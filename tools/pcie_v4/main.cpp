/*
 * Linux PCIe 视频查看器，Qt 5 / Qt 6，配套 my_pciedriver.c ABI v2。
 * 对应 IMAGE_ACK_20261017：保留 read/mmap 和显示缓冲优化。
 * 新驱动使用独立 DMA 邮箱确认 RECEIVED / APPLIED，超时保留在途事务。
 * 兼容驱动 7.0 的单向提交；这种模式只显示已提交，不伪造硬件确认。
 * 编辑不自动发送，不自动重发；底部日志有行数上限，不逐帧输出。
 * 控制线程串行处理 STOP、PUT 和配置，STOP 优先。
 *
 * read：DMA -> 内核副本 -> QImage 像素内存，两次采集数据复制。
 * mmap 显示缓冲：GET_FRAME 后，接收线程把需要显示的帧顺序复制
 *       到普通 QImage 内存，再归还 DMA 槽；Qt 不直接缩放 DMA 映射。
 *       这是一次用户态像素复制，并非全程零复制。只准备 GUI 需要的新帧。
 * mmap 直接显示（对照）：QImage 引用映射地址，最后一个消费者结束后 PUT。
 *       不代表窗口缩放/合成也完全没有复制；在部分 ARM 平台反而更慢。
 * DMA-BUF + RGA（ARM64 默认）：四槽导出 fd，不建立 misc mmap；
 *       仅缩放 GUI 请求的新帧。RGB565 -> 小尺寸 RGB565，Qt 无需 BGR。
 *       三块整页独占的普通内存预览池；同步任务完成后归还 FPGA 槽。
 *       RGA 错误先请求 STOP，不能把未知完成状态的源槽提前归还。
 *
 * 切换方式必须停止后重新 open；驱动不允许同一次 open 混用不同方式。
 * 按本机 RGB565 字节序显示，像素字节序和图像内容仍需上板验证。
 *
 * 线程：
 * GUI 负责窗口和绘图；CaptureSession::run 是控制线程；
 * 内部另有接收线程执行阻塞 read/GET_FRAME。
 * 控制线程可在接收阻塞时，通过同一个 fd 发出 STOP。
 *
 * mmap 直接显示的帧寿命：
 * mailbox 和显示窗口保存 shared_ptr<VideoFrame>。
 * 最后一个引用销毁时，先释放 QImage 视图，再把归还请求排入队列；
 * 控制线程发送 PUT，绝不在 GUI 线程执行可能阻塞的 ioctl。
 * 显示窗口只在 paintEvent 同步绘制，不单独保留外部 QImage 副本。
 * 显示缓冲模式复制结束后即可归还 DMA，窗口只保留普通内存中的副本。
 * 将来接 GPU/RGA/其他异步消费者时，必须让其完成事件也持有 VideoFrame。
 *
 * 停止顺序：请求停止并清空显示 -> STOP -> 接收线程退出 ->
 * 所有 VideoFrame 归还完成 -> munmap -> close。不使用 QThread::terminate。
 * 运行：./pcie_viewer [设备节点]；省略节点时查找唯一 /dev/fpga_frame_*。
 */
#include <QApplication>
#include <QByteArray>
#include <QCloseEvent>
#include <QComboBox>
#include <QCommandLineParser>
#include <QDir>
#include <QDateTime>
#include <QElapsedTimer>
#include <QHBoxLayout>
#include <QImage>
#include <QMessageBox>
#include <QMutex>
#include <QMutexLocker>
#include <QPainter>
#include <QPushButton>
#include <QPlainTextEdit>
#include <QThread>
#include <QTimer>
#include <QWidget>
#include <QCheckBox>
#include <QGridLayout>
#include <QLabel>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QVBoxLayout>

#include <array>
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <functional>
#include <cstdlib>

#include <fcntl.h>
#include <linux/types.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "fpga_uapi.h"
#include "image_protocol.h"
#ifndef PCIE_HAVE_RGA
#define PCIE_HAVE_RGA 0
#endif
#if PCIE_HAVE_RGA
#include <rga/im2d.h>
#include <rga/rga.h>
#endif

static_assert(sizeof(fpga_info) == 72, "fpga_info ABI mismatch");
static_assert(sizeof(fpga_frame) == 32, "fpga_frame ABI mismatch");
static_assert(sizeof(fpga_image_submit) == 32, "config ABI mismatch");
static_assert(sizeof(fpga_caps) == 16, "caps ABI mismatch");
static_assert(sizeof(fpga_image_ack_status) == 32, "image ACK ABI mismatch");

constexpr int FrameWidth = 1920;
constexpr int FrameHeight = 1080;
constexpr int FrameBytes = FrameWidth * FrameHeight * 2;
// 保留 Mmap 的原始语义，便于与 v2 直接映射显示做对比。
enum class ReceiveMode { Read, Mmap, MmapBuffered, RgaScaled };

/*
 * Qt 直接支持 RGB565，无需先转 BGR。只把需要显示的帧交给 RGA 缩放。
 * 32x18 为一个 16:9 单元，保证 RGB565 行跨度满足 RGA 的 4 字节对齐。
 * 最多输出原图尺寸；极大的窗口仍由 Qt 完成最后的放大。
 */
static QSize previewSize(QSize viewport)
{
    // RGA2 最多缩小 16 倍；最小 128x72，避免极小窗口超出缩放能力。
    const int units = std::max(4, std::min({viewport.width() / 32,
                                          viewport.height() / 18, 60}));
    return QSize(units * 32, units * 18);
}

static QImage allocatePreview(QSize size)
{
    // 整页独占：RGA 映射/维护缓存时，不与其他堆对象或 malloc 元数据共页。
    const long page = ::sysconf(_SC_PAGESIZE);
    if (page <= 0)
        return {};
    const size_t stride = static_cast<size_t>(size.width()) * 2;
    const size_t bytes = stride * size.height();
    const size_t allocated = (bytes + page - 1) / page * page;
    void *pixels = nullptr;
    if (::posix_memalign(&pixels, static_cast<size_t>(page), allocated) != 0)
        return {};
    std::memset(pixels, 0, allocated);
    QImage image(static_cast<uchar *>(pixels), size.width(), size.height(),
                 static_cast<int>(stride), QImage::Format_RGB16,
                 [](void *data) { std::free(data); }, pixels);
    if (image.isNull())
        std::free(pixels);
    return image; // QImage 的最后一个共享引用销毁时才释放像素。
}

static QString scalePreview(int sourceFd, QImage &destination)
{
#if PCIE_HAVE_RGA
    const auto source = wrapbuffer_fd_t(sourceFd, FrameWidth, FrameHeight,
                                        FrameWidth, FrameHeight, RK_FORMAT_RGB_565);
    const auto target = wrapbuffer_virtualaddr_t(destination.bits(), destination.width(),
        destination.height(), destination.bytesPerLine() / 2,
        destination.height(), RK_FORMAT_RGB_565);
    // sync=1：成功返回前不可发布预览，也不可归还 FPGA 源槽。
    const auto status = imresize_t(source, target, 0, 0, IM_INTERP_DEFAULT, 1);
    if (status != IM_STATUS_SUCCESS)
        return QStringLiteral("RGA 预览缩放失败：") + QString::fromLocal8Bit(imStrError_t(status));
    return {};
#else
    (void)sourceFd;
    (void)destination;
    return QStringLiteral("本次构建未启用 RGA；请选择 read/mmap 或重新编译");
#endif
}

static QString systemError(const QString &operation, int error)
{
    return operation + QStringLiteral("：") +
           QString::fromLocal8Bit(std::strerror(error));
}

static int deviceIoctl(int fd, unsigned long command, void *argument = nullptr)
{
    int result;
    do {
        result = ::ioctl(fd, command, argument);
    } while (result < 0 && errno == EINTR);
    return result;
}

/* 所有 return 都会执行此析构，避免部分 mmap 失败时遗留映射。 */
struct DeviceMemory {
    int fd = -1;
    fpga_info info{};
    std::array<void *, FPGA_FRAME_SLOTS> mapped{};
    std::array<int, FPGA_FRAME_SLOTS> exported{};

    DeviceMemory() { mapped.fill(MAP_FAILED); exported.fill(-1); }
    DeviceMemory(const DeviceMemory &) = delete;
    DeviceMemory &operator=(const DeviceMemory &) = delete;
    ~DeviceMemory()
    {
        for (void *address : mapped) {
            if (address != MAP_FAILED)
                ::munmap(address, static_cast<size_t>(info.map_bytes));
        }
        // 接收线程已结束，RGA 同步任务已完成，才释放导出引用和控制 fd。
        for (int descriptor : exported)
            if (descriptor >= 0)
                ::close(descriptor);
        if (fd >= 0)
            ::close(fd);
    }
};

/*
 * QImage 的释放线程不确定，因此归还请求只排队，不直接 ioctl。
 * 每槽最多一个待归还请求：驱动收到 PUT 前不可能再次交付同一槽。
 */
struct SessionControl {
    std::atomic<bool> stopRequested{false};
    std::mutex mutex;
    std::condition_variable changed;
    bool readerDone = false;
    // GUI 只排队；控制线程是配置 ioctl 的唯一调用者。
    bool configQueued = false;
    bool configBusy = false; // 确认模式下直到 APPLIED 才解除忙标记。
    bool configReady = false;
    fpga_image_submit nextConfig{};
    QString configMessage = QStringLiteral("正在准备参数接口");
    // 后台线程只排队文本，GUI 定时取走；两个队列都限制大小，避免积累。
    std::deque<QString> logs;
    unsigned int activeLeases = 0;
    std::array<bool, FPGA_FRAME_SLOTS> pending{};
    std::array<fpga_frame, FPGA_FRAME_SLOTS> returned{};

    bool hasReturns() const
    {
        for (bool value : pending) {
            if (value)
                return true;
        }
        return false;
    }

    void acquireLease()
    {
        std::lock_guard<std::mutex> guard(mutex);
        ++activeLeases;
    }

    void returnLease(const fpga_frame &frame)
    {
        {
            std::lock_guard<std::mutex> guard(mutex);
            returned[frame.slot] = frame;
            pending[frame.slot] = true;
        }
        changed.notify_all();
    }

    void requestStop()
    {
        {
            std::lock_guard<std::mutex> guard(mutex);
            stopRequested.store(true);
        }
        changed.notify_all();
    }
};

/* 图像与槽使用权一起传递，不能仅取出 image 后丢掉 VideoFrame。 */
struct VideoFrame {
    QImage image;
    std::shared_ptr<SessionControl> owner;
    fpga_frame lease{};

    ~VideoFrame()
    {
        image = QImage(); // 先结束本对象的像素视图
        if (owner)
            owner->returnLease(lease);
    }
};
using FramePtr = std::shared_ptr<VideoFrame>;

class CaptureSession final : public QThread
{
public:
    CaptureSession(QString device, ReceiveMode mode, QObject *parent)
        : QThread(parent), device_(std::move(device)), mode_(mode),
          control_(std::make_shared<SessionControl>())
    {
    }

    void requestStop()
    {
        control_->requestStop();
        clearLatest();
    }

    bool submitImage(const fpga_image_submit &config)
    {
        {
            std::lock_guard<std::mutex> guard(control_->mutex);
            if (isStopping() || !control_->configReady ||
                control_->configQueued || control_->configBusy)
                return false;
            control_->nextConfig = config;
            control_->configQueued = true;
            control_->configBusy = true;
            control_->configMessage = QStringLiteral("等待发送参数");
        }
        control_->changed.notify_all();
        return true;
    }

    QString imageMessage(bool &busy)
    {
        std::lock_guard<std::mutex> guard(control_->mutex);
        busy = isStopping() || !control_->configReady ||
               control_->configQueued || control_->configBusy;
        return control_->configMessage;
    }

    QStringList takeLogs()
    {
        QStringList result;
        std::lock_guard<std::mutex> guard(control_->mutex);
        for (const auto &line : control_->logs)
            result.append(line);
        control_->logs.clear();
        return result;
    }

    bool isStopping() const { return control_->stopRequested.load(); }

    void requestPreview(QSize viewport)
    {
        const QSize size = previewSize(viewport);
        const quint64 packed = (quint64(size.width()) << 32) | quint32(size.height());
        if (previewGeometry_.exchange(packed) != packed)
            displayRequested_.store(true);
    }

    // 只统计完整帧；短读片段、帧超时和参数提交不计数。
    quint64 receivedFrames() const
    {
        return receivedFrames_.load(std::memory_order_relaxed);
    }

    FramePtr takeLatest()
    {
        FramePtr frame;
        {
            QMutexLocker guard(&frameMutex_);
            frame.swap(latest_);
        }
        // GUI 取走上一张预览后才请求下一张，避免按输入帧率盲目复制。
        // 若 GUI 暂时忙，接收线程继续取帧并归还，不积压旧预览。
        if (frame && (mode_ == ReceiveMode::MmapBuffered || mode_ == ReceiveMode::RgaScaled))
            displayRequested_.store(true, std::memory_order_relaxed);
        return frame;
    }

    // 仅在 finished + wait 返回后读取。
    QString errorText() const { return error_; }

protected:
    void run() override
    {
        DeviceMemory memory;
        const QByteArray path = device_.toLocal8Bit();
        memory.fd = ::open(path.constData(), O_RDONLY | O_CLOEXEC);
        if (memory.fd < 0) {
            error_ = systemError(QStringLiteral("无法打开 %1").arg(device_),
                                 errno);
            return;
        }
        log(QStringLiteral("已打开设备：%1").arg(device_));
        if (isStopping())
            return;

        // read 模式也核对 ABI，避免搭配旧驱动却误认为调参已支持。
        if (deviceIoctl(memory.fd, FPGA_IOC_GET_INFO, &memory.info) < 0) {
            error_ = systemError(QStringLiteral("查询驱动接口失败"), errno);
            return;
        }
        if (memory.info.abi_version != FPGA_ABI_VERSION ||
            memory.info.width != FrameWidth || memory.info.height != FrameHeight ||
            memory.info.image_bytes != FrameBytes) {
            error_ = QStringLiteral("请加载视频 ABI v2 驱动");
            return;
        }

        // 视频 ABI 2 保持兼容，参数能力单独查询；这不是探测 FPGA 版本。
        fpga_caps caps{};
        if (deviceIoctl(memory.fd, FPGA_IOC_GET_CAPS, &caps) < 0 ||
            caps.version != FPGA_CAPS_VERSION ||
            !(caps.flags & FPGA_CAP_IMAGE_SUBMIT) || caps.reserved[0] || caps.reserved[1]) {
            error_ = QStringLiteral("请加载驱动 7.0 或更新版本（缺少参数提交能力）");
            return;
        }
        const bool ackMode = (caps.flags & FPGA_CAP_IMAGE_ACK) != 0;
        log(ackMode ? QStringLiteral("驱动支持参数两阶段确认；仍需匹配的 FPGA 位流")
                    : QStringLiteral("兼容单向参数接口：只能确认已提交，不能确认硬件生效"));

        if (mode_ == ReceiveMode::RgaScaled) {
            error_ = exportBuffers(memory, caps);
            if (!error_.isEmpty())
                return;
            if (isStopping())
                return;
            log(QStringLiteral("RGA：已导出四槽，不建立 misc mmap；按窗口需求缩放 RGB565"));
        } else if (mode_ != ReceiveMode::Read) {
            error_ = mapBuffers(memory);
            if (!error_.isEmpty()) {
                appendError(stopDevice(memory.fd));
                return;
            }
            if (isStopping())
                return;
        }

        // 接收资源准备后再登记邮箱；未 START 的配置 DMA 也必须由 STOP 排空。
        if (ackMode && deviceIoctl(memory.fd, FPGA_IOC_PREPARE_IMAGE_ACK) < 0) {
            error_ = systemError(QStringLiteral("准备参数回执邮箱失败"), errno);
            appendError(stopDevice(memory.fd));
            return;
        }
        if (ackMode)
            log(QStringLiteral("回执邮箱登记命令已发送；首个 RECEIVED 才能证实回路有效"));
        if (deviceIoctl(memory.fd, FPGA_IOC_START) < 0) {
            error_ = systemError(QStringLiteral("启动采集失败"), errno);
            appendError(stopDevice(memory.fd));
            return;
        }
        log(QStringLiteral("START 成功，开始接收视频"));

        QString readError;
        std::thread reader;
        try {
            reader = std::thread([this, &memory, &readError] {
                try {
                    readError = mode_ == ReceiveMode::Read
                                    ? receiveCopied(memory.fd)
                                    : receiveMapped(memory);
                } catch (const std::exception &error) {
                    readError = QStringLiteral("接收线程异常：") +
                                QString::fromLocal8Bit(error.what());
                }
                {
                    std::lock_guard<std::mutex> guard(control_->mutex);
                    control_->readerDone = true;
                }
                control_->changed.notify_all();
            });
        } catch (const std::exception &error) {
            error_ = QStringLiteral("无法创建接收线程：") +
                     QString::fromLocal8Bit(error.what());
            appendError(stopDevice(memory.fd));
            return;
        }

        /*
         * 同一个控制线程处理 STOP 和 PUT。
         * 即使读取线程已经退出，GUI 还可能持有最后一帧；
         * 必须等所有引用释放并处理完归还队列后，才能 munmap/close。
         */
        bool stopIssued = false;
        bool imagePending = false;
        bool imageTimedOut = false;
        bool imageQueryFailed = false;
        __u32 imageCookie = 0, imageFlags = 0;
        QElapsedTimer imageClock;
        QElapsedTimer ackPollClock;
        QString stopError;
        QString returnError;
        {
            std::lock_guard<std::mutex> guard(control_->mutex);
            control_->configReady = true;
            control_->configMessage = ackMode
                ? QStringLiteral("可提交参数；等待真实 RECEIVED / APPLIED 确认")
                : QStringLiteral("可提交参数；当前驱动不回传生效确认");
        }
        for (;;) {
            fpga_frame returned{};
            bool doStop = false;
            bool doReturn = false;
            bool doConfig = false;
            fpga_image_submit config{};
            {
                std::unique_lock<std::mutex> guard(control_->mutex);
                control_->changed.wait_for(guard, std::chrono::milliseconds(imagePending ? 20 : 250),
                                          [this, &stopIssued] {
                    return (!stopIssued &&
                            (isStopping() || control_->readerDone)) ||
                           control_->hasReturns() ||
                           (!stopIssued && control_->configQueued) ||
                           (stopIssued && control_->readerDone &&
                            control_->activeLeases == 0);
                });

                if (!stopIssued && (isStopping() || control_->readerDone)) {
                    control_->stopRequested.store(true);
                    stopIssued = true;
                    doStop = true;
                } else if (control_->hasReturns()) {
                    for (unsigned int i = 0; i < FPGA_FRAME_SLOTS; ++i) {
                        if (control_->pending[i]) {
                            returned = control_->returned[i];
                            control_->pending[i] = false;
                            doReturn = true;
                            break;
                        }
                    }
                } else if (!stopIssued && control_->configQueued) {
                    config = control_->nextConfig;
                    control_->configQueued = false;
                    doConfig = true;
                } else if (stopIssued && control_->readerDone &&
                           control_->activeLeases == 0) {
                    break;
                }
            }

            if (doStop) {
                log(QStringLiteral("发送 STOP，等待视频与参数回执 DMA 排空"));
                stopError = stopDevice(memory.fd);
                log(stopError.isEmpty() ? QStringLiteral("STOP 已确认；本次邮箱登记已失效")
                                        : stopError);
                if (imagePending)
                    log(QStringLiteral("结束尚未确认的参数等待；不据此判断该参数是否曾生效"));
                imagePending = false;
                clearLatest();
                // GUI 的定时器发现 isStopping 后会清空正在显示的帧。
            } else if (doReturn) {
                if (deviceIoctl(memory.fd, FPGA_IOC_PUT_FRAME, &returned) < 0 &&
                    returnError.isEmpty()) {
                    returnError = systemError(QStringLiteral("归还图像失败"),
                                              errno);
                    log(returnError);
                    control_->requestStop();
                }
                {
                    std::lock_guard<std::mutex> guard(control_->mutex);
                    --control_->activeLeases;
                }
                control_->changed.notify_all();
            } else if (doConfig) {
                const auto command = ackMode ? FPGA_IOC_SUBMIT_IMAGE_ACK
                                             : FPGA_IOC_SUBMIT_IMAGE;
                if (deviceIoctl(memory.fd, command, &config) < 0) {
                    const int submitError = errno;
                    // EBUSY 可能代表设备仍有未完成事务，不应通过连点覆盖。
                    setImageMessage(systemError(QStringLiteral("参数提交失败"), submitError),
                                    ackMode && submitError == EBUSY);
                    log(systemError(QStringLiteral("参数提交失败"), submitError));
                } else if (ackMode) {
                    imagePending = true;
                    imageTimedOut = imageQueryFailed = false;
                    imageCookie = imageFlags = 0;
                    imageClock.start();
                    ackPollClock.invalidate();
                    setImageMessage(QStringLiteral("参数已提交，正在查询提交编号"), true);
                } else {
                    setImageMessage(QStringLiteral("参数已提交；FPGA 不回传生效确认"), false);
                    log(QStringLiteral("参数已提交（单向接口，无硬件生效回执）"));
                }
            }
            // 与 PUT 共用控制线程；不在接收/GUI 线程阻塞等回执，STOP 始终优先。
            if (!stopIssued && imagePending &&
                (!ackPollClock.isValid() || ackPollClock.elapsed() >= 20)) {
                ackPollClock.start(); // PUT 高频归还时也不超过每秒 50 次查询
                fpga_image_ack_status status{};
                if (deviceIoctl(memory.fd, FPGA_IOC_GET_IMAGE_ACK, &status) < 0) {
                    if (!imageQueryFailed) {
                        const QString message = systemError(QStringLiteral("查询参数回执失败"), errno)
                            + QStringLiteral("；请停止后重试，不自动重发");
                        setImageMessage(message, true);
                        log(message);
                    }
                    imageQueryFailed = true;
                } else if (!status.cookie || (imageCookie && status.cookie != imageCookie) ||
                           status.reserved || (status.flags & FPGA_IMAGE_STOPPED)) {
                    setImageMessage(QStringLiteral("参数事务编号/状态异常，请停止后重试"), true);
                    if (!imageQueryFailed)
                        log(QStringLiteral("参数事务编号/状态异常，保留在途事务"));
                    imageQueryFailed = true;
                } else {
                    if (!imageCookie) {
                        imageCookie = status.cookie;
                        log(QStringLiteral("参数 #%1 已提交，等待 FPGA 接收").arg(imageCookie));
                        setImageMessage(QStringLiteral("参数 #%1 已提交，等待接收").arg(imageCookie), true);
                    }
                    if ((status.flags & FPGA_IMAGE_RECEIVED) && !(imageFlags & FPGA_IMAGE_RECEIVED)) {
                        log(QStringLiteral("参数 #%1：FPGA 已接收（RECEIVED）").arg(imageCookie));
                        setImageMessage(QStringLiteral("参数 #%1 已接收，等待帧边界生效").arg(imageCookie), true);
                    }
                    imageFlags |= status.flags;
                    if ((imageFlags & (FPGA_IMAGE_RECEIVED | FPGA_IMAGE_APPLIED)) ==
                        (FPGA_IMAGE_RECEIVED | FPGA_IMAGE_APPLIED)) {
                        const QString message = QStringLiteral("参数 #%1 已生效（APPLIED%2）")
                            .arg(imageCookie).arg(imageTimedOut ? QStringLiteral("，迟到回执") : QString());
                        setImageMessage(message, false);
                        log(message + QStringLiteral("；不代表该帧已在窗口显示"));
                        imagePending = false;
                    }
                }
                if (imagePending && !imageTimedOut && imageClock.elapsed() >= 3000) {
                    imageTimedOut = true;
                    const QString message = QStringLiteral("参数确认超时（3 秒）；继续观察迟到回执，禁止覆盖，请停止后恢复");
                    setImageMessage(message, true);
                    log(message);
                }
            }
        }

        reader.join();
        error_ = readError;
        appendError(stopError);
        appendError(returnError);
        // memory 析构：先 munmap 所有槽，再 close。此时没有图像引用。
    }

private:
    void log(const QString &message)
    {
        const QString line = QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz "))
                             + message;
        std::lock_guard<std::mutex> guard(control_->mutex);
        if (control_->logs.size() >= 500)
            control_->logs.pop_front();
        control_->logs.push_back(line);
    }

    void setImageMessage(const QString &message, bool busy)
    {
        std::lock_guard<std::mutex> guard(control_->mutex);
        // 保留排队请求的忙标记，消息和状态始终由同一把锁保护。
        if (control_->configQueued)
            return;
        control_->configMessage = message;
        control_->configBusy = busy;
    }

    void appendError(const QString &text)
    {
        if (text.isEmpty())
            return;
        if (!error_.isEmpty())
            error_ += QStringLiteral("\n");
        error_ += text;
    }

    void clearLatest()
    {
        // 析构旧帧会取 control_->mutex，不要在持有该锁时调用本函数。
        FramePtr frame = takeLatest();
    }

    void publish(FramePtr frame)
    {
        {
            QMutexLocker guard(&frameMutex_);
            if (!isStopping()) {
                latest_.swap(frame);
            }
        }
        // 旧 mailbox 帧在锁外析构，mmap 帧会自动排入归还队列。
    }

    QString mapBuffers(DeviceMemory &memory)
    {
        if (deviceIoctl(memory.fd, FPGA_IOC_GET_INFO, &memory.info) < 0)
            return systemError(QStringLiteral("查询 mmap 接口失败"), errno);
        const fpga_info &info = memory.info;
        const long pageSize = ::sysconf(_SC_PAGESIZE);
        if (info.abi_version != FPGA_ABI_VERSION ||
            info.slot_count != FPGA_FRAME_SLOTS ||
            info.width != FrameWidth || info.height != FrameHeight ||
            info.stride != FrameWidth * 2 || info.image_bytes != FrameBytes ||
            info.pixel_format != FPGA_FORMAT_RGB565 || info.reserved ||
            pageSize <= 0 || info.map_bytes < FrameBytes + 64U ||
            info.map_bytes % static_cast<unsigned long>(pageSize) ||
            info.map_bytes > std::numeric_limits<size_t>::max() ||
            info.map_bytes >
                static_cast<unsigned long long>(
                    std::numeric_limits<off_t>::max()) / FPGA_FRAME_SLOTS) {
            return QStringLiteral("驱动版本、图像格式或映射布局不匹配");
        }

        for (unsigned int i = 0; i < FPGA_FRAME_SLOTS; ++i) {
            if (info.offset[i] != info.map_bytes * i)
                return QStringLiteral("驱动返回的槽偏移不匹配");
            for (int attempt = 0; attempt < 100; ++attempt) {
                if (isStopping())
                    return {};
                memory.mapped[i] = ::mmap(
                    nullptr, static_cast<size_t>(info.map_bytes),
                    PROT_READ, MAP_SHARED, memory.fd,
                    static_cast<off_t>(info.offset[i]));
                if (memory.mapped[i] != MAP_FAILED)
                    break;
                const int mapError = errno;
                if ((mapError != EAGAIN && mapError != EINTR) || attempt == 99)
                    return systemError(QStringLiteral("映射槽 %1 失败").arg(i),
                                       mapError);
                QThread::msleep(1);
            }
        }
        return {};
    }

    QString exportBuffers(DeviceMemory &memory, const fpga_caps &caps)
    {
        const auto &info = memory.info;
        if (!PCIE_HAVE_RGA || !(caps.flags & FPGA_CAP_DMABUF))
            return QStringLiteral("RGA 模式需要启用 RGA 的上位机和支持 DMA-BUF 的 8.2 驱动");
        if (info.slot_count != FPGA_FRAME_SLOTS || info.stride != FrameWidth * 2 ||
            info.pixel_format != FPGA_FORMAT_RGB565 || info.reserved)
            return QStringLiteral("RGA 输入格式不匹配");
        // 在 PREPARE/START 之前导出；部分失败由 DeviceMemory 统一关闭。
        for (unsigned slot = 0; slot < FPGA_FRAME_SLOTS; ++slot) {
            fpga_dmabuf_export request{};
            request.slot = slot;
            request.fd = -1;
            if (deviceIoctl(memory.fd, FPGA_IOC_EXPORT_DMABUF, &request) < 0)
                return systemError(QStringLiteral("导出 DMA-BUF 槽 %1 失败").arg(slot), errno);
            if (request.fd < 0)
                return QStringLiteral("驱动返回无效 DMA-BUF fd");
            memory.exported[slot] = request.fd;
        }
        return {};
    }

    QString stopDevice(int fd)
    {
        int lastError = 0;
        for (int attempt = 0; attempt < 3; ++attempt) {
            if (deviceIoctl(fd, FPGA_IOC_STOP) == 0)
                return {};
            lastError = errno;
            if (lastError != ETIMEDOUT)
                break;
            log(QStringLiteral("STOP 等待超时（第 %1 次）；继续等待原编号，不重新发送命令")
                .arg(attempt + 1));
        }
        return systemError(QStringLiteral("停止采集未得到确认"), lastError) +
               QStringLiteral("\n请检查设备；驱动会保留相关 DMA 资源。");
    }

    QString receiveCopied(int fd)
    {
        bool stalled = false;
        while (!isStopping()) {
            FramePtr frame = std::make_shared<VideoFrame>();
            frame->image = QImage(FrameWidth, FrameHeight, QImage::Format_RGB16);
            if (frame->image.isNull())
                return QStringLiteral("无法分配图像内存");
            if (frame->image.bytesPerLine() != FrameWidth * 2)
                return QStringLiteral("图像行跨度与驱动不一致");

            uchar *pixels = frame->image.bits();
            int received = 0;
            while (received < FrameBytes && !isStopping()) {
                const ssize_t count = ::read(fd, pixels + received,
                                            FrameBytes - received);
                if (count > 0) {
                    received += static_cast<int>(count);
                    continue;
                }
                if (isStopping())
                    return {};
                if (count == 0)
                    return QStringLiteral("设备已经停止输出图像");
                const int readError = errno;
                if (readError == ETIMEDOUT && !stalled) {
                    log(QStringLiteral("等待视频帧超时，继续接收；请检查 FPGA 视频输入与 DMA 状态"));
                    stalled = true;
                }
                if (readError == EINTR || readError == ETIMEDOUT)
                    continue;
                return systemError(QStringLiteral("读取图像失败"), readError);
            }
            if (isStopping())
                return {};
            noteFrameReceived(stalled);
            publish(std::move(frame));
        }
        return {};
    }

    QString receiveMapped(DeviceMemory &memory)
    {
        // 普通内存缓冲池：窗口、待显示帧、正在准备的帧各一个即可。
        // QImage 隐式共享：只有 isDetached() 为真时才可覆盖像素。
        // 因而 GUI 即使绘制较慢，也不会读到正在被接收线程改写的画面。
        std::array<QImage, 3> displayBuffers;
        const bool useRga = mode_ == ReceiveMode::RgaScaled;
        QElapsedTimer performance;
        performance.start();
        quint64 scaled = 0;
        qint64 scaleNs = 0;
        bool stalled = false;
        while (!isStopping()) {
            fpga_frame lease{};
            if (deviceIoctl(memory.fd, FPGA_IOC_GET_FRAME, &lease) < 0) {
                const int getError = errno;
                if (isStopping())
                    return {};
                if (getError == ETIMEDOUT && !stalled) {
                    log(QStringLiteral("等待完整 DMA 帧超时，继续接收；请检查 FPGA 输入与帧尾回写"));
                    stalled = true;
                }
                if (getError == EINTR || getError == ETIMEDOUT)
                    continue;
                if (getError == ENOBUFS) {
                    // 所有槽暂被显示或归还队列持有；休眠，避免忙轮询。
                    std::unique_lock<std::mutex> guard(control_->mutex);
                    control_->changed.wait_for(guard, std::chrono::milliseconds(5));
                    continue;
                }
                return systemError(QStringLiteral("获取映射帧失败"), getError);
            }

            if (lease.slot >= FPGA_FRAME_SLOTS || !lease.session ||
                !(lease.token >> 2) || (lease.token & 3U) != lease.slot ||
                lease.sequence != (lease.token >> 2) ||
                lease.bytes_used != FrameBytes || lease.reserved) {
                // 不访问非法描述指向的内存；控制线程将 STOP 并关闭会话。
                return QStringLiteral("驱动返回了无效的帧描述");
            }

            control_->acquireLease();
            FramePtr frame;
            try {
                frame = std::make_shared<VideoFrame>();
            } catch (...) {
                control_->returnLease(lease);
                throw;
            }
            frame->owner = control_;
            frame->lease = lease;
            if (!useRga) frame->image = QImage(
                static_cast<const uchar *>(memory.mapped[lease.slot]),
                FrameWidth, FrameHeight, FrameWidth * 2, QImage::Format_RGB16);
            if (!useRga && frame->image.isNull())
                return QStringLiteral("无法创建映射图像视图");
            if (isStopping())
                return {}; // frame 析构时归还，不遗留刚取得的槽
            // 采集 FPS 统计完整 GET，不把为保持低延迟而跳过的预览漏算。
            noteFrameReceived(stalled);
            if (mode_ == ReceiveMode::Mmap) {
                publish(std::move(frame));
                continue;
            }

            // 最多准备一个尚未被 GUI 取走的预览；其他帧直接归还槽。
            if (!displayRequested_.exchange(false, std::memory_order_relaxed))
                continue;
            const quint64 geometry = previewGeometry_.load();
            const QSize wanted = useRga ? QSize(int(geometry >> 32), int(quint32(geometry)))
                                        : QSize(FrameWidth, FrameHeight);
            QImage *destination = nullptr;
            for (QImage &buffer : displayBuffers) {
                if (buffer.isNull() || buffer.isDetached()) {
                    if (buffer.size() != wanted) {
                        buffer = useRga ? allocatePreview(wanted)
                                        : QImage(wanted, QImage::Format_RGB16);
                        if (buffer.isNull())
                            return QStringLiteral("无法分配显示缓冲区");
                    }
                    destination = &buffer;
                    break;
                }
            }
            if (!destination) {
                // 消费者仍持有全部副本时跳过当前帧，不额外分配大块内存。
                displayRequested_.store(true, std::memory_order_relaxed);
                continue;
            }

            FramePtr preview = std::make_shared<VideoFrame>();
            // 顺序读 DMA 一遍，后续缩放和重绘都访问普通内存。
            // 复制在接收线程执行，不放进 GUI 的 paintEvent。
            if (useRga) {
                QElapsedTimer conversion;
                conversion.start();
                QString scaleError;
                try {
                    scaleError = scalePreview(memory.exported[lease.slot], *destination);
                } catch (...) {
                    control_->requestStop(); // 未知完成状态先 STOP，不能先 PUT。
                    throw;
                }
                if (!scaleError.isEmpty()) {
                    control_->requestStop();
                    return scaleError;
                }
                scaleNs += conversion.nsecsElapsed();
                ++scaled;
                if (performance.elapsed() >= 5000) {
                    log(QStringLiteral("RGA 预览 %1×%2：%3 次/秒，平均 %4 ms；仅处理显示需要的帧")
                        .arg(wanted.width()).arg(wanted.height())
                        .arg(scaled * 1000.0 / performance.elapsed(), 0, 'f', 1)
                        .arg(double(scaleNs) / scaled / 1000000.0, 0, 'f', 2));
                    scaled = 0; scaleNs = 0; performance.restart();
                }
            } else {
                std::memcpy(destination->bits(), frame->image.constBits(), FrameBytes);
            }
            preview->image = *destination; // 共享普通内存，不再复制像素。
            frame.reset(); // 结束映射视图并排队 PUT，预览不再持有 DMA 槽。
            if (isStopping())
                return {};
            publish(std::move(preview));
        }
        return {};
    }

    void noteFrameReceived(bool &stalled)
    {
        // 每帧只累加计数，只有首帧或超时恢复时输出一次日志。
        if (receivedFrames_.fetch_add(1, std::memory_order_relaxed) == 0)
            log(QStringLiteral("收到第一幅完整图像：1920×1080 RGB565"));
        else if (stalled)
            log(QStringLiteral("视频帧接收已恢复"));
        stalled = false;
    }

    QString device_;
    ReceiveMode mode_;
    QString error_;
    std::shared_ptr<SessionControl> control_;
    QMutex frameMutex_;
    FramePtr latest_;
    std::atomic<quint64> receivedFrames_{0};
    std::atomic<bool> displayRequested_{true};
    std::atomic<quint64> previewGeometry_{(quint64(960) << 32) | 540U};
};

// 简单黑底图像窗口；只同步读取当前 VideoFrame 的像素。
class VideoWindow final : public QWidget
{
public:
    explicit VideoWindow(QWidget *parent) : QWidget(parent, Qt::Window)
    {
        setWindowTitle(QStringLiteral("PCIe 图像"));
        resize(960, 540);
        setMinimumSize(320, 180);
        // paintEvent 会填满整个窗口，不必让 Qt 再预先擦一次背景。
        setAttribute(Qt::WA_OpaquePaintEvent);
    }

    std::function<void()> onClosed;
    void setFrame(FramePtr frame)
    {
        frame_ = std::move(frame);
        freshFrame_ = bool(frame_);
        update();
    }

    quint64 paintedFrames() const { return paintedFrames_; }
    quint64 paintingNs() const { return paintingNs_; }
    void resetStatistics()
    {
        paintedFrames_ = 0;
        paintingNs_ = 0;
        freshFrame_ = false;
    }

protected:
    void paintEvent(QPaintEvent *) override
    {
        QElapsedTimer drawing;
        drawing.start();
        QPainter painter(this);
        painter.fillRect(rect(), Qt::black);
        if (!frame_ || frame_->image.isNull())
            return;
        const QImage &image = frame_->image;
        const QSize size = image.size().scaled(this->size(), Qt::KeepAspectRatio);
        const QRect target((width() - size.width()) / 2,
                           (height() - size.height()) / 2,
                           size.width(), size.height());
        painter.drawImage(target, image);
        // 同一张图因遮挡、缩放而重绘不重复计数；只算真正绘制的新帧。
        if (freshFrame_) {
            ++paintedFrames_;
            paintingNs_ += quint64(drawing.nsecsElapsed());
            freshFrame_ = false;
        }
        // GUI 单线程：绘制完成后，后续 setFrame 才可能释放这一帧。
    }

    void closeEvent(QCloseEvent *event) override
    {
        event->accept();
        if (onClosed)
            onClosed();
    }

private:
    FramePtr frame_;
    quint64 paintedFrames_ = 0;
    quint64 paintingNs_ = 0;
    bool freshFrame_ = false;
};


/*
 * 参数只在点击“应用参数”时发送，不因拖动控件连续提交。
 * 场景预设来自 FPGA 新版指南，是待验证的起点；不是自动场景识别。
 */
class ImageControls final : public QWidget
{
public:
    explicit ImageControls(QWidget *parent) : QWidget(parent)
    {
        auto *outer = new QVBoxLayout(this);
        auto *presets = new QHBoxLayout;
        preset_ = new QComboBox(this);
        const char *names[] = {
            "全旁路", "温和对比度 / 高光保护", "背光提亮", "雾天：仅去雾",
            "雾天：综合增强", "夜间 / 隧道", "轻度降噪",
            "仅暗光增强", "仅高光压制", "暗光增强 + 高光压制"
        };
        for (const char *name : names)
            preset_->addItem(QString::fromUtf8(name));
        presets->addWidget(new QLabel(QStringLiteral("场景预设"), this));
        presets->addWidget(preset_, 1);
        outer->addLayout(presets);

        const char *switches[] = {
            "去雾 DCP", "Gamma", "CLAHE", "高光保护", "亮度降噪",
            "透射率滤波", "天空保护", "防光晕", "阴影保护",
            "原高光压制", "原暗光增强"
        };
        auto *grid = new QGridLayout;
        for (int i = 0; i < 11; ++i) {
            checks_[i] = new QCheckBox(QString::fromUtf8(switches[i]), this);
            grid->addWidget(checks_[i], i / 4, i % 4);
            connect(checks_[i], &QCheckBox::toggled, this,
                    [this] { markCustom(); });
        }
        outer->addLayout(grid);

        // 参数顺序对应 words() 中的打包顺序，范围与 FPGA 钳位规则一致。
        const char *labels[] = {
            "去雾下限", "天空下限", "去雾强度", "降噪混合",
            "降噪 sigma", "暗光阈值", "高光阈值", "CLAHE clip",
            "CLAHE 强度", "阴影增益", "提亮限制", "高光膝点", "高光斜率"
        };
        const int lo[] = {1, 1, 0, 0, 16, 0, 0, 1, 0, 0, 0, 0, 0};
        const int hi[] = {255, 255, 255, 256, 48, 255, 255, 65535,
                          4, 64, 255, 255, 256};
        auto *parameters = new QGridLayout;
        for (int i = 0; i < 13; ++i) {
            values_[i] = new QSpinBox(this);
            values_[i]->setRange(lo[i], hi[i]);
            if (i == 4)
                values_[i]->setSingleStep(16);
            parameters->addWidget(new QLabel(QString::fromUtf8(labels[i]), this),
                                  i / 3, (i % 3) * 2);
            parameters->addWidget(values_[i], i / 3, (i % 3) * 2 + 1);
            connect(values_[i], QOverload<int>::of(&QSpinBox::valueChanged),
                    this, [this] { markCustom(); });
        }
        outer->addLayout(parameters);
        auto *note = new QLabel(QStringLiteral(
            "参数作用于整张图像；修改后点击应用。天空下限不低于去雾下限；"
            "sigma 非 16/32/48 时按 32 处理。"), this);
        note->setWordWrap(true);
        outer->addWidget(note);
        connect(preset_, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, [this](int index) { if (index >= 0) loadPreset(index); });
        loadPreset(0);
    }

    fpga_image_submit words() const
    {
        fpga_image_submit result{};
        __u32 raw[5]{};
        for (int i = 0; i < 11; ++i)
            if (checks_[i]->isChecked())
                raw[0] |= 1U << bits_[i];
        const auto value = [this](int i) { return __u32(values_[i]->value()); };
        raw[1] = value(0) | (value(1) << 8) | (value(2) << 16);
        raw[2] = value(3) | (value(4) << 9) |
                 (value(5) << 16) | (value(6) << 24);
        raw[3] = value(7) | (value(8) << 16) | (value(9) << 19);
        raw[4] = value(10) | (value(11) << 8) | (value(12) << 16);
        fpga_cfg_sanitize(raw, result.words);
        return result; // reserved 留零；新接口的 cookie 由驱动生成
    }

private:
    void markCustom()
    {
        if (!loading_) {
            QSignalBlocker block(preset_);
            preset_->setCurrentIndex(-1);
        }
    }

    void loadPreset(int index)
    {
        static const __u32 presets[][5] = {
            {0x1c0, 0x00f38034, 0x00004100, 0x02030190, 0x0080c040},
            {0x1cc, 0x00f38034, 0x00004100, 0x010100c8, 0x0080c020},
            {0x1ce, 0x00f38034, 0x00004100, 0x010100c8, 0x0080c020},
            {0x1e1, 0x00e0a060, 0x00004100, 0x02030190, 0x0080c040},
            {0x1ff, 0x00e0a060, 0x00004080, 0x010100c8, 0x0080c020},
            {0x1de, 0x00f38034, 0x000040c0, 0x010100c8, 0x0080c020},
            {0x1d0, 0x00f38034, 0x00002080, 0x010100c8, 0x0080c020},
            {0x11c0,0x00f38034, 0x00604100, 0x02030190, 0x0080c040},
            {0x09c0,0x00f38034, 0xc0004100, 0x02030190, 0x0080c040},
            {0x19c0,0x00f38034, 0xc0604100, 0x02030190, 0x0080c040}
        };
        if (index < 0 || index >= 10)
            return;
        loading_ = true;
        const __u32 *w = presets[index];
        for (int i = 0; i < 11; ++i)
            checks_[i]->setChecked(w[0] & (1U << bits_[i]));
        const __u32 v[] = {
            w[1] & 255U, (w[1] >> 8) & 255U, (w[1] >> 16) & 255U,
            w[2] & 511U, (w[2] >> 9) & 63U, (w[2] >> 16) & 255U,
            w[2] >> 24, w[3] & 65535U, (w[3] >> 16) & 7U,
            (w[3] >> 19) & 127U, w[4] & 255U,
            (w[4] >> 8) & 255U, (w[4] >> 16) & 511U
        };
        for (int i = 0; i < 13; ++i)
            values_[i]->setValue(static_cast<int>(v[i]));
        loading_ = false;
    }

    const int bits_[11] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 11, 12};
    QComboBox *preset_ = nullptr;
    QCheckBox *checks_[11]{};
    QSpinBox *values_[13]{};
    bool loading_ = false;
};

class MainWindow final : public QWidget
{
public:
    explicit MainWindow(QString device, ReceiveMode initialMode =
                        PCIE_HAVE_RGA ? ReceiveMode::RgaScaled : ReceiveMode::MmapBuffered)
        : device_(std::move(device))
    {
        setWindowTitle(QStringLiteral("PCIe 视频 v4 · RGA 预览与参数日志"));
        resize(780, 720);
        modeBox_ = new QComboBox(this);
        modeBox_->addItem(QStringLiteral("read（复制）"), int(ReceiveMode::Read));
        modeBox_->addItem(QStringLiteral("mmap（显示缓冲）"), int(ReceiveMode::MmapBuffered));
        modeBox_->addItem(QStringLiteral("mmap（直接显示，对照）"), int(ReceiveMode::Mmap));
        if (PCIE_HAVE_RGA)
            modeBox_->addItem(QStringLiteral("DMA-BUF + RGA（预览缩放）"), int(ReceiveMode::RgaScaled));
        modeBox_->setCurrentIndex(std::max(0, modeBox_->findData(int(initialMode))));
        modeBox_->setToolTip(QStringLiteral(
            "显示缓冲：在接收线程复制一次到普通内存再绘图。\n"
            "直接显示：保持 v2 的映射视图，用于比较性能。\n"
            "RGA：只缩放需要显示的帧，RGB565 直接生成小预览，不转 BGR。"));
        startButton_ = new QPushButton(QStringLiteral("开始"), this);
        stopButton_ = new QPushButton(QStringLiteral("停止"), this);
        stopButton_->setEnabled(false);

        auto *outer = new QVBoxLayout(this);
        auto *layout = new QHBoxLayout;
        outer->addLayout(layout);
        layout->setContentsMargins(20, 24, 20, 24);
        layout->setSpacing(12);
        layout->addWidget(modeBox_);
        layout->addWidget(startButton_);
        layout->addWidget(stopButton_);

        fpsLabel_ = new QLabel(QStringLiteral("采集：0.0 FPS   显示：0.0 FPS"), this);
        outer->addWidget(fpsLabel_);
        imageControls_ = new ImageControls(this);
        outer->addWidget(imageControls_);
        applyButton_ = new QPushButton(QStringLiteral("应用参数"), this);
        applyButton_->setEnabled(false);
        outer->addWidget(applyButton_);
        configStatus_ = new QLabel(QStringLiteral("开始采集后可应用参数"), this);
        configStatus_->setWordWrap(true);
        outer->addWidget(configStatus_);
        auto *lifetime = new QLabel(QStringLiteral(
            "新驱动回传已接收/已生效；APPLIED 不代表窗口已显示对应帧。超时不自动重发。"), this);
        lifetime->setWordWrap(true);
        outer->addWidget(lifetime);
        outer->addWidget(new QLabel(QStringLiteral("运行日志"), this));
        logView_ = new QPlainTextEdit(this);
        logView_->setObjectName(QStringLiteral("runtimeLog"));
        logView_->setReadOnly(true);
        logView_->setMaximumBlockCount(500); // 超过上限自动淘汰最旧行
        logView_->setMinimumHeight(130);
        outer->addWidget(logView_, 1);
        appendLog(QStringLiteral("查看器就绪；请选择接收方式后开始采集"));
        connect(applyButton_, &QPushButton::clicked, this, [this] {
            if (session_ && session_->submitImage(imageControls_->words())) {
                applyButton_->setEnabled(false);
                configStatus_->setText(QStringLiteral("参数已排队"));
                appendLog(QStringLiteral("用户提交参数；等待控制线程发送"));
            }
        });

        video_ = new VideoWindow(this);
        video_->onClosed = [this] { stopCapture(); };
        connect(startButton_, &QPushButton::clicked,
                this, [this] { startCapture(); });
        connect(stopButton_, &QPushButton::clicked,
                this, [this] { stopCapture(); });

        // 原来的 33 ms 将更新频率限制在约 30 FPS；现在以约 60 Hz 取预览。
        // update() 可合并绘制事件，邮箱也只保留最新帧，不排队每一帧。
        timer_.setTimerType(Qt::PreciseTimer);
        timer_.setInterval(16);
        connect(&timer_, &QTimer::timeout, this, [this] {
            if (!session_)
                return;
            drainLogs(session_);
            if (stopping_)
                return;
            if (session_->isStopping()) {
                stopCapture(); // 后台异常时也释放 GUI 持有的 mmap 帧
                return;
            }
            bool pending = false;
            configStatus_->setText(session_->imageMessage(pending));
            applyButton_->setEnabled(!pending);
            // 仅传递窗口尺寸，后台自己处理缩放；GUI 不等待硬件任务。
            session_->requestPreview(video_->size());
            FramePtr frame = session_->takeLatest();
            if (frame)
                video_->setFrame(std::move(frame));
            updateFrameRate();
        });
    }

    ~MainWindow() override
    {
        if (session_) {
            session_->requestStop();
            video_->setFrame({}); // wait 前必须释放 GUI 的槽引用
            session_->wait();
        }
    }

protected:
    void closeEvent(QCloseEvent *event) override
    {
        if (session_) {
            event->ignore();
            closing_ = true;
            stopCapture();
        } else {
            event->accept();
        }
    }

private:
    // 只有 GUI 线程触碰文本控件；后台日志通过有界队列转交。
    void appendLog(const QString &message)
    {
        logView_->appendPlainText(QDateTime::currentDateTime().toString(
            QStringLiteral("HH:mm:ss.zzz ")) + message);
    }

    void drainLogs(CaptureSession *session)
    {
        for (const QString &line : session->takeLogs())
            logView_->appendPlainText(line); // 已包含事件发生时间，不能重复加时间
    }

    void updateFrameRate()
    {
        // QElapsedTimer 是单调时钟；系统校时不会造成负数或突增。
        // 每约一秒用实际经过时间求平均，不能直接把帧数当成 FPS。
        const qint64 ms = fpsClock_.elapsed();
        if (!session_ || ms < 1000)
            return;
        const quint64 received = session_->receivedFrames();
        const quint64 painted = video_->paintedFrames();
        const quint64 paintingNs = video_->paintingNs();
        fpsLabel_->setText(QStringLiteral("采集：%1 FPS   显示：%2 FPS")
            .arg(double(received - lastReceived_) * 1000.0 / double(ms), 0, 'f', 1)
            .arg(double(painted - lastPainted_) * 1000.0 / double(ms), 0, 'f', 1));
        if (performanceClock_.elapsed() >= 5000) {
            const auto newPaints = painted - lastPainted_;
            appendLog(QStringLiteral("%1：采集 %2 FPS，显示 %3 FPS，绘图平均 %4 ms（不含屏幕呈现等待）")
                .arg(modeBox_->currentText())
                .arg(double(received - lastReceived_) * 1000.0 / ms, 0, 'f', 1)
                .arg(double(newPaints) * 1000.0 / ms, 0, 'f', 1)
                .arg(newPaints ? double(paintingNs - lastPaintingNs_) / newPaints / 1000000.0 : 0, 0, 'f', 2));
            performanceClock_.restart();
        }
        lastReceived_ = received;
        lastPainted_ = painted;
        lastPaintingNs_ = paintingNs;
        fpsClock_.restart();
    }

    void startCapture()
    {
        if (session_)
            return;

        QString path = device_;
        if (path.isEmpty()) {
            const QDir dev(QStringLiteral("/dev"));
            const QStringList matches = dev.entryList(
                QStringList{QStringLiteral("fpga_frame_*")},
                QDir::Files | QDir::System | QDir::NoDotAndDotDot);
            if (matches.size() != 1) {
                appendLog(matches.isEmpty() ? QStringLiteral("无法开始：未找到设备节点")
                                           : QStringLiteral("无法开始：存在多个设备，请指定节点"));
                QMessageBox::warning(this, QStringLiteral("无法开始"),
                    matches.isEmpty()
                        ? QStringLiteral("未找到 /dev/fpga_frame_* 设备。\n请先加载驱动。")
                        : QStringLiteral("发现多个设备，请在启动时指定节点："
                                         "\npcie_viewer /dev/fpga_frame_..."));
                return;
            }
            path = dev.filePath(matches.first());
        }

        stopping_ = false;
        lastReceived_ = 0;
        lastPainted_ = 0;
        lastPaintingNs_ = 0;
        video_->resetStatistics();
        fpsClock_.start();
        performanceClock_.start();
        fpsLabel_->setText(QStringLiteral("采集：0.0 FPS   显示：0.0 FPS"));
        configStatus_->setText(QStringLiteral("正在启动；编辑参数不会自动发送"));
        applyButton_->setEnabled(false);
        modeBox_->setEnabled(false);
        startButton_->setEnabled(false);
        stopButton_->setEnabled(true);
        video_->setFrame({});
        video_->show();
        video_->raise();
        const ReceiveMode mode = ReceiveMode(modeBox_->currentData().toInt());
        appendLog(QStringLiteral("请求开始：%1；%2").arg(path, modeBox_->currentText()));
        session_ = new CaptureSession(path, mode, this);
        session_->requestPreview(video_->size());
        CaptureSession *session = session_;
        connect(session, &QThread::finished, this, [this, session] {
            session->wait();
            drainLogs(session);
            const QString error = session->errorText();
            appendLog(error.isEmpty() ? QStringLiteral("采集会话已结束，设备已关闭") : error);
            timer_.stop();
            video_->hide();
            video_->setFrame({});
            session_ = nullptr;
            session->deleteLater();
            stopping_ = false;
            fpsLabel_->setText(QStringLiteral("采集：0.0 FPS   显示：0.0 FPS（已停止）"));
            modeBox_->setEnabled(!closing_);
            startButton_->setEnabled(!closing_);
            stopButton_->setEnabled(false);
            applyButton_->setEnabled(false);
            configStatus_->setText(QStringLiteral(
                "采集已停止；视频 STOP 不清除 FPGA 图像参数"));
            if (!error.isEmpty())
                QMessageBox::warning(this, QStringLiteral("采集提示"), error);
            if (closing_)
                close();
        });
        timer_.start();
        session->start();
    }

    void stopCapture()
    {
        if (session_ && !stopping_)
            appendLog(QStringLiteral("请求停止采集"));
        stopping_ = true;
        fpsLabel_->setText(QStringLiteral("采集：0.0 FPS   显示：0.0 FPS（已停止）"));
        applyButton_->setEnabled(false);
        // 保留定时器以显示等待 STOP 时的日志，finished 后再停止定时器。
        if (session_)
            session_->requestStop();
        video_->hide();
        video_->setFrame({});
        if (!session_)
            return;
        modeBox_->setEnabled(false);
        startButton_->setEnabled(false);
        stopButton_->setEnabled(false);
    }

    QString device_;
    QPlainTextEdit *logView_ = nullptr;
    QLabel *fpsLabel_ = nullptr;
    QElapsedTimer fpsClock_;
    QElapsedTimer performanceClock_;
    quint64 lastReceived_ = 0;
    quint64 lastPainted_ = 0;
    quint64 lastPaintingNs_ = 0;
    ImageControls *imageControls_ = nullptr;
    QPushButton *applyButton_ = nullptr;
    QLabel *configStatus_ = nullptr;
    QComboBox *modeBox_ = nullptr;
    QPushButton *startButton_ = nullptr;
    QPushButton *stopButton_ = nullptr;
    VideoWindow *video_ = nullptr;
    CaptureSession *session_ = nullptr;
    QTimer timer_;
    bool stopping_ = false;
    bool closing_ = false;
};

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QApplication::setApplicationName(QStringLiteral("pcie_viewer"));
    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral("PCIe read / mmap / DMA-BUF RGA 视频查看器"));
    parser.addHelpOption();
    QCommandLineOption modeOption(QStringLiteral("mode"),
        QStringLiteral("初始接收方式：read / mmap / mmap-direct / rga"), QStringLiteral("mode"));
    QCommandLineOption startOption(QStringLiteral("start"), QStringLiteral("打开窗口后自动开始采集"));
    parser.addOption(modeOption);
    parser.addOption(startOption);
    parser.addPositionalArgument(QStringLiteral("device"),
                                 QStringLiteral("驱动节点；省略时自动查找"),
                                 QStringLiteral("[device]"));
    parser.process(app);
    const QStringList arguments = parser.positionalArguments();
    if (arguments.size() > 1)
        parser.showHelp(1);
    ReceiveMode initialMode = PCIE_HAVE_RGA ? ReceiveMode::RgaScaled : ReceiveMode::MmapBuffered;
    if (parser.isSet(modeOption)) {
        const QString value = parser.value(modeOption);
        if (value == QStringLiteral("read")) initialMode = ReceiveMode::Read;
        else if (value == QStringLiteral("mmap")) initialMode = ReceiveMode::MmapBuffered;
        else if (value == QStringLiteral("mmap-direct")) initialMode = ReceiveMode::Mmap;
        else if (value == QStringLiteral("rga") && PCIE_HAVE_RGA) initialMode = ReceiveMode::RgaScaled;
        else parser.showHelp(1);
    }
    MainWindow window(arguments.isEmpty() ? QString() : arguments.first(), initialMode);
    window.show();
    if (parser.isSet(startOption))
        QTimer::singleShot(0, &window, [&window] {
            for (auto *button : window.findChildren<QPushButton *>())
                if (button->text() == QStringLiteral("开始")) { button->click(); break; }
        });
    return app.exec();
}
