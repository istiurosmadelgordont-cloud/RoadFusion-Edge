// 真实 PcieCapture + RGA 接收器；设备和 RGA 系统边界被替换，不访问硬件。
#include "adas/pcie_capture.hpp"
#include <algorithm>
#include <cassert>
#include <cstdarg>
#include <future>
#include <set>

namespace mock {
std::mutex mutex;
std::condition_variable changed;
bool stopped=false, waiting=false, held=false, rga_active=false, block_rga=false;
bool no_cap=false, ack=false, fail_start=false, fail_get=false, fail_rga=false, fail_put=false;
int fail_export=-1;
unsigned grants=0, issued=0, returned=0, converted=0, starts=0, stops=0, prepared=0;
unsigned exports=0, closed=0;
fpga_frame current{};
std::set<int> open_fds;
void reset() {
  std::lock_guard<std::mutex> lock(mutex);
  assert(open_fds.empty());
  stopped=waiting=held=rga_active=block_rga=no_cap=ack=fail_start=fail_get=fail_rga=fail_put=false;
  fail_export=-1;grants=issued=returned=converted=starts=stops=prepared=exports=closed=0;
  current={};
}
void grant(unsigned count) {
  std::lock_guard<std::mutex> lock(mutex);grants=count;changed.notify_all();
}
void wait_for(const std::function<bool()>& predicate) {
  std::unique_lock<std::mutex> lock(mutex);
  assert(changed.wait_for(lock,std::chrono::seconds(4),predicate));
}
void done() {
  std::lock_guard<std::mutex> lock(mutex);
  assert(open_fds.empty() && !rga_active && closed==exports+1);
}
int error(int value) {errno=value;return -1;}
}

extern "C" int __wrap_open(const char*,int flags,...) {
  assert(flags==(O_RDONLY|O_CLOEXEC));
  std::lock_guard<std::mutex> lock(mock::mutex);
  assert(mock::open_fds.empty());mock::open_fds.insert(123);return 123;
}
extern "C" int __wrap_close(int fd) {
  std::lock_guard<std::mutex> lock(mock::mutex);
  assert(mock::open_fds.erase(fd)==1 && !mock::rga_active);
  if(fd==123)assert(mock::open_fds.empty());
  ++mock::closed;return 0;
}
extern "C" ssize_t __wrap_read(int,void*,size_t) {
  assert(false && "RGA input must never fall back to read");return -1;
}
extern "C" int __wrap_ioctl(int fd,unsigned long command,...) {
  va_list args;va_start(args,command);void* value=va_arg(args,void*);va_end(args);
  std::unique_lock<std::mutex> lock(mock::mutex);
  using namespace mock;
  assert(fd==123 && open_fds.count(fd));
  if(command==FPGA_IOC_GET_INFO) {
    auto& i=*static_cast<fpga_info*>(value);i={};
    i.abi_version=2;i.width=1920;i.height=1080;i.stride=3840;i.image_bytes=4147200;
    i.slot_count=4;i.pixel_format=FPGA_FORMAT_RGB565;i.map_bytes=4149248;return 0;
  }
  if(command==FPGA_IOC_GET_CAPS) {
    auto& c=*static_cast<fpga_caps*>(value);c={};c.version=1;
    c.flags=no_cap?0:FPGA_CAP_DMABUF;
    if(ack)c.flags|=FPGA_CAP_IMAGE_SUBMIT|FPGA_CAP_IMAGE_ACK;
    return 0;
  }
  if(command==FPGA_IOC_EXPORT_DMABUF) {
    auto& e=*static_cast<fpga_dmabuf_export*>(value);
    assert(!starts && e.slot==exports);
    if(static_cast<int>(e.slot)==fail_export)return error(ENOMEM);
    e.fd=200+e.slot;open_fds.insert(e.fd);++exports;return 0;
  }
  if(command==FPGA_IOC_PREPARE_IMAGE_ACK){assert(!starts&&exports==4);++prepared;return 0;}
  if(command==FPGA_IOC_START){++starts;assert(exports==4);return fail_start?error(EBUSY):0;}
  if(command==FPGA_IOC_STOP){++stops;stopped=true;changed.notify_all();return 0;}
  if(command==FPGA_IOC_GET_FRAME) {
    assert(!held && !rga_active);
    if(fail_get)return error(ETIMEDOUT);
    waiting=true;changed.notify_all();
    changed.wait(lock,[]{return stopped||issued<grants;});waiting=false;
    if(stopped)return error(EPIPE);
    current={};current.session=7;current.sequence=++issued;
    current.slot=(issued-1)%4;current.token=(issued<<2)|current.slot;current.bytes_used=4147200;
    *static_cast<fpga_frame*>(value)=current;held=true;return 0;
  }
  if(command==FPGA_IOC_PUT_FRAME) {
    const auto& frame=*static_cast<const fpga_frame*>(value);
    assert(held&&!rga_active && frame.token==current.token && frame.session==current.session);
    if(fail_put)return error(ESTALE);
    held=false;++returned;changed.notify_all();return 0;
  }
  assert(false && "Unexpected ioctl");return -1;
}

extern "C" rga_buffer_t wrapbuffer_fd_t(int fd,int w,int h,int ws,int hs,int format) {
  rga_buffer_t b{};b.fd=fd;b.width=w;b.height=h;b.wstride=ws;b.hstride=hs;b.format=format;return b;
}
extern "C" rga_buffer_t wrapbuffer_virtualaddr_t(void* address,int w,int h,int ws,int hs,int format) {
  rga_buffer_t b{};b.vir_addr=address;b.width=w;b.height=h;b.wstride=ws;b.hstride=hs;b.format=format;return b;
}
extern "C" const char* imStrError_t(IM_STATUS) {return "mock RGA failure";}
extern "C" IM_STATUS imcvtcolor_t(rga_buffer_t source,rga_buffer_t target,
                                  int sfmt,int dfmt,int mode,int sync) {
  unsigned sequence;
  {
    std::unique_lock<std::mutex> lock(mock::mutex);
    assert(mock::held && !mock::rga_active && source.fd==200+int(mock::current.slot));
    assert(sync==1 && mode==0 && sfmt==RK_FORMAT_BGR_565 && dfmt==RK_FORMAT_BGR_888);
    assert(source.width==1920 && source.height==1080 && source.wstride==1920 && source.hstride==1080);
    assert(target.width==1920 && target.height==1080 && target.wstride==1920 && target.hstride==1080 && target.vir_addr);
    assert(reinterpret_cast<uintptr_t>(target.vir_addr)%static_cast<size_t>(::sysconf(_SC_PAGESIZE))==0);
    mock::rga_active=true;sequence=mock::current.sequence;mock::changed.notify_all();
    mock::changed.wait(lock,[]{return !mock::block_rga;});
    if(mock::fail_rga){mock::rga_active=false;return IM_STATUS_FAILED;}
  }
  cv::Mat image(target.height,target.width,CV_8UC3,target.vir_addr,target.wstride*3);
  image.setTo(cv::Scalar(sequence,sequence+1,sequence+2));
  {
    std::lock_guard<std::mutex> lock(mock::mutex);
    mock::rga_active=false;++mock::converted;mock::changed.notify_all();
  }
  return IM_STATUS_SUCCESS;
}

void pixels(const cv::Mat& image,unsigned sequence) {
  assert(!image.empty() && image.type()==CV_8UC3);
  assert(image.at<cv::Vec3b>(0,0)==cv::Vec3b(sequence,sequence+1,sequence+2));
  assert(image.at<cv::Vec3b>(image.rows-1,image.cols-1)==cv::Vec3b(sequence,sequence+1,sequence+2));
}
int main() {
  cv::setNumThreads(1);
  volatile sig_atomic_t quit=0;
  using Clock=std::chrono::steady_clock;
  using Result=adas::PcieCapture::ReadResult;
  // 全部槽持有时必须跳帧；ROI 自己也持有引用，释放原 Mat 不代表能复用。
  {
    mock::reset();adas::PcieCapture capture(quit);capture.open("mock",3);
    Clock::time_point stamp;cv::Mat first,second,third,next;
    mock::grant(1);assert(capture.read(first,stamp));pixels(first,1);
    cv::Mat roi=first(cv::Rect(10,10,50,50));first.release();
    mock::grant(2);assert(capture.read(second,stamp));pixels(second,2);
    mock::grant(3);assert(capture.read(third,stamp));pixels(third,3);
    mock::grant(4);mock::wait_for([]{return mock::returned==4&&mock::waiting;});
    assert(mock::converted==3 && capture.poll_read(next,stamp,16)==Result::WAITING);
    pixels(roi,1);pixels(second,2);pixels(third,3);
    const auto reusable=second.data;second.release();
    mock::grant(5);assert(capture.read(next,stamp));pixels(next,5);
    assert(next.data==reusable);pixels(roi,1);pixels(third,3);
    capture.close();mock::done();pixels(roi,1);pixels(third,3);pixels(next,5);
    assert(mock::converted==4 && mock::returned==5 && mock::stops==1);
  }
  // STOP 可能在 RGA 执行期间完成；必须等 RGA 返回再 PUT、join、关闭导出 fd。
  {
    mock::reset();mock::block_rga=true;mock::grant(1);
    adas::PcieCapture capture(quit);capture.open("mock",3);
    auto reader=std::async(std::launch::async,[&]{cv::Mat m;Clock::time_point t;return capture.read(m,t);});
    mock::wait_for([]{return mock::rga_active;});
    auto closer=std::async(std::launch::async,[&]{capture.close();});
    mock::wait_for([]{return mock::stops==1;});
    assert(closer.wait_for(std::chrono::milliseconds(20))==std::future_status::timeout);
    {std::lock_guard<std::mutex> lock(mock::mutex);assert(!mock::returned && !mock::closed);mock::block_rga=false;mock::changed.notify_all();}
    closer.get();assert(!reader.get());mock::done();assert(mock::returned==1);
  }
  // 初始化、取帧、RGA 和 PUT 错误均显式失败，不静默降级或归还未完成的转换。
  for(int failure=0;failure<6;++failure) {
    mock::reset();mock::no_cap=failure==0;mock::fail_export=failure==1?2:-1;
    mock::fail_start=failure==2;mock::fail_get=failure==3;mock::fail_rga=failure==4;mock::fail_put=failure==5;
    mock::grant(1);adas::PcieCapture capture(quit);bool rejected=false;
    try{capture.open("mock",3);}catch(const std::exception&){rejected=true;}
    if(failure<2)assert(rejected && !mock::starts);
    else {
      cv::Mat image;Clock::time_point stamp;
      assert(!capture.read(image,stamp) && !capture.error().empty());
      capture.close();assert(mock::stops==(failure==2?0U:1U));
    }
    mock::done();if(failure==4||failure==5)assert(!mock::returned);
  }
  {
    mock::reset();mock::ack=true;adas::PcieCapture capture(quit);capture.open("mock",3);
    assert(mock::prepared==1 && !mock::starts);capture.close();mock::done();assert(mock::stops==1);
  }
  // close/reopen 后新池不能覆盖仍由旧消费者持有的图像。
  {
    mock::reset();mock::grant(1);adas::PcieCapture capture(quit);capture.open("mock",3);
    cv::Mat old,fresh;Clock::time_point stamp;
    assert(capture.read(old,stamp));capture.close();mock::done();
    mock::reset();mock::grant(1);capture.open("mock",3);
    assert(capture.read(fresh,stamp));assert(fresh.data!=old.data);capture.close();mock::done();pixels(old,1);
  }
  std::cout<<"PCIE_RGA_CAPTURE_PASS: actual capture, source lease, sync-before-PUT, bounded BGR pool, ROI/future lifetime, pool-full drop/reuse, STOP/join/error/reopen\n";
}
