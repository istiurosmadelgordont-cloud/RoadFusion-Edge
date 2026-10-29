// 使用正式 Qt / RGA 头文件，只替换 PCIe 系统调用和 RGA 执行边界。
// 不连接 FPGA，不把模拟计时当作 RGA 硬件性能。
#define PCIE_HAVE_RGA 1
#define PCIE_RGA_REGRESSION
#include "qt_control_regression.cpp"
#include <set>

namespace rgaMock {
std::condition_variable changed;
unsigned calls=0;
bool block=false,entered=false,release=false,fail=false;
void reset() { calls=0;block=entered=release=fail=false; }
}

extern "C" rga_buffer_t wrapbuffer_fd_t(int fd,int width,int height,int ws,int hs,int format) {
 rga_buffer_t result{};result.fd=fd;result.width=width;result.height=height;
 result.wstride=ws;result.hstride=hs;result.format=format;return result;
}
extern "C" rga_buffer_t wrapbuffer_virtualaddr_t(void* address,int width,int height,int ws,int hs,int format) {
 rga_buffer_t result{};result.vir_addr=address;result.width=width;result.height=height;
 result.wstride=ws;result.hstride=hs;result.format=format;return result;
}
extern "C" const char* imStrError_t(IM_STATUS) { return "injected RGA failure"; }
extern "C" IM_STATUS imresize_t(rga_buffer_t source,rga_buffer_t target,
                                double fx,double fy,int interpolation,int sync) {
 assert(fx==0 && fy==0 && interpolation==IM_INTERP_DEFAULT && sync==1);
 assert(source.fd>=600 && source.fd<604 && source.width==1920 && source.height==1080);
 assert(source.wstride==1920 && source.hstride==1080 && source.format==RK_FORMAT_RGB_565);
 assert(target.format==RK_FORMAT_RGB_565 && target.wstride==target.width && target.hstride==target.height);
 assert(target.width%32==0 && target.height==target.width*9/16);
 assert(reinterpret_cast<uintptr_t>(target.vir_addr)%static_cast<size_t>(sysconf(_SC_PAGESIZE))==0);
 std::unique_lock<std::mutex> lock(mock::mutex);
 const unsigned slot=source.fd-600;
 assert(mock::held[slot] && mock::exported[slot] && !mock::rgaActive[slot]);
 mock::rgaActive[slot]=true;++rgaMock::calls;rgaMock::entered=true;
 rgaMock::changed.notify_all();
 if(rgaMock::block)rgaMock::changed.wait(lock,[]{return rgaMock::release;});
 assert(mock::held[slot] && mock::exported[slot]);
 // 填入源帧样本，归还后源槽立即覆写，便于发现源/预览寿命混淆。
 std::memset(target.vir_addr,mock::pixels[slot][0],size_t(target.wstride)*target.height*2);
 mock::rgaActive[slot]=false;
 if(rgaMock::fail){mock::failedRgaMustStop=true;return IM_STATUS_FAILED;}
 return IM_STATUS_SUCCESS;
}

template<class Predicate> static void until(QApplication& app,Predicate predicate) {
 QElapsedTimer clock;clock.start();
 while(!predicate()) {
  assert(clock.elapsed()<6000);
  app.processEvents();QThread::msleep(1);
 }
}
static FramePtr next(QApplication& app,CaptureSession& session) {
 FramePtr frame;
 until(app,[&]{frame=session.takeLatest();return bool(frame);});
 return frame;
}
static void finished(QApplication& app,CaptureSession& session) {
 until(app,[&]{return session.isFinished();});session.wait();
}
static void closed() {
 std::lock_guard<std::mutex> lock(mock::mutex);
 assert(!mock::opened && mock::gets==mock::puts && !mock::bound);
 for(bool value:mock::exported)assert(!value);
 for(bool value:mock::held)assert(!value);
 for(bool value:mock::rgaActive)assert(!value);
}

int main(int argc,char** argv) {
 QApplication app(argc,argv);app.setQuitOnLastWindowClosed(false);
 assert(previewSize({960,540})==QSize(960,540));
 assert(previewSize({1280,720})==QSize(1280,720));
 assert(previewSize({1000,700})==QSize(992,558));
 assert(previewSize({4096,2160})==QSize(1920,1080));
 assert(previewSize({1,1})==QSize(128,72));
 mock::reset();rgaMock::reset();
 {
  CaptureSession session("mock",ReceiveMode::RgaScaled,&app);
  session.requestPreview({960,540});session.start();
  std::vector<FramePtr> held;
  std::vector<uchar> values;
  std::set<const uchar*> addresses;
  for(unsigned i=0;i<3;++i) {
   auto frame=next(app,session);
   assert(!frame->owner && frame->image.size()==QSize(960,540));
   assert(frame->image.format()==QImage::Format_RGB16);
   assert(addresses.insert(frame->image.constBits()).second);
   values.push_back(frame->image.constBits()[0]);held.push_back(frame);
  }
  unsigned before;
  {std::lock_guard<std::mutex> lock(mock::mutex);before=mock::gets;assert(rgaMock::calls==3 && mock::maps==0 && mock::reads==0);}
  // 全部预览还在使用时，不覆盖、不无限分配，输入仍持续 GET/PUT。
  QThread::msleep(180);
  {std::lock_guard<std::mutex> lock(mock::mutex);assert(unsigned(mock::gets)>before+3 && rgaMock::calls==3);}
  for(size_t i=0;i<held.size();++i) {
   const auto& image=held[i]->image;
   assert(image.constBits()[0]==values[i] && image.constBits()[image.sizeInBytes()-1]==values[i]);
  }
  held.front().reset();session.requestPreview({640,360});
  auto resized=next(app,session);assert(resized->image.size()==QSize(640,360));
  unsigned calls;
  {std::lock_guard<std::mutex> lock(mock::mutex);calls=rgaMock::calls;}
  QThread::msleep(100);
  {std::lock_guard<std::mutex> lock(mock::mutex);assert(rgaMock::calls==calls);}
  // 预览图不持有 DMA 槽，可在 STOP、关闭导出 fd 后继续读取。
  session.requestStop();finished(app,session);
  assert(session.errorText().isEmpty());closed();
  assert(held[1]->image.constBits()[0]==values[1]);
  assert(resized->image.width()==640);
 }
 // STOP 与一个尚未返回的同步 RGA 任务并发，fd 不能提前关闭。
 mock::reset();rgaMock::reset();rgaMock::block=true;
 {
  CaptureSession session("mock",ReceiveMode::RgaScaled,&app);session.start();
  until(app,[]{std::lock_guard<std::mutex> lock(mock::mutex);return rgaMock::entered;});
  session.requestStop();
  until(app,[]{std::lock_guard<std::mutex> lock(mock::mutex);return mock::stops==1;});
  {std::lock_guard<std::mutex> lock(mock::mutex);assert(!session.isFinished() && mock::puts==0);for(bool value:mock::exported)assert(value);rgaMock::release=true;}
  rgaMock::changed.notify_all();finished(app,session);assert(session.errorText().isEmpty());closed();
 }
 // 出错明确停止，不静默退回 read；错误槽先 STOP，之后才能 PUT。
 mock::reset();rgaMock::reset();rgaMock::fail=true;
 {
  CaptureSession session("mock",ReceiveMode::RgaScaled,&app);session.start();finished(app,session);
  assert(session.errorText().contains(QStringLiteral("RGA 预览缩放失败")));closed();
  assert(mock::stops==1 && mock::reads==0);
 }
 mock::reset();rgaMock::reset();mock::failExport=2;
 {
  CaptureSession session("mock",ReceiveMode::RgaScaled,&app);session.start();finished(app,session);
  assert(session.errorText().contains(QStringLiteral("导出 DMA-BUF")));closed();
  assert(mock::starts==0 && rgaMock::calls==0);
 }
 mock::reset();rgaMock::reset();mock::dmaCap=false;
 {
  CaptureSession session("mock",ReceiveMode::RgaScaled,&app);session.start();finished(app,session);
  assert(session.errorText().contains(QStringLiteral("8.2")));closed();
  assert(mock::starts==0);
 }
 // 主窗口默认 RGA，模式可切换；用实际按钮开始/停止，核对线程日志交付。
 mock::reset();rgaMock::reset();
 {
  MainWindow window("mock");window.show();
  QComboBox* modes=nullptr;
  for(auto* combo:window.findChildren<QComboBox*>())if(combo->findData(int(ReceiveMode::RgaScaled))>=0)modes=combo;
  assert(modes && modes->count()==4 && modes->currentData().toInt()==int(ReceiveMode::RgaScaled));
  QPushButton* start=nullptr;QPushButton* stop=nullptr;
  for(auto* button:window.findChildren<QPushButton*>()) {
   if(button->text()==QStringLiteral("开始"))start=button;
   if(button->text()==QStringLiteral("停止"))stop=button;
  }
  assert(start && stop);auto* logs=window.findChild<QPlainTextEdit*>(QStringLiteral("runtimeLog"));
  start->click();
  until(app,[&]{return logs->toPlainText().contains(QStringLiteral("收到第一幅完整图像"));});
  stop->click();until(app,[&]{return start->isEnabled();});closed();
  assert(logs->toPlainText().contains(QStringLiteral("已导出四槽")));
 }
 puts("PASS: Qt/RGA demand resize, RGB565, bounded pool, page alignment, resize, retained output, STOP concurrency, error/export cleanup and GUI");
}
