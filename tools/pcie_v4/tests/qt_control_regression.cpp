// 编译真实查看器代码，仅通过链接器替换设备系统调用；不访问 FPGA。
#define main pcie_viewer_main
#include "../main.cpp"
#undef main
#include <cassert>
#include <cstdarg>
#include <cstdio>
#include <vector>

namespace mock {
std::mutex mutex;
bool opened=false,running=false,oldDriver=false,failSubmit=false;
bool ackCap=false,bound=false;
bool dmaCap=true;
int failExport=-1;
std::array<bool,4> exported{};
std::array<bool,4> rgaActive{};
bool failedRgaMustStop=false;
int maps=0;
unsigned ackCookie=0,ackStage=0;int prepares=0,ackSubmits=0;
int starts=0,stops=0,gets=0,puts=0,reads=0,submits=0,forbidden=0;
unsigned sequence=0;
std::array<bool,4> held{};
std::array<std::vector<unsigned char>,4> pixels;
std::vector<fpga_image_submit> configurations;
void reset(){
 std::lock_guard<std::mutex> guard(mutex);
 assert(!opened); running=oldDriver=failSubmit=false;
 starts=stops=gets=puts=reads=submits=forbidden=0;sequence=0;held.fill(false);
 configurations.clear();for(auto& p:pixels)p.resize(4149248);
 ackCap=bound=false;ackCookie=ackStage=0;prepares=ackSubmits=0;
 dmaCap=true;failExport=-1;exported.fill(false);
 rgaActive.fill(false);failedRgaMustStop=false;maps=0;
}
}
extern "C" int __wrap_open(const char*,int,...) {
 std::lock_guard<std::mutex> guard(mock::mutex);
 assert(!mock::opened);mock::opened=true;return 501;
}
extern "C" int __wrap_close(int fd) {
 if(fd>=600 && fd<604){std::lock_guard<std::mutex> guard(mock::mutex);assert(mock::exported[fd-600]);assert(!mock::rgaActive[fd-600]);mock::exported[fd-600]=false;return 0;}
 assert(fd==501);std::lock_guard<std::mutex> guard(mock::mutex);
 for(bool h:mock::held)assert(!h);
 assert(!mock::bound);
 for(bool value:mock::exported)assert(!value);
 mock::opened=false;return 0;
}
extern "C" void* __wrap_mmap(void*,size_t len,int prot,int flags,int fd,off_t off) {
 ++mock::maps;
 assert(fd==501 && len==4149248 && prot==PROT_READ && flags==MAP_SHARED);
 assert(off>=0 && off/4149248<4);return mock::pixels[off/4149248].data();
}
extern "C" int __wrap_munmap(void*,size_t) {return 0;}
extern "C" ssize_t __wrap_read(int fd,void* data,size_t len) {
 assert(fd==501);QThread::msleep(8);std::lock_guard<std::mutex> guard(mock::mutex);
 if(!mock::running)return 0;
 ++mock::reads;
 // 短读接收路径也必须完整组帧，返回上限 700001。
 const auto bytes=std::min<size_t>(len,700001);std::memset(data,0x55,bytes);return bytes;
}
extern "C" int __wrap_ioctl(int fd,unsigned long cmd,...) {
 assert(fd==501);va_list args;va_start(args,cmd);void* arg=va_arg(args,void*);va_end(args);
 if(cmd==FPGA_IOC_GET_FRAME)QThread::msleep(8);
 std::lock_guard<std::mutex> guard(mock::mutex);
 switch(cmd){
 case FPGA_IOC_GET_INFO:{
  auto& i=*static_cast<fpga_info*>(arg);i={};i.abi_version=2;i.width=1920;i.height=1080;
  i.stride=3840;i.image_bytes=4147200;i.slot_count=4;i.pixel_format=1;i.map_bytes=4149248;
  for(unsigned n=0;n<4;++n)i.offset[n]=n*i.map_bytes;
  return 0;
 }
 case FPGA_IOC_GET_CAPS:{
  if(mock::oldDriver){errno=ENOTTY;return -1;}
  auto& caps=*static_cast<fpga_caps*>(arg);caps={};caps.version=1;
  // 8.2 额外声明 DMA-BUF 能力；原 read/mmap 查看器应继续正常工作。
  caps.flags=FPGA_CAP_IMAGE_SUBMIT|(mock::dmaCap?FPGA_CAP_DMABUF:0)|
             (mock::ackCap?FPGA_CAP_IMAGE_ACK:0);return 0;
 }
 case FPGA_IOC_EXPORT_DMABUF:{
  auto& request=*static_cast<fpga_dmabuf_export*>(arg);
  assert(!mock::running && request.slot<4 && !request.flags && !request.reserved);
  if(int(request.slot)==mock::failExport){errno=ENOMEM;return -1;}
  assert(!mock::exported[request.slot]);mock::exported[request.slot]=true;
  request.fd=600+int(request.slot);return 0;
 }
 case FPGA_IOC_PREPARE_IMAGE_ACK:
  assert(!mock::running && !mock::bound);mock::bound=true;mock::ackStage=0;++mock::prepares;return 0;
 case FPGA_IOC_SUBMIT_IMAGE_ACK:
  assert(mock::bound);++mock::ackSubmits;++mock::ackCookie;mock::ackStage=0;return 0;
 case FPGA_IOC_GET_IMAGE_ACK:{
  assert(mock::bound);auto& s=*static_cast<fpga_image_ack_status*>(arg);s={};s.cookie=mock::ackCookie;
  s.flags=FPGA_IMAGE_BOUND|(mock::ackStage<2?FPGA_IMAGE_PENDING:0);
  if(mock::ackStage>=1)s.flags|=FPGA_IMAGE_RECEIVED;
  if(mock::ackStage>=2)s.flags|=FPGA_IMAGE_APPLIED;
  return 0;
 }
 case FPGA_IOC_START:mock::running=true;++mock::starts;return 0;
 case FPGA_IOC_STOP:mock::running=false;mock::bound=false;++mock::stops;return 0;
 case FPGA_IOC_GET_FRAME:{
  if(!mock::running){errno=EPIPE;return -1;}
  for(unsigned n=0;n<4;++n)if(!mock::held[n]){
   auto& f=*static_cast<fpga_frame*>(arg);f={};f.session=mock::starts;f.slot=n;
   f.sequence=++mock::sequence;f.token=(f.sequence<<2)|n;f.bytes_used=4147200;
   // 模拟 FPGA 为新帧写入不同像素；PUT 后立即覆盖，暴露提前归还问题。
   std::memset(mock::pixels[n].data(),int(f.sequence%200+1),4147200);
   mock::held[n]=true;++mock::gets;return 0;
  }
  errno=ETIMEDOUT;return -1;
 }
 case FPGA_IOC_PUT_FRAME:{
  const auto& f=*static_cast<fpga_frame*>(arg);assert(f.slot<4 && mock::held[f.slot]);
  assert(!mock::rgaActive[f.slot]);
  if(mock::failedRgaMustStop)assert(!mock::running); // RGA 失败必须先 STOP。
  std::memset(mock::pixels[f.slot].data(),0xee,4147200);
  mock::held[f.slot]=false;++mock::puts;return 0;
 }
 case FPGA_IOC_SUBMIT_IMAGE:{
  ++mock::submits;mock::configurations.push_back(*static_cast<fpga_image_submit*>(arg));
  if(mock::failSubmit){errno=EBUSY;return -1;}return 0;
 }
 case FPGA_IOC_SET_IMAGE:case FPGA_IOC_GET_IMAGE:
  ++mock::forbidden;assert(false);return -1;
 default:assert(false);return -1;
 }
}

#ifndef PCIE_RGA_REGRESSION
int main(int argc,char** argv){
 QApplication app(argc,argv);app.setQuitOnLastWindowClosed(false);mock::reset();
 ImageControls panel(nullptr);auto* presets=panel.findChild<QComboBox*>();assert(presets && presets->count()==10);
 const unsigned expected[]={0x1c0,0x1cc,0x1ce,0x1e1,0x1ff,0x1de,0x1d0,0x11c0,0x9c0,0x19c0};
 for(int n=0;n<10;++n){presets->setCurrentIndex(n);auto c=panel.words();assert(c.words[0]==expected[n]);for(auto v:c.reserved)assert(!v);}
 auto config=panel.words();
 VideoWindow paint(nullptr);auto image=std::make_shared<VideoFrame>();image->image=QImage(32,32,QImage::Format_RGB16);
 paint.setFrame(image);paint.show();app.processEvents();paint.grab();assert(paint.paintedFrames()==1);
 paint.grab();assert(paint.paintedFrames()==1);paint.resetStatistics();assert(!paint.paintedFrames());paint.setFrame({});paint.hide();
 CaptureSession* session=new CaptureSession("mock",ReceiveMode::Mmap,&app);
 assert(!session->submitImage(config));session->start();
 int phase=0;FramePtr displayed;QElapsedTimer deadline;deadline.start();
 QStringList logs;
 MainWindow *window=nullptr;QPlainTextEdit *logView=nullptr;
 QPushButton *startButton=nullptr,*stopButton=nullptr;
 std::vector<FramePtr> bufferedHeld;
 std::vector<unsigned char> bufferedValues;
 quint64 beforePaused=0;
 QElapsedTimer paused;
 QTimer timer;timer.setInterval(10);
 QObject::connect(&timer,&QTimer::timeout,&app,[&]{
  assert(deadline.elapsed()<12000);
  // phase 9 模拟 GUI 暂停消费，并保留全部三个显示副本。
  if(phase<18 && phase!=9 && phase!=10)
   if(auto f=session->takeLatest())displayed=std::move(f);
  bool busy=false;const QString message=phase<18?session->imageMessage(busy):QString();
  if(phase<18)logs.append(session->takeLogs());
  if(phase==0 && displayed && !busy){
   assert(session->submitImage(config));assert(!session->submitImage(config));phase=1;
  }else if(phase==1 && message.contains(QStringLiteral("参数已提交")) && !busy){
   assert(!message.contains(QStringLiteral("参数已生效")));
   {std::lock_guard<std::mutex> g(mock::mutex);assert(mock::submits==1);mock::failSubmit=true;}
   assert(session->submitImage(config));phase=2;
  }else if(phase==2 && message.contains(QStringLiteral("参数提交失败")) && !busy){
   assert(!message.contains(QStringLiteral("等待生效")));
   {std::lock_guard<std::mutex> g(mock::mutex);assert(mock::submits==2);mock::failSubmit=false;}
   assert(session->submitImage(config));phase=3;
  }else if(phase==3 && message.contains(QStringLiteral("参数已提交")) && !busy){
   {std::lock_guard<std::mutex> g(mock::mutex);assert(mock::submits==3);}
   session->requestStop();displayed.reset();phase=4;
  }else if(phase==4 && session->isFinished()){
   displayed.reset();session->wait();assert(session->errorText().isEmpty());assert(session->receivedFrames()>0);delete session;
   assert(mock::gets==mock::puts && mock::forbidden==0);
   session=new CaptureSession("mock",ReceiveMode::Read,&app);session->start();phase=5;
  }else if(phase==5 && displayed){session->requestStop();displayed.reset();phase=6;
  }else if(phase==6 && session->isFinished()){
   session->wait();assert(session->errorText().isEmpty());delete session;
   assert(mock::reads>0 && mock::starts==2 && mock::stops==2 && mock::gets==mock::puts && !mock::opened);
   mock::oldDriver=true;session=new CaptureSession("mock",ReceiveMode::Read,&app);session->start();phase=7;
  }else if(phase==7 && session->isFinished()){
   session->wait();assert(session->errorText().contains(QStringLiteral("7.0")));delete session;
   assert(mock::starts==2 && mock::forbidden==0 && !mock::opened);
   mock::oldDriver=false;displayed.reset();
   session=new CaptureSession("mock",ReceiveMode::MmapBuffered,&app);session->start();phase=8;
  }else if(phase==8 && displayed){
   assert(!displayed->owner); // 显示副本不再持有 DMA 槽。
   assert(displayed->image.format()==QImage::Format_RGB16);
   const auto* data=displayed->image.constBits();
   for(const auto& p:mock::pixels)assert(data!=p.data());
   if(bufferedHeld.empty() || displayed!=bufferedHeld.back()){
    bufferedHeld.push_back(displayed);bufferedValues.push_back(data[0]);
   }
   if(bufferedHeld.size()==3){
    beforePaused=session->receivedFrames();paused.start();phase=9;
   }
  }else if(phase==9 && paused.elapsed()>150){
   // 全部显示副本被占用时仍要继续 GET/PUT，不得覆盖旧副本或无限分配。
   assert(session->receivedFrames()>beforePaused+3);
   for(size_t n=0;n<bufferedHeld.size();++n){
    const auto& image=bufferedHeld[n]->image;const auto* data=image.constBits();
    assert(data[0]==bufferedValues[n] && data[FrameBytes/2]==bufferedValues[n]);
    assert(data[FrameBytes-1]==bufferedValues[n] && data[0]!=0xee);
   }
   displayed=bufferedHeld.front();bufferedHeld.clear();
   session->requestStop();phase=10;
  }else if(phase==10 && session->isFinished()){
   // 普通内存副本可在 STOP、munmap、close 后继续使用。
   session->wait();assert(session->errorText().isEmpty());delete session;
   assert(displayed->image.constBits()[0]==bufferedValues.front());displayed.reset();
   assert(mock::starts==3 && mock::stops==3 && mock::gets==mock::puts && !mock::opened);
   mock::reset();mock::ackCap=true;logs.clear();
   session=new CaptureSession("mock",ReceiveMode::Read,&app);session->start();phase=11;
  }else if(phase==11 && !busy){
   assert(mock::prepares==1);assert(session->submitImage(config));phase=12;
  }else if(phase==12 && message.contains(QStringLiteral("等待接收"))){
   assert(busy && !session->submitImage(config));
   std::lock_guard<std::mutex> g(mock::mutex);assert(mock::ackSubmits==1);mock::ackStage=1;phase=13;
  }else if(phase==13 && message.contains(QStringLiteral("等待帧边界"))){
   assert(busy && !session->submitImage(config));phase=14;
  }else if(phase==14 && message.contains(QStringLiteral("超时"))){
   assert(busy && !session->submitImage(config));
   std::lock_guard<std::mutex> g(mock::mutex);assert(mock::ackSubmits==1);mock::ackStage=2;phase=15;
  }else if(phase==15 && message.contains(QStringLiteral("已生效")) && !busy){
   assert(message.contains(QStringLiteral("迟到回执")));
   assert(session->submitImage(config));phase=16;
  }else if(phase==16 && message.contains(QStringLiteral("#2"))){
   assert(busy);session->requestStop();displayed.reset();phase=17;
  }else if(phase==17 && session->isFinished()){
   session->wait();logs.append(session->takeLogs());assert(session->errorText().isEmpty());delete session;
   assert(mock::ackSubmits==2 && mock::stops==1 && !mock::opened);
   const auto text=logs.join('\n');assert(text.contains(QStringLiteral("RECEIVED")));
   assert(text.contains(QStringLiteral("APPLIED")) && text.contains(QStringLiteral("超时")));
   assert(text.contains(QStringLiteral("STOP 已确认")));
   // 验证真实主窗口底部的有界日志，以及启动/停止日志从工作线程到 GUI 的传递。
   window=new MainWindow("mock",ReceiveMode::MmapBuffered);window->show();
   logView=window->findChild<QPlainTextEdit*>(QStringLiteral("runtimeLog"));
   assert(logView && logView->isReadOnly() && logView->maximumBlockCount()==500);
   for(auto* button:window->findChildren<QPushButton*>()){
    if(button->text()==QStringLiteral("开始"))startButton=button;
    if(button->text()==QStringLiteral("停止"))stopButton=button;
   }
   assert(startButton && stopButton);startButton->click();phase=18;
  }else if(phase==18 && logView->toPlainText().contains(QStringLiteral("START 成功"))){
   stopButton->click();phase=19;
  }else if(phase==19 && logView->toPlainText().contains(QStringLiteral("设备已关闭"))){
   assert(logView->toPlainText().contains(QStringLiteral("STOP 已确认")));
   // 输出截图到环境指定的临时路径，便于本地核对布局，不污染交付目录。
   const auto shot=qgetenv("PCIE_TEST_SCREENSHOT");if(!shot.isEmpty())window->grab().save(QString::fromLocal8Bit(shot));
   delete window;assert(!mock::opened);
   puts("PASS: real Qt read/mmap; two-stage ACK; timeout/late ACK; no auto resend; STOP; bounded bottom log and GUI delivery");app.quit();
  }
 });
 timer.start();return app.exec();
}
#endif
