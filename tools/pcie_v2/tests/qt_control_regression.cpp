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
}
}
extern "C" int __wrap_open(const char*,int,...) {
 std::lock_guard<std::mutex> guard(mock::mutex);
 assert(!mock::opened);mock::opened=true;return 501;
}
extern "C" int __wrap_close(int fd) {
 assert(fd==501);std::lock_guard<std::mutex> guard(mock::mutex);
 for(bool h:mock::held)assert(!h);
 mock::opened=false;return 0;
}
extern "C" void* __wrap_mmap(void*,size_t len,int prot,int flags,int fd,off_t off) {
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
  auto& caps=*static_cast<fpga_caps*>(arg);caps={};caps.version=1;caps.flags=FPGA_CAP_IMAGE_SUBMIT;return 0;
 }
 case FPGA_IOC_START:mock::running=true;++mock::starts;return 0;
 case FPGA_IOC_STOP:mock::running=false;++mock::stops;return 0;
 case FPGA_IOC_GET_FRAME:{
  if(!mock::running){errno=EPIPE;return -1;}
  for(unsigned n=0;n<4;++n)if(!mock::held[n]){
   auto& f=*static_cast<fpga_frame*>(arg);f={};f.session=mock::starts;f.slot=n;
   f.sequence=++mock::sequence;f.token=(f.sequence<<2)|n;f.bytes_used=4147200;
   mock::held[n]=true;++mock::gets;return 0;
  }
  errno=ETIMEDOUT;return -1;
 }
 case FPGA_IOC_PUT_FRAME:{
  const auto& f=*static_cast<fpga_frame*>(arg);assert(f.slot<4 && mock::held[f.slot]);
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
 QTimer timer;timer.setInterval(10);
 QObject::connect(&timer,&QTimer::timeout,&app,[&]{
  assert(deadline.elapsed()<10000);
  if(auto f=session->takeLatest())displayed=std::move(f);
  bool busy=false;const QString message=session->imageMessage(busy);
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
   puts("PASS: submit-only UI, manual retry, no query/automatic resend, old driver rejection, mmap/read leases, FPS");app.quit();
  }
 });
 timer.start();return app.exec();
}
