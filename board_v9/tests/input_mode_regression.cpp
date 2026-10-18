// 检查主程序真实控件、选灯和预警入口；不打开设备或模型。
#define main four_view_application_entry_for_test
#include "../src/four_view.cpp"
#undef main
#include <cassert>

namespace {
using Direction=adas::ManeuverDirection;
using Action=adas::IntersectionAction;
using Simulator=adas::LaneSignalSimulator;
using Alarm=Simulator::AlarmType;
adas::Detection lamp(int id,float x=310,float score=.99f) {
  adas::Detection d; d.class_id=id; d.score=score; d.box=cv::Rect2f(x,90,20,20); return d;
}
void confirm(Simulator& s,int id,double begin=0,float x=310) {
  for(double step : {0.,.3,.65}) s.observe(begin+step,{lamp(id,x)},640,360);
}
void controls(Simulator& s,adas::MomentaryInput& input,bool& visible,int key=-1) {
  apply_signal_demo_controls(s,visible,key,input.take());
}
void click(Simulator& s,adas::MomentaryInput& input,bool& visible,const cv::Rect& r) {
  input.press(r.x+5,r.y+5); input.release(); controls(s,input,visible);
}
adas::WarningSummary warning(const Simulator& s) {
  adas::WarningManager manager;
  auto result=manager.update(false,false,false,false,false,adas::LaneChangeResult(),s.motion_intersection());
  apply_signal_demo_warning(s,result); return result;
}
bool parse_test(std::initializer_list<const char*> values,Options& options) {
  std::vector<std::string> storage; for(auto v:values) storage.emplace_back(v);
  std::vector<char*> args; for(auto& v:storage) args.push_back(&v[0]);
  return parse(args.size(),args.data(),options);
}
}

int main(int argc,char** argv) {
  cv::setNumThreads(1);
  Options o;
  assert(parse_test({"viewer","--pcie","/dev/mock","--scene","missing/profile","--display-fps","0"},o));
  assert(o.display_fps_limit==0 && o.pcie_device=="/dev/mock");
  assert(parse_test({"viewer","--demo-lane","right"},o) && o.demo_direction=="right");
  for(const char* bad:{"-1","nan","inf","bad"}) assert(!parse_test({"viewer","--display-fps",bad},o));
  assert(!parse_test({"viewer","--demo-force-move"},o));

  // 三车道 x 三方向：红灯任何行驶都报闯红灯，绿灯方向不符报违规操作。
  for(int lane=0;lane<3;++lane) {
    Simulator s; bool visible=true; adas::MomentaryInput input;
    click(s,input,visible,kDemoLanes[lane]);
    assert(s.lane()==static_cast<Direction>(lane+1) && !s.moving());
    const int red=lane==0?8:lane==1?7:9;
    confirm(s,red);
    assert(s.action()==Action::STOP);
    for(int direction=0;direction<3;++direction) {
      click(s,input,visible,kDemoDirections[direction]);
      assert(s.moving() && s.direction()==static_cast<Direction>(direction+1));
      assert(s.lane()==static_cast<Direction>(lane+1));
      assert(s.alarm() && s.alarm_type()==Alarm::RED_LIGHT);
      const auto selected=s.intersection().selected_signal;
      assert(selected.box==cv::Rect(310,90,20,20) && selected.stable);
      const auto w=warning(s);
      assert(w.priority==110 && std::string(w.text).find("闯红灯")!=std::string::npos);
      controls(s,input,visible); input.cancel(); controls(s,input,visible);
      assert(s.moving()); // 松开/移动/窗口鼠标状态取消不取消已选择的行驶动作。
      // 再点同一个方向停车，按钮不会要求持续按住。
      click(s,input,visible,kDemoDirections[direction]);
      assert(!s.moving() && !s.alarm());
      assert(warning(s).priority==(lane==direction?60:0)); // 匹配方向停车后只保留普通红灯提示。
    }
    confirm(s,red+4,1.);
    assert(s.action()==Action::GO && !s.moving());
    for(int direction=0;direction<3;++direction) {
      click(s,input,visible,kDemoDirections[direction]);
      assert(s.moving() && s.alarm()==(lane!=direction));
      assert(s.alarm_type()==(lane==direction?Alarm::NONE:Alarm::INVALID_DIRECTION));
      const auto w=warning(s);
      assert(w.priority==(lane==direction?0:110));
      if(lane!=direction) {
        assert(std::string(w.text).find("违规操作")!=std::string::npos);
        assert(std::string(w.text).find("闯红灯")==std::string::npos);
        assert(std::find(w.active.begin(),w.active.end(),adas::WarningCode::INTERSECTION)!=w.active.end());
        assert(std::string(s.alarm_title())=="违规操作");
      }
      assert(s.motion_intersection().action==(lane==direction?Action::GO:Action::UNKNOWN));
      click(s,input,visible,kDemoDirections[direction]); assert(!s.moving());
    }
  }

  Simulator s; bool visible=true; adas::MomentaryInput input;
  click(s,input,visible,kDemoLanes[2]); confirm(s,9);
  // 右转红灯时改选左转/直行，也必须维持“闯红灯”报警，而非改成违规操作。
  click(s,input,visible,kDemoDirections[2]); assert(s.alarm());
  click(s,input,visible,kDemoDirections[0]); assert(s.moving() && s.alarm_type()==Alarm::RED_LIGHT);
  click(s,input,visible,kDemoDirections[1]); assert(s.moving() && s.alarm_type()==Alarm::RED_LIGHT);
  click(s,input,visible,kDemoDirections[2]); assert(s.alarm());
  click(s,input,visible,kDemoDirections[2]); assert(!s.moving() && !s.alarm());
  // 两个车辆状态栏只显示状态，点击不会修改运动。
  for(const auto& status:kDemoStatus) { click(s,input,visible,status); assert(!s.moving()); }
  // 同一周期两次完整点击应先开始再停止，不能合并成一次开始。
  const auto b=kDemoDirections[2].tl()+cv::Point(5,5);
  input.press(b.x,b.y); input.release(); input.press(b.x,b.y); input.release();
  controls(s,input,visible); assert(!s.moving());
  // OpenCV 与 GLES 都通过同一个事件队列，按下松开组成一次单击。
  opencv_mouse_input(cv::EVENT_LBUTTONDOWN,b.x,b.y,cv::EVENT_FLAG_LBUTTON,&input);
  opencv_mouse_input(cv::EVENT_LBUTTONUP,b.x,b.y,0,&input);
  controls(s,input,visible); assert(s.moving());
  opencv_mouse_input(cv::EVENT_MOUSEMOVE,20,20,0,&input);
  controls(s,input,visible); assert(s.moving());
  click(s,input,visible,kDemoDirections[2]); assert(!s.moving());
  // 车道与方向独立：换车道会清除旧灯态，不会自动改行驶方向。
  click(s,input,visible,kDemoDirections[2]);
  click(s,input,visible,kDemoLanes[0]);
  assert(s.moving() && s.direction()==Direction::RIGHT && s.action()==Action::UNKNOWN);
  assert(s.alarm_type()==Alarm::INVALID_DIRECTION);
  s.reset(); assert(!s.moving() && s.direction()==Direction::RIGHT && s.lane()==Direction::LEFT);

  // 保留不依赖配置的选灯、时间确认与中心圆灯规则。
  s.set_lane(Direction::STRAIGHT);
  for(double t:{0.,.3,.65}) s.observe(t,{lamp(7,30),lamp(11,310),lamp(9,470)},640,360);
  assert(s.action()==Action::GO && s.intersection().selected_signal.box.x==310);
  s.reset(); confirm(s,10); assert(s.action()==Action::STOP);
  for(Direction d:{Direction::LEFT,Direction::RIGHT}) {
    s.set_lane(d); confirm(s,7); assert(s.action()==Action::UNKNOWN);
  }
  // 直行新增直行箭头：箭头优先，圆灯颜色不覆盖箭头，短时漏检不立即切灯。
  Simulator straight;
  for(double t:{0.,.3,.65}) straight.observe(t,{lamp(7,310),lamp(14,430)},640,360);
  assert(straight.action()==Action::GO && straight.intersection().selected_signal.box.x==430);
  straight.toggle_direction(Direction::STRAIGHT); assert(!straight.alarm());
  for(double t:{1.,1.3}) straight.observe(t,{lamp(11,310),lamp(10,430)},640,360);
  assert(straight.alarm() && straight.intersection().selected_signal.box.x==430);
  straight.observe(1.6,{lamp(11,310)},640,360);
  assert(straight.action()==Action::UNKNOWN && !straight.alarm());
  for(double t:{2.2,2.5,2.85}) straight.observe(t,{lamp(11,310)},640,360);
  assert(straight.action()==Action::GO && straight.intersection().selected_signal.box.x==310);
  for(double t:{1.,1.3,1.65}) s.observe(t,{lamp(7,310),lamp(13,470)},640,360);
  assert(s.action()==Action::GO && s.intersection().selected_signal.box.x==470);
  s.toggle_direction(Direction::RIGHT); assert(s.moving() && !s.alarm());
  s.tick(2.6); assert(s.action()==Action::UNKNOWN && !s.alarm());
  s.set_lane(Direction::STRAIGHT);
  s.observe(3.,{lamp(7,100)},640,360); s.observe(3.3,{lamp(7,310)},640,360);
  assert(s.confirmations()==1 && s.action()==Action::WAIT);
  for(int i=0;i<100;++i) { s.observe(3.3,{lamp(7,310)},640,360); s.tick(3.35); }
  assert(s.confirmations()==1);
  s.observe(3.55,{lamp(7,311)},640,360); assert(s.action()==Action::STOP);
  s.observe(3.8,{},640,360); assert(s.action()==Action::UNKNOWN);
  s.observe(4.,{lamp(7),lamp(11)},640,360); assert(s.action()==Action::UNKNOWN);
  s.observe(4.3,{lamp(7,310,.4f)},640,360); assert(s.action()==Action::UNKNOWN);
  controls(s,input,visible,'H'); assert(!visible);
  s.stop(); controls(s,input,visible,'1'); assert(s.moving() && s.direction()==Direction::LEFT);
  controls(s,input,visible,'1'); assert(!s.moving());
  controls(s,input,visible,'6'); assert(s.lane()==Direction::RIGHT && s.direction()==Direction::LEFT);
  for(int key:{'a','g','s',' '}) { controls(s,input,visible,key); assert(!s.moving()); }

  // 未知灯态不伪报闯红灯，但仍可依据已选择的用途车道判断方向违规。
  for(int lane=0;lane<3;++lane) for(int direction=0;direction<3;++direction) {
    Simulator unknown; unknown.set_lane(static_cast<Direction>(lane+1));
    unknown.toggle_direction(static_cast<Direction>(direction+1));
    assert(unknown.alarm_type()==(lane==direction?Alarm::NONE:Alarm::INVALID_DIRECTION));
    unknown.toggle_direction(static_cast<Direction>(direction+1)); assert(!unknown.alarm());
  }
  // 红灯切绿灯时，错误方向的报警文字切换为违规操作；正确方向切换可解除报警。
  Simulator transition; transition.set_lane(Direction::LEFT); confirm(transition,8);
  transition.toggle_direction(Direction::RIGHT); assert(transition.alarm_type()==Alarm::RED_LIGHT);
  const int red_signature=signal_demo_signature(transition,true);
  confirm(transition,12,1.); assert(transition.alarm_type()==Alarm::INVALID_DIRECTION);
  assert(signal_demo_signature(transition,true)!=red_signature);
  transition.toggle_direction(Direction::LEFT); assert(!transition.alarm());

  if(argc>1) {
    if(argc>2) assert(adas::ui::initialize_text(argv[2]));
    cv::Mat canvas(1080,1920,CV_8UC3,cv::Scalar(24,15,7));
    confirm(s,13,5.); s.toggle_direction(Direction::LEFT);
    const auto intersection=s.intersection();
    draw_sidebar(canvas,26,8,0,120,intersection.selected_signal,adas::LaneResult(),adas::LaneResult(),false,
        adas::RiskResult(),adas::RiskResult(),adas::BlindSpotResult(),adas::BlindSpotResult(),true,true,
        0,std::array<double,2>{{-1,-1}},adas::VehicleState(),adas::LaneSemantic(),adas::LaneChangeResult(),
        intersection,true,false,false);
    draw_signal_demo_controls(canvas,s,true);
    cv::Mat view(360,640,CV_8UC3,cv::Scalar(65,65,65));
    draw_signal_demo_view(view,s,true,false,{lamp(13)},s.direction_matches_lane(),s.alarm_title());
    const auto band=adas::ui::compact_signal_hud(view.size(),{lamp(13)});
    const auto pixel=view.at<cv::Vec3b>(band.y+1,band.x+band.width/2);
    assert(pixel[2]>180 && pixel[0]<150); // 方向不匹配时的报警仍须显示为红色，而非被灰色预览覆盖。
    view.copyTo(canvas(cv::Rect(16,124,640,360))); assert(cv::imwrite(argv[1],canvas));
  }
  std::cout << "INPUT_MODE_REGRESSION_PASS: 9 red/green/unknown pairs, violation text, red-light priority, toggle stop\n";
}
