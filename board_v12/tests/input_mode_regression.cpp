// 检查主程序真实控件、选灯和预警入口；不打开设备或模型。
#define main four_view_application_entry_for_test
#define ADAS_INPUT_MODE_TEST
#include "../src/four_view.cpp"
#undef main
#include "../src/warning_logic.cpp"
#include <cassert>

namespace {
using Direction=adas::ManeuverDirection;
using Action=adas::IntersectionAction;
using Simulator=adas::LaneSignalSimulator;
using Alarm=Simulator::AlarmType;
std::chrono::steady_clock::time_point at(int ms) {
  return std::chrono::steady_clock::time_point(std::chrono::seconds(1))+std::chrono::milliseconds(ms);
}
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
  {
    RoadFusionUi ui;assert(!ui.lane_assist_enabled);
    const auto button=RoadFusionUi::lane_assist();
    assert((button & RoadFusionUi::fcw()).area()==0);
    assert((button & cv::Rect(878,30,178,38)).area()==0);
    adas::NavigationDemo nav;adas::MomentaryInput mouse;
    int x=-1,y=-1,top=0;bool avm=false,verify=true;
    auto press=[&](){
      mouse.press(button.x+5,button.y+5);mouse.release();auto input=mouse.take();
      ui.route(input,x,y,top,avm,verify,nav);assert(input.clicks.empty());
    };
    press();assert(ui.lane_assist_enabled && ui.lane_assist_changed);
    assert(top==0 && !avm && verify && !nav.enabled());
    auto empty=mouse.take();ui.route(empty,x,y,top,avm,verify,nav);
    assert(ui.lane_assist_enabled); // a held/consumed click cannot toggle twice
    adas::LaneResult measured;measured.valid=true;measured.offset_ratio=.3f;
    measured.left={{100,350},{260,150}};measured.right={{540,350},{380,150}};
    adas::LaneDepartureMonitor monitor;
    auto at=std::chrono::steady_clock::now();
    monitor.update(measured,at);
    auto departed=monitor.update(measured,at+std::chrono::milliseconds(200));
    assert(departed.departure);
    const auto on=center_lane_display(departed,true);
    const auto off=center_lane_display(departed);
    assert(on.departure && on.departure_side==departed.departure_side);
    assert(!off.departure && off.offset_ratio==0.f);
    assert(on.left==off.left && on.right==off.right);
    adas::WarningManager manager;
    const auto alarm=manager.update(false,false,false,false,true,{},{});
    assert(alarm.primary==adas::WarningCode::LDW);
    const auto collision=manager.update(true,false,false,false,true,{},{});
    assert(collision.primary==adas::WarningCode::FCW);
    press();assert(!ui.lane_assist_enabled);
    std::cout<<"LANE_ASSIST_TOGGLE_PASS: independent consumed clicks; default off; real departure monitor; drawing and warning priority\n";
  }
  {
    bool fcw=false;adas::MomentaryInput clicks;
    clicks.press(kFcwToggle.x+10,kFcwToggle.y+10);clicks.release();
    assert(apply_fcw_controls(fcw,clicks.take()) && fcw);
    assert(!apply_fcw_controls(fcw,clicks.take()) && fcw);
    clicks.press(kFcwToggle.x+10,kFcwToggle.y+10);clicks.release();
    assert(apply_fcw_controls(fcw,clicks.take()) && !fcw);
    cv::Mat canvas(1080,1920,CV_8UC3,cv::Scalar());
    // FCW is rendered in the dashboard popover, never at the action token.
    const char* words[]={"adas","--pcie-camera-order","left,front,right,rear","--enable-fcw"};
    std::vector<std::string> text(words,words+4);std::vector<char*> args;
    for(auto& word:text)args.push_back(&word[0]);
    Options parsed;assert(parse(args.size(),args.data(),parsed) && parsed.fcw_enabled);
    assert(adas::CameraSlotOrder(parsed.pcie_camera_order).slot(0)==1);
    std::cout<<"FCW_CONTROLS_PASS: original button, single click consumption and native front slot CLI\n";
  }
  {
    Options options;assert(options.pcie_buffers==6);
    assert(parse_test({"test","--pcie-buffers","4"},options) && options.pcie_buffers==4);
    assert(!parse_test({"test","--pcie-buffers","2"},options));
    assert(!parse_test({"test","--pcie-io","read"},options));
    assert(!parse_test({"test","--pcie-preview","cpu"},options));
    std::cout<<"PCIE_RGA_DEFAULT_PASS: fixed RGA input/preview; bounded pool and former mode switches rejected\n";
  }
  {
    adas::LaneResult lane;lane.valid=true;lane.departure=true;
    lane.departure_side=adas::LaneDepartureSide::RIGHT;lane.offset_ratio=.3f;
    lane.left={{100,350},{260,150}};lane.right={{540,350},{380,150}};
    suppress_lane_departure_alarm(lane);
    assert(!lane.departure && lane.departure_side==adas::LaneDepartureSide::NONE);
    assert(lane.valid && lane.offset_ratio==.3f && lane.left.front()==cv::Point(100,350));
    adas::LaneSemantic semantic;semantic.trend=adas::CrossingSide::RIGHT;semantic.tlc_s=.5f;
    semantic.left=adas::LaneMarking::DASHED;semantic.right=adas::LaneMarking::SOLID;
    auto levels=view_alert_levels({}, {},lane,semantic,{}, {},false,{});
    assert(levels[0]==0); // approach warnings are disabled as well
    adas::RiskResult risk;risk.target=risk.reliable=true;risk.warning=true;risk.ttc_s=1.f;
    levels=view_alert_levels(risk,{},lane,semantic,{}, {},false,{});
    assert(levels[0]==3); // disabling LDW must not disable object warnings
    const auto drawing=center_lane_display(lane);
    assert(drawing.offset_ratio==0.f && drawing.left==lane.left && drawing.right==lane.right);
    assert(semantic.left==adas::LaneMarking::DASHED && semantic.right==adas::LaneMarking::SOLID);
    std::cout<<"LDW_DISABLED_PASS: no departure/TLC alarm; geometry, paint evidence and object warnings retained\n";
  }
  {
    adas::Test4Safety safety; bool show=true; Simulator demo;
    adas::MomentaryInput input;
    auto press=[&](const cv::Rect& b,double t) {
      input.press(b.x+3,b.y+3); input.release();
      apply_test4_controls(safety,show,-1,input.take(),t);
    };
    press(kTest4Enable,0); assert(safety.current().enabled);
    adas::Detection ped;ped.class_id=0;ped.track_id=1;ped.score=.9f;ped.box={300,140,40,160};
    auto second=ped;second.track_id=2;second.box.x=360;
    safety.update({ped,second},640,360,nullptr,false,true,0,0);
    safety.update({ped,second},640,360,nullptr,false,true,.3,.3);
    assert(safety.current().hazards.size()==2);
    demo.toggle_direction(Direction::STRAIGHT);assert(demo.moving());
    apply_test4_interlock(safety.current(),demo);assert(!demo.moving());
    cv::Mat front(360,640,CV_8UC3,cv::Scalar(0,0,0));
    draw_test4_view(front,safety.current(),{ped,second});
    assert(front.at<cv::Vec3b>(140,300)[2]>200); // target follows visible tracker box
    assert(front.at<cv::Vec3b>(140,360)[2]>200); // both hazards have red outlines
    press(kTest4Resume,.31);assert(safety.current().brake);
    for(double t:{.6,.95,1.3,1.65}) safety.update({},640,360,nullptr,false,true,t,t);
    assert(!safety.current().brake && safety.current().can_resume); // no release click
    press(kTest4Resume,1.66);assert(!safety.current().brake && !demo.moving());
    demo.toggle_direction(Direction::STRAIGHT);assert(demo.moving());
    apply_test4_interlock(safety.current(),demo);assert(demo.moving());
    press(kTest4Tab,1.7);assert(!show);
    press(kTest4Enable,1.8);assert(safety.current().enabled);
    for(const auto& b:kDemoDirections) assert((b&kTest4Enable).area()==0 && (b&kTest4Resume).area()==0);
    for(const auto& b:kTest2Directions) assert((b&kTest4Enable).area()==0 && (b&kTest4Resume).area()==0);
    std::cout<<"TEST4_CONTROLS_PASS: multiple red targets, automatic stop/clear interlock, manual restart, hidden controls and original button isolation\n";
  }
  cv::setNumThreads(1);
  Options o;
  assert(parse_test({"viewer","--pcie","/dev/mock","--scene","missing/profile","--display-fps","0"},o));
  assert(o.display_fps_limit==0 && o.pcie_device=="/dev/mock");
  assert(parse_test({"viewer","--demo-lane","right"},o) && o.demo_direction=="right");
  for(const char* bad:{"-1","nan","inf","bad"}) assert(!parse_test({"viewer","--display-fps",bad},o));
  assert(!parse_test({"viewer","--demo-force-move"},o));

  {
    // 使用界面真实点击入口；按住或松开不会重复切换，其他按钮不会开启车道线。
    LaneToggleState toggle; adas::MomentaryInput lane_input;
    assert(!toggle.requested && !toggle.reserves_npu(false));
    lane_input.press(kLaneDetectionToggle.x+5,kLaneDetectionToggle.y+5);
    assert(apply_lane_toggle_controls(toggle,-1,lane_input.take()));
    assert(toggle.requested && toggle.reserves_npu(false));
    assert(!apply_lane_toggle_controls(toggle,-1,lane_input.take()));
    lane_input.release();
    assert(!apply_lane_toggle_controls(toggle,-1,lane_input.take()));
    const unsigned running_generation=toggle.generation;
    assert(toggle.accepts(true,running_generation));
    lane_input.press(kDemoDirections[0].x+5,kDemoDirections[0].y+5);
    assert(!apply_lane_toggle_controls(toggle,-1,lane_input.take()));
    // 模型在后台加载时可以取消；取消后仍先等加载线程退出，再恢复 NPU 提交。
    toggle.loading=true;
    lane_input.press(kLaneDetectionToggle.x+5,kLaneDetectionToggle.y+5);
    lane_input.release();
    assert(apply_lane_toggle_controls(toggle,-1,lane_input.take()));
    assert(!toggle.requested && toggle.reserves_npu(false));
    assert(!toggle.accepts(false,running_generation));
    toggle.loading=false;
    assert(!toggle.reserves_npu(false));
    assert(apply_lane_toggle_controls(toggle,'L',lane_input.take()));
    assert(toggle.requested && !toggle.accepts(true,running_generation));
    // 关闭再开启后，旧工作线程的结果仍不能恢复车道线。
    assert(toggle.accepts(true,toggle.generation));
    toggle.fail("missing lane model");
    assert(!toggle.requested && !toggle.reserves_npu(false) && !toggle.error.empty());
    assert(std::string(toggle.label(false)).find("失败")!=std::string::npos);
    lane_input.press(kLaneDetectionToggle.x+5,kLaneDetectionToggle.y+5);
    lane_input.release();
    assert(apply_lane_toggle_controls(toggle,-1,lane_input.take()));
    assert(toggle.requested && toggle.error.empty()); // 点击失败按钮可以重试。
    const cv::Point inside(kLaneDetectionToggle.x+5,kLaneDetectionToggle.y+5);
    lane_input.press(inside.x,inside.y); lane_input.release();
    lane_input.press(inside.x,inside.y); lane_input.release();
    const unsigned before_double_click=toggle.generation;
    assert(apply_lane_toggle_controls(toggle,-1,lane_input.take()));
    assert(toggle.requested && toggle.generation==before_double_click+2);
    std::cout<<"LANE_TOGGLE_REGRESSION_PASS: clicks, cancellation, retry and stale-result rejection\n";
  }

  {
    // Real Test Two click entry: independent of the Test One lane/light simulator.
    Simulator signal_one; bool overlay=true;
    adas::VehicleStateManager vehicle; adas::LaneChangeFsm fsm;
    adas::MomentaryInput buttons;
    const auto press=[&](const cv::Rect& b,bool enabled) {
      buttons.press(b.x+4,b.y+4);buttons.release();auto snapshot=buttons.take();
      apply_signal_demo_controls(signal_one,overlay,-1,snapshot);
      return apply_test2_controls(vehicle,fsm,snapshot,at(100),enabled);
    };
    assert(!press(kTest2Directions[0],false));
    assert(fsm.current().state==adas::LaneChangeState::IDLE);
    adas::LaneSemantic semantic;semantic.lane_valid=true;
    semantic.left=adas::LaneMarking::SOLID;
    semantic.right=adas::LaneMarking::DASHED;semantic.right_change_allowed=true;
    assert(press(kTest2Force[0],true));
    auto change=fsm.update(vehicle.snapshot(at(100)),semantic,false,false,true,false,at(100));
    assert(change.solid_alarm && change.forced && !change.permitted);
    auto alarm=adas::WarningManager().update(false,false,false,false,false,change,adas::IntersectionResult());
    assert(alarm.primary==adas::WarningCode::LANE_CHANGE && alarm.priority==95);
    assert(!signal_one.moving() && signal_one.lane()==Direction::STRAIGHT);
    assert(press(kTest2Directions[1],true));
    assert(!fsm.current().solid_alarm && fsm.current().state==adas::LaneChangeState::IDLE);
    assert(vehicle.snapshot(at(100)).turn==adas::TurnSignal::OFF);
    assert(press(kTest2Force[1],true));
    semantic.right=adas::LaneMarking::UNKNOWN;semantic.right_change_allowed=false;
    change=fsm.update(vehicle.snapshot(at(100)),semantic,false,false,true,false,at(100));
    assert(!change.solid_alarm && !change.permitted);
    assert(press(kTest2Directions[2],true));
    semantic.right=adas::LaneMarking::DASHED;semantic.right_change_allowed=true;
    change=fsm.update(vehicle.snapshot(at(100)),semantic,false,false,true,false,at(100));
    assert(!change.permitted && !change.forced);
    change=fsm.update(vehicle.snapshot(at(700)),semantic,false,false,true,false,at(700));
    assert(change.permitted);
    change=fsm.update(vehicle.snapshot(at(750)),semantic,false,true,true,false,at(750));
    assert(!change.permitted && change.state==adas::LaneChangeState::BLOCKED);
    assert(press(kTest2Directions[1],true));
    assert(fsm.current().state==adas::LaneChangeState::IDLE);
    // No Test One control overlaps either row of Test Two buttons.
    for(const auto& one:kDemoDirections)for(const auto& two:kTest2Directions)assert((one&two).area()==0);
    std::cout<<"TEST2_CONTROLS_PASS: solid alarm, cancel, dashed permission, blind block, independent Test One\n";
  }

  // 三车道 x 三方向：红灯任何行驶都报闯红灯，绿灯方向不符报违规操作。
  {
    // 当前测试二只按虚实线演示，不依赖 YOLO/盲区状态放行。
    for(int side=0;side<2;++side) {
      adas::VehicleState vehicle;vehicle.turn_valid=true;
      vehicle.turn=side==0?adas::TurnSignal::LEFT:adas::TurnSignal::RIGHT;
      const auto direction=side==0?Direction::LEFT:Direction::RIGHT;
      adas::LaneSemantic lane;lane.lane_valid=true;
      lane.left=lane.right=adas::LaneMarking::SOLID;
      adas::LaneChangeFsm rules;rules.request(direction,at(100));
      auto result=rules.update_rules(vehicle,lane,at(100));
      assert(!result.permitted && !result.solid_alarm && test2_motion_direction(result)==Direction::STRAIGHT);
      rules.request(direction,at(100),true);
      result=rules.update_rules(vehicle,lane,at(100));
      assert(result.solid_alarm && test2_motion_direction(result)==Direction::STRAIGHT);
      auto alarm=adas::WarningManager().update(false,false,false,false,false,result,adas::IntersectionResult());
      cv::Mat canvas(1080,1920,CV_8UC3,cv::Scalar(0,0,0));
      cv::Mat header(76,980,CV_8UC3,cv::Scalar(0,0,0));
      draw_global_warning(canvas,alarm,adas::RiskResult(),adas::RiskResult(),std::array<int,4>{{0,0,0,0}},header);
      const auto pixel=canvas.at<cv::Vec3b>(2,500);assert(pixel[2]>180 && pixel[0]<100);
      rules.request(direction,at(100));
      lane.left=lane.right=adas::LaneMarking::DASHED;
      lane.left_change_allowed=lane.right_change_allowed=true;
      result=rules.update_rules(vehicle,lane,at(100));
      assert(result.permitted && test2_motion_direction(result)==direction);
      lane.left=lane.right=adas::LaneMarking::UNKNOWN;
      lane.left_change_allowed=lane.right_change_allowed=false;
      result=rules.update_rules(vehicle,lane,at(200));
      assert(!result.permitted && !result.solid_alarm && test2_motion_direction(result)==Direction::STRAIGHT);
    }
    adas::LaneResult full;full.valid=true;
    for(auto& line:full.boundaries)line={{1,2},{3,4}};
    for(auto& area:full.lane_polygons)area={{1,2},{3,4},{5,6}};
    full.left=full.boundaries[1];full.right=full.boundaries[2];full.polygon=full.lane_polygons[1];
    const auto center=center_lane_display(full);
    assert(center.boundaries[0].empty() && center.boundaries[3].empty());
    assert(center.lane_polygons[0].empty() && center.lane_polygons[2].empty());
    assert(center.left==full.left && center.right==full.right && center.polygon==full.polygon);
    assert(!full.boundaries[0].empty() && !full.lane_polygons[2].empty());
    adas::LaneSemantic observed;observed.lane_valid=true;
    observed.left=adas::LaneMarking::DASHED;observed.left_change_allowed=true;
    assert(!adas::lane_decision_fresh(at(100),at(400))); // original LDW deadline unchanged
    assert(test2_rule_evidence(observed,full,at(100),at(749)).left_change_allowed);
    assert(!test2_rule_evidence(observed,full,at(100),at(750)).lane_valid);
    full.identity_uncertain=true;
    assert(!test2_rule_evidence(observed,full,at(100),at(200)).lane_valid);
    full.identity_uncertain=false;
    observed.left=adas::LaneMarking::UNKNOWN;observed.left_change_allowed=false;
    assert(!test2_rule_evidence(observed,full,at(100),at(400)).left_change_allowed);
    std::cout<<"TEST2_RULES_PASS: dashed arrows, solid straight, forced-solid red alarm, unknown hold, ego-only display\n";
  }

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
  // Explicit movement buttons are idempotent and preserve the selected direction.
  click(s,input,visible,kDemoStatus[0]);assert(s.moving() && s.direction()==Direction::RIGHT);
  click(s,input,visible,kDemoStatus[0]);assert(s.moving());
  click(s,input,visible,kDemoStatus[1]);assert(!s.moving());
  click(s,input,visible,kDemoStatus[1]);assert(!s.moving());
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

  // The actual dashboard router must own popover presses and preserve ordered clicks.
  RoadFusionUi dashboard;adas::NavigationDemo nav;nav.set_enabled(true);
  int top=0,x=-1,y=-1;bool avm=false,verify=true,fcw=false;
  auto route=[&](cv::Rect rect){
    adas::MomentaryInput events;auto p=rect.tl()+cv::Point(4,4);
    events.press(p.x,p.y);events.release();auto packet=events.take();
    x=p.x;y=p.y;dashboard.route(packet,x,y,top,avm,verify,nav);return packet;
  };
  auto packet=route(RoadFusionUi::fcw());assert(dashboard.utility==1 && packet.clicks.empty());
  packet=route(RoadFusionUi::utility_switch());assert(apply_fcw_controls(fcw,packet)&&fcw);
  packet=route(adas::NavigationDemo::distance_buttons()[0]);
  assert(packet.clicks.empty() && nav.distance()==120); // covered controls never activate
  dashboard.escape();route(adas::NavigationDemo::distance_buttons()[0]);assert(nav.distance()==90);
  route(adas::NavigationDemo::direction_buttons()[0]);assert(nav.direction()==Direction::LEFT);
  route(adas::NavigationDemo::distance_buttons()[2]);assert(nav.distance()==120);
  route(RoadFusionUi::surround());assert(top==2);
  route(kTest4Tab);assert(top==2);route(RoadFusionUi::overview());assert(top==0);
  route(kSignalVerify);assert(!verify && dashboard.verify_changed);
  route(RoadFusionUi::info());assert(dashboard.utility==2);assert(dashboard.escape());
  route(RoadFusionUi::themes());assert(dashboard.theme_menu);assert(dashboard.escape());
  adas::CameraViewLayout view_layout;assert(view_layout.select(-3)&&view_layout.priority());
  for(const auto& r:view_layout.rects())assert(r.area()>0 && (r&cv::Rect(0,0,1920,1080))==r);
  assert(view_layout.select(2)&&!view_layout.priority());assert(view_layout.select(-1));
  // New start button cannot bypass the existing danger-stop interlock.
  Simulator stopped;click(stopped,input,visible,kDemoStatus[0]);assert(stopped.moving());
  adas::Test4Result brake;brake.enabled=true;brake.brake=true;
  apply_test4_interlock(brake,stopped);assert(!stopped.moving());

  if(argc>1) {
    if(argc>2) assert(adas::ui::initialize_text(argv[2]));
    cv::Mat canvas(1080,1920,CV_8UC3,cv::Scalar(24,15,7));
    caption(canvas,"RoadFusion",24,39,cv::Scalar(250,209,95),1.03);
    caption(canvas,"Drive Smarter. A Safer Tomorrow.",26,62,cv::Scalar(185,160,130),.43);
    LaneToggleState preview_toggle;
    draw_lane_detection_toggle(canvas,preview_toggle,false);
    confirm(s,13,5.); s.toggle_direction(Direction::LEFT);
    const auto intersection=s.intersection();
    draw_sidebar(canvas,26,8,0,120,intersection.selected_signal,adas::LaneResult(),adas::LaneResult(),false,
        adas::RiskResult(),adas::RiskResult(),adas::BlindSpotResult(),adas::BlindSpotResult(),true,true,
        0,std::array<double,2>{{-1,-1}},adas::VehicleState(),adas::LaneSemantic(),adas::LaneChangeResult(),
        intersection,true,false,false);
    draw_signal_demo_controls(canvas,s,true);
    adas::LaneSemantic preview_semantic;preview_semantic.left=adas::LaneMarking::SOLID;
    preview_semantic.right=adas::LaneMarking::DASHED;
    draw_test2_controls(canvas,adas::LaneChangeResult(),preview_semantic,true);
    cv::Mat view(360,640,CV_8UC3,cv::Scalar(65,65,65));
    draw_signal_demo_view(view,s,true,false,{lamp(13)},s.direction_matches_lane(),s.alarm_title());
    draw_maneuver_hud(view,adas::LaneChangeResult(),preview_semantic);
    const auto band=adas::ui::compact_signal_hud(view.size(),{lamp(13)});
    const auto pixel=view.at<cv::Vec3b>(band.y+1,band.x+band.width/2);
    assert(pixel[2]>180 && pixel[0]<150); // 方向不匹配时的报警仍须显示为红色，而非被灰色预览覆盖。
    view.copyTo(canvas(cv::Rect(16,124,640,360))); assert(cv::imwrite(argv[1],canvas));
  }
  std::cout << "INPUT_MODE_REGRESSION_PASS: 9 red/green/unknown pairs, violation text, red-light priority, toggle stop\n";
}
