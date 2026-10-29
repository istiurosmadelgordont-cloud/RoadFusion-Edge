// Included inside four_view.cpp's anonymous namespace: presentation and input
// routing use the application's real controls and safety state.
class RoadFusionUi {
 public:
  bool dirty=true,theme_menu=false;
  int utility=0; // 0 closed, 1 FCW, 2 runtime information
  static cv::Rect overview(){return {604,30,126,40};}
  static cv::Rect surround(){return {742,30,126,40};}
  static cv::Rect fcw(){return {1280,30,110,36};}
  static cv::Rect info(){return {1400,30,110,36};}
  static cv::Rect themes(){return {1522,30,184,36};}
  static cv::Rect theme_item(int i){return {1522,86+i*43,350,39};}
  static cv::Rect utility_switch(){return {1410,248,458,38};}
  static cv::Rect pedestrian_switch(){return {1410,350,458,38};}
  static cv::Rect utility_close(){return {1834,185,34,30};}
  RoadFusionUi(){int t=0;std::ifstream f("config/ui_theme.txt");if(f>>t && t>=0 && t<7)adas::dashboard::theme()=t;}
  bool escape(){if(!utility&&!theme_menu)return false;utility=0;theme_menu=false;dirty=true;return true;}
  void route(adas::MomentaryInput::Snapshot& input,int& x,int& y,int& top,
             bool& avm_visible,bool& verify,adas::NavigationDemo& nav) {
    std::vector<cv::Point> remaining;
    for(const auto& p:input.clicks){
      bool used=true;
      if(themes().contains(p)){theme_menu=!theme_menu;utility=0;}
      else if(theme_menu){
        for(int i=0;i<7;++i)if(theme_item(i).contains(p)){
          adas::dashboard::theme()=i;std::ofstream("config/ui_theme.txt")<<i<<'\n';
        }
        theme_menu=false;
      } else if(fcw().contains(p)){utility=utility==1?0:1;}
      else if(info().contains(p)){utility=utility==2?0:2;}
      else if(utility){
        if(utility==1 && utility_switch().contains(p))remaining.push_back(kFcwToggle.tl()+cv::Point(2,2));
        else if(utility==1 && pedestrian_switch().contains(p))remaining.push_back(kTest4Toggle.tl()+cv::Point(2,2));
        else if(utility_close().contains(p) || !cv::Rect(1390,178,498,280).contains(p))utility=0;
      } else if(overview().contains(p) || cv::Rect(1410,186,146,30).contains(p)){top=0;avm_visible=false;}
      else if(surround().contains(p) || cv::Rect(1566,186,146,30).contains(p)){top=2;avm_visible=false;}
      else if(top==2 && cv::Rect(1410,424,206,26).contains(p)){avm_visible=!avm_visible;}
      else if(kSignalVerify.contains(p)){verify=!verify;verify_changed=true;}
      else if(top==0 && nav.click(p)){}
      else {used=false;remaining.push_back(p);}
      if(used){dirty=true;x=y=-1;input.click={-1,-1};input.down=false;}
    }
    input.clicks.swap(remaining);
  }
  bool verify_changed=false;
  void draw(cv::Mat& canvas,const adas::CameraViewLayout& layout,
      const adas::LaneSignalSimulator& demo,const adas::NavigationDemo& nav,
      const adas::NavigationDemo::Advice& advice,const adas::FcwState& fcw_state,
      const adas::Test4Result& test4,const adas::LaneChangeResult& change,
      const adas::LaneSemantic& semantic,const LaneToggleState& lane_toggle,
      const Options& options,bool verify,bool overlay,bool live,int top,
      double fps,double yolo,double ufld,double npu,bool fresh,
      const cv::Mat& avm,const adas::ManualAvmClock& avm_clock,double seconds,
      bool manual,bool avm_visible,const adas::WarningSummary& warning,
      const std::string& renderer) {
    using namespace adas::dashboard;
    canvas.setTo(bg());
    // Subtle header gradient is rasterized only when UI state/metrics refresh.
    for(int x=0;x<canvas.cols;++x){const double a=double(x)/canvas.cols*.5;
      cv::line(canvas,{x,0},{x,96},bg()*(1-a)+button()*a);}
    cv::line(canvas,{0,97},{1919,97},line());
    cv::line(canvas,{40,64},{50,34},accent(),2,cv::LINE_AA);
    cv::line(canvas,{60,64},{53,34},accent(),2,cv::LINE_AA);
    label(canvas,"RoadFusion",{84,60},30);label(canvas,"E D G E",{250,54},11,accent());
    cv::line(canvas,{296,34},{296,66},line());
    label(canvas,"四视角高级辅助驾驶系统",{322,42},16);
    label(canvas,"FPGA + RK3568 · EDGE INTELLIGENCE",{322,67},12,accent());
    control(canvas,overview(),"驾驶总览",top==0);control(canvas,surround(),"环视拼接",top==2);
    control(canvas,fcw(),"碰撞预警",fcw_state.enabled||test4.enabled);control(canvas,info(),"运行信息",utility==2);
    control(canvas,themes(),theme_name(theme()),theme_menu,14);
    control(canvas,{1720,30,168,36},live?"● PCIe 实时输入":"● 视频回放",true,14);
    label(canvas,"道路感知",{32,146},24);label(canvas,"ROAD PERCEPTION",{146,145},12,muted());
    control(canvas,kLaneDetectionToggle,lane_toggle.label(options.lanes_enabled),lane_toggle.requested,14);
    control(canvas,adas::CameraViewLayout::priority_button(),"主视角优先",layout.priority(),14);
    control(canvas,adas::CameraViewLayout::return_button(),layout.focused()<0?"四路均分":"返回四路",layout.focused()<0&&!layout.priority(),14);
    if(!live){control(canvas,kPrevSceneButton,"上一场景",false,12);control(canvas,kNextSceneButton,"下一场景",false,12);}
    const char* names[]={"前视","后视","左视","右视"};
    const char* english[]={"FRONT CAMERA","REAR CAMERA","LEFT CAMERA","RIGHT CAMERA"};
    const char* numbers[]={"01","04","02","03"};
    const auto views=layout.rects();
    if(layout.focused()>=0)for(int i=0;i<4;++i)control(canvas,adas::CameraViewLayout::select_button(i),names[i],i==layout.focused(),14);
    for(int i=0;i<4;++i)if(views[i].area()>0){
      const auto r=views[i];card(canvas,{r.x-1,r.y-39,r.width+2,r.height+(layout.priority()&&i==0?40:66)});
      control(canvas,{r.x+14,r.y-29,28,24},numbers[i],false,12);
      label(canvas,names[i],{r.x+54,r.y-10},18);label(canvas,english[i],{r.x+108,r.y-11},12,muted());
      if(!(layout.priority()&&i==0))label(canvas,"● 目标感知",{r.x+16,r.br().y+19},12,accent());
      control(canvas,adas::CameraViewLayout::zoom_button(r),"[ ]",layout.focused()==i,15);
    }
    card(canvas,{1390,178,498,280});
    control(canvas,{1410,186,146,30},"模拟导航",top==0,14);
    control(canvas,{1566,186,146,30},"环视拼接",top==2,14);
    if(top==0){
      control(canvas,adas::NavigationDemo::toggle_button(),nav.enabled()?"导航模拟 开":"导航模拟 关",nav.enabled(),13);
      const auto dir=nav.direction();
      cv::Point base(1432,286),bend(1432,246);
      if(dir==adas::ManeuverDirection::STRAIGHT)cv::arrowedLine(canvas,base,{1432,236},accent(),5,cv::LINE_AA,0,.3);
      else{cv::line(canvas,base,bend,accent(),5,cv::LINE_AA);cv::arrowedLine(canvas,bend,{dir==adas::ManeuverDirection::LEFT?1412:1456,246},accent(),5,cv::LINE_AA,0,.45);}
      label(canvas,std::to_string(nav.distance()),{1480,274},48);label(canvas,"m",{1600,273},20,muted());
      label(canvas,std::string("前方路口")+nav.direction_text(),{1480,302},17);
      label(canvas,std::string("当前匹配方向：")+adas::LaneSignalSimulator::text(nav.current_route()),{1410,322},13,muted());
      control(canvas,{1410,330,458,29},advice.text,true,14);
      const char* ns[]={"↑ 左转","↑ 直行","↑ 右转"};const char* ds[]={"距离 -30 m","距离 +30 m","重置 120 m"};
      for(int i=0;i<3;++i){control(canvas,adas::NavigationDemo::direction_buttons()[i],ns[i],int(dir)==i+1,14);control(canvas,adas::NavigationDemo::distance_buttons()[i],ds[i],false,13);}
    } else if(top==2){
      if(!avm.empty()){cv::Mat thumb;cv::resize(avm,thumb,{202,202});thumb.copyTo(canvas({1412,220,202,202}));}
      label(canvas,manual?"七路素材 / 手动同步":"实时地面投影",{1640,239},15,accent());
      if(manual){
      control(canvas,kAvmPlay,manual?(avm_clock.playing()?"暂停补播":"开始 / 继续补播"):"未加载补播素材",manual&&avm_clock.playing(),14);
      control(canvas,kAvmReset,"停止并重置",false,14);
      const char* labels[]={"-0.1 秒","+0.1 秒","-0.5 秒","+0.5 秒"};
      for(int i=0;i<4;++i)control(canvas,kAvmAdjust[i],labels[i],false,13);
      }else{
        label(canvas,"手动补播未启用",{1640,280},16,muted());
        label(canvas,"需加载本地 AVM 素材",{1640,314},15,muted());
        label(canvas,"实时拼接继续更新",{1640,350},15,muted());
      }
      label(canvas,manual?number(avm_clock.position(seconds))+" s":"实时画面 · 近似纹理补位",{1640,416},13,muted());
      control(canvas,kIntentLeft,"左",change.direction==adas::ManeuverDirection::LEFT,13);
      control(canvas,kIntentOff,"直行",change.state==adas::LaneChangeState::IDLE,13);
      control(canvas,kIntentRight,"右",change.direction==adas::ManeuverDirection::RIGHT,13);
      control(canvas,{1410,424,206,26},avm_visible?"返回四路 [V]":"放大环视 [V]",avm_visible,13);
    } else {
      label(canvas,"危险模拟停车 / 测试四",{1410,236},17,accent());
      control(canvas,kTest4Enable,test4.enabled?"危险监测 开 [B]":"危险监测 关 [B]",test4.enabled,15);
      label(canvas,test4.can_resume?"刹车已自动解除":"安全后自动解除",{1640,268},15,test4.can_resume?accent():muted());
      label(canvas,test4_reason(test4),{1410,312},16,test4.brake?rgb(0xef7777):text());
      label(canvas,"连续安全至少 1 秒、4 次新观测后解除",{1410,346},15,muted());
      label(canvas,"解除后手动起步；未连接车辆制动执行器",{1410,378},14,muted());
      label(canvas,test4.brake?"模拟停车中":"模拟监测状态",{1410,424},18,test4.brake?rgb(0xef7777):accent());
    }
    card(canvas,{1390,474,498,110});label(canvas,"交通灯与路口决策",{1410,505},17);label(canvas,"SIGNAL",{1575,504},11,muted());
    control(canvas,kSignalVerify,verify?"HSV 复核 开":"HSV 复核 关",verify,12);
    rounded(canvas,{1410,526,94,38},bg(),18);
    for(int i=0;i<3;++i){bool on=(i==0&&demo.action()==adas::IntersectionAction::STOP)||(i==2&&demo.action()==adas::IntersectionAction::GO);
      cv::circle(canvas,{1428+i*29,545},9,on?(i==0?rgb(0xf17575):rgb(0x75d6a3)):line(),-1,cv::LINE_AA);}
    label(canvas,adas::intersection_action_caption(demo.action()),{1520,543},18,accent());
    label(canvas,std::string("当前路线：")+demo.lane_text()+" / "+(demo.moving()?"模拟行驶":"模拟停止"),{1520,568},13,muted());
    card(canvas,{1390,598,498,218});label(canvas,"用途车道与模拟行驶",{1410,630},17);
    control(canvas,kSignalOverlayToggle,overlay?"图形叠加 · 开":"图形叠加 · 关",overlay,13);
    const char* lanes[]={"左转车道","↑ 直行车道","右转车道"};const char* dirs[]={"1 左转","2 直行","3 右转"};
    for(int i=0;i<3;++i){control(canvas,kDemoLanes[i],lanes[i],int(demo.lane())==i+1,16);control(canvas,kDemoDirections[i],dirs[i],demo.moving()&&int(demo.direction())==i+1,14);}
    label(canvas,"模拟车辆状态",{1410,775},13,muted());control(canvas,kDemoStatus[0],"车辆行驶",demo.moving(),13);control(canvas,kDemoStatus[1],"车辆停止",!demo.moving(),13);
    label(canvas,std::string("左侧 ")+adas::lane_marking_caption(semantic.left)+" / 右侧 "+adas::lane_marking_caption(semantic.right),{1410,803},13,muted());
    label(canvas,demo.alarm()?demo.alarm_title():demo.moving()?"模拟 · 行驶":"模拟 · 停止",{1762,803},13,demo.alarm()?rgb(0xef7777):accent());
    card(canvas,{1390,830,498,132});label(canvas,"车道线控制",{1410,860},17);
    label(canvas,!options.lanes_enabled?"请先开启车道检测":test2_action_caption(change),{1660,860},13,change.solid_alarm?rgb(0xef7777):accent());
    const char* changes[]={"← 左变道","↑ 保持直行","右变道 →","强制左变道","强制右变道"};
    for(int i=0;i<5;++i){bool selected=i==1?change.state==adas::LaneChangeState::IDLE:
      (i==0||i==3?change.direction==adas::ManeuverDirection::LEFT:change.direction==adas::ManeuverDirection::RIGHT)&&change.forced==(i>2);
      control(canvas,i<3?kTest2Directions[i]:kTest2Force[i-3],changes[i],selected,14);}
    label(canvas,options.lanes_enabled?"车道检测已开启":"车道检测已关闭",{1577,940},12,muted());
    metrics(canvas,fps,yolo,ufld,npu,fresh,options.lanes_enabled,live);
    if(warning.priority>0){
      rounded(canvas,{1044,30,222,38},rgb(0x49251d),7);
      label(canvas,warning.priority>=100?"模拟驾驶报警":warning.priority>=70?"危险预警":"驾驶提示",{1058,56},18,rgb(0xffb295));
      cv::Mat detail=canvas(cv::Rect(1044,72,844,23));
      label(detail,warning.text,{0,17},14,rgb(0xffb295));
    }
    if(avm_visible){cv::rectangle(canvas,adas::CameraViewLayout::area(),bg(),-1);if(!avm.empty()){cv::Mat enlarged;cv::resize(avm,enlarged,{800,800});enlarged.copyTo(canvas({296,214,800,800}));}label(canvas,"2D 环视 · 近似地面投影 / V 返回",{48,202},19,accent());}
    if(utility){card(canvas,{1390,178,498,280});control(canvas,utility_close(),"×",false,20);
      label(canvas,utility==1?"碰撞预警":"运行信息",{1410,212},21);
      if(utility==1){
        label(canvas,"车辆碰撞预警",{1410,239},14,muted());
        control(canvas,utility_switch(),fcw_state.enabled?"FCW 已开启 · 点击关闭":"FCW 已关闭 · 点击开启",fcw_state.enabled,17);
        label(canvas,!fcw_state.enabled?"已关闭":fcw_state.level==adas::FcwLevel::DANGER?"碰撞危险 · 请停车":fcw_state.level==adas::FcwLevel::CAUTION?"注意前车距离":fcw_state.status==adas::FcwStatus::WAIT_LANE?"等待车道检测":fcw_state.status==adas::FcwStatus::STALE?"等待新观测":"监测中 / 等待有效前车",{1410,310},16,fcw_state.level==adas::FcwLevel::DANGER?rgb(0xef7777):accent());
        label(canvas,"行人碰撞预警",{1410,341},14,muted());
        control(canvas,pedestrian_switch(),test4.enabled?"危险监测 开 [B]":"危险监测 关 [B]",test4.enabled,17);
        label(canvas,test4_reason(test4),{1410,413},15,test4.brake?rgb(0xef7777):muted());
        label(canvas,"安全后自动解除模拟停车；解除后手动起步",{1410,439},13,muted());
      }else{label(canvas,"渲染器："+renderer,{1410,252},17);label(canvas,"采集 / 预览：同步 RGA",{1410,288},17);label(canvas,"模型：YOLO INT8 + MO FP16",{1410,324},17);label(canvas,"显示 "+number(fps)+" FPS / 识别 "+number(yolo)+" FPS",{1410,360},17,accent());label(canvas,"L 车道线 · Z 放大 · V 环视 · F11 窗口 · Q 退出",{1410,416},13,muted());}
    }
    if(theme_menu){card(canvas,{1514,78,366,314});for(int i=0;i<7;++i)control(canvas,theme_item(i),theme_name(i),i==theme(),17);}
    dirty=false;
  }
  void metrics(cv::Mat& canvas,double fps,double yolo,double ufld,double npu,bool fresh,bool lanes,bool live){
    using namespace adas::dashboard;
    card(canvas,{1390,976,498,84});const char* metric[]={"DISPLAY","YOLO","UFLD","INFERENCE"};
    const double values[]={fps,yolo,ufld,npu};
    for(int i=0;i<4;++i){int x=1410+118*i;label(canvas,metric[i],{x,998},11,muted());label(canvas,i==3?std::to_string(int(std::round(values[i]))):number(values[i]),{x,1027},26);label(canvas,i==3?"ms":"FPS",{x+64,1027},11,muted());}
    label(canvas,fresh?"● NPU 正常":"● NPU 等待新观测",{1410,1050},11,accent());label(canvas,lanes?"● UFLD 已开启":"○ UFLD 已关闭",{1570,1050},11,muted());label(canvas,live?"● PCIe CAMERA":"● REPLAY",{1750,1050},11,accent());
  }
 private:
  static std::string number(double n){std::ostringstream s;s<<std::fixed<<std::setprecision(1)<<n;return s.str();}
};
