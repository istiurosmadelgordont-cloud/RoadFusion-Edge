// 编译真实主程序；链接时丢弃未使用的采集/NPU 入口，不访问 PCIe 或模型。
#define main board_application_main
#include "../src/four_view.cpp"
#undef main
#include <cassert>
#include <sys/stat.h>

int main() {
  const auto parsed = [](std::vector<std::string> words, Options& options) {
    words.insert(words.begin(), "adas_four_view");
    std::vector<char*> argv;
    for (auto& word : words) argv.push_back(&word[0]);
    return parse(static_cast<int>(argv.size()), argv.data(), options);
  };
  Options options;
  assert(parsed({"--pcie", "mock", "--display-fps", "0"}, options));
  assert(options.display_fps_limit == 0.0 && options.pcie_mode == "read");
  assert(parsed({"--pcie-mode", "mmap"}, options) && options.pcie_mode == "mmap");
  assert(parsed({"--display-fps", "15", "--display-fps", "0"}, options));
  assert(options.display_fps_limit == 0.0); // 后置参数可解除已有上限。
  assert(parsed({"--display-fps", "29.97"}, options));
  assert(std::abs(options.display_fps_limit - 29.97) < 1e-9);
  for (const char* invalid : {"-1", "nan", "inf", "15junk", ""})
    assert(!parsed({"--display-fps", invalid}, options));

  char original[4096]; assert(::getcwd(original, sizeof(original)));
  char temporary[] = "/tmp/board-presentation.XXXXXX";
  const char* root = ::mkdtemp(temporary); assert(root);
  assert(::chdir(root) == 0);
  const std::vector<std::string> directories{
      "work", "work/four_view_sample", "work/four_view_sample/sample",
      "board_v8", "board_v8/four_view_sample", "board_v8/four_view_sample/sample",
      "work/config", "work/config/simulated_test_1_scenes",
      "work/config/simulated_test_1_scenes/sample"};
  for (const auto& path : directories) assert(::mkdir(path.c_str(), 0700) == 0);
  const auto write_roi = [](const std::string& file) {
    std::ofstream output(file.c_str()); output << "roi 0.10 0.05 0.90 0.70\n";
    assert(output.good());
  };
  write_roi("board_v8/four_view_sample/sample/signal_demo.txt");
  assert(::chdir("work") == 0);
  const std::string scene = "four_view_sample/sample";
  const std::string shared = "../board_v8/" + scene + "/signal_demo.txt";
  assert(adas::scene_asset_path(scene, "signal_demo.txt") == shared);

  // PCIe 的配置无需 MP4；同一 ROI 让两种输入都能显示并操作行驶/停车控件。
  adas::SignalDemo live_demo, replay_demo;
  assert(live_demo.load(adas::scene_asset_path(scene, "signal_demo.txt")));
  assert(replay_demo.load(shared));
  live_demo.set_motion(1); replay_demo.set_motion(1);
  cv::Mat live_ui(1080, 1920, CV_8UC3, cv::Scalar());
  cv::Mat replay_ui = live_ui.clone();
  draw_camera_cards(live_ui); draw_camera_cards(replay_ui);
  draw_signal_demo_controls(live_ui, live_demo, true);
  draw_signal_demo_controls(replay_ui, replay_demo, true);
  assert(cv::norm(live_ui, replay_ui, cv::NORM_INF) == 0);
  assert(live_demo.enabled() && live_demo.motion_mode() == 1);

  // 两套输入不再只比较“画得一样”，还覆盖真实点击 -> 灯态 -> 预警输出。
  for (bool mouse : {false, true}) {
    adas::SignalDemo demo; assert(demo.load(shared));
    bool overlay = true, verify = true;
    const cv::Point nowhere(-1,-1);
    const auto motion = [&](int mode) {
      const auto rect = kDemoMotion[mode];
      OpenCvPointer pointer;
      on_opencv_mouse(cv::EVENT_LBUTTONDOWN, rect.x+rect.width/2,
                      rect.y+rect.height/2, 0, &pointer);
      return apply_demo_controls(demo, overlay, verify,
          mouse ? -1 : "AGS"[mode], mouse ? pointer.click : nowhere);
    };
    const auto lamp = [](int id) {
      adas::Detection result; result.class_id=id; result.score=.99f;
      result.box=cv::Rect2f(250,100,30,30); return result;
    };
    const auto warning = [&]() {
      adas::WarningManager manager;
      auto result=manager.update(false,false,false,false,false,
          adas::LaneChangeResult(),demo.intersection());
      demo.update_warning(result); return result;
    };
    assert(motion(1) & DemoMotion);
    assert(demo.moving() && !demo.alarm()); // 单按 G 不能伪造红灯报警。
    assert(std::string(demo.alarm_status())=="待确认" && warning().priority==0);
    demo.observe(0,{lamp(7)},640,360); demo.observe(.4,{lamp(7)},640,360);
    assert(demo.alarm() && demo.intersection().action==adas::IntersectionAction::STOP);
    const auto red_alarm=warning();
    assert(red_alarm.primary==adas::WarningCode::INTERSECTION && red_alarm.priority==110);
    assert(std::string(demo.alarm_status())=="告警");
    cv::Mat alarm_canvas(1080,1920,CV_8UC3,cv::Scalar());
    const cv::Mat header(76,980,CV_8UC3,cv::Scalar());
    draw_global_warning(alarm_canvas,red_alarm,adas::RiskResult(),adas::RiskResult(),
        {{0,0,0,0}},header);
    const auto pixel=alarm_canvas.at<cv::Vec3b>(2,435);
    assert(pixel[2]>200 && pixel[1]<100); // 高优先级报警为红色，不能降成黄色。
    assert(motion(2) & DemoMotion);
    assert(!demo.moving() && !demo.alarm() && warning().priority==60);
    assert(std::string(demo.alarm_status())=="已停车");
    assert(motion(0) & DemoMotion);
    demo.observe(.8,{lamp(11)},640,360); assert(!demo.moving());
    demo.observe(1.2,{lamp(11)},640,360); demo.observe(1.6,{lamp(11)},640,360);
    assert(demo.action()==adas::IntersectionAction::GO && demo.moving());
    assert(!demo.alarm() && warning().priority==0);
    assert(motion(2) & DemoMotion); assert(!demo.moving());
    assert(motion(1) & DemoMotion); assert(demo.moving() && !demo.alarm());
    const auto left=kDemoDirections[0];
    assert(apply_demo_controls(demo,overlay,verify,mouse?-1:'1',mouse?
        cv::Point(left.x+5,left.y+5):nowhere) & DemoDirection);
    assert(demo.direction()==adas::ManeuverDirection::LEFT && demo.confirmations()==0);
    assert(demo.action()!=adas::IntersectionAction::GO && !demo.alarm());
    demo.observe(2,{lamp(9)},640,360); // 右箭头红灯不能触发左转预警。
    demo.observe(2.4,{lamp(8)},640,360); demo.observe(2.8,{lamp(8)},640,360);
    assert(demo.alarm());
    assert(apply_demo_controls(demo,overlay,verify,'H',nowhere) & DemoOverlay);
    assert(!overlay && warning().priority==110); // 隐藏图形不关闭预警逻辑。
    assert(apply_demo_controls(demo,overlay,verify,-1,kSignalVerify.tl()+cv::Point(5,5)) & DemoVerify);
    assert(!verify && demo.confirmations()==0 && !demo.alarm() && demo.motion_mode()==1);
  }

  write_roi(scene + "/signal_demo.txt");
  assert(adas::scene_asset_path(scene, "signal_demo.txt") == scene + "/signal_demo.txt");
  assert(std::remove((scene + "/signal_demo.txt").c_str()) == 0);
  assert(std::remove(shared.c_str()) == 0);
  const std::string bundled = "config/simulated_test_1_scenes/sample/signal_demo.txt";
  write_roi(bundled);
  assert(adas::scene_asset_path(scene, "signal_demo.txt") == bundled);
  const std::string absolute = std::string(root) + "/missing/sample";
  assert(adas::scene_asset_path(absolute, "signal_demo.txt") == absolute + "/signal_demo.txt");

  // 标定中即使有额外相机，实时四路也只能使用前四个映射。
  const std::string avm_file = scene + "/avm.yml";
  {
    cv::FileStorage file(avm_file, cv::FileStorage::WRITE);
    file << "version" << 2 << "pixels_per_metre" << 25 << "vehicle_width" << 2.0
         << "vehicle_length" << 4.72;
    const cv::Mat map(256, 256, CV_32F, cv::Scalar(0.5));
    const cv::Mat mask(256, 256, CV_8U, cv::Scalar(255));
    for (const char* camera : {"front", "rear_left"})
      file << camera << "{" << "map_x" << map << "map_y" << map << "mask" << mask << "}";
  }
  adas::AvmRenderer avm;
  assert(avm.load(adas::scene_asset_path(scene, "avm.yml")));
  assert(avm.count() == 2 && avm.count(4) == 1);

  assert(std::remove(avm_file.c_str()) == 0);
  assert(std::remove(bundled.c_str()) == 0);
  assert(::chdir(root) == 0);
  for (auto path = directories.rbegin(); path != directories.rend(); ++path)
    assert(::rmdir(path->c_str()) == 0);
  assert(::chdir(original) == 0); assert(::rmdir(root) == 0);
  std::cout << "PASS: default read, unlimited FPS, scene profiles, mouse/keys -> red alarm/stop/green auto, direction reset, HSV and live AVM\n";
}
