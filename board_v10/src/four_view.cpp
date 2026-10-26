#include "adas/config.hpp"
#include "adas/signal_demo.hpp"
#include "adas/lane_signal_simulator.hpp"
#include "adas/momentary_input.hpp"
#include "adas/avm_renderer.hpp"
#include "adas/pcie_capture.hpp"
#include "adas/image_processing_toolbar.hpp"
#include "adas/preview_resize.hpp"
#include "adas/light_verification_frames.hpp"
#include "adas/adas_logic_v2.hpp"
#include "adas/experimental_adas.hpp"
#include "adas/perception_schedule.hpp"
#include "adas/npu_executor.hpp"
#include "adas/npu_run_guard.hpp"
#include "adas/lane_freshness.hpp"
#include "adas/byte_tracker.hpp"
#include "adas/box_motion_compensator.hpp"
#include "adas/drive_decision.hpp"
#include "adas/lane_detector.hpp"
#include "adas/lane_geometry_tracker.hpp"
#include "adas/lane_visual_tracker.hpp"
#include "adas/lane_outer_style_tracker.hpp"
#include "adas/lane_paint_display.hpp"
#include "adas/lane_paint_contrast.hpp"
#include "adas/overlay.hpp"
#include "adas/risk_estimator.hpp"
#include "adas/fcw_monitor.hpp"
#include "adas/test4_safety.hpp"
#include "adas/navigation_demo.hpp"
#include "adas/rknn_detector.hpp"
#include "adas/signal_logic.hpp"
#include "adas/warning_logic.hpp"
#include "adas/text_renderer.hpp"
#include "adas/lamp_annotation.hpp"
#include "adas/ufld_lane_detector.hpp"
#ifdef ADAS_HAVE_GLES
#include "adas/gl_presenter.hpp"
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <future>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rga/rga.h>
#include <rga/im2d.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>

namespace {
// Accepted new results only: redraws and tracker predictions never count.
class RecognitionRate {
 public:
  using Clock = std::chrono::steady_clock;
  void reset() { samples_.clear(); started_ = Clock::now(); }
  void record() { samples_.push_back(Clock::now()); }
  double fps() {
    const auto now = Clock::now();
    while (!samples_.empty() && now - samples_.front() >= std::chrono::seconds(2))
      samples_.pop_front();
    const double elapsed = std::chrono::duration<double>(now - started_).count();
    return elapsed >= 1.0 ? samples_.size() / std::min(2.0, elapsed) : 0.0;
  }
 private:
  Clock::time_point started_ = Clock::now();
  std::deque<Clock::time_point> samples_;
};

volatile sig_atomic_t quit_requested = 0;
void request_quit(int) { quit_requested = 1; }

const int kTileW = 640;
const int kTileH = 360;
const int kHeaderH = 76;
const int kSideW = 640;
const int kCanvasW = 2 * kTileW + kSideW;
const int kCanvasH = 1080;
const int kFpgaViewW = 960;
const int kFpgaViewH = 540;
const int kFpgaCompositeW = 2 * kFpgaViewW;
const int kFpgaCompositeH = 2 * kFpgaViewH;
const char* kFiles[] = {"front.mp4", "rear.mp4", "left.mp4", "right.mp4"};
const cv::Rect kIntentLeft(1740, 372, 47, 35);
const cv::Rect kIntentOff(1791, 372, 47, 35);
const cv::Rect kIntentRight(1842, 372, 47, 35);
// 保持三列：用途车道、方向按钮、车辆状态。车道选择与模拟方向相互独立。
const std::array<cv::Rect,3> kDemoLanes{{{1408,650,148,28},{1568,650,148,28},{1728,650,148,28}}};
const std::array<cv::Rect,3> kDemoDirections{{{1408,686,148,28},{1568,686,148,28},{1728,686,148,28}}};
const std::array<cv::Rect,2> kDemoStatus{{{1408,722,148,28},{1568,722,148,28}}};
const cv::Rect kSignalOverlayToggle(1770,622,118,28);
const cv::Rect kSignalVerify(1404, 563, 125, 29);
const std::array<cv::Rect,3> kTest2Directions{{{1408,806,148,28},{1568,806,148,28},{1728,806,148,28}}};
const std::array<cv::Rect,2> kTest2Force{{{1408,840,148,27},{1728,840,148,27}}};
const cv::Rect kTest4Tab(1714,92,174,28);
const cv::Rect kTest4Enable(1408,144,220,30);
const cv::Rect kTest4Resume(1640,144,246,30);
// 左上角标题栏内，不遮挡前视画面，也不占用原有车道/方向按钮。
const cv::Rect kLaneDetectionToggle(240, 12, 176, 36);
const cv::Rect kFcwToggle(750,1026,300,32);
// 右上标题栏空位，不遮挡测试一、测试二、导航和车道线开关。
const cv::Rect kPrevSceneButton(1040, 980, 150, 38);
const cv::Rect kNextSceneButton(1200, 980, 150, 38);
// Display geometry is independent of the 640x360 inference/overlay images.
const std::array<cv::Rect, 4> kViewRects{{
    {16, 124, 672, 378}, {700, 124, 672, 378},
    {16, 568, 672, 378}, {700, 568, 672, 378}}};

struct Options {
  std::string pcie_device;
  std::string model = "models/yolo14_960x544_ptq_int8.rknn";
  std::string scene = "four_view_sample/e716f3ed";
  std::string snapshot;
  std::string demo_output;
  std::string demo_direction = "straight";
  bool signal_overlay_visible = true;
  std::string ufld_model = "models/ufldv2_culane_res18_1600x320_int8.rknn";
  int cpu_threads = 2;
  int npu_workers = 2;
  int detect_every = 2;
  int ufld_every = 5;
  int max_frames = 0;
  int snapshot_frame = -1;
  double display_fps_limit = 0.0;
  bool headless = false;
  bool benchmark = false;
  bool once = false;
  bool separate = false;
  bool dump_detections = false;
  bool opencv_display = false;
  bool software_decode = false;
  bool legacy_mosaic = false;
  bool ufld_rear = true;
  bool rear_lane_enabled = false;
  bool lanes_enabled = false;
  bool ttc_enabled = false;
  bool fcw_enabled = false;
  bool test4_enabled = false;
  bool demo_navigation = false;
};

// requested 表示用户意愿，Options::lanes_enabled 表示已经可以提交检测任务。
// 首次加载模型时二者不同；关闭后保留模型缓存，再次开启无需重复加载。
struct LaneToggleState {
  bool requested = false;
  bool loading = false;
  unsigned generation = 0;
  std::string error;

  void toggle() { requested = !requested; error.clear(); ++generation; }
  void fail(const std::string& reason) { requested = false; error = reason; }
  bool accepts(bool enabled, unsigned task_generation) const {
    return enabled && requested && task_generation == generation;
  }
  bool needs_model(bool enabled) const { return requested && !enabled && error.empty(); }
  bool reserves_npu(bool enabled) const { return loading || needs_model(enabled); }
  int signature(bool enabled) const {
    return (requested ? 1 : 0) | (loading ? 2 : 0) | (enabled ? 4 : 0) | (!error.empty() ? 8 : 0);
  }
  const char* label(bool enabled) const {
    if (!error.empty()) return "车道线：失败重试";
    if (!requested) return "车道线：关闭";
    return enabled ? "车道线：开启" : "车道线：开启中";
  }
};

bool apply_lane_toggle_controls(LaneToggleState& state, int key,
                                const adas::MomentaryInput::Snapshot& input) {
  const unsigned before = state.generation;
  if (key == 'l' || key == 'L') state.toggle();
  for (const auto& click : input.clicks)
    if (kLaneDetectionToggle.contains(click)) state.toggle();
  return state.generation != before;
}

struct LaneModelLoadResult {
  std::unique_ptr<adas::UfldLaneDetector> model;
  std::string error;
};

LaneModelLoadResult load_lane_model(const std::string& path, bool live) {
  LaneModelLoadResult result;
  try {
    if (path.empty()) { result.error = "未指定车道线模型，请设置 --ufld-model"; return result; }
    std::unique_ptr<adas::UfldLaneDetector> candidate(new adas::UfldLaneDetector(path));
    if (!candidate->ready()) result.error = candidate->error();
    else if (live && (candidate->input_width() != 1600 || candidate->input_height() != 320))
      result.error = "PCIe 车道线模型需要使用 1600x320 UFLDv2";
    else result.model = std::move(candidate);
  } catch (const std::exception& error) { result.error = error.what(); }
  return result;
}

bool parse(int argc, char** argv, Options& o) {
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--enable-lanes") { o.lanes_enabled = true; continue; }
    if (key == "--enable-test4") { o.test4_enabled = true; continue; }
    if (key == "--demo-navigation") { o.demo_navigation = true; continue; }
    if (key == "--no-test4") { o.test4_enabled = false; continue; }
    if (key == "--enable-fcw") { o.fcw_enabled=true; continue; }
    if (key == "--no-fcw") { o.fcw_enabled=false; continue; }
    if (key == "--enable-ttc") { o.ttc_enabled = true; continue; }
    if (key == "--no-signal-overlay") { o.signal_overlay_visible = false; continue; }
    if (key == "--signal-overlay") { o.signal_overlay_visible = true; continue; }
    if (key == "--no-ttc") { o.ttc_enabled = false; continue; }
    if (key == "--no-lanes") { o.lanes_enabled = false; continue; }
    if (key == "--demo-force-move") {
      std::cerr << "Forced movement removed; click a direction button to simulate driving\n";
      return false;
    }
    if (key == "--headless") { o.headless = true; continue; }
    if (key == "--benchmark") { o.benchmark = true; continue; }
    if (key == "--once") { o.once = true; continue; }
    if (key == "--separate") { o.separate = true; continue; }
    if (key == "--dump-detections") { o.dump_detections = true; continue; }
    if (key == "--opencv") { o.opencv_display = true; continue; }
    if (key == "--software-decode") { o.software_decode = true; continue; }
    if (key == "--legacy-mosaic") { o.legacy_mosaic = true; continue; }
    if (key == "--ufld-rear") { o.ufld_rear = true; o.rear_lane_enabled = true; continue; }
    if (key == "--no-rear-lane") { o.rear_lane_enabled = false; continue; }
    if (key == "--help") {
      std::cout << "--pcie /dev/fpga_frame_<BDF> --scene DIR --model FILE --detect-every N --cpu-threads N "
                   "--ufld-model FILE --ufld-every N --display-fps N --max-frames N --snapshot FILE --snapshot-frame N "
                   "--demo-lane left|straight|right --headless --benchmark --once "
                   "--separate --dump-detections --opencv --software-decode --legacy-mosaic --ufld-rear --no-rear-lane --enable-lanes --no-lanes\n";
      return false;
    }
    if (++i >= argc) { std::cerr << "Missing value for " << key << '\n'; return false; }
    const std::string value = argv[i];
    if (key == "--pcie") o.pcie_device = value;
    else if (key == "--scene") o.scene = value;
    else if (key == "--model") o.model = value;
    else if (key == "--snapshot") o.snapshot = value;
    else if (key == "--demo-output") o.demo_output = value;
    else if (key == "--demo-lane" || key == "--demo-direction") {
      if (value!="left" && value!="straight" && value!="right") return false;
      o.demo_direction=value;
    }
    else if (key == "--ufld-model") o.ufld_model = value;
    else if (key == "--detect-every") o.detect_every = std::max(1, std::atoi(value.c_str()));
    else if (key == "--ufld-every") o.ufld_every = std::max(2, std::atoi(value.c_str()));
    else if (key == "--display-fps") {
      // 0 表示不额外限制显示帧率；文件回放仍遵守原视频时间轴。
      char* end = nullptr;
      const double limit = std::strtod(value.c_str(), &end);
      if (end == value.c_str() || *end != '\0' || !std::isfinite(limit) || limit < 0.0) {
        std::cerr << "--display-fps requires a non-negative finite number\n";
        return false;
      }
      o.display_fps_limit = limit;
    }
    else if (key == "--cpu-threads") o.cpu_threads = std::max(1, std::min(4, std::atoi(value.c_str())));
    else if (key == "--npu-workers") o.npu_workers = std::max(1, std::min(2, std::atoi(value.c_str())));
    else if (key == "--max-frames") o.max_frames = std::max(0, std::atoi(value.c_str()));
    else if (key == "--snapshot-frame") o.snapshot_frame = std::max(0, std::atoi(value.c_str()));
    else { std::cerr << "Unknown option: " << key << '\n'; return false; }
  }
  return true;
}

// Replay decoder helpers inherit 0/1 at creation. Restore the caller so
// inference and UI keep their original placement; PCIe never uses this scope.
class ReplayDecodeCpuScope {
 public:
  ReplayDecodeCpuScope() {
    const int error=pthread_getaffinity_np(pthread_self(),sizeof(previous_),&previous_);
    if(error) throw std::runtime_error("Cannot read replay CPU affinity");
    cpu_set_t target; CPU_ZERO(&target); CPU_SET(0,&target); CPU_SET(1,&target);
    if(pthread_setaffinity_np(pthread_self(),sizeof(target),&target))
      throw std::runtime_error("Cannot bind replay decoder to CPU 0/1");
  }
  ~ReplayDecodeCpuScope() { pthread_setaffinity_np(pthread_self(),sizeof(previous_),&previous_); }
 private:
  cpu_set_t previous_;
};

class SyncCapture {
 public:
  ~SyncCapture() { close_pipe(); }

  void release() {
    close_pipe();
    software_.release();
  }

  bool open(const std::string& path, bool hardware) {
    ReplayDecodeCpuScope decode_cpu;
    close_pipe();
    software_.release();
    path_ = path;
    hardware_ = hardware;
    // Replay assets provide verified container metadata; the board's legacy
    // OpenCV probe may fail even when MPP can decode the same MP4 correctly.
    std::ifstream metadata((path + ".info").c_str());
    if (!(metadata >> width_ >> height_ >> fps_ >> frames_)) {
      cv::VideoCapture probe(path);
      if (!probe.isOpened()) return false;
      fps_ = probe.get(cv::CAP_PROP_FPS);
      frames_ = static_cast<int>(probe.get(cv::CAP_PROP_FRAME_COUNT));
      width_ = static_cast<int>(probe.get(cv::CAP_PROP_FRAME_WIDTH));
      height_ = static_cast<int>(probe.get(cv::CAP_PROP_FRAME_HEIGHT));
      probe.release();
    }
    if (!std::isfinite(fps_) || fps_ <= 0.0 || frames_ <= 0) return false;
    if (width_ < 2 || height_ < 2 || width_ > 8192 || height_ > 4320 ||
        (hardware_ && ((width_ & 1) != 0 || (height_ & 1) != 0))) {
      release();
      return false;
    }
    return hardware_ ? start_pipe() : software_.open(path);
  }

  bool read(cv::Mat& frame) {
    ReplayDecodeCpuScope decode_cpu;
    if (!hardware_) return software_.read(frame);
    GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink_), 10 * GST_SECOND);
    if (!sample) return false;
    GstVideoInfo info;
    gst_video_info_init(&info);
    GstBuffer* buffer = gst_sample_get_buffer(sample);
    GstMapInfo map;
    if (!gst_video_info_from_caps(&info, gst_sample_get_caps(sample)) ||
        GST_VIDEO_INFO_FORMAT(&info) != GST_VIDEO_FORMAT_NV12 ||
        GST_VIDEO_INFO_WIDTH(&info) != width_ || GST_VIDEO_INFO_HEIGHT(&info) != height_ ||
        !gst_buffer_map(buffer, &map, GST_MAP_READ)) {
      gst_sample_unref(sample);
      return false;
    }
    // MPP's visible 1080-row image uses an aligned 1088-row Y plane.
    // Read buffer metadata instead of treating caps dimensions as storage size.
    const GstVideoMeta* meta = gst_buffer_get_video_meta(buffer);
    const int y_stride = meta ? meta->stride[0] : GST_VIDEO_INFO_PLANE_STRIDE(&info, 0);
    const int uv_stride = meta ? meta->stride[1] : GST_VIDEO_INFO_PLANE_STRIDE(&info, 1);
    const size_t y_offset = meta ? meta->offset[0] : GST_VIDEO_INFO_PLANE_OFFSET(&info, 0);
    const size_t uv_offset = meta ? meta->offset[1] : GST_VIDEO_INFO_PLANE_OFFSET(&info, 1);
    const bool valid = y_stride >= width_ && uv_stride >= width_ &&
        y_offset + size_t(height_ - 1) * y_stride + width_ <= map.size &&
        uv_offset + size_t(height_ / 2 - 1) * uv_stride + width_ <= map.size;
    if (valid) {
      nv12_.create(height_ + height_ / 2, width_, CV_8UC1);
      for (int row = 0; row < height_; ++row)
        std::memcpy(nv12_.ptr(row), map.data + y_offset + size_t(row) * y_stride, width_);
      for (int row = 0; row < height_ / 2; ++row)
        std::memcpy(nv12_.ptr(height_ + row), map.data + uv_offset + size_t(row) * uv_stride, width_);
      cv::cvtColor(nv12_, frame, cv::COLOR_YUV2BGR_NV12);
    }
    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    if (!valid) return false;
    return true;
  }

  bool grab() {
    ReplayDecodeCpuScope decode_cpu;
    if (!hardware_) return software_.grab();
    GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink_), 10 * GST_SECOND);
    if (!sample) return false;
    gst_sample_unref(sample);
    return true;
  }

  void reset() {
    ReplayDecodeCpuScope decode_cpu;
    if (!hardware_) {
      software_.set(cv::CAP_PROP_POS_FRAMES, 0);
    } else {
      close_pipe();
      start_pipe();
    }
  }

  bool seek(int frame) {
    ReplayDecodeCpuScope decode_cpu;
    if (hardware_) return false;
    return software_.set(cv::CAP_PROP_POS_FRAMES, frame);
  }

  double fps() const { return fps_; }
  int frames() const { return frames_; }

 private:
  static std::string shell_quote(const std::string& value) {
    std::string quoted = "'";
    for (char c : value) quoted += c == '\'' ? "'\\''" : std::string(1, c);
    return quoted + "'";
  }

  bool start_pipe() {
    gst_init(nullptr, nullptr);
    GError* error = nullptr;
    pipeline_ = gst_parse_launch(
        "filesrc name=source ! qtdemux ! h264parse ! mppvideodec ! "
        "video/x-raw,format=NV12 ! appsink name=frames sync=false max-buffers=2 drop=false", &error);
    if (error || !pipeline_) {
      if (error) { std::cerr << "MPP pipeline: " << error->message << '\n'; g_error_free(error); }
      close_pipe();
      return false;
    }
    GstElement* source = gst_bin_get_by_name(GST_BIN(pipeline_), "source");
    g_object_set(source, "location", path_.c_str(), nullptr);
    gst_object_unref(source);
    sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "frames");
    if (!sink_ || gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      close_pipe();
      return false;
    }
    return true;
  }

  void close_pipe() {
    if (pipeline_) gst_element_set_state(pipeline_, GST_STATE_NULL);
    if (sink_) { gst_object_unref(sink_); sink_ = nullptr; }
    if (pipeline_) { gst_object_unref(pipeline_); pipeline_ = nullptr; }
  }

  bool hardware_ = false;
  GstElement* pipeline_ = nullptr;
  GstElement* sink_ = nullptr;
  cv::Mat nv12_;
  cv::VideoCapture software_;
  std::string path_;
  double fps_ = 0.0;
  int frames_ = 0;
  int width_ = 0;
  int height_ = 0;
};

// Rear-corner replay streams are optional and decoded only by the BEV worker.
// Their board copies are aligned to the primary replay's frame timeline offline.
struct AvmReplayWorker {
  adas::AvmRenderer renderer;
  std::array<cv::VideoCapture,3> extra;
  std::array<int,3> next{{0,0,0}};
  AvmReplayWorker(const adas::AvmRenderer& source, const std::string& scene, bool live)
      : renderer(source) {
    if (live) return;
    ReplayDecodeCpuScope decode_cpu;
    const char* names[]={"rear_left.mp4","rear_right.mp4","front_tele.mp4"};
    for (int i=0;i<3;++i) {
      const std::string path=scene+"/"+names[i];
      if (std::ifstream(path.c_str()).good()) extra[i].open(path);
    }
  }
  cv::Mat render(const std::array<cv::Mat,4>& primary, int frame) {
    std::array<cv::Mat,7> images;
    for(int i=0;i<4;++i) images[i]=primary[i];
    for(int i=0;i<3;++i) {
      if(!extra[i].isOpened()) continue;
      ReplayDecodeCpuScope decode_cpu;
      if(frame<next[i]) { extra[i].set(cv::CAP_PROP_POS_FRAMES,frame); next[i]=frame; }
      bool ok=true;
      while(next[i]<frame) { if(!extra[i].grab()) {ok=false;break;} ++next[i]; }
      if(ok && extra[i].read(images[i+4])) ++next[i];
    }
    return renderer.render(images);
  }
};

bool scene_complete(const std::string& scene) {
  std::ifstream synchronized((scene + "/sync_720p.mp4").c_str(), std::ios::binary);
  if (synchronized) return true;
  std::ifstream composite((scene + "/fpga_1080p.mp4").c_str(), std::ios::binary);
  if (composite) return true;
  for (int i = 0; i < 4; ++i) {
    std::ifstream input((scene + "/" + kFiles[i]).c_str(), std::ios::binary);
    if (!input) return false;
  }
  return true;
}

// 输入来源与场景配置分开：PCIe 取帧不依赖 MP4，但使用同一场景的 ROI/标定。
// 优先使用素材目录的配置；仅在文件不存在时按同名场景查找随源码附带的配置。
// 不猜测其他场景，也不把整张画面伪装成已经绑定的主灯组。
std::string scene_asset_path(const std::string& scene, const std::string& filename) {
  const std::string direct = scene + "/" + filename;
  if (std::ifstream(direct).good()) return direct;
  const size_t last = scene.find_last_not_of("/\\");
  if (last == std::string::npos) return direct;
  const size_t slash = scene.find_last_of("/\\", last);
  const std::string name = scene.substr(slash == std::string::npos ? 0 : slash + 1,
                                      last - (slash == std::string::npos ? 0 : slash + 1) + 1);
  const std::string bundled = "config/simulated_test_1_scenes/" + name + "/" + filename;
  return std::ifstream(bundled).good() ? bundled : direct;
}

bool open_scene(const std::string& scene, bool hardware,
                std::array<SyncCapture, 4>& streams,
                double& source_fps, int& source_frames,
                bool& single_composite) {
  if (!scene_complete(scene)) return false;
  const std::string synchronized_path = scene + "/sync_720p.mp4";
  const std::string fpga_path = scene + "/fpga_1080p.mp4";
  std::ifstream synchronized(synchronized_path.c_str(), std::ios::binary);
  const std::string composite_path = synchronized ? synchronized_path : fpga_path;
  std::ifstream composite(composite_path.c_str(), std::ios::binary);
  single_composite = static_cast<bool>(composite);
  if (single_composite) {
    for (int i = 1; i < 4; ++i) streams[i].release();
    if (!streams[0].open(composite_path, hardware)) return false;
    source_fps = streams[0].fps();
    source_frames = streams[0].frames();
    return source_fps >= 1.0 && source_fps <= 60.0 && source_frames > 0;
  }
  double expected_fps = 0.0;
  int expected_frames = -1;
  for (int i = 0; i < 4; ++i) {
    const std::string path = scene + "/" + kFiles[i];
    if (!streams[i].open(path, hardware)) return false;
    const double fps = streams[i].fps();
    const int frames = streams[i].frames();
    if (i == 0) {
      expected_fps = fps;
      expected_frames = frames;
    } else if (std::abs(fps - expected_fps) > 0.01 || frames != expected_frames) {
      return false;
    }
  }
  if (expected_fps < 1.0 || expected_fps > 60.0) return false;
  source_fps = expected_fps;
  source_frames = expected_frames;
  return true;
}

bool read_synchronized_group(std::array<SyncCapture, 4>& streams,
                             bool single_composite,
                             cv::Mat& composite,
                             std::array<cv::Mat, 4>& frames) {
  if (!single_composite) {
    // Publish a group only after all decoders produced the same next index.
    std::array<cv::Mat, 4> pending;
    for (int i = 0; i < 4; ++i) {
      if (!streams[i].read(pending[i])) return false;
    }
    frames = std::move(pending);
    return true;
  }
  if (!streams[0].read(composite) || composite.empty()) return false;
  const int half_w = composite.cols / 2;
  const int half_h = composite.rows / 2;
  if (half_w < 1 || half_h < 1) return false;
  // Keep the native composite for YOLO. Only the display copy is reduced.
  // The old OpenCV generic 3:2 area resize costs ~100 ms for four tiles.
  if (composite.cols == kTileW * 3 && composite.rows == kTileH * 3) {
    cv::Mat preview;
    adas::resize_preview(composite, preview);
    for (int i = 0; i < 4; ++i)
      preview(cv::Rect((i % 2) * kTileW, (i / 2) * kTileH, kTileW, kTileH)).copyTo(frames[i]);
    return true;
  }
  for (int i = 0; i < 4; ++i) {
    const cv::Rect quadrant((i % 2) * half_w, (i / 2) * half_h,
                            half_w, half_h);
    cv::resize(composite(quadrant), frames[i], cv::Size(kTileW, kTileH),
               0.0, 0.0, cv::INTER_AREA);
  }
  return true;
}

struct InferenceResult {
  std::array<cv::Mat,4> motion_reference;
  std::vector<adas::Detection> verified_lights;
  std::array<std::vector<adas::Detection>,4> verified_views;
  int hsv_checked=0, hsv_rejected=0;
  double hsv_ms=0.0;
  double source_seconds = -1.0;
  std::array<std::vector<adas::Detection>, 4> detections;
  double npu_ms = 0.0;
  std::chrono::steady_clock::time_point captured_at{};
};

struct LaneTaskResult {
  int source_frame = -1;
  int camera = 0;
  adas::LaneResult lane;
  adas::LaneMarkingObservation markings;
  int image_width = 0;
  int image_height = 0;
  bool neural = false;
  unsigned generation = 0;  // 丢弃开关切换前的在途结果，防止旧车道线重新出现。
  double elapsed_ms = 0.0;
  std::chrono::steady_clock::time_point captured_at{};
  std::chrono::steady_clock::time_point completed_at{};
  std::chrono::steady_clock::time_point media_at{};
};

adas::LaneResult scale_lane_result(const adas::LaneResult& source,
                                   int source_width, int source_height,
                                   int target_width, int target_height) {
  if (!source.valid || source_width <= 0 || source_height <= 0 ||
      (source_width == target_width && source_height == target_height)) {
    return source;
  }
  adas::LaneResult result = source;
  const float sx = target_width / static_cast<float>(source_width);
  const float sy = target_height / static_cast<float>(source_height);
  const auto scale_points = [sx, sy](std::vector<cv::Point>& points) {
    for (cv::Point& point : points) {
      point.x = cvRound(point.x * sx);
      point.y = cvRound(point.y * sy);
    }
  };
  scale_points(result.left);
  scale_points(result.right);
  scale_points(result.polygon);
  for (auto& boundary : result.boundaries) scale_points(boundary);
  for (auto& polygon : result.lane_polygons) scale_points(polygon);
  result.binary.release();
  result.bird_eye.release();
  result.curve_fit.release();
  return result;
}

enum class CalibrationTarget { NONE, FRONT, REAR };

std::vector<cv::Point2f> order_roi(std::vector<cv::Point2f> points) {
  std::sort(points.begin(), points.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
    return a.y == b.y ? a.x < b.x : a.y < b.y;
  });
  if (points.size() != 4) return {};
  cv::Point2f tl = points[0].x < points[1].x ? points[0] : points[1];
  cv::Point2f tr = points[0].x < points[1].x ? points[1] : points[0];
  cv::Point2f bl = points[2].x < points[3].x ? points[2] : points[3];
  cv::Point2f br = points[2].x < points[3].x ? points[3] : points[2];
  // A level top/bottom pair makes the bird-eye transform stable even when the
  // user clicks the same painted line a few pixels higher on one side.
  const float top_y = 0.5f * (tl.y + tr.y);
  const float bottom_y = 0.5f * (bl.y + br.y);
  tl.y = tr.y = top_y;
  bl.y = br.y = bottom_y;
  return {tl, tr, br, bl};
}

bool valid_roi(const std::vector<cv::Point2f>& points) {
  if (points.size() != 4) return false;
  const float top_width = points[1].x - points[0].x;
  const float bottom_width = points[2].x - points[3].x;
  const float top_y = 0.5f * (points[0].y + points[1].y);
  const float bottom_y = 0.5f * (points[2].y + points[3].y);
  return top_width > 0.03f && bottom_width > 0.15f && top_width < bottom_width &&
         bottom_y - top_y > 0.12f;
}

bool save_roi(const std::string& path, const std::vector<cv::Point2f>& points) {
  std::ofstream output(path.c_str());
  if (!output || points.size() != 4) return false;
  for (const auto& point : points) output << point.x << ' ' << point.y << '\n';
  return true;
}

bool load_roi(const std::string& path, adas::LaneDetector& detector,
              std::vector<cv::Point2f>* loaded = nullptr) {
  std::ifstream input(path.c_str());
  std::vector<cv::Point2f> points;
  float x = 0.0f, y = 0.0f;
  while (input >> x >> y) points.emplace_back(x, y);
  points = order_roi(points);
  if (!valid_roi(points)) return false;
  detector.set_roi(points[0], points[1], points[2], points[3]);
  if (loaded) *loaded = points;
  return true;
}

void draw_lane_geometry(cv::Mat& frame, const adas::LaneResult& lane) {
  if (!lane.valid) return;
  cv::Mat layer = frame.clone();
  cv::fillPoly(layer, std::vector<std::vector<cv::Point>>(1, lane.polygon),
               lane.departure ? cv::Scalar(0, 70, 255) : cv::Scalar(20, 190, 70));
  cv::addWeighted(layer, 0.24, frame, 0.76, 0, frame);
  const cv::Scalar normal(230, 210, 50), danger(35, 45, 245);
  cv::polylines(frame, lane.left, false,
                lane.departure_side == adas::LaneDepartureSide::LEFT ? danger : normal,
                lane.departure_side == adas::LaneDepartureSide::LEFT ? 8 : 5, cv::LINE_AA);
  cv::polylines(frame, lane.right, false,
                lane.departure_side == adas::LaneDepartureSide::RIGHT ? danger : normal,
                lane.departure_side == adas::LaneDepartureSide::RIGHT ? 8 : 5, cv::LINE_AA);
}

cv::Point2f sample_lane_line(const std::vector<cv::Point>& line, float position) {
  if (line.empty()) return cv::Point2f();
  if (line.size() == 1) return cv::Point2f(line.front());
  const float scaled = std::max(0.0f, std::min(1.0f, position)) * (line.size() - 1);
  const size_t first = static_cast<size_t>(scaled);
  const size_t second = std::min(first + 1, line.size() - 1);
  const float mix = scaled - first;
  return cv::Point2f(line[first]) * (1.0f - mix) + cv::Point2f(line[second]) * mix;
}

adas::LaneResult interpolate_lane(const adas::LaneResult& previous,
                                  const adas::LaneResult& target,
                                  float target_weight) {
  if (!target.valid) return target;
  if (!previous.valid || previous.left.empty() || previous.right.empty()) return target;
  adas::LaneResult result = target;
  result.left.clear();
  result.right.clear();
  const int samples = 28;
  for (int i = 0; i < samples; ++i) {
    const float position = i / static_cast<float>(samples - 1);
    const cv::Point2f old_left = sample_lane_line(previous.left, position);
    const cv::Point2f new_left = sample_lane_line(target.left, position);
    const cv::Point2f old_right = sample_lane_line(previous.right, position);
    const cv::Point2f new_right = sample_lane_line(target.right, position);
    result.left.emplace_back(cvRound(old_left.x * (1.0f - target_weight) + new_left.x * target_weight),
                             cvRound(old_left.y * (1.0f - target_weight) + new_left.y * target_weight));
    result.right.emplace_back(cvRound(old_right.x * (1.0f - target_weight) + new_right.x * target_weight),
                              cvRound(old_right.y * (1.0f - target_weight) + new_right.y * target_weight));
  }
  result.polygon = result.left;
  result.polygon.insert(result.polygon.end(), result.right.begin(), result.right.end());
  result.offset_ratio = previous.offset_ratio * (1.0f - target_weight) +
                        target.offset_ratio * target_weight;
  result.departure = target.departure;  // Warning state belongs to LaneDepartureMonitor.
  return result;
}

adas::LaneResult stabilize_lane(const adas::LaneResult& measured,
                                const adas::LaneResult& previous,
                                int& missed_updates, bool vehicle_occlusion) {
  bool geometry_plausible = measured.valid;
  if (geometry_plausible && previous.valid && !measured.left.empty() &&
      !measured.right.empty() && !previous.left.empty() && !previous.right.empty()) {
    const float old_bottom_width = previous.right.back().x - previous.left.front().x;
    const float new_bottom_width = measured.right.back().x - measured.left.front().x;
    const float old_top_width = previous.right.front().x - previous.left.back().x;
    const float new_top_width = measured.right.front().x - measured.left.back().x;
    const float old_center = 0.5f * (previous.right.back().x + previous.left.front().x);
    const float new_center = 0.5f * (measured.right.back().x + measured.left.front().x);
    // A fixed camera cannot produce a sudden lane-width collapse. This is
    // normally a close vehicle edge entering the ROI, especially in REAR.
    geometry_plausible = old_bottom_width > 1.0f && old_top_width > 1.0f &&
        new_bottom_width > old_bottom_width * 0.72f &&
        new_bottom_width < old_bottom_width * 1.35f &&
        new_top_width > old_top_width * 0.60f &&
        new_top_width < old_top_width * 1.55f &&
        std::abs(new_center - old_center) < old_bottom_width * 0.22f;
  }
  if (!geometry_plausible) {
    ++missed_updates;
    // Hold short occlusions, but accept a persistent new geometry after a few
    // updates so a real lane change cannot remain stuck on the old lane.
    const int hold_updates = vehicle_occlusion ? 5 : 2;
    if (previous.valid && missed_updates <= hold_updates) {
      adas::LaneResult held = previous;
      held.partial = true;
      return held;
    }
    if (measured.valid) {
      missed_updates = 0;
      return measured;
    }
    return measured;
  }
  missed_updates = 0;
  if (!previous.valid || previous.left.empty() || previous.right.empty()) return measured;

  // UFLD already produces fitted curves. Keep only light suppression here;
  // heavier filtering makes the 90-120 ms NPU result look another update late.
  return interpolate_lane(previous, measured, 0.88f);
}

bool lane_blocked_by_vehicle(const std::vector<adas::Detection>& detections,
                             int width, int height) {
  for (const auto& detection : detections) {
    if (detection.class_id < 0 || detection.class_id > 6) continue;
    const float center_x = detection.box.x + detection.box.width * 0.5f;
    const float bottom = detection.box.y + detection.box.height;
    const float area = detection.box.area() /
        std::max(1.0f, static_cast<float>(width * height));
    if (center_x > width * 0.16f && center_x < width * 0.84f &&
        bottom > height * 0.48f && area > 0.010f) return true;
  }
  return false;
}

bool near_lane_change_regulation(const std::vector<adas::Detection>& detections) {
  for (const auto& detection : detections) {
    if (detection.score < 0.4f) continue;
    // Traffic lights, crosswalk and guide-arrow detections are the only
    // intersection proximity evidence available before map/CAN integration.
    if ((detection.class_id >= 7 && detection.class_id <= 14) ||
        detection.class_id == 16 || detection.class_id == 17) return true;
  }
  return false;
}

void draw_calibration(cv::Mat& frame, const std::vector<cv::Point2f>& points,
                      const std::string& camera) {
  cv::rectangle(frame, cv::Rect(0, 28, frame.cols, 34), cv::Scalar(20, 90, 170), cv::FILLED);
  adas::ui::draw_text(frame, "标定" + camera + "：请在车道线上点击4个点",
                      cv::Point(12, 52), 17, cv::Scalar(255, 255, 255));
  for (size_t i = 0; i < points.size(); ++i) {
    const cv::Point point(static_cast<int>(points[i].x * frame.cols),
                          static_cast<int>(points[i].y * frame.rows));
    cv::circle(frame, point, 8, cv::Scalar(0, 255, 255), cv::FILLED, cv::LINE_AA);
    cv::putText(frame, std::to_string(i + 1), point + cv::Point(10, -8),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
  }
}

void caption(cv::Mat& canvas, const std::string& text, int x, int y,
             const cv::Scalar& color = cv::Scalar(225, 235, 240), double scale = 0.55) {
  const bool utf8 = std::any_of(text.begin(), text.end(), [](char c) {
    return static_cast<unsigned char>(c) >= 0x80;
  });
  if (utf8)
    adas::ui::draw_text(canvas, text, cv::Point(x, y),
                        std::max(12, cvRound(scale * 28.0)), color);
  else
    cv::putText(canvas, text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX,
                scale, color, 1, cv::LINE_AA);
}

void panel(cv::Mat& canvas, int x, int y, int w, int h) {
  cv::rectangle(canvas, cv::Rect(x, y, w, h), cv::Scalar(32, 22, 11), cv::FILLED);
  cv::rectangle(canvas, cv::Rect(x, y, w, h), cv::Scalar(115, 76, 28), 1);
}

void draw_lane_detection_toggle(cv::Mat& canvas, const LaneToggleState& state, bool enabled) {
  const auto& button = kLaneDetectionToggle;
  const cv::Scalar color = !state.error.empty() ? cv::Scalar(80,80,245) :
      enabled ? cv::Scalar(130,230,90) : state.requested ? cv::Scalar(70,210,245) :
                                                         cv::Scalar(185,185,185);
  panel(canvas,button.x,button.y,button.width,button.height);
  cv::rectangle(canvas,button,color,1,cv::LINE_AA);
  caption(canvas,state.label(enabled),button.x+10,button.y+24,color,.50);
}

cv::Scalar target_risk_color(const adas::RiskResult& risk) {
  return adas::risk_overlay_color(risk);
}

void draw_boxes(cv::Mat& frame, const std::vector<adas::Detection>& detections,
                const adas::RiskResult* risk = nullptr, int attention_track = -1,
                bool danger_attention = false) {
  adas::ui::LampAnnotationLayout lamp_layout(frame.size(),detections);
  for (const auto& d : detections) {
    // UI preference: hide unclassified lamp housings; retain all observations for logic.
    if (d.class_id == 21) continue;
    const bool risk_target = risk && risk->target && risk->reliable && risk->track_id >= 0 &&
                             d.track_id == risk->track_id;
    const bool attention = attention_track >= 0 && d.track_id == attention_track;
    const cv::Scalar color = risk_target ? target_risk_color(*risk) :
        attention ? (danger_attention ? cv::Scalar(35, 45, 245)
                                      : cv::Scalar(50, 225, 245)) : adas::normal_object_color(d.class_id);
    if (risk_target) adas::draw_risk_halo(frame, d.box, *risk);
    cv::rectangle(frame, d.box, color, risk_target || attention ? 3 : 1);
    const bool mo_light=d.class_id==21 || d.name.find(" MO ")!=std::string::npos;
    if (!attention && !mo_light) continue;  // Target risk cards carry rear labels.
    if(mo_light) {adas::ui::draw_compact_lamp_caption(frame,d,lamp_layout);continue;}
    std::ostringstream label;
    label << d.name;
    if (d.track_id >= 0) label << " #" << d.track_id;
    if (d.name.find(" MO ")==std::string::npos)
      label << ' ' << int(std::round(d.score*100)) << '%';
    const int x = std::max(2, static_cast<int>(d.box.x));
    const int y = std::max(23, static_cast<int>(d.box.y));
    const cv::Size size = adas::ui::measure_text(label.str(),18);
    cv::rectangle(frame, cv::Rect(x, y - 22, std::min(size.width + 7, frame.cols - x), 22),
                  cv::Scalar(15, 20, 25), cv::FILLED);
    const cv::Scalar lamp_color=d.class_id==21 ? cv::Scalar(40,210,245) :
        d.class_id<=10 ? cv::Scalar(45,65,245) : cv::Scalar(80,235,95);
    caption(frame,label.str(),x+3,y-4,mo_light?lamp_color:color,.64);
  }
}

void draw_rear_risk_overlay(cv::Mat& frame, const adas::RiskResult& risk,
                             const std::vector<adas::Detection>& detections) {
  // No rear lane geometry is available with --no-rear-lane. A fixed
  // trapezoid would falsely imply a road-aligned collision corridor.
  // Show object-level risk only until calibrated geometry is available.
  adas::draw_target_card(frame, detections, risk, "RCW");
  if (!risk.target || !risk.reliable || !std::isfinite(risk.relative_speed_kmh) ||
      risk.relative_speed_kmh < 3.0f) return;
  for (const auto& d : detections) {
    if (risk.track_id < 0 || d.track_id != risk.track_id) continue;
    const int x = std::max(15, std::min(frame.cols - 15, cvRound(d.box.x + d.box.width + 14)));
    const int y = std::max(65, std::min(frame.rows - 45, cvRound(d.box.y + d.box.height)));
    const int length = cvRound(std::min(60.0f, 20.0f + risk.relative_speed_kmh));
    // Down-screen is a symbolic approach toward ego, not a world-space vector.
    cv::arrowedLine(frame, {x, y - length}, {x, y}, target_risk_color(risk), 3, cv::LINE_AA, 0, .3);
    caption(frame, "CLOSING", std::max(5, x - 65), y + 18, target_risk_color(risk), .4);
    break;
  }
}

void draw_blind_spot_overlay(cv::Mat& frame, const adas::BlindSpotResult& blind,
                             bool turn_intent, const char* side, bool fresh,
                             const std::vector<adas::Detection>& detections) {
  const bool occupied = fresh && blind.occupied;
  const bool lca = occupied && turn_intent;
  const cv::Scalar color = lca ? cv::Scalar(35, 45, 245) :
      occupied ? cv::Scalar(50, 225, 245) : cv::Scalar(145, 160, 170);
  const std::string text = std::string(side) + (!fresh ? " BSD UNAVAILABLE" :
      lca ? " LCA BLOCKED" : occupied ? " BSD TARGET" : " BSD CLEAR");
  int baseline = 0;
  const cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX,
                                              0.53, 1, &baseline);
  const cv::Rect label_area(8, 6,
                            std::min(frame.cols - 16, label_size.width + 22), 31);
  cv::Mat label_roi = frame(label_area);
  cv::Mat label_layer = label_roi.clone();
  cv::rectangle(frame, label_area, cv::Scalar(15, 20, 27), cv::FILLED);
  cv::addWeighted(label_roi, .82, label_layer, .18, 0, label_roi);
  if (occupied) cv::rectangle(frame, label_area, color, 1, cv::LINE_AA);
  if (!occupied) {
    caption(frame, text, 18, 28, color, 0.53);
    return;
  }
  for (const auto& d : detections) {
    if (d.track_id != blind.track_id) continue;
    // This cue follows the detected object in image space. The side cameras
    // are not calibrated for a road-aligned polygon or metric trajectory.
    adas::draw_side_risk_cue(frame, d.box, lca);
    break;
  }
  caption(frame, text, 18, 28, color, 0.53);
}

void warning_banner(cv::Mat& frame, const std::string& text) {
  const cv::Rect area(8, frame.rows - 38, std::min(frame.cols - 16, 330), 30);
  cv::rectangle(frame, area, cv::Scalar(20, 35, 190), cv::FILLED);
  adas::ui::draw_text(frame, text, cv::Point(area.x + 8, area.y + 22),
                      16, cv::Scalar(255, 255, 255));
}

adas::LaneResult center_lane_display(const adas::LaneResult& source) {
  adas::LaneResult result=source;
  for(int slot:{0,3}) {result.boundaries[slot].clear();result.boundary_coverage[slot]=0;}
  result.lane_polygons[0].clear();result.lane_polygons[2].clear();
  // LDW is disabled in the demo. Keep measured coordinates and paint styles,
  // but omit departure/approach colouring in this drawing-only copy.
  result.departure=false;
  result.departure_side=adas::LaneDepartureSide::NONE;
  result.offset_ratio=0.f;
  return result;
}

void suppress_lane_departure_alarm(adas::LaneResult& lane) {
  lane.departure=false;
  lane.departure_side=adas::LaneDepartureSide::NONE;
}

adas::LaneSemantic test2_rule_evidence(const adas::LaneSemantic& observed,
                                      const adas::LaneResult& geometry,
                                      std::chrono::steady_clock::time_point captured,
                                      std::chrono::steady_clock::time_point now) {
  // 按键规则演示约每 240ms 更新，不能套用仅 300ms 的动态 LDW 截止时间。
  // 只用最近观测，不把显示保持/未知补成虚线；灯态与 LDW 仍走原时限。
  if(!observed.lane_valid || captured==std::chrono::steady_clock::time_point() || now<captured ||
      now-captured>=std::chrono::milliseconds(650) || !geometry.valid || geometry.identity_uncertain)
    return adas::LaneSemantic();
  return observed;
}

adas::ManeuverDirection test2_motion_direction(const adas::LaneChangeResult& change) {
  return change.permitted && (change.direction==adas::ManeuverDirection::LEFT ||
      change.direction==adas::ManeuverDirection::RIGHT)?change.direction:adas::ManeuverDirection::STRAIGHT;
}

const char* test2_action_caption(const adas::LaneChangeResult& change) {
  if(change.solid_alarm)return "实线变道报警";
  const auto motion=test2_motion_direction(change);
  return motion==adas::ManeuverDirection::LEFT?"模拟左变道":
      motion==adas::ManeuverDirection::RIGHT?"模拟右变道":"保持直行";
}

void draw_maneuver_hud(cv::Mat& frame, const adas::LaneChangeResult& change,
                        const adas::LaneSemantic& lane) {
  const auto motion=test2_motion_direction(change);
  const cv::Scalar color = change.solid_alarm ? cv::Scalar(35,45,245) :
      change.state==adas::LaneChangeState::CHECK_RULE && !lane.lane_valid ?
      cv::Scalar(35,225,245):cv::Scalar(90,215,100);
  const int y = frame.rows - 84;
  const cv::Rect area(10, y, 220, 58);
  cv::rectangle(frame, area, cv::Scalar(20, 25, 28), cv::FILLED);
  cv::rectangle(frame, area, color, 1);
  const bool straight=motion==adas::ManeuverDirection::STRAIGHT;
  const bool left=motion==adas::ManeuverDirection::LEFT;
  cv::arrowedLine(frame, straight?cv::Point(25,y+23):cv::Point(left?34:17,y+16),
      straight?cv::Point(25,y+7):cv::Point(left?17:34,y+16),color,2,cv::LINE_AA,0,.4);
  caption(frame,std::string("测试二 / ")+test2_action_caption(change),43,y+18,color,.43);
  caption(frame,std::string("左 ")+adas::lane_marking_caption(lane.left)+
      " / 右 "+adas::lane_marking_caption(lane.right),18,y+35,cv::Scalar(215,215,215),.38);
  const auto requested=change.direction==adas::ManeuverDirection::LEFT?lane.left:lane.right;
  const char* detail=change.solid_alarm?"强制实线变道，报警":change.permitted?"虚线允许变道":
      change.state==adas::LaneChangeState::IDLE?"按键模拟左右变道":
      lane.lane_valid && requested==adas::LaneMarking::SOLID?"实线保持直行":"线型待确认，保持直行";
  caption(frame,detail,18,y+51,color,.38);
}

int risk_level(const adas::RiskResult& risk) {
  if (!risk.target || !risk.reliable) return 0;
  if (std::isfinite(risk.ttc_s) && risk.ttc_s > 0.0f) {
    if (risk.ttc_s < 1.5f) return 3;
    if (risk.ttc_s < 3.0f) return 2;
    if (risk.ttc_s < 4.0f) return 1;
  }
  return risk.warning ? 2 : 0;
}

std::array<int, 4> view_alert_levels(const adas::RiskResult& front,
    const adas::RiskResult& rear, const adas::LaneResult& lane,
    const adas::LaneSemantic& semantic, const adas::BlindSpotResult& left,
    const adas::BlindSpotResult& right, bool side_fresh,
    const adas::VehicleState& vehicle) {
  std::array<int, 4> level{{risk_level(front), risk_level(rear), 0, 0}};
  // Automatic departure/TLC warnings are disabled; object risk stays active.
  (void)lane; (void)semantic;
  if (side_fresh && left.occupied)
    level[2] = vehicle.turn_valid && vehicle.turn == adas::TurnSignal::LEFT ? 3 : 1;
  if (side_fresh && right.occupied)
    level[3] = vehicle.turn_valid && vehicle.turn == adas::TurnSignal::RIGHT ? 3 : 1;
  return level;
}

cv::Scalar alert_color(int level) {
  return level >= 3 ? cv::Scalar(35, 45, 245) :
         level == 2 ? cv::Scalar(30, 145, 245) :
         level == 1 ? cv::Scalar(35, 205, 245) : cv::Scalar(82, 69, 47);
}

void draw_view_alerts(cv::Mat& canvas, const std::array<int, 4>& levels) {
  const double seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  const double pulse = .78 + .22 * std::sin(seconds * 2.0 * 3.141592653589793);
  for (int i = 0; i < 4; ++i) {
    const cv::Rect& r = kViewRects[i];
    const int level = levels[i];
    const cv::Scalar base = alert_color(level);
    const cv::Scalar color = level >= 2 ? base * pulse : base;
    // The canvas is reused across frames. Erase the full former stroke,
    // including its inner antialiased pixels, before drawing a new severity.
    cv::rectangle(canvas, cv::Rect(r.x - 3, r.y - 3, r.width + 6, r.height + 6),
                  cv::Scalar(24, 15, 7), 10);
    cv::rectangle(canvas, cv::Rect(r.x - 3, r.y - 3, r.width + 6, r.height + 6),
                  color, level >= 2 ? 4 : level == 1 ? 3 : 1, cv::LINE_AA);
  }
}

void draw_global_warning(cv::Mat& canvas, const adas::WarningSummary& warning,
                         const adas::RiskResult& front, const adas::RiskResult& rear,
                         const std::array<int, 4>& levels,
                         const cv::Mat& normal_header) {
  // Restore the normal title after an alert clears; the canvas is persistent.
  normal_header.copyTo(canvas(cv::Rect(430, 0, 980, 76)));
  const int maximum = *std::max_element(levels.begin(), levels.end());
  if (maximum == 0 && warning.priority < 60) return;
  // A confirmed caution level must have a readable yellow message even when
  // it is based on distance and TTC is unavailable. Higher-priority traffic
  // and Test Four messages keep their existing ownership of the header.
  const bool front_red=warning.priority<=100 &&
      ((warning.primary==adas::WarningCode::FCW && front.warning) ||
       (warning.priority<60 && risk_level(front)>=2));
  const bool front_yellow=levels[0]==1 && warning.priority<60 && !front_red;
  const int level = std::max(maximum, warning.primary==adas::WarningCode::LANE_CHANGE &&
      warning.priority>=95?3:warning.priority>=60?1:0);
  const cv::Scalar color = alert_color(front_red?3:front_yellow?1:level);
  cv::rectangle(canvas, cv::Rect(430, 0, 980, level >= 3 ? 8 : 5),
                color, cv::FILLED);
  if (!front_yellow && warning.priority < 60 && risk_level(front) < 2 && risk_level(rear) < 2) return;
  const cv::Rect area(430, 8, 980, 59);
  cv::rectangle(canvas, area, cv::Scalar(20, 30, 38), cv::FILLED);
  std::ostringstream text;
  if(front_red) {
    text << "红色碰撞预警  |  前方碰撞危险";
  } else if(front_yellow) {
    text << "黄色碰撞预警  |  注意车距";
  } else if ((warning.primary == adas::WarningCode::RCW && rear.warning) ||
             (warning.priority < 60 && risk_level(rear) >= 2)) {
    text << "RCW";
    if (rear.ttc_s > 0.0f && std::isfinite(rear.ttc_s))
      text << "  |  TTC ~" << std::fixed << std::setprecision(1) << rear.ttc_s << "s";
  } else text << warning.text;
  const int text_x=front_red || front_yellow ?
      430+std::max(20,(980-adas::ui::measure_text(text.str(),27).width)/2):478;
  caption(canvas, text.str(), text_x, 48, color, .98);
}

const cv::Rect kProgressBar(24, 1050, 1320, 10);

void draw_progress_bar(cv::Mat& canvas, int frame, int total, bool locked) {
  cv::rectangle(canvas, kProgressBar, cv::Scalar(55, 65, 72), cv::FILLED);
  const float ratio = total > 1 ? std::max(0.0f, std::min(1.0f,
      frame / static_cast<float>(total - 1))) : 0.0f;
  const int filled = static_cast<int>(ratio * kProgressBar.width);
  if (filled > 0)
    cv::rectangle(canvas, cv::Rect(kProgressBar.x, kProgressBar.y, filled,
                  kProgressBar.height), locked ? cv::Scalar(90, 120, 140)
                                               : cv::Scalar(65, 205, 245), cv::FILLED);
  const int knob_x = kProgressBar.x + filled;
  cv::circle(canvas, cv::Point(knob_x, kProgressBar.y + kProgressBar.height / 2),
             8, locked ? cv::Scalar(150, 170, 180) : cv::Scalar(90, 235, 255), cv::FILLED);
  std::ostringstream label;
  const double seconds = frame / 30.0;
  const double duration = total / 30.0;
  label << std::fixed << std::setprecision(1) << seconds << " / " << duration << " s"
        << (locked ? "  标定暂停" : "  点击 / 拖动");
  caption(canvas, label.str(), 1030, 1038, cv::Scalar(175, 205, 215), 0.43);
}

cv::Mat make_mosaic(const std::array<cv::Mat, 4>& frames) {
  // The RKNN model has a fixed 1x3x640x640 input. Each camera gets one
  // 320x320 quadrant, with 16:9 picture content centered in 320x180.
  cv::Mat mosaic(640, 640, CV_8UC3, cv::Scalar(114, 114, 114));
  for (int i = 0; i < 4; ++i) {
    cv::Mat small;
    cv::resize(frames[i], small, cv::Size(320, 180), 0, 0, cv::INTER_AREA);
    const int x = (i % 2) * 320;
    const int y = (i / 2) * 320 + 70;
    small.copyTo(mosaic(cv::Rect(x, y, 320, 180)));
  }
  return mosaic;
}

cv::Mat make_fpga_composite(const std::array<cv::Mat, 4>& frames) {
  // Match the production FPGA interface: a single 1920x1080 frame containing
  // four 960x540 camera quadrants. The fixed 640x640 detector letterboxes this
  // whole frame, so each camera occupies 320x180 pixels in the model tensor.
  cv::Mat composite(kFpgaCompositeH, kFpgaCompositeW, CV_8UC3);
  for (int i = 0; i < 4; ++i) {
    const cv::Rect destination((i % 2) * kFpgaViewW,
                               (i / 2) * kFpgaViewH,
                               kFpgaViewW, kFpgaViewH);
    cv::resize(frames[i], composite(destination), destination.size(),
               0.0, 0.0, cv::INTER_LINEAR);
  }
  return composite;
}

std::array<std::vector<adas::Detection>, 4> split_mosaic(
    const std::vector<adas::Detection>& combined) {
  std::array<std::vector<adas::Detection>, 4> result;
  for (const auto& d : combined) {
    const float cx = d.box.x + d.box.width * 0.5f;
    const float cy = d.box.y + d.box.height * 0.5f;
    const int col = static_cast<int>(cx / 320.0f);
    const int row = static_cast<int>(cy / 320.0f);
    if (col < 0 || col > 1 || row < 0 || row > 1) continue;
    const float x0 = col * 320.0f;
    const float y0 = row * 320.0f + 70.0f;
    if (cy < y0 || cy >= y0 + 180.0f) continue;
    const float x1 = std::max(0.0f, d.box.x - x0);
    const float y1 = std::max(0.0f, d.box.y - y0);
    const float x2 = std::min(320.0f, d.box.x + d.box.width - x0);
    const float y2 = std::min(180.0f, d.box.y + d.box.height - y0);
    const float area = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
    if (area < d.box.area() * 0.5f || area < 3.0f) continue;
    adas::Detection mapped = d;
    mapped.box = cv::Rect2f(x1 * 2.0f, y1 * 2.0f,
                            (x2 - x1) * 2.0f, (y2 - y1) * 2.0f);
    result[row * 2 + col].push_back(mapped);
  }
  return result;
}

std::array<std::vector<adas::Detection>, 4> split_fpga_composite(
    const std::vector<adas::Detection>& combined) {
  std::array<std::vector<adas::Detection>, 4> result;
  const float scale_x = static_cast<float>(kTileW) / kFpgaViewW;
  const float scale_y = static_cast<float>(kTileH) / kFpgaViewH;
  for (const auto& detection : combined) {
    const float center_x = detection.box.x + detection.box.width * 0.5f;
    const float center_y = detection.box.y + detection.box.height * 0.5f;
    const int column = static_cast<int>(center_x / kFpgaViewW);
    const int row = static_cast<int>(center_y / kFpgaViewH);
    if (column < 0 || column > 1 || row < 0 || row > 1) continue;

    const float origin_x = column * static_cast<float>(kFpgaViewW);
    const float origin_y = row * static_cast<float>(kFpgaViewH);
    const float x1 = std::max(0.0f, detection.box.x - origin_x);
    const float y1 = std::max(0.0f, detection.box.y - origin_y);
    const float x2 = std::min(static_cast<float>(kFpgaViewW),
                              detection.box.x + detection.box.width - origin_x);
    const float y2 = std::min(static_cast<float>(kFpgaViewH),
                              detection.box.y + detection.box.height - origin_y);
    const float clipped_area = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
    if (clipped_area < detection.box.area() * 0.5f || clipped_area < 9.0f) continue;

    adas::Detection mapped = detection;
    mapped.box = cv::Rect2f(x1 * scale_x, y1 * scale_y,
                            (x2 - x1) * scale_x, (y2 - y1) * scale_y);
    result[row * 2 + column].push_back(mapped);
  }
  return result;
}

void draw_tile(cv::Mat& canvas, const cv::Mat& frame, int index) {
  cv::Mat tile;
  cv::resize(frame, tile, kViewRects[index].size(), 0, 0, cv::INTER_LINEAR);
  tile.copyTo(canvas(kViewRects[index]));
}

void draw_camera_cards(cv::Mat& canvas) {
  const char* titles[] = {"前视   Front", "后视   Rear", "左视   Left", "右视   Right"};
  for (int i = 0; i < 4; ++i) {
    const auto& r = kViewRects[i];
    panel(canvas, r.x - 1, r.y - 38, r.width + 2, r.height + 40);
    caption(canvas, titles[i], r.x + 18, r.y - 12, cv::Scalar(240, 240, 235), 0.70);
    caption(canvas, "RoadFusion", r.x + r.width - 135, r.y - 12,
            cv::Scalar(165, 135, 100), 0.46);
  }
}

void draw_top_car(cv::Mat& canvas, const cv::Point& center, int width, int height,
                  const cv::Scalar& body, bool horizontal, bool ego) {
  const cv::Size size = horizontal ? cv::Size(height, width) : cv::Size(width, height);
  const cv::Rect box(center.x - size.width / 2, center.y - size.height / 2,
                     size.width, size.height);
  cv::rectangle(canvas, box, body, ego ? 2 : cv::FILLED);
  if (!ego) cv::rectangle(canvas, box, cv::Scalar(215, 225, 232), 1);
  const cv::Rect glass = horizontal
      ? cv::Rect(center.x - size.width / 4, center.y - size.height / 2 + 2,
                 size.width / 2, std::max(2, size.height - 4))
      : cv::Rect(center.x - size.width / 2 + 2, center.y - size.height / 4,
                 std::max(2, size.width - 4), size.height / 3);
  cv::rectangle(canvas, glass, cv::Scalar(45, 65, 78), cv::FILLED);
  if (ego) {
    cv::circle(canvas, cv::Point(box.x + 3, box.y + box.height - 3), 2,
               cv::Scalar(45, 60, 245), cv::FILLED);
    cv::circle(canvas, cv::Point(box.x + box.width - 3, box.y + box.height - 3), 2,
               cv::Scalar(45, 60, 245), cv::FILLED);
  }
}

void draw_surround_map(cv::Mat& canvas, const cv::Rect& area,
                       const adas::RiskResult& front_risk,
                       const adas::RiskResult& rear_risk,
                       const adas::BlindSpotResult& left_blind,
                       const adas::BlindSpotResult& right_blind,
                       const adas::VehicleState& vehicle, bool side_fresh) {
  // This is a directional risk indicator, not a metric map. No camera pixel
  // position is presented as a physical distance or cross-camera track.
  cv::rectangle(canvas, area, cv::Scalar(38, 29, 19), cv::FILLED);
  cv::rectangle(canvas, area, cv::Scalar(70, 78, 84), 1);
  const cv::Point ego(area.x + area.width / 2, area.y + 82);
  const cv::Scalar idle(92, 70, 48);
  const int front_level = risk_level(front_risk);
  const int rear_level = risk_level(rear_risk);
  const int left_level = side_fresh && left_blind.occupied
      ? (vehicle.turn_valid && vehicle.turn == adas::TurnSignal::LEFT ? 3 : 1) : 0;
  const int right_level = side_fresh && right_blind.occupied
      ? (vehicle.turn_valid && vehicle.turn == adas::TurnSignal::RIGHT ? 3 : 1) : 0;
  cv::circle(canvas, ego, 65, cv::Scalar(53, 68, 79), 1, cv::LINE_AA);
  cv::circle(canvas, ego, 39, cv::Scalar(53, 68, 79), 1, cv::LINE_AA);
  const auto sector = [&](int level, int begin, int end) {
    const cv::Scalar color = level ? alert_color(level) : idle;
    cv::ellipse(canvas, ego, cv::Size(59, 59), 0, begin, end,
                color, level ? 11 : 7, cv::LINE_AA);
  };
  sector(front_level, 225, 315);
  sector(rear_level, 45, 135);
  sector(left_level, 135, 225);
  sector(right_level, -45, 45);
  draw_top_car(canvas, ego, 25, 42, cv::Scalar(208, 220, 232), false, true);
  const auto risk_label = [&](const char* direction, const adas::RiskResult& risk,
                              int level, int x, int y) {
    caption(canvas, direction, x, y, level ? alert_color(level) : cv::Scalar(180, 165, 140), .52);
    if (!level) return;
    std::ostringstream line;
    line << "TTC ";
    if (std::isfinite(risk.ttc_s) && risk.ttc_s > 0.0f)
      line << "~" << std::fixed << std::setprecision(1) << risk.ttc_s << "s";
    else line << "--";
    caption(canvas, line.str(), x, y + 18, alert_color(level), .43);
  };
  risk_label("FRONT", front_risk, front_level, area.x + 17, area.y + 25);
  risk_label("REAR", rear_risk, rear_level, area.x + area.width - 105,
             area.y + 25);
  caption(canvas, "LEFT", area.x + 17, area.y + 94,
          left_level ? alert_color(left_level) : idle, .52);
  caption(canvas, side_fresh ? (left_level ? "BSD / LCA" : "CLEAR") : "STALE",
          area.x + 17, area.y + 113,
          left_level ? alert_color(left_level) : idle, .42);
  caption(canvas, "RIGHT", area.x + area.width - 105, area.y + 94,
          right_level ? alert_color(right_level) : idle, .52);
  caption(canvas, side_fresh ? (right_level ? "BSD / LCA" : "CLEAR") : "STALE",
          area.x + area.width - 105, area.y + 113,
          right_level ? alert_color(right_level) : idle, .42);
}

void draw_sidebar(cv::Mat& canvas, double fps, double yolo_fps, double ufld_fps, double npu_ms,
                  const adas::SignalResult& signal,
                  const adas::LaneResult& front_lane, const adas::LaneResult& rear_lane,
                  bool rear_lane_enabled,
                  const adas::RiskResult& front_risk,
                  const adas::RiskResult& rear_risk,
                  const adas::BlindSpotResult& left_blind,
                  const adas::BlindSpotResult& right_blind, bool side_fresh,
                  bool health_ready,
                  double lane_ms, const std::array<double, 2>& lane_age_ms,
                  const adas::VehicleState& vehicle,
                  const adas::LaneSemantic& semantics,
                  const adas::LaneChangeResult& lane_change,
                  const adas::IntersectionResult& intersection,
                  bool verify_colors, bool lanes_enabled, bool ttc_enabled) {
  const int x = 1390, w = 514;
  const cv::Scalar muted(180, 159, 129), white(242, 237, 227);
  const cv::Scalar cyan(245, 201, 70), green(135, 229, 75), red(85, 85, 245);
  const auto card = [&](int y, int h, const char* title) {
    panel(canvas, x, y, w, h);
    caption(canvas, title, x + 16, y + 28, white, 0.62);
    cv::line(canvas, {x + 1, y + 40}, {x + w - 2, y + 40}, cv::Scalar(85, 59, 29));
  };
  card(86, 376, "2D 鸟瞰环视    AVM");
  caption(canvas, "转向意图", 1740, 292, white, 0.54);
  caption(canvas, vehicle.demo ? "DEMO 输入" : "CAN 未接",
          1740, 319, vehicle.demo ? cyan : muted, 0.48);
  caption(canvas, side_fresh ? "盲区数据有效" : "盲区数据过期",
          1740, 343, side_fresh ? green : muted, 0.46);
  const cv::Rect buttons[] = {kIntentLeft,kIntentOff,kIntentRight};
  const char* names[] = {"左", "关闭", "右"};
  for (int i=0; i<3; ++i) {
    const auto& b=buttons[i];
    panel(canvas,b.x,b.y,b.width,b.height);
    const bool selected = (i==0 && vehicle.turn==adas::TurnSignal::LEFT) ||
        (i==1 && (!vehicle.turn_valid || vehicle.turn==adas::TurnSignal::OFF)) ||
        (i==2 && vehicle.turn==adas::TurnSignal::RIGHT);
    caption(canvas,names[i],b.x+10,b.y+24,selected?cyan:muted,.49);
  }
  caption(canvas, lanes_enabled ? adas::lane_change_state_caption(lane_change.state) : "变道判断已关闭",
          1740, 438, lane_change.permitted ? green : muted, 0.46);

  card(474, 128, "交通灯与路口决策    Traffic Light");
  const char* signal_text = "未确认";
  cv::Scalar sig = muted;
  if (signal.state == adas::SignalState::RED) { signal_text = "红灯"; sig = red; }
  if (signal.state == adas::SignalState::GREEN) { signal_text = "绿灯"; sig = green; }
  if (signal.state == adas::SignalState::YELLOW) { signal_text = "黄灯"; sig = cv::Scalar(60, 220, 245); }
  for (int i = 0; i < 3; ++i) {
    const bool on = (i == 0 && signal.state == adas::SignalState::RED) ||
      (i == 1 && signal.state == adas::SignalState::YELLOW) ||
      (i == 2 && signal.state == adas::SignalState::GREEN);
    cv::circle(canvas, {x + 36 + i * 37, 551}, 13, on ? sig : cv::Scalar(53, 39, 22), cv::FILLED, cv::LINE_AA);
  }
  caption(canvas, std::string("识别：") + signal_text, x + 160, 546, sig, 0.66);
  const char* decision = adas::intersection_action_caption(intersection.action);
  const char* route = intersection.route == adas::ManeuverDirection::LEFT ? "左转" :
                      intersection.route == adas::ManeuverDirection::RIGHT ? "右转" :
                      intersection.route == adas::ManeuverDirection::STRAIGHT ? "直行" : "--";
  caption(canvas, std::string("路口方向 ") +
          (intersection.route_valid ? route : "未知") + "  |  " + decision, x + 160, 580,
          intersection.action == adas::IntersectionAction::STOP ? red : white, 0.56);
  panel(canvas,kSignalVerify.x,kSignalVerify.y,kSignalVerify.width,kSignalVerify.height);
  caption(canvas,verify_colors?"HSV复核 开":"HSV复核 关",kSignalVerify.x+5,kSignalVerify.y+21,
          verify_colors?cyan:muted,.47);

  if (lanes_enabled) {
  card(614, 136, "车道状态    Lane Status");
  caption(canvas, "前视", x + 17, 683, muted, 0.54);
  caption(canvas, front_lane.valid ? (front_lane.departure
              ? adas::lane_departure_name(front_lane.departure_side) : "已检测") : "未检测",
          x + 83, 683, front_lane.valid ? (front_lane.departure ? red : green) : muted, 0.62);
  caption(canvas, "后视", x + 285, 683, muted, 0.54);
  caption(canvas, !rear_lane_enabled ? "已关闭" :
          (rear_lane.valid ? "已检测" : "未检测"), x + 355, 683,
          rear_lane_enabled && rear_lane.valid ? green : muted, 0.62);
  std::ostringstream lane_semantic;
  lane_semantic << "左 " << adas::lane_marking_caption(semantics.left)
                << "   右 " << adas::lane_marking_caption(semantics.right)
                << "   TLC ";
  if (semantics.tlc_s > 0.0f) lane_semantic << std::fixed << std::setprecision(1) << semantics.tlc_s << "s";
  else lane_semantic << "--";
  caption(canvas, lane_semantic.str(), x + 17, 703, muted, 0.48);
  const bool changing_left = lane_change.direction == adas::ManeuverDirection::LEFT;
  const bool changing_right = lane_change.direction == adas::ManeuverDirection::RIGHT;
  const char* visual_state = !front_lane.valid ? "不可用" :
      front_lane.departure && front_lane.departure_side == adas::LaneDepartureSide::LEFT
          ? "左偏离" :
      front_lane.departure && front_lane.departure_side == adas::LaneDepartureSide::RIGHT
          ? "右偏离" :
      front_lane.partial || front_lane.identity_uncertain ? "待确认" : "未见偏离";
  const char* request_state = changing_left ? "模拟左" :
                              changing_right ? "模拟右" : "未提供";
  caption(canvas, std::string("视觉 ") + visual_state + "   |   变道请求 " + request_state,
          x + 17, 723, front_lane.departure ? red : muted, 0.46);
  const bool rule_ok = changing_left ? semantics.left_change_allowed :
                       changing_right ? semantics.right_change_allowed : false;
  const bool blind_clear = side_fresh &&
      !(changing_left ? left_blind.occupied : changing_right ? right_blind.occupied : false);
  std::ostringstream change_hud;
  change_hud << "规则 " << (rule_ok ? "OK" : "--")
             << "  盲区 " << (!side_fresh ? "STALE" : blind_clear ? "CLEAR" : "VEHICLE")
             << "  " << adas::lane_change_state_caption(lane_change.state);
  caption(canvas, change_hud.str(), x + 17, 743,
          lane_change.permitted ? green :
          lane_change.state == adas::LaneChangeState::BLOCKED ? red : muted, 0.42);
  } else {
    card(614, 136, "识别图例");
    caption(canvas,"黄色：交通灯定位框",x+17,683,cv::Scalar(40,210,245),.55);
    caption(canvas,"红色 / 绿色：已确认灯态",x+17,711,green,.55);
    caption(canvas,"灰色：当前灯态无法确认",x+17,739,muted,.50);
  }

  // 测试二按钮在每帧绘制，避免点击后等待侧栏缓存刷新。

  card(886, 132, "系统性能    System Performance");
  std::ostringstream perf, timing;
  perf << std::fixed << std::setprecision(1) << "识别 FPS  YOLO " << yolo_fps;
  if (lanes_enabled) perf << "  UFLD " << ufld_fps;
  timing << std::fixed << std::setprecision(1) << "显示 " << fps << " FPS  "
         << std::setprecision(0) << "YOLO " << npu_ms << " ms";
  if (lanes_enabled) timing << "  车道 " << lane_ms << " ms";
  caption(canvas, perf.str(), x + 17, 952, green, 0.57);
  caption(canvas, timing.str(), x + 17, 980, white, 0.48);
  std::ostringstream age;
  const bool lane_fresh = lane_age_ms[0] >= 0.0 && lane_age_ms[0] <= 1000.0;
  if (!health_ready) {
    age << "Health  INITIALIZING";
  } else {
    age << "Health  YOLO/NPU " << (side_fresh ? "OK" : "STALE");
    if (lanes_enabled) age << "  UFLD " << (lane_fresh ? "OK" : "STALE");
    age << "  CAMERA OK";
  }
  caption(canvas, age.str(), x + 17, 1006,
          health_ready && (!side_fresh || (lanes_enabled && !lane_fresh)) ? red : muted, 0.46);
}

template <class Demo>
void draw_signal_demo_view(cv::Mat& frame,const Demo& demo,bool main_view,bool direction_preview,const std::vector<adas::Detection>& detections,bool applies=true,const char* alarm_title="闯红灯报警") {
  const cv::Scalar red(80,80,245),green(120,230,75),muted(190,190,190),amber(70,210,245);
  const bool pending_green=demo.action()==adas::IntersectionAction::STOP && demo.raw()==adas::SignalState::GREEN;
  const cv::Scalar color=demo.alarm()?red:!applies?muted:pending_green?amber:demo.action()==adas::IntersectionAction::STOP?red:
      demo.action()==adas::IntersectionAction::GO?green:muted;
  const cv::Rect band=adas::ui::compact_signal_hud(frame.size(),detections);
  cv::Mat shade=frame(band).clone();shade.setTo(cv::Scalar(23,25,29));
  cv::addWeighted(frame(band),.30,shade,.70,0,frame(band));
  cv::rectangle(frame,band,color,demo.alarm()?3:1,cv::LINE_AA);
  const cv::Point offset(band.x,band.y);
  const cv::Point origin=offset+cv::Point(16,33);
  const cv::Point tip=demo.direction()==adas::ManeuverDirection::LEFT?offset+cv::Point(5,19):
      demo.direction()==adas::ManeuverDirection::RIGHT?offset+cv::Point(27,19):offset+cv::Point(16,7);
  if(applies && demo.action()==adas::IntersectionAction::GO && demo.moving())
    cv::arrowedLine(frame,origin,tip,green,4,cv::LINE_AA,0,.48);
  else {
    cv::circle(frame,offset+cv::Point(16,21),9,color,2,cv::LINE_AA);
    cv::line(frame,offset+cv::Point(10,21),offset+cv::Point(22,21),color,2,cv::LINE_AA);
  }
  const std::string state=demo.alarm()?alarm_title:!applies?std::string("方向 / ")+demo.direction_text():pending_green?"绿灯确认中":
      demo.action()==adas::IntersectionAction::STOP?(main_view?"红灯停车":"红灯亮"):
      demo.action()==adas::IntersectionAction::GO?(main_view?std::string("通行 / ")+demo.direction_text():direction_preview?std::string("绿灯亮 / ")+demo.direction_text():"绿灯亮"):"等待确认";
  caption(frame,state,band.x+31,band.y+17,color,.57);
  caption(frame,std::string(main_view?(demo.moving()?"模拟行驶":"模拟停止"):direction_preview?std::string(demo.direction_text())+" / 未选主灯组":"本视角灯态")+
      (demo.held()?" / 暂存":""),band.x+31,band.y+35,cv::Scalar(235,235,235),.45);
}

// 同一方向按钮再次点击即停车，点击另一方向则切换动作并继续行驶。
// 松开鼠标不影响运动；第三排仅显示状态，不接受行驶/停车命令。
void apply_signal_demo_controls(adas::LaneSignalSimulator& demo, bool& visible,
                                int key, const adas::MomentaryInput::Snapshot& input) {
  if (key >= 'A' && key <= 'Z') key += 'a' - 'A';
  if (key == 'h') visible = !visible;
  for (int i = 0; i < 3; ++i) {
    if (key == '4' + i) demo.set_lane(static_cast<adas::ManeuverDirection>(i + 1));
    if (key == '1' + i) demo.toggle_direction(static_cast<adas::ManeuverDirection>(i + 1));
  }
  // 消费完整点击队列，快速选择车道再选择方向时也不丢事件。
  for (const auto& click : input.clicks) {
    if (kSignalOverlayToggle.contains(click)) visible = !visible;
    for (int i=0;i<3;++i) {
      if (kDemoLanes[i].contains(click)) demo.set_lane(static_cast<adas::ManeuverDirection>(i+1));
      if (kDemoDirections[i].contains(click)) demo.toggle_direction(static_cast<adas::ManeuverDirection>(i+1));
    }
  }
}

bool apply_fcw_controls(bool& enabled,const adas::MomentaryInput::Snapshot& input) {
  bool changed=false;
  for(const auto& point:input.clicks)if(kFcwToggle.contains(point)){enabled=!enabled;changed=true;}
  return changed;
}

void draw_fcw_controls(cv::Mat& canvas,const adas::FcwState& state) {
  const char* status=state.status==adas::FcwStatus::OFF?"关":
      state.status==adas::FcwStatus::WAIT_LANE?"开 | 等待车道":
      state.status==adas::FcwStatus::STALE?"开 | 等待新观测":
      state.status==adas::FcwStatus::NO_TARGET?"开 | 等待前车":
      state.status==adas::FcwStatus::CONFIRMING?"开 | 确认前车":
      state.level==adas::FcwLevel::DANGER?"开 | 碰撞危险":
      state.level==adas::FcwLevel::CAUTION?"开 | 注意车距":"开 | 正常跟车";
  const std::string text=std::string("碰撞预警 FCW：")+status;
  static cv::Mat button;static std::string previous;
  if(button.empty() || previous!=text) {
    previous=text;button=cv::Mat(kFcwToggle.size(),CV_8UC3,cv::Scalar(24,15,7));
    const cv::Scalar color=state.enabled?adas::fcw_color(state.level):cv::Scalar(180,185,190);
    cv::rectangle(button,cv::Rect(0,0,button.cols,button.rows),color,1);
    caption(button,text,9,23,color,.48);
  }
  button.copyTo(canvas(kFcwToggle));
}

void opencv_mouse_input(int event, int x, int y, int flags, void* context) {
  auto& input = *static_cast<adas::MomentaryInput*>(context);
  if (event == cv::EVENT_LBUTTONDOWN) input.press(x,y);
  else if (event == cv::EVENT_LBUTTONUP) input.release();
  else if (event == cv::EVENT_MOUSEMOVE) input.move(x,y,(flags & cv::EVENT_FLAG_LBUTTON) != 0);
}

// 确保异常退出时也先解绑回调，再销毁它引用的鼠标状态。
class OpenCvInputWindow {
 public:
  void open(adas::MomentaryInput& input) {
    cv::namedWindow(name(),cv::WINDOW_NORMAL); active_=true;
    cv::setMouseCallback(name(),opencv_mouse_input,&input);
    cv::setWindowProperty(name(),cv::WND_PROP_FULLSCREEN,cv::WINDOW_FULLSCREEN);
  }
  ~OpenCvInputWindow() {
    if (!active_) return;
    try { cv::setMouseCallback(name(),nullptr); cv::destroyWindow(name()); }
    catch (const cv::Exception&) {} // 用户已经关闭窗口时无需再次销毁。
  }
 private:
  static const char* name() { return "RK3568 V7 | NVIDIA 4 VIEW"; }
  bool active_=false;
};

// 区分两种独立报警，避免绿灯下的方向违规被误写成“闯红灯”。
void apply_signal_demo_warning(const adas::LaneSignalSimulator& demo, adas::WarningSummary& warning) {
  if (demo.alarm()) {
    warning.priority = 110;
    warning.primary = adas::WarningCode::INTERSECTION;
    if (std::find(warning.active.begin(),warning.active.end(),adas::WarningCode::INTERSECTION)==warning.active.end())
      warning.active.push_back(adas::WarningCode::INTERSECTION);
    warning.text = demo.alarm_type() == adas::LaneSignalSimulator::AlarmType::INVALID_DIRECTION ?
        "违规操作：行驶方向与用途车道不符，请停车" : "模拟闯红灯报警：请立即停车";
  }
}

int signal_demo_signature(const adas::LaneSignalSimulator& demo,bool visible) {
  // 每种状态各占一位，报警类型改变时即使 alarm() 仍为 true，也必须立即刷新。
  return static_cast<int>(demo.action()) + 10*static_cast<int>(demo.alarm_type()) +
      100*static_cast<int>(demo.direction()) + 1000*demo.moving() +
      10000*static_cast<int>(demo.lane()) + 100000*visible;
}

void draw_signal_demo_controls(cv::Mat& canvas,const adas::LaneSignalSimulator& demo,bool visible) {
  panel(canvas,1390,614,514,144);
  caption(canvas,"用途车道 / 单击模拟行驶",1406,641,cv::Scalar(242,237,227),.60);
  panel(canvas,kSignalOverlayToggle.x,kSignalOverlayToggle.y,kSignalOverlayToggle.width,kSignalOverlayToggle.height);
  caption(canvas,visible?"图形叠加 开":"图形叠加 关",kSignalOverlayToggle.x+7,kSignalOverlayToggle.y+21,
      visible?cv::Scalar(130,230,90):cv::Scalar(185,185,185),.49);
  const char* lanes[]={"左转车道","直行车道","右转车道"};
  const char* directions[]={"1 左转","2 直行","3 右转"};
  for(int i=0;i<3;++i) {
    const auto& lane=kDemoLanes[i]; panel(canvas,lane.x,lane.y,lane.width,lane.height);
    caption(canvas,lanes[i],lane.x+10,lane.y+21,
        static_cast<int>(demo.lane())==i+1?cv::Scalar(70,205,245):cv::Scalar(185,185,185),.54);
    const auto& b=kDemoDirections[i];panel(canvas,b.x,b.y,b.width,b.height);
    const bool selected=demo.moving() && static_cast<int>(demo.direction())==i+1;
    caption(canvas,directions[i],b.x+7,b.y+21,selected?cv::Scalar(130,230,90):cv::Scalar(185,185,185),.51);
  }
  for(int i=0;i<2;++i) {
    const auto& b=kDemoStatus[i]; panel(canvas,b.x,b.y,b.width,b.height);
    const bool on=i==0?demo.moving():!demo.moving();
    caption(canvas,i==0?"车辆行驶":"车辆停止",b.x+10,b.y+21,
        on?(demo.alarm()?cv::Scalar(80,80,245):cv::Scalar(130,230,90)):cv::Scalar(125,125,125),.54);
  }
}

bool apply_test2_controls(adas::VehicleStateManager& vehicle, adas::LaneChangeFsm& fsm,
                         const adas::MomentaryInput::Snapshot& input,
                         std::chrono::steady_clock::time_point now, bool enabled, bool intent_panel = true) {
  bool changed=false;
  for(const auto& click:input.clicks) {
    adas::ManeuverDirection direction=adas::ManeuverDirection::NONE;
    bool request=false,forced=false;
    if(kTest2Directions[1].contains(click) || (intent_panel && kIntentOff.contains(click)))request=true;
    if(enabled)for(int side=0;side<2;++side) {
      if(kTest2Directions[side==0?0:2].contains(click) ||
          (intent_panel && (side==0?kIntentLeft:kIntentRight).contains(click)) || kTest2Force[side].contains(click)) {
        direction=side==0?adas::ManeuverDirection::LEFT:adas::ManeuverDirection::RIGHT;
        forced=kTest2Force[side].contains(click);request=true;
      }
    }
    if(!request)continue;
    vehicle.set_demo_turn(direction==adas::ManeuverDirection::LEFT?adas::TurnSignal::LEFT:
        direction==adas::ManeuverDirection::RIGHT?adas::TurnSignal::RIGHT:adas::TurnSignal::OFF,now);
    fsm.request(direction,now,forced);changed=true;
  }
  return changed;
}

void apply_test4_controls(adas::Test4Safety& safety, bool& show_panel,
    int key, const adas::MomentaryInput::Snapshot& input, double now, bool allow_tab=true) {
  if (key=='b' || key=='B') safety.set_enabled(!safety.current().enabled);
  for (const auto& click:input.clicks) {
    if (allow_tab && kTest4Tab.contains(click)) show_panel=!show_panel;
    else if (show_panel && kTest4Enable.contains(click)) safety.set_enabled(!safety.current().enabled);
  }
}

const char* test4_reason(const adas::Test4Result& r) {
  if (!r.enabled) return "手动开启后参与模拟停车";
  if (r.brake && r.stale) return "检测过期，保持停车";
  if (r.can_resume) return "区域安全，已自动解除刹车";
  if (r.brake) return r.hazard==adas::Test4Hazard::PEDESTRIAN?
      "行人进入行驶区域，模拟刹车":"前方车辆危险，模拟刹车";
  return r.stale?"等待新的前视检测":"监测行人及本车道前方车辆";
}

void apply_test4_interlock(const adas::Test4Result& r, adas::LaneSignalSimulator& demo) {
  if (r.enabled && r.brake) demo.stop();
}

void draw_test4_controls(cv::Mat& canvas,const adas::Test4Result& r,bool show_panel) {
  const cv::Scalar muted(180,159,129),white(242,237,227),cyan(245,201,70),red(85,85,245),green(135,229,75);
  if (show_panel) {
    cv::rectangle(canvas,cv::Rect(1390,86,514,376),cv::Scalar(24,15,7),cv::FILLED);
    panel(canvas,1390,86,514,376);
    caption(canvas,"测试四 / 危险模拟停车",1406,114,white,.62);
    cv::line(canvas,{1391,126},{1902,126},cv::Scalar(85,59,29));
    panel(canvas,kTest4Enable.x,kTest4Enable.y,kTest4Enable.width,kTest4Enable.height);
    caption(canvas,r.enabled?"危险监测 开  [B]":"危险监测 关  [B]",1418,166,r.enabled?green:muted,.52);
    panel(canvas,kTest4Resume.x,kTest4Resume.y,kTest4Resume.width,kTest4Resume.height);
    caption(canvas,r.can_resume?"刹车已自动解除":"安全后自动解除",1653,166,r.can_resume?green:muted,.52);
    caption(canvas,r.brake?"车辆状态：模拟停车":r.enabled?"车辆状态：监测中":"车辆状态：测试四未启用",
        1408,211,r.brake?red:white,.64);
    caption(canvas,test4_reason(r),1408,246,r.brake?red:muted,.55);
    caption(canvas,r.lane_based?"行驶区域：当前车道线":"行驶区域：中心估计区域",1408,282,cyan,.51);
    caption(canvas,"连续安全至少1秒、4次新观测后解除",1408,318,muted,.48);
    caption(canvas,"解除后用原方向按钮手动起步",1408,352,muted,.51);
    caption(canvas,"模拟输出 / 未连接车辆制动执行器",1408,390,muted,.47);
    caption(canvas,r.brake?"危险目标："+std::to_string(r.hazards.size())+"，持续确认安全后解除":
        "行人横穿 / 前车接近及近距阻挡",1408,429,white,.47);
  }
  panel(canvas,kTest4Tab.x,kTest4Tab.y,kTest4Tab.width,kTest4Tab.height);
  caption(canvas,show_panel?"切换 AVM":"切换测试四",kTest4Tab.x+10,kTest4Tab.y+21,cyan,.49);
}

void draw_test4_view(cv::Mat& frame,const adas::Test4Result& r,
                     const std::vector<adas::Detection>& visible) {
  if (!r.enabled || frame.empty()) return;
  const cv::Scalar color=r.brake?cv::Scalar(80,80,245):cv::Scalar(220,190,80);
  const cv::Rect band(std::max(0,frame.cols-270),8,std::min(260,frame.cols),44);
  cv::Mat shade=frame(band).clone();shade.setTo(cv::Scalar(23,25,29));
  cv::addWeighted(frame(band),.25,shade,.75,0,frame(band));
  cv::rectangle(frame,band,color,1);
  caption(frame,r.brake?"测试四 / 模拟停车":"测试四 / 危险监测",band.x+7,band.y+17,color,.47);
  caption(frame,r.stale?"等待新检测，禁止自动解除":r.can_resume?"已自动解除，等待手动起步":
      r.brake?(r.hazard==adas::Test4Hazard::PEDESTRIAN?"行人进入行驶区域":"前方车辆危险"):"行人 / 前车",band.x+7,band.y+35,color,.40);
  if (r.brake) {
    cv::rectangle(frame,cv::Rect(1,1,frame.cols-2,frame.rows-2),color,2);
    // Rendering may follow the existing bounded tracker display; it never
    // feeds a new danger/clear vote back into Test4Safety.
    for (const auto& target:r.hazards)
      for (const auto& d:visible)
        if (d.track_id==target.track_id) {
          cv::rectangle(frame,d.box,color,3,cv::LINE_AA); break;
        }
  }
}

void draw_navigation_symbol(cv::Mat& frame,cv::Rect area,adas::ManeuverDirection direction,
                            int distance,bool inset) {
  cv::rectangle(frame,area,cv::Scalar(8,12,15),cv::FILLED);
  cv::rectangle(frame,area,cv::Scalar(180,150,65),1);
  const int x=area.x,y=area.y;
  const cv::Scalar white(245,245,245);
  const float scale=inset?.62f:1.f;
  const cv::Point base(x+int(28*scale),y+int(78*scale));
  const cv::Point bend(base.x,y+int(40*scale));
  if(direction==adas::ManeuverDirection::STRAIGHT)
    cv::arrowedLine(frame,base,{base.x,y+int(20*scale)},white,inset?3:5,cv::LINE_AA,0,.30);
  else {
    cv::line(frame,base,bend,white,inset?3:5,cv::LINE_AA);
    const int sign=direction==adas::ManeuverDirection::LEFT?-1:1;
    cv::arrowedLine(frame,bend,{bend.x+int(24*scale)*sign,bend.y},white,inset?3:5,cv::LINE_AA,0,.40);
  }
  caption(frame,std::to_string(distance)+" m",x+int(76*scale),y+int(48*scale),white,inset?.64:.95);
  const char* label=direction==adas::ManeuverDirection::LEFT?"左转":direction==adas::ManeuverDirection::RIGHT?"右转":"直行";
  caption(frame,label,x+int(76*scale),y+int(78*scale),white,inset?.45:.60);
  caption(frame,"模拟导航",x+8,y+area.height-8,cv::Scalar(95,205,245),inset?.35:.43);
}

void draw_navigation_controls(cv::Mat& canvas,const adas::NavigationDemo& nav,
                              const adas::NavigationDemo::Advice& advice) {
  const cv::Scalar muted(180,159,129),white(242,237,227),cyan(245,201,70),green(135,229,75),red(85,85,245);
  cv::rectangle(canvas,cv::Rect(1390,86,514,376),cv::Scalar(24,15,7),cv::FILLED);
  panel(canvas,1390,86,514,376);
  caption(canvas,"测试三 / 导航模拟驾驶",1406,114,white,.62);
  cv::line(canvas,{1391,126},{1902,126},cv::Scalar(85,59,29));
  draw_navigation_symbol(canvas,{1408,144,210,110},nav.direction(),nav.distance(),false);
  caption(canvas,std::string("方向：")+nav.direction_text(),1640,169,white,.62);
  caption(canvas,"距离："+std::to_string(nav.distance())+" m",1640,201,white,.62);
  caption(canvas,!nav.enabled()?"状态：关闭":nav.distance()>20?
      (nav.direction()==adas::ManeuverDirection::STRAIGHT?"状态：前方直行":"状态：准备转弯"):
      "状态：模拟到达路口",1640,236,cyan,.47);
  const cv::Scalar action_color=advice.action==adas::NavigationDemo::Action::STOP?red:
      advice.action==adas::NavigationDemo::Action::WAIT?cv::Scalar(75,210,245):nav.enabled()?green:muted;
  cv::rectangle(canvas,cv::Rect(1408,264,478,36),cv::Scalar(35,45,16),cv::FILLED);
  cv::rectangle(canvas,cv::Rect(1408,264,478,36),action_color,1);
  caption(canvas,std::string("模拟建议：")+advice.text,1420,289,action_color,.65);
  auto b=adas::NavigationDemo::toggle_button();panel(canvas,b.x,b.y,b.width,b.height);
  caption(canvas,nav.enabled()?"导航模拟 开":"导航模拟 关",b.x+10,b.y+22,nav.enabled()?green:muted,.52);
  caption(canvas,std::string("匹配灯向：")+adas::LaneSignalSimulator::text(nav.current_route()),1640,334,cyan,.49);
  const auto directions=adas::NavigationDemo::direction_buttons(),distances=adas::NavigationDemo::distance_buttons();
  const char* names[]={"左转","直行","右转"};const char* distance_names[]={"距离 -30 m","距离 +30 m","重置 120 m"};
  for(int i=0;i<3;++i){
    const auto d=directions[i];panel(canvas,d.x,d.y,d.width,d.height);
    caption(canvas,names[i],d.x+12,d.y+22,int(nav.direction())==i+1?green:muted,.53);
    const auto a=distances[i];panel(canvas,a.x,a.y,a.width,a.height);
    caption(canvas,distance_names[i],a.x+8,a.y+22,white,.48);
  }
  caption(canvas,"方向与距离为模拟输入 / 尚未接入导航识别",1408,449,muted,.43);
}

void draw_navigation_view(cv::Mat& frame,const adas::NavigationDemo& nav,
                          const adas::NavigationDemo::Advice& advice) {
  if(!nav.enabled())return;
  draw_navigation_symbol(frame,{frame.cols-146,frame.rows-112,136,84},nav.direction(),nav.distance(),true);
  const cv::Rect band(frame.cols-270,59,260,42);
  cv::Mat shade=frame(band).clone();shade.setTo(cv::Scalar(23,25,29));
  cv::addWeighted(frame(band),.25,shade,.75,0,frame(band));
  const cv::Scalar color=advice.action==adas::NavigationDemo::Action::STOP?cv::Scalar(80,80,245):cv::Scalar(80,210,245);
  cv::rectangle(frame,band,color,1);
  caption(frame,"模拟：前方 "+std::to_string(nav.distance())+" m "+nav.direction_text(),band.x+7,band.y+16,color,.44);
  caption(frame,advice.text,band.x+7,band.y+34,color,.44);
}

void draw_test2_controls(cv::Mat& canvas,const adas::LaneChangeResult& change,
                         const adas::LaneSemantic& lane,bool enabled) {
  const cv::Scalar muted(150,150,150),green(130,230,90),red(80,80,245),amber(70,205,245);
  panel(canvas,1390,766,514,108);
  caption(canvas,"测试二 / 车道线控制",1406,793,cv::Scalar(242,237,227),.56);
  caption(canvas,!enabled?"请先开启车道检测":change.solid_alarm?"实线变道报警":
      test2_action_caption(change),1650,793,change.solid_alarm?red:muted,.49);
  const char* labels[]={"左变道","保持直行","右变道","强制左变道","强制右变道"};
  for(int i=0;i<5;++i) {
    const auto& b=i<3?kTest2Directions[i]:kTest2Force[i-3];
    panel(canvas,b.x,b.y,b.width,b.height);
    const bool left=change.direction==adas::ManeuverDirection::LEFT;
    const bool right=change.direction==adas::ManeuverDirection::RIGHT;
    const bool selected=i==1?change.state==adas::LaneChangeState::IDLE:
        i==0?left&&!change.forced:i==2?right&&!change.forced:i==3?left&&change.forced:right&&change.forced;
    caption(canvas,labels[i],b.x+10,b.y+21,!enabled&&i!=1?cv::Scalar(100,100,100):
        selected?(change.solid_alarm?red:change.permitted?green:amber):muted,.51);
  }
  const std::string markings=std::string("左")+adas::lane_marking_caption(lane.left)+
      " / 右"+adas::lane_marking_caption(lane.right);
  caption(canvas,markings,1570,860,muted,.43);
}

void draw_bottom_left(cv::Mat& canvas, int scene_index, int scene_count,
                      const std::string& scene_name, bool live = false, bool two_stage = false,
                      bool lanes_enabled = false) {
  cv::rectangle(canvas, {0, 964, 1380, 116}, cv::Scalar(26, 17, 8), cv::FILLED);
  caption(canvas, live ? "实时采集" : "同步回放", 24, 1003, cv::Scalar(235, 199, 100), 0.62);
  std::ostringstream scene;
  if (live) scene << "PCIe 1920 x 1080";
  else {
    std::string display_name=scene_name.substr(0,16);
    if(scene_name.find("recommended_wait_a")!=std::string::npos)display_name="推荐A：外滩/直行/左转/圆灯";
    if(scene_name.find("recommended_wait_b")!=std::string::npos)display_name="推荐B：陆家嘴/徐家汇/夜景";
    if(scene_name.find("recommended_wait_c")!=std::string::npos)display_name="推荐C：深圳圆灯/直行箭头";
    if(scene_name.find("recommended_control_d")!=std::string::npos)display_name="推荐D：短箭头/国外对照";
    if(scene_name.find("near_china_a")!=std::string::npos)display_name="精选A：陆家嘴/外滩/夜景/箭头";
    if(scene_name.find("near_china_a_driving")!=std::string::npos)display_name="驾驶视角：陆家嘴/外滩/夜景/外滩";
    if(scene_name.find("near_control_b")!=std::string::npos)display_name="精选B：箭头/陆家嘴/国外/外滩";
    if(scene_name.find("near_left_straight")!=std::string::npos)display_name="精选B：近拍左转与直行分相位";
    scene << "场景 " << scene_index+1 << '/' << scene_count << "   " << display_name;
  }
  caption(canvas, scene.str(), 150, 1003, cv::Scalar(215, 202, 180), 0.55);
  caption(canvas, lanes_enabled ? "C 前视标定   R 后视标定   Q 退出" : "V 环视放大   Q 退出", 578, 1003, cv::Scalar(183, 163, 137), 0.51);
  if (!live) {
  for (const auto& rect : {kPrevSceneButton, kNextSceneButton}) panel(canvas, rect.x, rect.y, rect.width, rect.height);
  caption(canvas, "< 上一场景", 1057, 1006, cv::Scalar(238, 223, 191), 0.52);
  caption(canvas, "下一场景 >", 1217, 1006, cv::Scalar(238, 223, 191), 0.52);
  }
  std::string models=two_stage ? "YOLO INT8  /  MO FP16  /  ByteTrack" : "YOLOv8n-P2  /  ByteTrack";
  if (lanes_enabled) models += "  /  UFLD V2";
  caption(canvas,models,24,1037,cv::Scalar(174,150,111),.48);
  cv::rectangle(canvas, {1390, 1028, 514, 52}, cv::Scalar(24, 15, 7), cv::FILLED);

}

}  // namespace

#ifndef ADAS_INPUT_MODE_TEST
int run_four_view(int argc, char** argv) {
  Options options;
  if (!parse(argc, argv, options)) return 1;
  const bool live = !options.pcie_device.empty();
  if (live && (options.separate || options.legacy_mosaic))
    throw std::runtime_error("PCIe requires one full 1080p composite inference");
  if (live && options.lanes_enabled && options.ufld_model.empty())
    throw std::runtime_error("PCIe requires --ufld-model (1600x320 UFLDv2); no traditional lane fallback");
  if (live) options.ufld_rear = true;
  if (live) adas::bind_cpu_pair(2, "adas-logic");
  else adas::bind_cpu_pair(2, "replay-logic");
  std::signal(SIGINT, request_quit);
  std::signal(SIGTERM, request_quit);
  adas::PcieCapture pcie(quit_requested);
  if (live) pcie.open(options.pcie_device);
  if (!adas::ui::initialize_text("/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc"))
    std::cerr << "Chinese font unavailable; UI text will use fallback rendering\n";
  // Keep the launcher default at one; allow controlled CPU-thread comparisons.
  cv::setNumThreads(std::max(1,options.cpu_threads));
  std::vector<std::string> scene_paths;
  const char* bundled_scenes[] = {
      "four_view_sample/recommended_wait_a_20261012",
      "four_view_sample/recommended_wait_b_20261012",
      "four_view_sample/recommended_wait_c_20261012",
      "four_view_sample/recommended_control_d_20261012",
      "four_view_sample/signal_ru_20261010",
      "four_view_sample/signal_sg_20261010",
      "four_view_sample/signal_close_20261010",
      "four_view_sample/china_lights_quad_20261010",
      "four_view_sample/china_lights_quad_b_20261010",
      "four_view_sample/china_lights_quad_c_20261010",
      "four_view_sample/66b5fa4b_30fps",
      "four_view_sample/e716f3ed_30fps",
      "four_view_sample/050541b1_30fps",
      "four_view_sample/668b1ac0_30fps",
      "four_view_sample/ddfc4b8d_30fps"};
  if (!live && scene_complete(options.scene)) scene_paths.push_back(options.scene);
  for (const char* bundled : bundled_scenes) {
    const std::string candidate = bundled;
    if (live || !scene_complete(candidate) ||
        std::find(scene_paths.begin(), scene_paths.end(), candidate) != scene_paths.end()) continue;
    scene_paths.push_back(candidate);
  }
  // A curated light-replay list keeps rejected/remote-light scenes off the UI.
  // Explicit --scene still permits an engineer to open a legacy road scene.
  if (!live) {
    std::ifstream curated("config/recommended_light_scenes.txt");
    std::vector<std::string> approved;
    std::string candidate;
    while (curated >> candidate) {
      if (scene_complete(candidate) && std::find(approved.begin(),approved.end(),candidate)==approved.end())
        approved.push_back(candidate);
    }
    if (!approved.empty()) {
      if (scene_complete(options.scene) && std::find(approved.begin(),approved.end(),options.scene)==approved.end())
        approved.insert(approved.begin(),options.scene);
      scene_paths.swap(approved);
    }
  }
  // PCIe 的 --scene 只指定当前视频对应的业务配置，不要求本地保存视频文件。
  // 保留实际路径，后面的灯组绑定、方向判断及 AVM 才能与回放复用同一配置。
  if (live) scene_paths.push_back(options.scene);
  if (scene_paths.empty()) {
    std::cerr << "No complete four-view scenes found\n";
    return 2;
  }
  int scene_index = static_cast<int>(std::find(scene_paths.begin(), scene_paths.end(), options.scene) -
                                     scene_paths.begin());
  if (scene_index < 0 || scene_index >= static_cast<int>(scene_paths.size())) scene_index = 0;
  options.scene = scene_paths[scene_index];
  std::array<SyncCapture, 4> streams;
  double source_fps = 0.0;
  int source_frames = -1;
  bool single_composite = live;
  if (!live && !open_scene(options.scene, !options.software_decode, streams,
                  source_fps, source_frames, single_composite)) {
    std::cerr << "Cannot open synchronized scene " << options.scene << '\n';
    return 2;
  }
  adas::DetectorConfig config;
  config.model_path = options.model;
  adas::RknnDetector detector(config);
  if (!detector.ready()) { std::cerr << detector.error() << '\n'; return 3; }
  std::unique_ptr<adas::UfldLaneDetector> ufld_lane_detector;
  if (options.lanes_enabled && !options.ufld_model.empty()) {
    auto loaded = load_lane_model(options.ufld_model, live);
    if (!loaded.model) {
      std::cerr << loaded.error << '\n';
      return 4;
    }
    ufld_lane_detector = std::move(loaded.model);
    std::cout << "Experimental lane model: " << ufld_lane_detector->profile()
              << " every >= " << options.ufld_every << " displayed frames\n";
  }
  if (live && options.lanes_enabled && ufld_lane_detector->input_width() != 1600)
    throw std::runtime_error("PCIe lane model must use 1600x320 UFLDv2");
  if (!options.lanes_enabled) std::cout << "Lane detection and overlay: disabled\n";
  if (!options.ttc_enabled) std::cout << "Legacy TTC/RCW disabled; front FCW has an independent dashboard switch\n";
  std::cout << "Front collision warning enabled=" << options.fcw_enabled << std::endl;
  if (live && options.lanes_enabled) std::cout << "Lane backend: front=UFLDv2/RKNN rear="
      << (options.rear_lane_enabled ? "UFLDv2/RKNN" : "disabled")
      << "; traditional OpenCV lane fallback=disabled; upstream=ed12960\n";
  const bool high_precision_ufld = ufld_lane_detector &&
                                   ufld_lane_detector->input_width() >= 1600;
  adas::PerceptionPeriods perception_periods =
      adas::lane_priority_periods(high_precision_ufld,live && options.npu_workers>1);
  if (ufld_lane_detector) {
    std::cout << "Perception service targets: objects="
              << perception_periods.object_ms << " ms front_lane="
              << perception_periods.front_lane_ms << " ms rear_lane="
              << perception_periods.rear_lane_ms << " ms; display_limit="
              << (options.display_fps_limit > 0.0 ? options.display_fps_limit
                                                  : source_fps)
              << " FPS\n";
  }
  bool verify_colors = true;
  int hsv_checked=0, hsv_rejected=0;
  double hsv_ms=0.0;
  adas::VehicleStateManager vehicle_state_manager;
  adas::VehicleState vehicle_state;
  adas::LaneSemanticTracker lane_semantic_tracker;
  adas::LaneSemantic lane_semantic;
  adas::LaneChangeFsm lane_change_fsm;
  adas::LaneChangeResult lane_change_result;
  adas::IntersectionResult intersection_result;
  std::vector<adas::Detection> current_lights;
  adas::LaneSignalSimulator signal_demo;
  std::cout << "Signal simulator: lane-based toggle controls v4; invalid-direction and red-light alarms, same-button stop\n";
  signal_demo.set_lane(options.demo_direction == "left" ? adas::ManeuverDirection::LEFT :
      options.demo_direction == "right" ? adas::ManeuverDirection::RIGHT : adas::ManeuverDirection::STRAIGHT);
  int demo_saved_signature = -1;
  int dashboard_demo_signature = -1; // 界面刷新与状态日志分别记录，避免互相吞掉变更。
  bool signal_overlay_visible=options.signal_overlay_visible;
  std::array<adas::SignalDemo,3> extra_demos;
  const auto configure_extra_demos=[&]() {
    for(auto& demo:extra_demos){demo.enable_full_view();demo.set_direction_filter(true);demo.set_direction(signal_demo.direction());}
  };
  configure_extra_demos();
  const auto accept_demo_signal = [&](double seconds) {
    signal_demo.observe(seconds,current_lights,640,360);
    std::cout << "SIGNAL_DEMO_OBS t=" << seconds << " raw=" << static_cast<int>(signal_demo.raw())
      << " action=" << static_cast<int>(signal_demo.action()) << " confirmations=" << signal_demo.confirmations()
      << " held=" << signal_demo.held() << " confidence=" << signal_demo.confidence() << std::endl;
  };
  adas::WarningManager warning_manager;
  adas::WarningSummary warning_summary;

  adas::SignalLogic signal_logic;
  adas::DriveConfig drive_config;
  adas::DriveDecisionLogic drive_logic(drive_config);
  adas::LaneConfig lane_config;
  lane_config.processing_width = 640;
  adas::LaneDetector front_lane_detector(lane_config);
  adas::LaneDetector rear_lane_detector(lane_config);
  std::vector<cv::Point2f> front_roi_points;
  std::vector<cv::Point2f> rear_roi_points;
  load_roi("config/front_lane_roi.txt", front_lane_detector, &front_roi_points);
  load_roi("config/rear_lane_roi.txt", rear_lane_detector, &rear_roi_points);
  adas::LaneDepartureMonitor front_lane_warning;
  adas::LaneDepartureMonitor rear_lane_warning;
  front_lane_warning.set_reference_from_roi(front_roi_points);
  rear_lane_warning.set_reference_from_roi(rear_roi_points);
  adas::RiskConfig front_risk_config;
  front_risk_config.focal_scale = 0.30f;  // NVIDIA front wide, about 120 degrees.
  adas::RiskConfig rear_risk_config;
  rear_risk_config.require_closing = true;
  rear_risk_config.focal_scale = 1.87f;   // NVIDIA rear tele, about 30 degrees.
  adas::RiskEstimator front_risk_estimator(front_risk_config);
  adas::FcwMonitor fcw_monitor;fcw_monitor.set_enabled(options.fcw_enabled);
  adas::FcwPaintTracker fcw_paint_tracker;
  double fcw_ms=0.;
  adas::RiskEstimator rear_risk_estimator(rear_risk_config);
  adas::Test4Safety test4_safety;
  test4_safety.set_enabled(options.test4_enabled);
  bool show_test4_panel=true;
  bool show_navigation_panel=options.demo_navigation;
  int navigation_top_panel=options.test4_enabled?1:0;
  adas::NavigationDemo navigation_demo;
  navigation_demo.set_enabled(options.demo_navigation);
  adas::LaneSignalSimulator navigation_signal;
  int test4_saved_signature=-1;
  adas::BlindSpotMonitor left_blind_monitor;
  adas::BlindSpotMonitor right_blind_monitor;
  std::array<adas::ByteTracker, 4> trackers;
  std::array<adas::BoxMotionCompensator,4> box_motion;
  double box_motion_ms=0;int box_motion_corrected=0;
  adas::SignalResult signal;
  adas::DriveResult drive;
  adas::LaneResult lane;
  adas::LaneResult rear_lane;
  adas::LaneResult front_lane_target;
  adas::LaneResult front_lane_observed;
  adas::LaneResult rear_lane_target;
  adas::NeuralLaneGeometryTracker front_neural_lane_tracker;
  adas::NeuralLaneGeometryTracker rear_neural_lane_tracker;
  adas::LaneVisualTracker front_lane_visual,rear_lane_visual;
  adas::LaneOuterStyleTracker front_outer_style;
  adas::LanePaintDisplayTracker front_paint_display;
  int front_lane_missed_updates = 0;
  int rear_lane_missed_updates = 0;
  adas::RiskResult front_risk;
  adas::RiskResult rear_risk;
  adas::BlindSpotResult left_blind;
  adas::BlindSpotResult right_blind;
  std::array<std::vector<adas::Detection>, 4> detections;
  std::array<std::vector<adas::Detection>, 4> observed;
  std::array<double, 4> detection_seconds{{-1,-1,-1,-1}};
  std::array<bool, 4> fresh_measurement{{false, false, false, false}};
  std::future<InferenceResult> inference_future;
  bool inference_running = false;
  std::future<LaneTaskResult> lane_future;
  adas::NpuExecutor npu_executor(options.npu_workers);
  bool lane_running = false;
  bool lane_task_uses_npu = false;
  LaneToggleState lane_toggle;
  lane_toggle.requested = options.lanes_enabled;
  std::future<LaneModelLoadResult> lane_model_future;
  bool lane_backend_reset_pending = false;
  int dashboard_lane_signature = -1;
  int ufld_task_sequence = 0;
  std::array<std::chrono::steady_clock::time_point, 3> task_launched{};
  int last_detector_start = -1000000;
  int last_ufld_start = -1000000;
  double npu_ms = 0.0;
  adas::AvmRenderer avm;
  avm.load(scene_asset_path(options.scene, "avm.yml"));
  avm.set_visual_fill(true);
  std::cout << "AVM calibrated cameras=" << avm.count()
            << "; PC presentation: full ground view, car 1.55x, bounded texture fill\n";
  std::shared_ptr<AvmReplayWorker> avm_worker(new AvmReplayWorker(avm,options.scene,live));
  bool avm_visible = false;
  cv::Mat avm_preview;
  auto avm_updated = std::chrono::steady_clock::time_point();
  std::future<cv::Mat> avm_future;
  bool avm_running = false;
  bool avm_changed = true;
  RecognitionRate yolo_rate, ufld_rate;
  double object_interval_ms = -1.0;
  double object_result_ms = -1.0;
  unsigned stale_object_drops=0,object_results=0;
  double object_gap_ms=-1.0,max_object_gap_ms=0.0;
  auto object_received_at=std::chrono::steady_clock::time_point();
  std::array<size_t,4> object_raw_counts{{0,0,0,0}};
  double display_fps = 0.0;
  double instant_fps = 0.0;
  auto previous_presentation = std::chrono::steady_clock::time_point();
  auto fps_window_start = std::chrono::steady_clock::time_point();
  int fps_window_intervals = 0;
  int source_frame_index = 0;
  int shown_frames = 0;
  bool capture_failed = false;
  CalibrationTarget calibration_target = CalibrationTarget::NONE;
  std::vector<cv::Point2f> calibration_points;
  adas::MomentaryInput mouse_input;
  OpenCvInputWindow opencv_window;
  adas::ImageProcessingDialog image_dialog;
  adas::ImageProcessingToolbar image_toolbar(image_dialog);
  std::unique_ptr<adas::GlPresenter> gl_presenter;
#ifdef ADAS_HAVE_GLES
  if (!options.headless && !options.opencv_display) {
    try {
      gl_presenter.reset(new adas::GlPresenter(kCanvasW, kCanvasH));
      std::cout << "GLES renderer: " << gl_presenter->renderer() << std::endl;
    } catch (const std::exception& error) {
      std::cerr << "GLES unavailable, using OpenCV display: " << error.what() << std::endl;
    }
  }
#endif
  if (!options.headless && !gl_presenter) {
    opencv_window.open(mouse_input);
  }
  if (live) std::cout << "PCIe pcie_v3 read 1080p RGB565; capture CPU=0,1; image controls/ACK=worker; YOLO/UFLD=RKNN NPU\n";
  else std::cout << "Four synchronized streams: " << source_frames << " frames at "
            << source_fps << " FPS; " << (options.separate ? "round-robin" : "mosaic")
            << " V7 inference every " << options.detect_every << " frame(s); decoder="
            << (options.software_decode ? "software" : "MPP hardware")
            << "; inference_source="
            << (options.legacy_mosaic ? "legacy 640x640 mosaic" : "FPGA-style 1920x1080")
            << "; video_ui=1280x720\n";
  const auto start = std::chrono::steady_clock::now();
  auto playback_epoch = start;
  int playback_dropped_frames = 0;
  cv::Mat canvas(kCanvasH, kCanvasW, CV_8UC3, cv::Scalar(24, 15, 7));
  cv::rectangle(canvas, cv::Rect(0, 0, kCanvasW, kHeaderH),
                cv::Scalar(37, 24, 10), cv::FILLED);
  caption(canvas, "RoadFusion", 24, 39, cv::Scalar(250, 209, 95), 1.03);
  caption(canvas, "Drive Smarter. A Safer Tomorrow.", 26, 62, cv::Scalar(185, 160, 130), 0.43);
  caption(canvas, "基于紫光同创 FPGA + RK3568 的四视角高级辅助驾驶系统",
          514, 36, cv::Scalar(242, 238, 229), 0.83);
  caption(canvas, "RoadFusion-Edge  /  Demo UI", 779, 62, cv::Scalar(202, 178, 140), 0.52);
  draw_camera_cards(canvas);
  const cv::Mat normal_header = canvas(cv::Rect(430, 0, 980, 76)).clone();
  std::array<cv::Mat, 4> source_images;
  cv::Mat source_composite;
  cv::Mat live_preview;
  double last_lane_ms = 0.0;
  std::array<std::chrono::steady_clock::time_point, 2> lane_captured_at{};
  std::array<int, 2> lane_source_frame{{-1, -1}};
  std::array<double, 2> lane_age_ms{{-1.0, -1.0}};
  int stale_lane_drops = 0;
  double lane_complete_age_ms=-1.0, lane_poll_delay_ms=-1.0;
  auto last_detection_at = std::chrono::steady_clock::time_point();
  const auto finish_lane_task = [&]() {
    // Draining both workers prevents pre-seek/pre-scene results from leaking.
    if (inference_running) {
      inference_future.wait();
      inference_running = false;
    }
    front_lane_target = rear_lane_target = adas::LaneResult();
    front_neural_lane_tracker.reset(); front_lane_visual.reset(); front_outer_style.reset(); front_paint_display.reset();
    rear_neural_lane_tracker.reset(); rear_lane_visual.reset();
    front_lane_warning.reset();
    rear_lane_warning.reset();
    const auto reset_at = std::chrono::steady_clock::now();
    vehicle_state_manager = adas::VehicleStateManager();
    lane_change_fsm.request(adas::ManeuverDirection::NONE, reset_at);
    lane_semantic_tracker.reset();
    lane_semantic = adas::LaneSemantic();
    intersection_result = adas::IntersectionResult();
    warning_summary = adas::WarningSummary();
    current_lights.clear();
    signal_demo.reset();
    test4_safety.reset();
    mouse_input.cancel();
    configure_extra_demos();
    demo_saved_signature=-1;
    dashboard_demo_signature=-1;
    task_launched = {};
    lane_captured_at = {};
    lane_source_frame = {{-1, -1}};
    last_detection_at = std::chrono::steady_clock::time_point();
    if (lane_running) {
      lane_future.wait();
      lane_running = false;
    }
  };
  const auto clear_lane_display = [&]() {
    // 只清车道相关状态，不调用 finish_lane_task()，以免重置灯态和车辆按钮。
    // 这里也不等待 NPU；在途车道任务通过 generation 丢弃。
    lane = rear_lane = front_lane_target = rear_lane_target = adas::LaneResult();
    front_neural_lane_tracker.reset(); front_lane_visual.reset(); front_outer_style.reset(); front_paint_display.reset(); rear_neural_lane_tracker.reset(); rear_lane_visual.reset();
    front_lane_warning.reset(); rear_lane_warning.reset();
    front_lane_missed_updates = rear_lane_missed_updates = 0;
    lane_semantic_tracker.reset(); lane_semantic = adas::LaneSemantic();
    lane_change_fsm.request(adas::ManeuverDirection::NONE, std::chrono::steady_clock::now());
    lane_change_result = adas::LaneChangeResult();
    lane_captured_at = {}; lane_source_frame = {{-1,-1}}; lane_age_ms = {{-1.,-1.}};
    task_launched[1] = task_launched[2] = std::chrono::steady_clock::time_point();
    last_ufld_start = -1000000; ufld_task_sequence = 0;
    last_lane_ms = 0; ufld_rate.reset();
    calibration_target = CalibrationTarget::NONE; calibration_points.clear();
    // OpenCV 检测器可能仍由旧工作线程使用，等其退出后再 reset，避免数据竞争。
    lane_backend_reset_pending = true;
  };
  auto last_frame_captured = std::chrono::steady_clock::time_point();
  while (!quit_requested) {
    // 先还原上帧菜单下面的侧栏，再正常更新界面，收起后不会留下残影。
    image_toolbar.restore_overlay(canvas);
    int key=-1, click_x=-1, click_y=-1;
    if (!options.headless) {
      if (gl_presenter) key=gl_presenter->poll_input(&click_x,&click_y,&mouse_input);
      else {
        key=cv::waitKey(1) & 0xff;
        if (cv::getWindowProperty("RK3568 V7 | NVIDIA 4 VIEW",cv::WND_PROP_VISIBLE)<1) break;
      }
    }
    // GLES 主窗口没有 HighGUI 事件循环；弹窗可见时单独泵送其事件。
    // 弹窗里的按键不再传给车辆按钮，避免输入参数时误触方向/报警。
    if (!options.headless && gl_presenter && image_dialog.visible()) {
      const int dialog_key=cv::waitKey(1)&0xff;
      if(dialog_key==27)image_dialog.close();
    } else if(image_dialog.visible() && key>=0 && key!=255) {
      if(key==27)image_dialog.close();
      key=-1;
    }
    if(key==27 && image_toolbar.dismiss())key=-1;
    if (key=='q' || key==27) break;
    auto input=mouse_input.take();
    const auto image_status=pcie.image_status();
    const auto submit_image=[&pcie](const fpga_image_submit& config) {
      return pcie.submit_image(config);
    };
    image_dialog.tick(image_status,live,submit_image);
    // 快捷菜单先消费鼠标事件，避免透过下拉列表触发驾驶模拟或标定。
    if(image_toolbar.route_input(input,click_x,click_y,image_status,live,submit_image)) {
      image_dialog.open();image_dialog.tick(image_status,live,submit_image);
    }
    // 拖动坐标仍供原标定/进度条使用；普通按钮只消费一次按下事件。
    if (!gl_presenter) { click_x=input.click.x; click_y=input.click.y; }
    if(apply_fcw_controls(options.fcw_enabled,input)) {
      fcw_monitor.set_enabled(options.fcw_enabled);
      std::cout<<"FCW_TOGGLE enabled="<<options.fcw_enabled<<std::endl;
    }
    apply_signal_demo_controls(signal_demo,signal_overlay_visible,key,input);
    if(options.demo_navigation) {
      for(const auto& point:input.clicks) {
        if(kTest4Tab.contains(point))navigation_top_panel=(navigation_top_panel+1)%3;
        else if(navigation_top_panel==0)navigation_demo.click(point);
      }
      show_navigation_panel=navigation_top_panel==0;
      show_test4_panel=navigation_top_panel==1;
    }
    apply_test4_controls(test4_safety,show_test4_panel,key,input,
        live?std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count():source_frame_index/source_fps,
        !options.demo_navigation);
    if (apply_lane_toggle_controls(lane_toggle, key, input)) {
      options.lanes_enabled = lane_toggle.requested &&
          (ufld_lane_detector || (!live && options.ufld_model.empty()));
      clear_lane_display();
      if(!lane_toggle.requested) {
        vehicle_state_manager.set_demo_turn(adas::TurnSignal::OFF,std::chrono::steady_clock::now());
        lane_change_fsm.request(adas::ManeuverDirection::NONE,std::chrono::steady_clock::now());
      }
      std::cout << "Lane detection requested=" << lane_toggle.requested << std::endl;
    }
    // 加载期间继续显示画面和处理按钮；取消开启后，完成的模型只缓存、不自动启用。
    if (lane_toggle.loading &&
        lane_model_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      auto loaded = lane_model_future.get();
      lane_toggle.loading = false;
      if (loaded.model) {
        ufld_lane_detector = std::move(loaded.model);
        perception_periods = adas::lane_priority_periods(ufld_lane_detector->input_width() >= 1600,
                                                      live && options.npu_workers>1);
        options.lanes_enabled = lane_toggle.requested;
        std::cout << "Lane model ready: " << ufld_lane_detector->profile()
                  << "; enabled=" << options.lanes_enabled << std::endl;
      } else {
        options.lanes_enabled = false;
        if (lane_toggle.requested) lane_toggle.fail(loaded.error);
        std::cerr << "车道线开启失败：" << loaded.error << std::endl;
      }
    }
    apply_test2_controls(vehicle_state_manager,lane_change_fsm,input,
        std::chrono::steady_clock::now(),options.lanes_enabled,!show_test4_panel && !show_navigation_panel);
    const auto frame_start = std::chrono::steady_clock::now();
    auto frame_captured = frame_start;
    double capture_ms = 0.0, wait_ms = 0.0, split_ms = 0.0;
    double compose_ms = 0.0;
    double render_ms = 0.0;
    const bool calibration_paused = calibration_target != CalibrationTarget::NONE &&
                                    !source_images[0].empty();
    std::array<cv::Mat, 4> frames;
    fresh_measurement.fill(false);
    bool ok = true;
    bool fresh_source = !calibration_paused;
    if (calibration_paused) { mouse_input.cancel(); signal_demo.stop(); }
    if (!calibration_paused) {
      if (live) {
        const auto result=pcie.poll_read(source_composite,frame_captured);
        ok=result!=adas::PcieCapture::ReadResult::CLOSED;
        fresh_source=result==adas::PcieCapture::ReadResult::FRAME;
        if (fresh_source) last_frame_captured=frame_captured;
        else frame_captured=last_frame_captured;
        const auto received_at = std::chrono::steady_clock::now();
        wait_ms = std::chrono::duration<double, std::milli>(received_at - frame_start).count();
        if (fresh_source) {
          adas::resize_preview(source_composite, live_preview);
          for (int i = 0; i < 4; ++i)
            live_preview(cv::Rect((i % 2) * 640, (i / 2) * 360, 640, 360)).copyTo(source_images[i]);
        }
        split_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - received_at).count();
      } else {
        ok = read_synchronized_group(streams, single_composite, source_composite, source_images);
      }
    }
    // 首帧尚未到达也显示界面；保留最后一帧时不重复送入推理。
    for (int i = 0; i < 4; ++i) {
      if (source_images[i].empty()) source_images[i]=cv::Mat(kTileH,kTileW,CV_8UC3,cv::Scalar());
      frames[i] = source_images[i].clone();
    }
    capture_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - frame_start).count();
    if (!ok) {
      if (live) { if (!quit_requested) { capture_failed = true; std::cerr << "PCIe: " << pcie.error() << std::endl; } break; }
      if (options.once) break;
      finish_lane_task();
      signal = adas::SignalResult();
      drive = adas::DriveResult();
      if (single_composite) streams[0].reset();
      else for (int i = 0; i < 4; ++i) streams[i].reset();
      yolo_rate.reset();
      ufld_rate.reset();
      source_frame_index = 0;
      test4_safety.reset(); signal_demo.stop();
      playback_epoch = std::chrono::steady_clock::now();
      signal_logic = adas::SignalLogic();
      drive_logic.reset();
      front_lane_detector.reset();
      rear_lane_detector.reset();
      lane = adas::LaneResult();
      rear_lane = adas::LaneResult();
      front_lane_target = adas::LaneResult();
      rear_lane_target = adas::LaneResult();
      front_neural_lane_tracker.reset(); front_lane_visual.reset(); front_outer_style.reset(); front_paint_display.reset();
      rear_neural_lane_tracker.reset(); rear_lane_visual.reset();
      front_lane_missed_updates = rear_lane_missed_updates = 0;
      front_risk_estimator.reset(); fcw_monitor.reset();
      rear_risk_estimator.reset();
      front_lane_warning.reset();
      rear_lane_warning.reset();
      left_blind_monitor.reset();
      right_blind_monitor.reset();
      for (auto& tracker : trackers) tracker.reset();
      for (auto& motion : box_motion) motion.reset();
      for (auto& view : detections) view.clear();
      continue;
    }
    if (!calibration_paused) {
    if (inference_running &&
        inference_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      InferenceResult result = inference_future.get();
      const auto tracking_now = std::chrono::steady_clock::now();
      ++object_results;
      object_result_ms = std::chrono::duration<double,std::milli>(tracking_now-result.captured_at).count();
      if (tracking_now - result.captured_at < std::chrono::milliseconds(500)) {
        if(object_received_at!=std::chrono::steady_clock::time_point()) {
          object_gap_ms=std::chrono::duration<double,std::milli>(tracking_now-object_received_at).count();
          max_object_gap_ms=std::max(max_object_gap_ms,object_gap_ms);
        }
        object_received_at=tracking_now;
        yolo_rate.record();
        object_result_ms = std::chrono::duration<double, std::milli>(
            tracking_now - result.captured_at).count();
        for (int i = 0; i < 4; ++i) {
          object_raw_counts[i]=result.detections[i].size();
          detections[i] = trackers[i].update(result.detections[i], result.captured_at, tracking_now);
          observed[i] = trackers[i].observations();
          if(live)box_motion[i].observe(result.motion_reference[i],observed[i],result.captured_at);
          detection_seconds[i] = result.source_seconds;
          fresh_measurement[i] = true;
        }
        last_detection_at = result.captured_at;
        npu_ms = result.npu_ms;
        hsv_checked = result.hsv_checked;
        hsv_rejected = result.hsv_rejected;
        hsv_ms = result.hsv_ms;
        signal = signal_logic.update(verify_colors ? result.verified_lights : observed[0],
                                     frames[0].cols, frames[0].rows);
        drive = drive_logic.update(signal);
        // 保留原始位置及 HSV 筛选结果，只补 ID；预测框绝不作为新灯态证据。
        current_lights = trackers[0].identify_measurements(
            verify_colors ? result.verified_lights : result.detections[0]);
        vehicle_state = vehicle_state_manager.snapshot(tracking_now);
        accept_demo_signal(result.source_seconds);
        {
          for(int view=1;view<4;++view) {
            extra_demos[view-1].observe(result.source_seconds,result.verified_views[view],640,360);
            std::cout << "SIGNAL_VIEW v=" << view << " t=" << result.source_seconds
                << " action=" << static_cast<int>(extra_demos[view-1].action()) << std::endl;
          }
        }

      } else ++stale_object_drops;
      inference_running = false;
    }
    const auto tracking_now = std::chrono::steady_clock::now();
    for (int i = 0; i < 4; ++i) detections[i] = trackers[i].predict(tracking_now);
    if(options.dump_detections) {
      std::cout<<"TRACK_FRAME frame="<<shown_frames<<" result="<<object_results
          <<" dropped="<<stale_object_drops<<" source_age_ms="
          <<(last_detection_at==std::chrono::steady_clock::time_point()?-1.0:
             std::chrono::duration<double,std::milli>(tracking_now-last_detection_at).count())
          <<" receipt_age_ms="
          <<(object_received_at==std::chrono::steady_clock::time_point()?-1.0:
             std::chrono::duration<double,std::milli>(tracking_now-object_received_at).count())
          <<" lamps=";
      for(int i=0;i<4;++i) {
        const auto count=[](const std::vector<adas::Detection>& values) {
          return std::count_if(values.begin(),values.end(),[](const adas::Detection& d) {
            return d.class_id>=7 && d.class_id<=14;
          });
        };
        std::cout<<(i?",":"")<<count(observed[i])<<':'<<count(detections[i]);
      }
      std::cout<<std::endl;
    }
    // Run the full PC estimator on a worker so its 40-100 ms CPU pass cannot
    // stall video presentation. The worker alternates front/rear and always
    // consumes the newest available frame.
    if (lane_running &&
        lane_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      LaneTaskResult result = lane_future.get();
      lane_running = false;
      lane_task_uses_npu = false;
      const bool current_lane_task = lane_toggle.accepts(options.lanes_enabled, result.generation);
      if (current_lane_task) last_lane_ms = result.elapsed_ms;
      const auto lane_received_at=std::chrono::steady_clock::now();
      lane_complete_age_ms=std::chrono::duration<double,std::milli>(result.completed_at-result.captured_at).count();
      lane_poll_delay_ms=std::chrono::duration<double,std::milli>(lane_received_at-result.completed_at).count();
      const bool fresh_lane = adas::lane_result_displayable(
          result.captured_at,result.completed_at,lane_received_at);
      if (current_lane_task && fresh_lane) {
        if (result.neural && result.camera == 0) ufld_rate.record();
        const int c = result.camera;
        auto& target = c == 0 ? front_lane_target : rear_lane_target;
        auto& monitor = c == 0 ? front_lane_warning : rear_lane_warning;
        auto& misses = c == 0 ? front_lane_missed_updates : rear_lane_missed_updates;
        auto& neural_tracker = c == 0 ? front_neural_lane_tracker
                                      : rear_neural_lane_tracker;
        // UFLD already fits curves, so only bridge short dropouts. The OpenCV
        // fallback also rejects implausible lane-width jumps caused by cars.
        const adas::LaneResult measured = scale_lane_result(
            result.lane, result.image_width, result.image_height,
            frames[c].cols, frames[c].rows);
        if (c == 0) front_lane_observed = measured;
        adas::LaneResult geometry = result.neural ?
            neural_tracker.update(measured) : stabilize_lane(
                measured, target, misses,
                lane_blocked_by_vehicle(detections[c], frames[c].cols, frames[c].rows));
        // Use the video timeline for LDW hold/clear times.  Wall-clock NPU
        // scheduling varies between replays of the same scene.
        const bool decision_fresh=adas::lane_decision_fresh(result.captured_at,lane_received_at);
        if(decision_fresh)target = monitor.update(geometry, result.media_at);
        else {target=geometry;target.departure=false;monitor.reset();}
        (c==0?front_lane_visual:rear_lane_visual).update(
            measured,target,result.media_at,frames[c].cols,frames[c].rows,
            result.media_at+(lane_received_at-result.captured_at));
        if (c == 0) {
          lane_semantic = lane_semantic_tracker.update(
              result.markings, target, result.media_at);
          front_outer_style.update(result.markings,target,
              live?result.media_at+(lane_received_at-result.captured_at):result.media_at);
        }
        lane_captured_at[c] = result.captured_at;
        lane_source_frame[c] = result.source_frame;
      } else if (current_lane_task) ++stale_lane_drops;
    }
    if (lane_backend_reset_pending && !lane_running) {
      front_lane_detector.reset(); rear_lane_detector.reset();
      lane_backend_reset_pending = false;
    }
    // 先让现有 YOLO/车道任务退出，再初始化新 RKNN 上下文。加载期间暂停提交
    // NPU 任务，避免模型初始化与共享 NPU 队列里的推理互相干扰。
    if (lane_toggle.needs_model(options.lanes_enabled) && !lane_toggle.loading &&
        !inference_running && !lane_running) {
      const std::string path = options.ufld_model;
      try {
        lane_model_future = std::async(std::launch::async, [path,live]() {
          return load_lane_model(path,live);
        });
        lane_toggle.loading = true;
      } catch (const std::exception& error) {
        lane_toggle.fail(error.what());
        std::cerr << "车道线模型加载线程启动失败：" << error.what() << std::endl;
      }
    }
    const int selected_task = options.lanes_enabled && ufld_lane_detector && !options.separate ?
        adas::next_perception_task(task_launched,
                                   options.ufld_rear && options.rear_lane_enabled,
                                   perception_periods) : -1;
    const bool detector_pending = selected_task >= 0 ? selected_task == 0 :
        shown_frames - last_detector_start >= options.detect_every;
    if (fresh_source && detector_pending && !inference_running &&
        !lane_toggle.reserves_npu(options.lanes_enabled) &&
        (live || !lane_running || !lane_task_uses_npu)) {
      last_detector_start = shown_frames;
      if (task_launched[0] != std::chrono::steady_clock::time_point())
        object_interval_ms = std::chrono::duration<double, std::milli>(
            frame_start - task_launched[0]).count();
      task_launched[0] = frame_start;
      if (options.separate) {
        const int camera = (shown_frames / options.detect_every) % 4;
        const auto measured_at = std::chrono::steady_clock::now();
        detections[camera] = trackers[camera].update(
            detector.detect(frames[camera], &npu_ms), measured_at, measured_at);
        yolo_rate.record();
        observed[camera] = trackers[camera].observations();
        detection_seconds[camera] = (live ? std::chrono::duration<double>(frame_captured - start).count() : source_frame_index / source_fps);
        fresh_measurement[camera] = true;
        last_detection_at = measured_at;
        if(camera>0) {
          const auto verified_view=adas::verify_light_colors(frames[camera],observed[camera]);
          extra_demos[camera-1].observe(detection_seconds[camera],verified_view,640,360);
        }
        if (camera == 0) {
          const auto color_start = std::chrono::steady_clock::now();
          const auto verified = adas::verify_light_colors(frames[0], observed[0], &hsv_checked, &hsv_rejected);
          hsv_ms = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-color_start).count();
          signal = signal_logic.update(verify_colors ? verified : observed[0], frames[0].cols, frames[0].rows);
          drive = drive_logic.update(signal);
          current_lights = verify_colors ? verified : observed[0];
          vehicle_state = vehicle_state_manager.snapshot(measured_at);
          accept_demo_signal(detection_seconds[0]);
  
        }
      } else {
        const bool fpga_composite = !options.legacy_mosaic;
        const auto compose_start = std::chrono::steady_clock::now();
        // Replay composites already have the correct quadrant layout. Avoid
        // splitting, enlarging to 1080p, then shrinking again for the detector.
        cv::Mat inference_image = live ? source_composite :
            (fpga_composite && single_composite ? source_composite.clone() :
             (fpga_composite ? make_fpga_composite(frames) : make_mosaic(frames)));
        compose_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - compose_start).count();
        const auto captured_at = frame_captured;
        const double source_seconds = (live ? std::chrono::duration<double>(frame_captured - start).count() : source_frame_index / source_fps);
        std::array<cv::Mat,4> light_frames;
        if(!live)for(int view=0;view<4;++view)light_frames[view]=frames[view].clone();
        const bool verify_extra_views=true;
        auto infer_objects =
            [&detector, &pcie, live, start, inference_image, captured_at, fpga_composite, source_seconds, light_frames, verify_extra_views]() mutable {
              if(live)adas::bind_cpu_pair(2,"object-worker");
              InferenceResult result;
              result.captured_at = captured_at;
              result.source_seconds = source_seconds;
              adas::NpuRunGuard input_slot(adas::npu_run_mutex(),std::defer_lock);
              if(live) {
                // Admit the task before selecting its input. Waiting behind
                // another context must not age a frame we have yet to infer.
                input_slot.lock();
                if(!pcie.latest_frame(inference_image,result.captured_at))return result;
                result.source_seconds=std::chrono::duration<double>(result.captured_at-start).count();
              }
              std::vector<adas::Detection> combined =
                  detector.detect(inference_image, &result.npu_ms);
              if(input_slot.owns_lock())input_slot.unlock();
              if (fpga_composite && (inference_image.cols != kFpgaCompositeW ||
                                    inference_image.rows != kFpgaCompositeH)) {
                const float sx=float(kFpgaCompositeW)/inference_image.cols;
                const float sy=float(kFpgaCompositeH)/inference_image.rows;
                for(auto& d:combined) {
                  d.box.x*=sx; d.box.width*=sx;
                  d.box.y*=sy; d.box.height*=sy;
                }
              }
              result.detections = fpga_composite
                  ? split_fpga_composite(combined) : split_mosaic(combined);
              if(live && fpga_composite)for(int view=0;view<4;++view) {
                bool needed=false;
                for(const auto& d:result.detections[view])
                  if(adas::BoxMotionCompensator::ordinary(d.class_id) && d.score>=.45f)needed=true;
                if(needed) {
                  const cv::Rect roi((view%2)*(inference_image.cols/2),(view/2)*(inference_image.rows/2),inference_image.cols/2,inference_image.rows/2);
                  result.motion_reference[view]=adas::BoxMotionCompensator::reference(inference_image(roi));
                }
              }
              if(live)adas::prepare_light_verification_frames(inference_image,result.detections,light_frames);
              const auto color_start = std::chrono::steady_clock::now();
              result.verified_lights = adas::verify_light_colors(light_frames[0],result.detections[0],
                  &result.hsv_checked,&result.hsv_rejected);
              if(verify_extra_views)for(int view=1;view<4;++view)
                result.verified_views[view]=adas::verify_light_colors(light_frames[view],result.detections[view]);
              result.hsv_ms = std::chrono::duration<double,std::milli>(
                  std::chrono::steady_clock::now()-color_start).count();
              return result;
            };
        inference_future = live ? npu_executor.submit(infer_objects) :
            std::async(std::launch::async,infer_objects);
        inference_running = true;
      }
      if (options.dump_detections && shown_frames < 4 * options.detect_every) {
        for (int i = 0; i < 4; ++i) {
          std::cout << "view=" << kFiles[i] << " frame=" << source_frame_index
                    << " detections=" << detections[i].size();
          for (const auto& d : detections[i])
            std::cout << ' ' << d.name << ':' << std::fixed << std::setprecision(2) << d.score;
          std::cout << '\n';
        }
      }
    }
    if (fresh_source && options.lanes_enabled && !lane_running && !lane_toggle.loading) {
      int camera = -1;
      bool use_ufld_task = false;
      if (ufld_lane_detector) {
        // The live executor serializes the contexts and starts pending work
        // without waiting for the next display refresh. Re-evaluate after an
        // object submission so a lane job can be ready behind that object job.
        const int lane_task = live ? adas::next_perception_task(task_launched,
            options.ufld_rear && options.rear_lane_enabled,perception_periods) : selected_task;
        if ((live || !inference_running) && (live ? lane_task!=0 : !detector_pending) &&
            (lane_task >= 0 || shown_frames - last_ufld_start >= options.ufld_every)) {
          camera = lane_task >= 0 ? lane_task - 1 :
              (options.ufld_rear && options.rear_lane_enabled &&
               (++ufld_task_sequence % 3 == 0) ? 1 : 0);
          use_ufld_task = true;
          last_ufld_start = shown_frames;
        }
      } else {
        // Rear geometry requires neural evidence; never fall back to the
        // traditional estimator when the model is absent.
        camera = 0;
      }
      if (camera >= 0) {
        task_launched[camera + 1] = frame_start;
        cv::Mat lane_frame = frames[camera].clone();
        // Preserve the FPGA composite's native 960x540 front pixels for the
        // full-width UFLDv2 model. UI tiles remain 640x360 and receive scaled
        // lane geometry after inference.
        if (camera < 2 && single_composite &&
            source_composite.cols >= kFpgaCompositeW &&
            source_composite.rows >= kFpgaCompositeH) {
          lane_frame = source_composite(cv::Rect(camera * (source_composite.cols / 2), 0, source_composite.cols / 2,
                                                 source_composite.rows / 2));
          // Live published frames are immutable. Keep the strided ROI: this
          // also bypasses unsupported RGA virtual-address blits upstream.
          // File capture may reuse its backing buffer, so it needs a copy.
          if (!live) lane_frame = lane_frame.clone();
        }
        const auto task_captured_at = frame_captured;
        const int task_source_frame = source_frame_index;
        const auto task_media_at = std::chrono::steady_clock::time_point(
            std::chrono::seconds(1)) +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>((live ? std::chrono::duration<double>(frame_captured - start).count() : task_source_frame / source_fps)));
        adas::UfldLaneDetector* ufld = ufld_lane_detector.get();
        const unsigned lane_generation = lane_toggle.generation;
        auto infer_lane =
            [&front_lane_detector, &pcie, live, start, ufld, lane_frame, camera,
             use_ufld_task, task_captured_at, task_media_at, task_source_frame, lane_generation]() {
              if(live)adas::bind_cpu_pair(0,"npu-worker");
              const auto started = std::chrono::steady_clock::now();
              LaneTaskResult result;
              result.camera = camera;
              result.neural = use_ufld_task;
              result.generation = lane_generation;
              result.captured_at = task_captured_at;
              result.media_at = task_media_at;
              result.source_frame = task_source_frame;
              cv::Mat input=lane_frame;
              adas::NpuRunGuard input_slot(adas::npu_run_mutex(),std::defer_lock);
              if(live) {
                if(use_ufld_task)input_slot.lock();
                cv::Mat latest;
                if(!pcie.latest_frame(latest,result.captured_at)) {
                  result.completed_at=std::chrono::steady_clock::now();return result;
                }
                input=latest(cv::Rect(camera*(latest.cols/2),0,latest.cols/2,latest.rows/2));
                result.media_at=std::chrono::steady_clock::time_point(std::chrono::seconds(1))+
                    (result.captured_at-start);
              }
              result.image_width = input.cols;
              result.image_height = input.rows;
              if (use_ufld_task) {
                result.lane = ufld->detect(input);
              } else if (camera == 0) {
                result.lane = front_lane_detector.detect(input);
              }
              if(input_slot.owns_lock())input_slot.unlock();
              if (camera == 0) {
                const auto supported=adas::LaneVisualTracker::supported_observation(result.lane,input.cols,input.rows);
                result.markings = adas::observe_lane_markings_contrast(input, supported);
              }
              result.completed_at=std::chrono::steady_clock::now();
              result.elapsed_ms = std::chrono::duration<double, std::milli>(
                  result.completed_at - started).count();
              return result;
            };
        lane_future = live && use_ufld_task ? npu_executor.submit(infer_lane) :
            std::async(std::launch::async,infer_lane);
        lane_running = true;
        lane_task_uses_npu = use_ufld_task;
      }
    }
    const auto presentation_now = std::chrono::steady_clock::now();
    for (int c = 0; c < 2; ++c) {
      auto& target = c == 0 ? front_lane_target : rear_lane_target;
      lane_age_ms[c] = lane_source_frame[c] >= 0 ?
          std::chrono::duration<double, std::milli>(presentation_now - lane_captured_at[c]).count() : -1.0;
      // Front UFLD shares one RK3568 NPU queue with YOLO. A valid update can
      // occasionally arrive just after 600 ms; clearing at 600 ms caused the
      // exact one-frame blank/yellow flash seen on screen.
      if (lane_age_ms[c] > (c == 0 ? 1000.0 : 1400.0)) {
        target = adas::LaneResult();
        (c == 0 ? front_lane_warning : rear_lane_warning).reset();
        (c == 0 ? front_neural_lane_tracker : rear_neural_lane_tracker).reset();
        if (c == 0) {
          lane_semantic.lane_valid = false;
          lane_semantic.left_change_allowed = false;
          lane_semantic.right_change_allowed = false;
          lane_semantic.crossing = adas::CrossingSide::NONE;
          lane_semantic.trend = adas::CrossingSide::NONE;
          lane_semantic.tlc_s = -1.0f;
        }
      }
    }
    lane = front_lane_target;
    rear_lane = rear_lane_target;
    // Retain internal geometry/semantic tracking for Test Two, while disabling
    // the automatic LDW alarm on every view and dashboard snapshot.
    suppress_lane_departure_alarm(lane);
    suppress_lane_departure_alarm(rear_lane);
    // Display hold must never authorize a lane change or a new LDW alert.
    if(!adas::lane_decision_fresh(lane_captured_at[1],presentation_now))rear_lane.departure=false;
    const auto detection_age = last_detection_at == std::chrono::steady_clock::time_point()
        ? std::chrono::steady_clock::duration::max()
        : presentation_now - last_detection_at;
    if (detection_age > std::chrono::milliseconds(800)) {
      signal_logic = adas::SignalLogic();
      drive_logic.reset();
      signal = adas::SignalResult();
      drive = adas::DriveResult();
      front_risk_estimator.reset(); fcw_monitor.reset();
      rear_risk_estimator.reset();
      current_lights.clear();
        intersection_result = adas::IntersectionResult();
    }
    // High-resolution UFLD shares one NPU queue with four-view YOLO. Object
    // updates may be more than 800 ms apart, so preserve the two-observation
    // BSD confirmation across that gap. Side warnings still disappear as soon
    // as their own freshness limit is reached.
    if (detection_age > std::chrono::milliseconds(2500)) {
      left_blind_monitor.reset();
      right_blind_monitor.reset();
    }
    if(options.ttc_enabled) {
    front_risk = front_risk_estimator.update(
        observed[0], frames[0].cols, frames[0].rows, &lane, fresh_measurement[0], detection_seconds[0]);
    rear_risk = rear_risk_estimator.update(
        observed[1], frames[1].cols, frames[1].rows, &rear_lane, fresh_measurement[1], detection_seconds[1]);
    } else { front_risk=adas::RiskResult();rear_risk=adas::RiskResult(); }
    left_blind=adas::BlindSpotResult();right_blind=adas::BlindSpotResult();
    }
    const auto fcw_started=std::chrono::steady_clock::now();
    const double fcw_now=live?std::chrono::duration<double>(fcw_started-start).count():source_frame_index/source_fps;
    fcw_monitor.set_enabled(options.fcw_enabled);
    const auto fcw_state=fcw_monitor.update(observed[0],frames[0].cols,frames[0].rows,lane,
        options.lanes_enabled && !calibration_paused && lane_age_ms[0]>=0 && lane_age_ms[0]<=650.,
        fresh_measurement[0],detection_seconds[0],fcw_now);
    const auto fcw_paint=fcw_paint_tracker.update(fcw_state,fcw_now);
    // The front FCW switch owns front collision warnings independently of RCW.
    front_risk=options.fcw_enabled?fcw_state.risk:adas::RiskResult();
    fcw_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-fcw_started).count();
    const auto advice_now = std::chrono::steady_clock::now();
    adas::LaneSemantic decision_semantic=lane_semantic;
    const adas::LaneSemantic rule_semantic=test2_rule_evidence(
        lane_semantic,lane,lane_captured_at[0],advice_now);
    if(!adas::lane_decision_fresh(lane_captured_at[0],advice_now)) {
      lane.departure=false;decision_semantic=adas::LaneSemantic();
    }
    adas::LaneSemantic display_semantic=lane_semantic;
    display_semantic.crossing=decision_semantic.crossing;
    display_semantic.trend=decision_semantic.trend;
    display_semantic.tlc_s=decision_semantic.tlc_s;
    display_semantic.left_change_allowed=decision_semantic.left_change_allowed;
    display_semantic.right_change_allowed=decision_semantic.right_change_allowed;
    display_semantic.crossing=adas::CrossingSide::NONE;
    display_semantic.trend=adas::CrossingSide::NONE;
    display_semantic.tlc_s=-1.f;
    vehicle_state = vehicle_state_manager.snapshot(advice_now);
    // 用途车道始终有效；灯态只由新观测确认，UI 重绘仅负责过期。
    const double demo_time=live?std::chrono::duration<double>(advice_now-start).count():source_frame_index/source_fps;
    const auto test4_result=test4_safety.update(observed[0],frames[0].cols,frames[0].rows,&lane,
        adas::lane_decision_fresh(lane_captured_at[0],advice_now),fresh_measurement[0],detection_seconds[0],demo_time);
    apply_test4_interlock(test4_result,signal_demo);
    const int test4_signature=int(test4_result.enabled)+2*int(test4_result.brake)+
        4*int(test4_result.stale)+8*int(test4_result.can_resume)+16*int(test4_result.hazard)+
        64*int(test4_result.hazards.size());
    if (test4_signature!=test4_saved_signature) {
      test4_saved_signature=test4_signature;
      std::cout<<"TEST4_STATE t="<<demo_time<<" enabled="<<test4_result.enabled
          <<" brake="<<test4_result.brake<<" stale="<<test4_result.stale
          <<" resume="<<test4_result.can_resume<<" hazard="<<int(test4_result.hazard)
          <<" track="<<test4_result.track_id<<" hazards="<<test4_result.hazards.size()
          <<" moving="<<signal_demo.moving()<<std::endl;
    }
    if (test4_result.enabled && test4_result.brake) {
      vehicle_state_manager.set_demo_turn(adas::TurnSignal::OFF,advice_now);
      vehicle_state=vehicle_state_manager.snapshot(advice_now);
      lane_change_fsm.request(adas::ManeuverDirection::NONE,advice_now);
    }
    signal_demo.tick(demo_time);
    navigation_signal.set_lane(navigation_demo.current_route());
    if(navigation_demo.enabled() && fresh_measurement[0])
      navigation_signal.observe(detection_seconds[0],current_lights,frames[0].cols,frames[0].rows);
    navigation_signal.tick(demo_time);
    const auto navigation_advice=navigation_demo.advice(navigation_signal.action(),test4_result.enabled && test4_result.brake);
    intersection_result=signal_demo.intersection();
    signal=intersection_result.selected_signal;
    const double overlay_time=live?std::chrono::duration<double>(advice_now-start).count():source_frame_index/source_fps;
    for(auto& demo:extra_demos){demo.set_direction(signal_demo.direction());demo.tick(overlay_time);}
    const bool side_fresh = last_detection_at != std::chrono::steady_clock::time_point() &&
        advice_now - last_detection_at < std::chrono::milliseconds(1800);
    lane_change_result = options.lanes_enabled ? lane_change_fsm.update_rules(
        vehicle_state, rule_semantic, advice_now) : adas::LaneChangeResult();
    warning_summary = warning_manager.update(
        front_risk.warning, rear_risk.warning, side_fresh && left_blind.occupied,
        side_fresh && right_blind.occupied, false /* LDW disabled */, lane_change_result,
        signal_demo.motion_intersection());
    apply_signal_demo_warning(signal_demo, warning_summary);
    if (test4_result.enabled && test4_result.brake) {
      warning_summary.primary=adas::WarningCode::FCW; warning_summary.priority=110;
      warning_summary.text=test4_reason(test4_result);
      warning_summary.active.push_back(adas::WarningCode::FCW);
    }
    auto view_levels = view_alert_levels(front_risk, rear_risk, lane,
        decision_semantic, left_blind, right_blind, side_fresh, vehicle_state);
    if(options.fcw_enabled)view_levels[0]=fcw_state.level==adas::FcwLevel::DANGER?3:
        fcw_state.level==adas::FcwLevel::CAUTION?1:0;
    if (test4_result.enabled && test4_result.brake) view_levels[0]=3;
    const auto render_start = std::chrono::steady_clock::now();
    const auto visual_at=std::chrono::steady_clock::time_point(std::chrono::seconds(1))+
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(
            live?std::chrono::duration<double>(advice_now-start).count():source_frame_index/source_fps));
    const adas::LaneResult visual_lane=front_lane_visual.render(lane,visual_at);
    front_outer_style.apply(display_semantic,visual_lane,visual_at);
    const adas::LaneResult center_lane=center_lane_display(visual_lane);
    const auto paint_display=front_paint_display.update(display_semantic,center_lane,visual_at);
    // Only adjust the boxes being drawn, after all safety decisions consumed
    // measured evidence. Invisible/expired ByteTrack entries stay invisible.
    const auto box_motion_start=std::chrono::steady_clock::now();box_motion_corrected=0;
    if(live)for(int view=0;view<4;++view)
      box_motion_corrected+=box_motion[view].apply(source_images[view],frame_captured,detections[view]);
    box_motion_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-box_motion_start).count();
    // Identity checks may hold a previous lane while fresh geometry changes
    // at a merge or marking transition. Show only the CURRENT measured edges
    // as subdued references; never draw the held pair as a confirmed corridor.
    if (options.lanes_enabled && lane.identity_uncertain &&
        lane_age_ms[0] >= 0.0 && lane_age_ms[0] < 650.0)
      adas::draw_lane_geometry_reference(frames[0], front_lane_observed);
    adas::draw_overlay(frames[0], detections[0], center_lane, intersection_result.selected_signal, front_risk,
                       adas::DriveResult(), yolo_rate.fps(), npu_ms, &display_semantic, &paint_display,
                       options.fcw_enabled?&fcw_paint:nullptr);
    if(signal_overlay_visible)draw_signal_demo_view(frames[0],signal_demo,true,false,detections[0],signal_demo.direction_matches_lane(),signal_demo.alarm_title());
    if (options.lanes_enabled) draw_lane_geometry(frames[1], rear_lane);
    draw_boxes(frames[1], detections[1], &rear_risk);
    const bool left_intent = vehicle_state.turn_valid &&
        vehicle_state.turn == adas::TurnSignal::LEFT;
    const bool right_intent = vehicle_state.turn_valid &&
        vehicle_state.turn == adas::TurnSignal::RIGHT;
    if(options.ttc_enabled) draw_rear_risk_overlay(frames[1], rear_risk, detections[1]);
    draw_boxes(frames[2], detections[2], nullptr, side_fresh && left_blind.occupied ? left_blind.track_id : -1,
               left_blind.occupied && left_intent);
    draw_boxes(frames[3], detections[3], nullptr, side_fresh && right_blind.occupied ? right_blind.track_id : -1,
               right_blind.occupied && right_intent);
    if(signal_overlay_visible)
      for(int view=1;view<4;++view)draw_signal_demo_view(frames[view],extra_demos[view-1],false,false,detections[view]);
    if (signal_overlay_visible && options.lanes_enabled && !test4_result.brake) draw_maneuver_hud(frames[0], lane_change_result, rule_semantic);
    draw_test4_view(frames[0],test4_result,detections[0]);
    // Navigation stays in the dashboard; keep the front camera image clear.
    if(options.lanes_enabled) {
      static std::string marking_label;
      static cv::Mat marking_footer;
      const std::string label=std::string("左边界：")+adas::lane_marking_caption(display_semantic.left)+
          "    右边界："+adas::lane_marking_caption(display_semantic.right);
      if(marking_footer.empty() || label!=marking_label) {
        marking_label=label;marking_footer=cv::Mat(24,640,CV_8UC3,cv::Scalar(18,23,27));
        caption(marking_footer,label,8,18,cv::Scalar(225,230,230),.44);
      }
      marking_footer.copyTo(frames[0](cv::Rect(0,frames[0].rows-24,640,24)));
    }
    if (rear_risk.warning) warning_banner(frames[1], "后车快速接近  TTC预警");
    if (side_fresh && left_blind.occupied && left_intent)
      warning_banner(frames[2], "左侧盲区有目标");
    if (side_fresh && right_blind.occupied && right_intent)
      warning_banner(frames[3], "右侧盲区有目标");
    if (calibration_target == CalibrationTarget::FRONT)
      draw_calibration(frames[0], calibration_points, "前视");
    if (calibration_target == CalibrationTarget::REAR)
      draw_calibration(frames[1], calibration_points, "后视");
    if (!gl_presenter) {
      for (int i = 0; i < 4; ++i) draw_tile(canvas, frames[i], i);
    }
    // Text rasterization is expensive on Cortex-A55. Camera textures and all
    // safety overlays still refresh every frame; dashboard text and timeline
    // refresh at one third of the video rate.
    const int demo_signature=signal_demo_signature(signal_demo,signal_overlay_visible);
    const int lane_signature=lane_toggle.signature(options.lanes_enabled);
    const bool refresh_dashboard = shown_frames < 2 || shown_frames % 3 == 0 ||
        demo_signature!=dashboard_demo_signature || lane_signature!=dashboard_lane_signature;
    draw_global_warning(canvas, warning_summary,
        options.fcw_enabled && fcw_state.level!=adas::FcwLevel::DANGER?adas::RiskResult():front_risk, rear_risk,
                        view_levels, normal_header);
    if (refresh_dashboard) {
      dashboard_demo_signature=demo_signature;
      dashboard_lane_signature=lane_signature;
      draw_lane_detection_toggle(canvas,lane_toggle,options.lanes_enabled);
      // PCIe 没有文件 FPS，用实际运行时间判断启动阶段，避免以 0 FPS 计时。
      const bool health_ready = live ? frame_start - start >= std::chrono::seconds(2) :
          shown_frames > std::max(10, static_cast<int>(std::max(1.0, source_fps) * 2.0));
      draw_sidebar(canvas, display_fps, yolo_rate.fps(), ufld_rate.fps(), npu_ms, signal, lane, rear_lane,
                   options.rear_lane_enabled,
                   front_risk, rear_risk, left_blind, right_blind, side_fresh,
                   health_ready,
                   last_lane_ms, lane_age_ms, vehicle_state, display_semantic,
                   lane_change_result, intersection_result, verify_colors, options.lanes_enabled, options.ttc_enabled);
      draw_bottom_left(canvas, scene_index,
                       static_cast<int>(scene_paths.size()),
                       options.scene.substr(options.scene.find_last_of("/\\") + 1), live, detector.two_stage(), options.lanes_enabled);
      caption(canvas, warning_summary.priority > 0 ? warning_summary.text : lane_change_result.reason,
          1404,1052, warning_summary.priority >= 70 ? cv::Scalar(80,80,245)
                                                    : cv::Scalar(215,188,130),.46);
      if (!live) draw_progress_bar(canvas, source_frame_index, source_frames, calibration_paused);
    }
    draw_signal_demo_controls(canvas,signal_demo,signal_overlay_visible);
    draw_test2_controls(canvas,lane_change_result,rule_semantic,options.lanes_enabled);
    draw_view_alerts(canvas, view_levels);
    // AVM warps run on a worker. The camera and UI presentation never wait for
    // a new BEV image; a late result simply stays on the last known preview.
    if (avm_running && avm_future.wait_for(std::chrono::milliseconds(0)) ==
                           std::future_status::ready) {
      avm_preview = avm_future.get();
      avm_running = false;
      avm_changed = true;
    }
    if (!avm_running && avm.count() > 0 && !calibration_paused &&
        frame_start - avm_updated >= std::chrono::milliseconds(500)) {
      std::array<cv::Mat,4> avm_input;
      for (int i=0; i<4; ++i) avm_input[i] = source_images[i].clone();
      const auto worker=avm_worker;
      const int avm_frame=source_frame_index;
      avm_future = std::async(std::launch::async, [worker,avm_input,avm_frame]() {
        return worker->render(avm_input,avm_frame);
      });
      avm_running = true;
      avm_updated = frame_start;
    }
    if (avm_preview.empty()) {
      avm_preview = avm.render(std::array<cv::Mat,4>());
      avm_changed = true;
    }
    if (!show_test4_panel && !show_navigation_panel && (refresh_dashboard || avm_changed)) {
      cv::rectangle(canvas,cv::Rect(1402,130,326,329),cv::Scalar(20,28,35),-1);
      cv::Mat avm_thumb;
      // Keep the same full ground extent as the PC AVM preview.
      cv::resize(avm_preview,avm_thumb,cv::Size(326,326),0,0,cv::INTER_AREA);
      avm_thumb.copyTo(canvas(cv::Rect(1402,132,326,326)));
      caption(canvas,"前 FRONT",1526,153,cv::Scalar(245,232,216),.37);
      caption(canvas,"后 REAR",1532,449,cv::Scalar(245,232,216),.37);
      caption(canvas,avm.count() ? (avm.visual_fill() ? "近似补位" : (avm.approximate() ? "近似投影" : "地面投影")) : "待标定",1740,158,cv::Scalar(90,210,245),.52);
      caption(canvas,std::to_string(avm.count()) + " 路相机",1740,184,cv::Scalar(190,195,200),.44);
      caption(canvas,avm.visual_fill() ? "纹理延展 / 车模放大" : "斜纹：盲区",1740,214,cv::Scalar(190,195,200),.43);
      caption(canvas,"V 放大查看",1740,247,cv::Scalar(190,195,200),.43);
      avm_changed = false;
    }
    if(show_navigation_panel) {
      if(refresh_dashboard)draw_navigation_controls(canvas,navigation_demo,navigation_advice);
    } else if (refresh_dashboard || !show_test4_panel) draw_test4_controls(canvas,test4_result,show_test4_panel);
    if(options.demo_navigation) {
      panel(canvas,kTest4Tab.x,kTest4Tab.y,kTest4Tab.width,kTest4Tab.height);
      caption(canvas,navigation_top_panel==0?"切换测试四":navigation_top_panel==1?"切换 AVM":"切换导航",
          kTest4Tab.x+10,kTest4Tab.y+21,cv::Scalar(245,201,70),.49);
    }
    const cv::Rect avm_button(800, 980, 220, 40);
    cv::rectangle(canvas, avm_button, cv::Scalar(24,15,7), cv::FILLED);
    cv::rectangle(canvas, avm_button, cv::Scalar(65,100,130), 1);
    caption(canvas, avm_visible ? "V 四路视频" : "V 2D环视 / AVM", 814,1007,cv::Scalar(240,220,170),.55);
    if (avm_visible) {
      cv::rectangle(canvas, cv::Rect(10,85,1365,870),cv::Scalar(20,28,35),-1);
      cv::Mat bird = avm_preview, enlarged;
      cv::resize(bird,enlarged,cv::Size(810,810));
      enlarged.copyTo(canvas(cv::Rect(285,125,810,810)));
      caption(canvas,"2D AVM  地面投影 / 近似纹理补位",30,112,cv::Scalar(240,220,170),.65);
      caption(canvas,std::string(avm.approximate() ? "估计参数 " : "已标定 ") + std::to_string(avm.count()) + " 路   车模放大 1.55 倍 / 补位为视觉近似",30,950,cv::Scalar(160,190,220),.55);
      if (!avm.count()) caption(canvas,"请加载当前素材组的 avm.yml 标定",360,500,cv::Scalar(70,210,255),.65);
    }
    // 最后画快捷菜单，使其覆盖侧栏，而不是被后续绘制的按钮盖住。
    draw_fcw_controls(canvas,fcw_state);
    image_toolbar.draw(canvas,image_status,live);
    render_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - render_start).count();
    const int snapshot_frame = options.snapshot_frame >= 0 ? options.snapshot_frame :
        (options.max_frames > 0 ? std::min(4, options.max_frames - 1) : 4);
    if (!options.snapshot.empty() && shown_frames == snapshot_frame) {
      cv::Mat snapshot = canvas;
      if (gl_presenter && !avm_visible) {
        snapshot = canvas.clone();
        for (int i = 0; i < 4; ++i) draw_tile(snapshot, frames[i], i);
      }
      if (!cv::imwrite(options.snapshot, snapshot)) std::cerr << "Could not save snapshot\n";
    }
    {
      const int signature=signal_demo_signature(signal_demo,signal_overlay_visible);
      if(signature!=demo_saved_signature) {
        demo_saved_signature=signature;
        std::cout << "SIGNAL_DEMO_STATE t=" << overlay_time
          << " action=" << static_cast<int>(signal_demo.action()) << " moving=" << signal_demo.moving()
          << " alarm=" << signal_demo.alarm() << " alarm_type=" << static_cast<int>(signal_demo.alarm_type())
          << " lane=" << signal_demo.lane_text()
          << " direction=" << signal_demo.direction_text() << std::endl;
        if(!options.demo_output.empty()) {
          cv::Mat picture=canvas.clone();
          if(gl_presenter && !avm_visible)for(int i=0;i<4;++i)draw_tile(picture,frames[i],i);
          const std::string name=signal_demo.alarm_type()==adas::LaneSignalSimulator::AlarmType::INVALID_DIRECTION?"invalid_direction":signal_demo.alarm()?"alarm":signal_demo.action()==adas::IntersectionAction::GO?"go":
              signal_demo.action()==adas::IntersectionAction::STOP?"stop":"wait";
          cv::imwrite(options.demo_output+"/"+(signal_overlay_visible?name:"hidden_"+name)+"_"+std::to_string(shown_frames)+".jpg",picture);
        }
      }
    }
    if (!options.headless) {
      int scene_delta = 0;
      if (gl_presenter)
        gl_presenter->present(canvas, avm_visible ? std::array<cv::Mat,4>() : frames, kViewRects);
      else cv::imshow("RK3568 V7 | NVIDIA 4 VIEW", canvas);
      if (key == 'v' || avm_button.contains(cv::Point(click_x,click_y))) {
        avm_visible = !avm_visible;
        if (!avm_visible) {
          // Remove the enlarged AVM from margins and camera-card headers.
          cv::rectangle(canvas,cv::Rect(10,85,1365,870),cv::Scalar(24,15,7),cv::FILLED);
          draw_camera_cards(canvas);
        }
        avm_changed = true;
      }
      if (!live && key == 'm') {
        options.separate = !options.separate;
        test4_safety.reset();
        signal_demo.reset();
        configure_extra_demos();
        for (auto& tracker : trackers) tracker.reset();
      for (auto& motion : box_motion) motion.reset();
        for (auto& view : detections) view.clear();
        signal_logic = adas::SignalLogic();
        drive_logic.reset();
        signal = adas::SignalResult();
        drive = adas::DriveResult();
        front_risk_estimator.reset(); fcw_monitor.reset();
        rear_risk_estimator.reset();
        left_blind_monitor.reset();
        right_blind_monitor.reset();
      }
      if (key == '[') scene_delta = -1;
      if (key == ']') scene_delta = 1;
      if (click_x >= 0 && click_y >= 0) {
        const cv::Point click(click_x, click_y);
        if (kSignalVerify.contains(click)) {
          verify_colors=!verify_colors;
          signal_logic=adas::SignalLogic(); drive_logic.reset();
          signal=adas::SignalResult(); drive=adas::DriveResult();
          std::cout << "HSV verification=" << verify_colors << std::endl;
        }
        if (kPrevSceneButton.contains(click)) scene_delta = -1;
        if (kNextSceneButton.contains(click)) scene_delta = 1;
      }
      if (options.lanes_enabled && !avm_visible && (key == 'c' || key == 'r')) {
        finish_lane_task();
        calibration_target = key == 'c' ? CalibrationTarget::FRONT : CalibrationTarget::REAR;
        calibration_points.clear();
      }
      if (!calibration_paused && source_frames > 1 &&
          click_x >= kProgressBar.x && click_x <= kProgressBar.x + kProgressBar.width &&
          click_y >= kProgressBar.y - 10 &&
          click_y <= kProgressBar.y + kProgressBar.height + 10) {
        const float ratio = std::max(0.0f, std::min(1.0f,
            (click_x - kProgressBar.x) / static_cast<float>(kProgressBar.width)));
        const int target = static_cast<int>(ratio * (source_frames - 1));
        finish_lane_task();
        bool seek_ok = streams[0].seek(target);
        if (!single_composite) {
          for (int i = 1; i < 4; ++i) seek_ok = streams[i].seek(target) && seek_ok;
        }
        if (seek_ok) {
          signal = adas::SignalResult();
          drive = adas::DriveResult();
          source_frame_index = target;
          playback_epoch = std::chrono::steady_clock::now() -
              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                  std::chrono::duration<double>(target / source_fps));
          avm_preview.release();
          avm_changed = true;
          for (auto& tracker : trackers) tracker.reset();
      for (auto& motion : box_motion) motion.reset();
          for (auto& view : detections) view.clear();
          signal_logic = adas::SignalLogic();
          drive_logic.reset();
          front_risk_estimator.reset(); fcw_monitor.reset();
          rear_risk_estimator.reset();
          front_lane_warning.reset();
          rear_lane_warning.reset();
          left_blind_monitor.reset();
          right_blind_monitor.reset();
          lane = adas::LaneResult();
          rear_lane = adas::LaneResult();
          front_lane_target = adas::LaneResult();
          rear_lane_target = adas::LaneResult();
          front_neural_lane_tracker.reset(); front_lane_visual.reset(); front_outer_style.reset(); front_paint_display.reset();
          rear_neural_lane_tracker.reset(); rear_lane_visual.reset();
          front_lane_missed_updates = rear_lane_missed_updates = 0;
        }
      }
      if (calibration_target != CalibrationTarget::NONE && click_x >= 0 && click_y >= kHeaderH) {
        const auto& view = kViewRects[calibration_target == CalibrationTarget::FRONT ? 0 : 1];
        if (view.contains(cv::Point(click_x, click_y))) {
          calibration_points.emplace_back((click_x - view.x) / static_cast<float>(view.width),
                                          (click_y - view.y) / static_cast<float>(view.height));
          if (calibration_points.size() == 4) {
            calibration_points = order_roi(calibration_points);
            if (valid_roi(calibration_points)) {
              finish_lane_task();
              adas::LaneDetector& lane_detector = calibration_target == CalibrationTarget::FRONT
                  ? front_lane_detector : rear_lane_detector;
              lane_detector.set_roi(calibration_points[0], calibration_points[1],
                                    calibration_points[2], calibration_points[3]);
              const std::string path = calibration_target == CalibrationTarget::FRONT
                  ? "config/front_lane_roi.txt" : "config/rear_lane_roi.txt";
              save_roi(path, calibration_points);
              if (calibration_target == CalibrationTarget::FRONT)
                front_roi_points = calibration_points;
              else
                rear_roi_points = calibration_points;
              if (calibration_target == CalibrationTarget::FRONT) {
                front_lane_warning.set_reference_from_roi(calibration_points);
                lane = adas::LaneResult();
                front_lane_target = adas::LaneResult();
                front_neural_lane_tracker.reset(); front_lane_visual.reset(); front_outer_style.reset(); front_paint_display.reset();
                front_lane_missed_updates = 0;
              } else {
                rear_lane_warning.set_reference_from_roi(calibration_points);
                rear_lane = adas::LaneResult();
                rear_lane_target = adas::LaneResult();
                rear_neural_lane_tracker.reset(); rear_lane_visual.reset();
                rear_lane_missed_updates = 0;
              }
              calibration_target = CalibrationTarget::NONE;
            } else {
              calibration_points.clear();
            }
          }
        }
      }
      if (!live && scene_delta != 0 && scene_paths.size() > 1) {
        finish_lane_task();
        if (inference_running) {
          inference_future.wait();
          inference_running = false;
        }
        const int count = static_cast<int>(scene_paths.size());
        const int target_scene = (scene_index + scene_delta + count) % count;
        double next_fps = 0.0;
        int next_frames = -1;
        bool next_single_composite = false;
        if (open_scene(scene_paths[target_scene], !options.software_decode,
                       streams, next_fps, next_frames, next_single_composite)) {
          scene_index = target_scene;
          options.scene = scene_paths[scene_index];
          signal_demo.reset(); // 保留用途车道，换场景后先停车并重新确认灯态。
          test4_safety.reset();
          mouse_input.cancel();
          configure_extra_demos();
          demo_saved_signature=-1;
          dashboard_demo_signature=-1;
          if (avm_running) { avm_future.wait(); avm_running = false; }
          avm.load(scene_asset_path(options.scene, "avm.yml"));
          avm_worker.reset(new AvmReplayWorker(avm,options.scene,live));
          avm_preview.release();
          avm_changed = true;
          avm_updated = std::chrono::steady_clock::time_point();
          source_fps = next_fps;
          source_frames = next_frames;
          single_composite = next_single_composite;
          source_frame_index = 0;
          playback_epoch = std::chrono::steady_clock::now();
          shown_frames = 0;
          ufld_task_sequence = 0;
          last_detector_start = -1000000;
          // shown_frames is scene-local. Keeping the previous scene's start
          // index blocks UFLD until the new scene reaches that old index.
          last_ufld_start = -1000000;
          calibration_target = CalibrationTarget::NONE;
          calibration_points.clear();
          for (cv::Mat& source : source_images) source.release();
          signal_logic = adas::SignalLogic();
          drive_logic.reset();
          signal = adas::SignalResult();
          drive = adas::DriveResult();
          front_lane_detector.reset();
          rear_lane_detector.reset();
          lane = adas::LaneResult();
          rear_lane = adas::LaneResult();
          front_lane_target = adas::LaneResult();
          rear_lane_target = adas::LaneResult();
          front_neural_lane_tracker.reset(); front_lane_visual.reset(); front_outer_style.reset(); front_paint_display.reset();
          rear_neural_lane_tracker.reset(); rear_lane_visual.reset();
          front_lane_missed_updates = rear_lane_missed_updates = 0;
          front_risk_estimator.reset(); fcw_monitor.reset();
          rear_risk_estimator.reset();
          front_lane_warning.reset();
          rear_lane_warning.reset();
          left_blind_monitor.reset();
          right_blind_monitor.reset();
          for (auto& tracker : trackers) tracker.reset();
      for (auto& motion : box_motion) motion.reset();
          for (auto& view : detections) view.clear();
          yolo_rate.reset();
          ufld_rate.reset();
          display_fps = 0.0;
          instant_fps = 0.0;
          previous_presentation = std::chrono::steady_clock::time_point();
          fps_window_start = std::chrono::steady_clock::time_point();
          fps_window_intervals = 0;
          std::cout << "Switched scene to " << options.scene << std::endl;
          continue;
        }
      }
    }
    const auto presented_at = std::chrono::steady_clock::now();
    if (previous_presentation == std::chrono::steady_clock::time_point()) {
      fps_window_start = presented_at;
    } else {
      const double interval = std::chrono::duration<double>(
          presented_at - previous_presentation).count();
      instant_fps = interval > 0.0 ? 1.0 / interval : 0.0;
      ++fps_window_intervals;
      const double window_seconds = std::chrono::duration<double>(
          presented_at - fps_window_start).count();
      if (window_seconds >= 1.0) {
        // Count only completed presentation intervals over real wall-clock
        // time. This is independent of source FPS and inference scheduling.
        display_fps = fps_window_intervals / window_seconds;
        fps_window_start = presented_at;
        fps_window_intervals = 0;
      }
    }
    previous_presentation = presented_at;
    ++shown_frames;
    if (fresh_source) ++source_frame_index;
    if (options.max_frames > 0 && shown_frames >= options.max_frames) break;
    if (calibration_paused) {
      // Keep the media clock still while a calibration frame is frozen.
      if (!live) playback_epoch = std::chrono::steady_clock::now() -
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(source_frame_index / source_fps));
      std::this_thread::sleep_for(std::chrono::milliseconds(33));
    } else if (!options.benchmark && (!live || options.display_fps_limit > 0)) {
      const double playback_fps = options.display_fps_limit > 0.0
          ? (live ? options.display_fps_limit : std::min(source_fps, options.display_fps_limit)) : source_fps;
      if (!live) {
        // Decode only the most recent frame needed for the source time line.
        // All four synchronized streams advance together; inference uses the
        // displayed source frame's timestamp and never invents dropped results.
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - playback_epoch).count();
        const int target = static_cast<int>(elapsed * playback_fps);
        const int wanted = std::max(0, std::min({target - source_frame_index,
                                                 source_frames - source_frame_index - 1, 8}));
        for (int dropped=0; dropped<wanted; ++dropped) {
          bool grabbed = streams[0].grab();
          if (!single_composite)
            for (int i=1; i<4; ++i) grabbed = streams[i].grab() && grabbed;
          if (!grabbed) break;
          ++source_frame_index;
          ++playback_dropped_frames;
        }
        const double until_next = (source_frame_index / playback_fps) -
            std::chrono::duration<double>(std::chrono::steady_clock::now() - playback_epoch).count();
        if (until_next > 0.0)
          std::this_thread::sleep_for(std::chrono::duration<double>(until_next));
      } else {
        const double period = 1.0 / playback_fps;
        const double spent = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - frame_start).count();
        if (spent < period)
          std::this_thread::sleep_for(std::chrono::duration<double>(period - spent));
      }
    }
    if (shown_frames % 30 == 0) std::cout << "shown=" << shown_frames
        << " source_frame=" << source_frame_index
        << " playback_dropped=" << playback_dropped_frames << " fps_1s=" << display_fps
        << " fps_instant=" << instant_fps
        << " yolo_result_fps=" << yolo_rate.fps()
        << " ufld_front_result_fps=" << ufld_rate.fps()
        << " npu_ms=" << npu_ms << " lanes=" << lane.valid << ',' << rear_lane.valid
        << " object_interval_ms=" << object_interval_ms
        << " object_result_ms=" << object_result_ms
        << " object_results=" << object_results << " stale_object_drops=" << stale_object_drops
        << " object_gap_ms=" << object_gap_ms << " max_object_gap_ms=" << max_object_gap_ms
        << " raw_boxes=" << object_raw_counts[0] << ',' << object_raw_counts[1] << ',' << object_raw_counts[2] << ',' << object_raw_counts[3]
        << " visible_boxes=" << detections[0].size() << ',' << detections[1].size() << ',' << detections[2].size() << ',' << detections[3].size()
        << " box_motion_ms=" << box_motion_ms << " box_motion_corrected=" << box_motion_corrected
        << " object_age_ms=" << (last_detection_at == std::chrono::steady_clock::time_point() ? -1.0 :
            std::chrono::duration<double, std::milli>(frame_start - last_detection_at).count())
        << " capture_ms=" << capture_ms << " wait_ms=" << wait_ms << " split_ms=" << split_ms << " lane_ms=" << last_lane_ms
        << " compose_ms=" << compose_ms << " render_ms=" << render_ms
        << " frame_ms=" << std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - frame_start).count()
        << " lane_age_ms=" << lane_age_ms[0] << ',' << lane_age_ms[1]
        << " lane_source_frame=" << lane_source_frame[0] << ',' << lane_source_frame[1]
        << " hsv=" << verify_colors << " hsv_checked=" << hsv_checked
        << " hsv_rejected=" << hsv_rejected << " hsv_ms=" << hsv_ms
        << " lane_change=" << static_cast<int>(lane_change_result.state)
        << " test2_permitted=" << lane_change_result.permitted
        << " test2_marking=" << static_cast<int>(rule_semantic.left) << ',' << static_cast<int>(rule_semantic.right)
        << " lane_marking=" << static_cast<int>(lane_semantic.left) << ','
        << static_cast<int>(lane_semantic.right)
        << " lane_marking_evidence=" << lane_semantic.left_evidence << ','
        << lane_semantic.right_evidence
        << " tlc_s=" << lane_semantic.tlc_s
        << " intersection=" << static_cast<int>(intersection_result.action)
        << " warning=" << static_cast<int>(warning_summary.primary)
        << " stale_lane_drops=" << stale_lane_drops
        << " test4_enabled=" << test4_result.enabled << " test4_brake=" << test4_result.brake
        << " test4_stale=" << test4_result.stale << " test4_resume=" << test4_result.can_resume
        << " test4_hazard=" << static_cast<int>(test4_result.hazard) << " test4_track=" << test4_result.track_id
        << " nav_mock=" << navigation_demo.enabled() << " nav_distance=" << navigation_demo.distance()
        << " nav_direction=" << int(navigation_demo.direction()) << " nav_advice=" << int(navigation_advice.action)
        << " lane_complete_age_ms=" << lane_complete_age_ms
        << " lane_poll_delay_ms=" << lane_poll_delay_ms
        << " lane_offset=" << lane.offset_ratio << ',' << rear_lane.offset_ratio
        << " lane_departure=" << lane.departure << ',' << rear_lane.departure
        << " departure_side=" << static_cast<int>(lane.departure_side)
        << " lane_identity=" << lane.identity_uncertain
        << " lane_partial=" << lane.partial
        << " lane_coverage=" << lane.boundary_coverage[0] << ','
        << lane.boundary_coverage[1] << ',' << lane.boundary_coverage[2] << ','
        << lane.boundary_coverage[3]
        << " adjacent_polygons=" << !lane.lane_polygons[0].empty() << ','
        << !lane.lane_polygons[2].empty()
        << " visual_adjacent=" << !visual_lane.lane_polygons[0].empty() << ',' << !visual_lane.lane_polygons[2].empty()
        << " confirmed_outer_marking=" << static_cast<int>(lane_semantic.boundaries[0]) << ',' << static_cast<int>(lane_semantic.boundaries[3])
        << " visual_outer_marking=" << static_cast<int>(display_semantic.boundaries[0]) << ',' << static_cast<int>(display_semantic.boundaries[3])
        << " outer_age_ms=" << front_lane_visual.observation_age_ms(visual_at)[0] << ','
        << front_lane_visual.observation_age_ms(visual_at)[1]
        << " lane_reassignment=" << static_cast<int>(lane.reassignment_side)
        << " fcw_enabled=" << options.fcw_enabled << " fcw_level=" << int(fcw_state.level)
        << " fcw_status=" << int(fcw_state.status) << " fcw_ms=" << fcw_ms
        << " risk_target=" << front_risk.target << ',' << rear_risk.target
        << " risk_warning=" << front_risk.warning << ',' << rear_risk.warning
        << " distance_m=" << front_risk.distance_m << ',' << rear_risk.distance_m
        << " ttc_s=" << front_risk.ttc_s << ',' << rear_risk.ttc_s
        << " blind=" << left_blind.occupied << ',' << right_blind.occupied
        << std::endl;
  }
  finish_lane_task();
  if (lane_toggle.loading) lane_model_future.wait();
  pcie.close();
  if (inference_running) inference_future.wait();
  const double seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();
  std::cout << "Done. shown=" << shown_frames << " wall_fps="
            << (seconds > 0.0 ? shown_frames / seconds : 0.0) << '\n';
  return capture_failed ? 5 : 0;
}

int main(int argc, char** argv) {
  try { return run_four_view(argc, argv); }
  catch (const std::exception& e) { std::cerr << "Fatal: " << e.what() << std::endl; return 1; }
}
#endif
