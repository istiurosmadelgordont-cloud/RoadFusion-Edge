#include "adas/config.hpp"
#include "adas/adas_logic_v2.hpp"
#include "adas/experimental_adas.hpp"
#include "adas/perception_schedule.hpp"
#include "adas/byte_tracker.hpp"
#include "adas/drive_decision.hpp"
#include "adas/lane_detector.hpp"
#include "adas/lane_geometry_tracker.hpp"
#include "adas/overlay.hpp"
#include "adas/risk_estimator.hpp"
#include "adas/rknn_detector.hpp"
#include "adas/signal_logic.hpp"
#include "adas/warning_logic.hpp"
#include "adas/text_renderer.hpp"
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
#include <future>
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

namespace {

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
const cv::Rect kIntentLeft(1535, 414, 78, 35);
const cv::Rect kIntentOff(1621, 414, 78, 35);
const cv::Rect kIntentRight(1707, 414, 78, 35);
const cv::Rect kSignalVerify(1404, 563, 125, 29);
const cv::Rect kPrevSceneButton(1040, 980, 150, 38);
const cv::Rect kNextSceneButton(1200, 980, 150, 38);
// Display geometry is independent of the 640x360 inference/overlay images.
const std::array<cv::Rect, 4> kViewRects{{
    {16, 124, 672, 378}, {700, 124, 672, 378},
    {16, 568, 672, 378}, {700, 568, 672, 378}}};

struct Options {
  std::string model = "models/unified21_p2_v7_640_int8.rknn";
  std::string scene = "four_view_sample/e716f3ed";
  std::string snapshot;
  std::string ufld_model;
  int cpu_threads = 2;
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
  bool ufld_rear = false;
  bool rear_lane_enabled = true;
};

bool parse(int argc, char** argv, Options& o) {
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--headless") { o.headless = true; continue; }
    if (key == "--benchmark") { o.benchmark = true; continue; }
    if (key == "--once") { o.once = true; continue; }
    if (key == "--separate") { o.separate = true; continue; }
    if (key == "--dump-detections") { o.dump_detections = true; continue; }
    if (key == "--opencv") { o.opencv_display = true; continue; }
    if (key == "--software-decode") { o.software_decode = true; continue; }
    if (key == "--legacy-mosaic") { o.legacy_mosaic = true; continue; }
    if (key == "--ufld-rear") { o.ufld_rear = true; continue; }
    if (key == "--no-rear-lane") { o.rear_lane_enabled = false; continue; }
    if (key == "--help") {
      std::cout << "--scene DIR --model FILE --detect-every N --cpu-threads N "
                   "--ufld-model FILE --ufld-every N --display-fps N --max-frames N --snapshot FILE --snapshot-frame N "
                   "--headless --benchmark --once "
                   "--separate --dump-detections --opencv --software-decode --legacy-mosaic --ufld-rear --no-rear-lane\n";
      return false;
    }
    if (++i >= argc) { std::cerr << "Missing value for " << key << '\n'; return false; }
    const std::string value = argv[i];
    if (key == "--scene") o.scene = value;
    else if (key == "--model") o.model = value;
    else if (key == "--snapshot") o.snapshot = value;
    else if (key == "--ufld-model") o.ufld_model = value;
    else if (key == "--detect-every") o.detect_every = std::max(1, std::atoi(value.c_str()));
    else if (key == "--ufld-every") o.ufld_every = std::max(2, std::atoi(value.c_str()));
    else if (key == "--display-fps") o.display_fps_limit = std::max(1.0, std::atof(value.c_str()));
    else if (key == "--cpu-threads") o.cpu_threads = std::max(1, std::min(4, std::atoi(value.c_str())));
    else if (key == "--max-frames") o.max_frames = std::max(0, std::atoi(value.c_str()));
    else if (key == "--snapshot-frame") o.snapshot_frame = std::max(0, std::atoi(value.c_str()));
    else { std::cerr << "Unknown option: " << key << '\n'; return false; }
  }
  return true;
}

class SyncCapture {
 public:
  ~SyncCapture() { close_pipe(); }

  void release() {
    close_pipe();
    software_.release();
  }

  bool open(const std::string& path, bool hardware) {
    close_pipe();
    software_.release();
    path_ = path;
    hardware_ = hardware;
    cv::VideoCapture probe(path);
    if (!probe.isOpened()) return false;
    fps_ = probe.get(cv::CAP_PROP_FPS);
    frames_ = static_cast<int>(probe.get(cv::CAP_PROP_FRAME_COUNT));
    width_ = static_cast<int>(probe.get(cv::CAP_PROP_FRAME_WIDTH));
    height_ = static_cast<int>(probe.get(cv::CAP_PROP_FRAME_HEIGHT));
    probe.release();
    if (width_ < 2 || height_ < 2 || width_ > 8192 || height_ > 4320 ||
        (hardware_ && ((width_ & 1) != 0 || (height_ & 1) != 0))) {
      release();
      return false;
    }
    return hardware_ ? start_pipe() : software_.open(path);
  }

  bool read(cv::Mat& frame) {
    if (!hardware_) return software_.read(frame);
    cv::Mat nv12(height_ * 3 / 2, width_, CV_8UC1);
    const size_t wanted = nv12.total();
    size_t received = 0;
    while (received < wanted) {
      const size_t count = std::fread(nv12.data + received, 1, wanted - received, pipe_);
      if (!count) return false;
      received += count;
    }
    frame.create(height_, width_, CV_8UC3);
    const rga_buffer_t source = wrapbuffer_virtualaddr(
        nv12.data, width_, height_, RK_FORMAT_YCbCr_420_SP);
    const rga_buffer_t target = wrapbuffer_virtualaddr(
        frame.data, width_, height_, RK_FORMAT_BGR_888);
    if (imcvtcolor(source, target, RK_FORMAT_YCbCr_420_SP,
                   RK_FORMAT_BGR_888,
                   IM_YUV_TO_RGB_BT601_LIMIT) != IM_STATUS_SUCCESS) {
      cv::cvtColor(nv12, frame, cv::COLOR_YUV2BGR_NV12);
    }
    return true;
  }

  bool grab() {
    if (!hardware_) return software_.grab();
    cv::Mat discarded;
    return read(discarded);
  }

  void reset() {
    if (!hardware_) {
      software_.set(cv::CAP_PROP_POS_FRAMES, 0);
    } else {
      close_pipe();
      start_pipe();
    }
  }

  bool seek(int frame) {
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
    const std::string command =
        "taskset -c 0,1 gst-launch-1.0 -q filesrc location=" + shell_quote(path_) +
        " ! qtdemux ! h264parse ! mppvideodec ! video/x-raw,format=NV12,width=" +
        std::to_string(width_) + ",height=" + std::to_string(height_) +
        " ! fdsink fd=1 sync=false 2>/dev/null";
    pipe_ = popen(command.c_str(), "r");
    return pipe_ != nullptr;
  }

  void close_pipe() {
    if (pipe_) {
      pclose(pipe_);
      pipe_ = nullptr;
    }
  }

  bool hardware_ = false;
  FILE* pipe_ = nullptr;
  cv::VideoCapture software_;
  std::string path_;
  double fps_ = 0.0;
  int frames_ = 0;
  int width_ = 0;
  int height_ = 0;
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
  for (int i = 0; i < 4; ++i) {
    const cv::Rect quadrant((i % 2) * half_w, (i / 2) * half_h,
                            half_w, half_h);
    cv::resize(composite(quadrant), frames[i], cv::Size(kTileW, kTileH),
               0.0, 0.0, cv::INTER_AREA);
  }
  return true;
}

struct InferenceResult {
  std::vector<adas::Detection> verified_lights;
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
  double elapsed_ms = 0.0;
  std::chrono::steady_clock::time_point captured_at{};
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

cv::Scalar box_color(int cls) {
  if (cls >= 7 && cls <= 10) return cv::Scalar(55, 75, 255);
  if (cls >= 11 && cls <= 14) return cv::Scalar(70, 230, 95);
  if (cls == 0 || cls == 1) return cv::Scalar(210, 100, 240);
  return cv::Scalar(90, 200, 255);
}

cv::Scalar target_risk_color(const adas::RiskResult& risk) {
  if (risk.warning || (risk.ttc_s > 0.0f && risk.ttc_s < 1.5f))
    return cv::Scalar(35, 45, 245);
  if (risk.ttc_s > 0.0f && risk.ttc_s < 3.0f)
    return cv::Scalar(30, 150, 245);
  return cv::Scalar(50, 225, 245);
}

void draw_boxes(cv::Mat& frame, const std::vector<adas::Detection>& detections,
                const adas::RiskResult* risk = nullptr, int attention_track = -1,
                bool danger_attention = false) {
  for (const auto& d : detections) {
    const bool risk_target = risk && risk->target && risk->track_id >= 0 &&
                             d.track_id == risk->track_id;
    const bool attention = attention_track >= 0 && d.track_id == attention_track;
    const cv::Scalar color = risk_target ? target_risk_color(*risk) :
        attention ? (danger_attention ? cv::Scalar(35, 45, 245)
                                      : cv::Scalar(50, 225, 245)) : box_color(d.class_id);
    cv::rectangle(frame, d.box, color, risk_target || attention ? 4 : 2);
    std::ostringstream label;
    label << d.name;
    if (d.track_id >= 0) label << " #" << d.track_id;
    label << ' ' << std::fixed << std::setprecision(2) << d.score;
    const int x = std::max(2, static_cast<int>(d.box.x));
    const int y = std::max(23, static_cast<int>(d.box.y));
    int baseline = 0;
    const cv::Size size = cv::getTextSize(label.str(), cv::FONT_HERSHEY_SIMPLEX, 0.43, 1, &baseline);
    cv::rectangle(frame, cv::Rect(x, y - 20, std::min(size.width + 7, frame.cols - x), 20),
                  cv::Scalar(15, 20, 25), cv::FILLED);
    caption(frame, label.str(), x + 3, y - 5, color, 0.43);
  }
}

void draw_rear_risk_overlay(cv::Mat& frame, const adas::RiskResult& risk) {
  // No rear lane geometry is available with --no-rear-lane. A fixed
  // trapezoid would falsely imply a road-aligned collision corridor.
  // Show object-level risk only until calibrated geometry is available.
  if (!risk.target) return;
  std::ostringstream text;
  text << "RCW #" << risk.track_id << "  " << std::fixed << std::setprecision(1)
       << risk.distance_m << "m";
  if (risk.ttc_s > 0.0f) text << "  TTC " << risk.ttc_s << "s";
  caption(frame, text.str(), 12, 28, target_risk_color(risk), 0.56);
}

void draw_blind_spot_overlay(cv::Mat& frame, const adas::BlindSpotResult& blind,
                             bool turn_intent, const char* side) {
  const std::vector<cv::Point> region{{cvRound(frame.cols * 0.08f), cvRound(frame.rows * 0.34f)},
      {cvRound(frame.cols * 0.92f), cvRound(frame.rows * 0.34f)},
      {cvRound(frame.cols * 0.98f), cvRound(frame.rows * 0.98f)},
      {cvRound(frame.cols * 0.02f), cvRound(frame.rows * 0.98f)}};
  const bool lca = blind.occupied && turn_intent;
  const cv::Scalar color = lca ? cv::Scalar(35, 45, 245) :
      blind.occupied ? cv::Scalar(50, 225, 245) : cv::Scalar(230, 210, 50);
  cv::Mat layer = frame.clone();
  cv::fillConvexPoly(layer, region, color);
  cv::addWeighted(layer, lca ? 0.18 : blind.occupied ? 0.12 : 0.05,
                  frame, lca ? 0.82 : blind.occupied ? 0.88 : 0.95, 0, frame);
  cv::polylines(frame, region, true, color, lca ? 4 : 2, cv::LINE_AA);
  std::string text = std::string(side) + (lca ? " LCA 变道危险" :
      blind.occupied ? " BSD 盲区占用" : " BSD ROI");
  caption(frame, text, 12, 28, color, 0.56);
}

void warning_banner(cv::Mat& frame, const std::string& text) {
  const cv::Rect area(8, frame.rows - 38, std::min(frame.cols - 16, 330), 30);
  cv::rectangle(frame, area, cv::Scalar(20, 35, 190), cv::FILLED);
  adas::ui::draw_text(frame, text, cv::Point(area.x + 8, area.y + 22),
                      16, cv::Scalar(255, 255, 255));
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
                       const std::array<std::vector<adas::Detection>, 4>& detections,
                       const adas::LaneResult& front_lane,
                       const adas::LaneResult& rear_lane,
                       const adas::SignalResult& signal,
                       const adas::RiskResult& front_risk,
                       const adas::RiskResult& rear_risk,
                       const adas::BlindSpotResult& left_blind,
                       const adas::BlindSpotResult& right_blind) {
  cv::rectangle(canvas, area, cv::Scalar(38, 41, 45), cv::FILLED);
  cv::rectangle(canvas, area, cv::Scalar(70, 78, 84), 1);
  const cv::Point ego(area.x + area.width / 2, area.y + area.height * 72 / 100);
  const int front_shift = front_lane.valid
      ? cvRound(-front_lane.offset_ratio * 18.0f) : 0;
  const int rear_shift = rear_lane.valid
      ? cvRound(-rear_lane.offset_ratio * 18.0f) : 0;

  const std::vector<cv::Point> road = {
      cv::Point(ego.x - 18 + front_shift, area.y + 4),
      cv::Point(ego.x + 18 + front_shift, area.y + 4),
      cv::Point(ego.x + 48, ego.y + 22), cv::Point(ego.x - 48, ego.y + 22)};
  cv::fillConvexPoly(canvas, road, cv::Scalar(29, 33, 37));
  const cv::Scalar lane_color = front_lane.valid
      ? cv::Scalar(205, 180, 80) : cv::Scalar(95, 105, 110);
  cv::line(canvas, cv::Point(ego.x - 30, ego.y + 17),
           cv::Point(ego.x - 14 + front_shift, area.y + 4), lane_color, 2, cv::LINE_AA);
  cv::line(canvas, cv::Point(ego.x + 30, ego.y + 17),
           cv::Point(ego.x + 14 + front_shift, area.y + 4), lane_color, 2, cv::LINE_AA);
  const cv::Scalar rear_color = rear_lane.valid
      ? cv::Scalar(160, 145, 80) : cv::Scalar(75, 82, 86);
  cv::line(canvas, cv::Point(ego.x - 30, ego.y + 17),
           cv::Point(ego.x - 19 + rear_shift, area.y + area.height - 3), rear_color, 1,
           cv::LINE_AA);
  cv::line(canvas, cv::Point(ego.x + 30, ego.y + 17),
           cv::Point(ego.x + 19 + rear_shift, area.y + area.height - 3), rear_color, 1,
           cv::LINE_AA);

  for (int y = area.y + 12; y < ego.y - 18; y += 13)
    cv::line(canvas, cv::Point(ego.x + front_shift / 2, y),
             cv::Point(ego.x + front_shift / 2, y + 5), cv::Scalar(105, 110, 112), 1);

  const cv::Rect light(ego.x - 5 + front_shift, area.y + 3, 10, 27);
  cv::rectangle(canvas, light, cv::Scalar(9, 13, 16), cv::FILLED);
  cv::rectangle(canvas, light, cv::Scalar(100, 110, 115), 1);
  const cv::Scalar off(42, 47, 49);
  cv::circle(canvas, cv::Point(light.x + 5, light.y + 5), 3,
             signal.state == adas::SignalState::RED ? cv::Scalar(45, 55, 255) : off, cv::FILLED);
  cv::circle(canvas, cv::Point(light.x + 5, light.y + 13), 3, off, cv::FILLED);
  cv::circle(canvas, cv::Point(light.x + 5, light.y + 21), 3,
             signal.state == adas::SignalState::GREEN ? cv::Scalar(65, 235, 90) : off,
             cv::FILLED);

  for (int camera = 0; camera < 4; ++camera) {
    std::vector<const adas::Detection*> objects;
    for (const adas::Detection& detection : detections[camera])
      if (detection.class_id >= 0 && detection.class_id <= 6) objects.push_back(&detection);
    std::sort(objects.begin(), objects.end(),
              [](const adas::Detection* left, const adas::Detection* right) {
                return left->box.area() > right->box.area();
              });
    const size_t limit = camera == 0 ? 3 : 2;
    if (objects.size() > limit) objects.resize(limit);
    for (const adas::Detection* object : objects) {
      const adas::Detection& detection = *object;
      const float nx = std::max(-1.0f, std::min(1.0f,
          (detection.box.x + detection.box.width * 0.5f) / kTileW * 2.0f - 1.0f));
      const float ny = std::max(-1.0f, std::min(1.0f,
          (detection.box.y + detection.box.height * 0.5f) / kTileH * 2.0f - 1.0f));
      const float apparent = std::max(0.0f, std::min(1.0f,
          std::sqrt(std::max(1.0f, detection.box.area()) /
                    static_cast<float>(kTileW * kTileH)) * 4.0f));
      const float far = 1.0f - apparent;
      cv::Point position = ego;
      if (camera == 0) {
        position.x += cvRound(nx * (24.0f + apparent * 28.0f));
        position.y = area.y + 34 + cvRound(apparent * (ego.y - area.y - 59));
      } else if (camera == 1) {
        position.x -= cvRound(nx * 34.0f);
        position.y = ego.y + 21 + cvRound(far * 7.0f);
      } else {
        const int side_distance = 52 + cvRound(far * (area.width / 2 - 65));
        position.x += camera == 2 ? -side_distance : side_distance;
        position.y += cvRound(ny * 22.0f);
      }
      position.x = std::max(area.x + 9, std::min(area.x + area.width - 10, position.x));
      position.y = std::max(area.y + 9, std::min(area.y + area.height - 10, position.y));
      const bool front_target = camera == 0 && front_risk.target &&
          detection.track_id == front_risk.track_id;
      const bool rear_target = camera == 1 && rear_risk.target &&
          detection.track_id == rear_risk.track_id;
      const bool blind_target = (camera == 2 && detection.track_id == left_blind.track_id) ||
          (camera == 3 && detection.track_id == right_blind.track_id);
      cv::Scalar object_color(165, 172, 176);
      if (front_target) object_color = target_risk_color(front_risk);
      if (rear_target) object_color = target_risk_color(rear_risk);
      if (blind_target) object_color = cv::Scalar(50, 225, 245);
      if (detection.class_id <= 1) {
        cv::circle(canvas, position, 5, object_color, cv::FILLED);
      } else {
        const int icon_width = 9 + cvRound(apparent * 5.0f);
        const int icon_height = 15 + cvRound(apparent * 8.0f);
        draw_top_car(canvas, position, icon_width, icon_height,
                     object_color, camera >= 2, false);
      }
    }
  }
  draw_top_car(canvas, ego, 18, 29, cv::Scalar(100, 230, 150), false, true);
}

void draw_sidebar(cv::Mat& canvas, double fps, double instant_fps, double npu_ms,
                  const adas::SignalResult& signal,
                  const std::array<std::vector<adas::Detection>, 4>& detections,
                  const adas::LaneResult& front_lane, const adas::LaneResult& rear_lane,
                  const adas::DriveResult& drive, const adas::RiskResult& front_risk,
                  const adas::RiskResult& rear_risk,
                  const adas::BlindSpotResult& left_blind,
                  const adas::BlindSpotResult& right_blind, bool side_fresh,
                  bool health_ready,
                  double lane_ms, const std::array<double, 2>& lane_age_ms,
                  const adas::VehicleState& vehicle,
                  const adas::LaneSemantic& semantics,
                  const adas::LaneChangeResult& lane_change,
                  const adas::IntersectionResult& intersection,
                  bool verify_colors) {
  const int x = 1390, w = 514;
  const cv::Scalar muted(180, 159, 129), white(242, 237, 227);
  const cv::Scalar cyan(245, 201, 70), green(135, 229, 75), red(85, 85, 245);
  const auto card = [&](int y, int h, const char* title) {
    panel(canvas, x, y, w, h);
    caption(canvas, title, x + 16, y + 28, white, 0.62);
    cv::line(canvas, {x + 1, y + 40}, {x + w - 2, y + 40}, cv::Scalar(85, 59, 29));
  };
  card(86, 220, "导航 / 周边感知    Navigation");
  draw_surround_map(canvas, {x + 12, 138, 258, 154}, detections, front_lane,
                    rear_lane, signal, front_risk, rear_risk, left_blind, right_blind);
  caption(canvas, "周边目标示意", x + 287, 170, cyan, 0.67);
  caption(canvas, "地图 / 路线未接入", x + 287, 210, muted, 0.51);
  caption(canvas, "非真实世界坐标", x + 287, 244, muted, 0.47);
  caption(canvas, "四路目标融合显示", x + 287, 276, muted, 0.47);

  card(318, 144, "车辆状态    Vehicle Status");
  cv::ellipse(canvas, {x + 77, 409}, {49, 49}, 0, 145, 395, cyan, 6, cv::LINE_AA);
  caption(canvas, "--", x + 53, 410, white, 1.1);
  caption(canvas, "km/h", x + 47, 439, muted, 0.49);
  caption(canvas, "模拟转向 / 10秒复位", x + 149, 395, white, 0.53);
  const cv::Rect buttons[] = {kIntentLeft,kIntentOff,kIntentRight};
  const char* names[] = {"左", "关闭", "右"};
  for (int i=0; i<3; ++i) {
    const auto& b=buttons[i];
    panel(canvas,b.x,b.y,b.width,b.height);
    const bool selected = (i==0 && vehicle.turn==adas::TurnSignal::LEFT) ||
        (i==1 && (!vehicle.turn_valid || vehicle.turn==adas::TurnSignal::OFF)) ||
        (i==2 && vehicle.turn==adas::TurnSignal::RIGHT);
    caption(canvas,names[i],b.x+18,b.y+24,selected?cyan:muted,.51);
  }
  caption(canvas, vehicle.demo ? "状态  模拟" : "CAN  未接", x + 370, 395,
          vehicle.demo ? cyan : muted, 0.54);
  caption(canvas, adas::lane_change_state_caption(lane_change.state),
          x + 370, 432, lane_change.permitted ? green : muted, 0.52);

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
  const char* decision = intersection.action == adas::IntersectionAction::UNKNOWN
      ? (drive.decision == adas::DriveDecision::STOP ? "STOP 停车提示" :
         drive.decision == adas::DriveDecision::GO ? "GO 通行提示" :
         drive.decision == adas::DriveDecision::SLOW ? "SLOW 减速提示" : "等待有效信号")
      : adas::intersection_action_caption(intersection.action);
  const char* route = intersection.route == adas::ManeuverDirection::LEFT ? "LEFT" :
                      intersection.route == adas::ManeuverDirection::RIGHT ? "RIGHT" :
                      intersection.route == adas::ManeuverDirection::STRAIGHT ? "STRAIGHT" : "--";
  caption(canvas, std::string("路口方向 ") +
          (intersection.route_valid ? route : "未知") + "  |  " + decision, x + 160, 580,
          intersection.action == adas::IntersectionAction::STOP ? red : white, 0.56);
  panel(canvas,kSignalVerify.x,kSignalVerify.y,kSignalVerify.width,kSignalVerify.height);
  caption(canvas,verify_colors?"HSV复核 开":"HSV复核 关",kSignalVerify.x+5,kSignalVerify.y+21,
          verify_colors?cyan:muted,.47);

  card(614, 136, "车道状态    Lane Status");
  caption(canvas, "前视", x + 17, 683, muted, 0.54);
  caption(canvas, front_lane.valid ? (front_lane.departure
              ? adas::lane_departure_name(front_lane.departure_side) : "已检测") : "未检测",
          x + 83, 683, front_lane.valid ? (front_lane.departure ? red : green) : muted, 0.62);
  caption(canvas, "后视", x + 285, 683, muted, 0.54);
  caption(canvas, rear_lane.valid ? "已检测" : "未检测", x + 355, 683,
          rear_lane.valid ? green : muted, 0.62);
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

  card(754, 120, "ADAS 视觉预警 / 未接 CAN");
  const char* labels[] = {"FCW", "RCW", "BSD-L", "BSD-R", "LDW"};
  const bool warnings[] = {front_risk.warning, rear_risk.warning, left_blind.occupied,
                           right_blind.occupied, front_lane.departure};
  for (int i = 0; i < 5; ++i) {
    const int bx = x + 15 + i * 100;
    caption(canvas, labels[i], bx, 822, warnings[i] ? red : white, 0.57);
    caption(canvas, warnings[i] ? "告警" : (i == 4 && !front_lane.valid ? "待检测" : "未触发"),
            bx, 852, warnings[i] ? red : muted, 0.49);
  }

  card(886, 132, "系统性能    System Performance");
  std::ostringstream perf, timing;
  perf << std::fixed << std::setprecision(1) << "实际显示 " << fps << " FPS（1秒计数）";
  timing << std::fixed << std::setprecision(1) << "瞬时 " << instant_fps << " FPS    "
         << std::setprecision(0) << "YOLO " << npu_ms << " ms  车道 " << lane_ms << " ms";
  caption(canvas, perf.str(), x + 17, 952, green, 0.57);
  caption(canvas, timing.str(), x + 17, 980, white, 0.48);
  std::ostringstream age;
  const bool lane_fresh = lane_age_ms[0] >= 0.0 && lane_age_ms[0] <= 1000.0;
  if (!health_ready) {
    age << "Health  INITIALIZING";
  } else {
    age << "Health  YOLO/NPU " << (side_fresh ? "OK" : "STALE")
        << "  UFLD " << (lane_fresh ? "OK" : "STALE")
        << "  CAMERA OK";
  }
  caption(canvas, age.str(), x + 17, 1006,
          health_ready && (!side_fresh || !lane_fresh) ? red : muted, 0.46);
}

void draw_bottom_left(cv::Mat& canvas, int scene_index, int scene_count,
                      const std::string& scene_name) {
  cv::rectangle(canvas, {0, 964, 1380, 116}, cv::Scalar(26, 17, 8), cv::FILLED);
  caption(canvas, "同步回放", 24, 1003, cv::Scalar(235, 199, 100), 0.62);
  std::ostringstream scene;
  scene << "场景 " << scene_index + 1 << '/' << scene_count << "   " << scene_name.substr(0, 16);
  caption(canvas, scene.str(), 150, 1003, cv::Scalar(215, 202, 180), 0.55);
  caption(canvas, "C 前视标定   R 后视标定   Q 退出", 578, 1003, cv::Scalar(183, 163, 137), 0.51);
  for (const auto& rect : {kPrevSceneButton, kNextSceneButton}) panel(canvas, rect.x, rect.y, rect.width, rect.height);
  caption(canvas, "< 上一场景", 1057, 1006, cv::Scalar(238, 223, 191), 0.52);
  caption(canvas, "下一场景 >", 1217, 1006, cv::Scalar(238, 223, 191), 0.52);
  caption(canvas, "YOLOv8n-P2  /  ByteTrack  /  UFLD V2", 24, 1037, cv::Scalar(174, 150, 111), 0.48);
  cv::rectangle(canvas, {1390, 1028, 514, 52}, cv::Scalar(24, 15, 7), cv::FILLED);

}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!parse(argc, argv, options)) return 1;
  if (!adas::ui::initialize_text("/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc"))
    std::cerr << "Chinese font unavailable; UI text will use fallback rendering\n";
  // The lane estimator and detector preprocessing already run as separate
  // application workers. A shared OpenCV worker pool caused rare multi-second
  // starvation on the two assigned Cortex-A55 cores.
  cv::setNumThreads(1);
  std::vector<std::string> scene_paths;
  const char* bundled_scenes[] = {
      "four_view_sample/66b5fa4b_30fps",
      "four_view_sample/e716f3ed_30fps",
      "four_view_sample/050541b1_30fps",
      "four_view_sample/668b1ac0_30fps",
      "four_view_sample/ddfc4b8d_30fps"};
  if (scene_complete(options.scene)) scene_paths.push_back(options.scene);
  for (const char* bundled : bundled_scenes) {
    const std::string candidate = bundled;
    if (!scene_complete(candidate) ||
        std::find(scene_paths.begin(), scene_paths.end(), candidate) != scene_paths.end()) continue;
    scene_paths.push_back(candidate);
  }
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
  bool single_composite = false;
  if (!open_scene(options.scene, !options.software_decode, streams,
                  source_fps, source_frames, single_composite)) {
    std::cerr << "Cannot open synchronized scene " << options.scene << '\n';
    return 2;
  }
  adas::DetectorConfig config;
  config.model_path = options.model;
  adas::RknnDetector detector(config);
  if (!detector.ready()) { std::cerr << detector.error() << '\n'; return 3; }
  std::unique_ptr<adas::UfldLaneDetector> ufld_lane_detector;
  if (!options.ufld_model.empty()) {
    ufld_lane_detector.reset(new adas::UfldLaneDetector(options.ufld_model));
    if (!ufld_lane_detector->ready()) {
      std::cerr << ufld_lane_detector->error() << '\n';
      return 4;
    }
    std::cout << "Experimental lane model: " << ufld_lane_detector->profile()
              << " every >= " << options.ufld_every << " displayed frames\n";
  }
  const bool high_precision_ufld = ufld_lane_detector &&
                                   ufld_lane_detector->input_width() >= 1600;
  const adas::PerceptionPeriods perception_periods =
      adas::lane_priority_periods(high_precision_ufld);
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
  adas::IntersectionLogic intersection_logic;
  adas::IntersectionResult intersection_result;
  adas::WarningManager warning_manager;
  adas::WarningSummary warning_summary;
  std::vector<adas::Detection> current_lights;
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
  adas::RiskEstimator rear_risk_estimator(rear_risk_config);
  adas::BlindSpotMonitor left_blind_monitor;
  adas::BlindSpotMonitor right_blind_monitor;
  std::array<adas::ByteTracker, 4> trackers;
  adas::SignalResult signal;
  adas::DriveResult drive;
  adas::LaneResult lane;
  adas::LaneResult rear_lane;
  adas::LaneResult front_lane_target;
  adas::LaneResult rear_lane_target;
  adas::NeuralLaneGeometryTracker front_neural_lane_tracker;
  adas::NeuralLaneGeometryTracker rear_neural_lane_tracker;
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
  bool lane_running = false;
  bool lane_task_uses_npu = false;
  int lane_task_sequence = 0;
  int ufld_task_sequence = 0;
  std::array<std::chrono::steady_clock::time_point, 3> task_launched{};
  int last_detector_start = -1000000;
  int last_ufld_start = -1000000;
  double npu_ms = 0.0;
  double display_fps = 0.0;
  double instant_fps = 0.0;
  auto previous_presentation = std::chrono::steady_clock::time_point();
  auto fps_window_start = std::chrono::steady_clock::time_point();
  int fps_window_intervals = 0;
  int source_frame_index = 0;
  int shown_frames = 0;
  CalibrationTarget calibration_target = CalibrationTarget::NONE;
  std::vector<cv::Point2f> calibration_points;
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
    cv::namedWindow("RK3568 V7 | NVIDIA 4 VIEW", cv::WINDOW_NORMAL);
    cv::setWindowProperty("RK3568 V7 | NVIDIA 4 VIEW", cv::WND_PROP_FULLSCREEN,
                          cv::WINDOW_FULLSCREEN);
  }
  std::cout << "Four synchronized streams: " << source_frames << " frames at "
            << source_fps << " FPS; " << (options.separate ? "round-robin" : "mosaic")
            << " V7 inference every " << options.detect_every << " frame(s); decoder="
            << (options.software_decode ? "software" : "MPP hardware")
            << "; inference_source="
            << (options.legacy_mosaic ? "legacy 640x640 mosaic" : "FPGA-style 1920x1080")
            << "; video_ui=1280x720\n";
  const auto start = std::chrono::steady_clock::now();
  cv::Mat canvas(kCanvasH, kCanvasW, CV_8UC3, cv::Scalar(24, 15, 7));
  cv::rectangle(canvas, cv::Rect(0, 0, kCanvasW, kHeaderH),
                cv::Scalar(37, 24, 10), cv::FILLED);
  caption(canvas, "RoadFusion", 24, 39, cv::Scalar(250, 209, 95), 1.03);
  caption(canvas, "Drive Smarter. A Safer Tomorrow.", 26, 62, cv::Scalar(185, 160, 130), 0.43);
  caption(canvas, "基于紫光同创 FPGA + RK3568 的四视角高级辅助驾驶系统",
          514, 36, cv::Scalar(242, 238, 229), 0.83);
  caption(canvas, "RoadFusion-Edge  /  Demo UI", 779, 62, cv::Scalar(202, 178, 140), 0.52);
  caption(canvas, "四路同步视频  |  演示模式", 1570, 43, cv::Scalar(225, 187, 115), 0.60);
  draw_camera_cards(canvas);
  std::array<cv::Mat, 4> source_images;
  cv::Mat source_composite;
  double last_lane_ms = 0.0;
  std::array<std::chrono::steady_clock::time_point, 2> lane_captured_at{};
  std::array<int, 2> lane_source_frame{{-1, -1}};
  std::array<double, 2> lane_age_ms{{-1.0, -1.0}};
  int stale_lane_drops = 0;
  auto last_detection_at = std::chrono::steady_clock::time_point();
  const auto finish_lane_task = [&]() {
    // Draining both workers prevents pre-seek/pre-scene results from leaking.
    if (inference_running) {
      inference_future.wait();
      inference_running = false;
    }
    front_lane_target = rear_lane_target = adas::LaneResult();
    front_neural_lane_tracker.reset();
    rear_neural_lane_tracker.reset();
    front_lane_warning.reset();
    rear_lane_warning.reset();
    const auto reset_at = std::chrono::steady_clock::now();
    vehicle_state_manager = adas::VehicleStateManager();
    lane_change_fsm.request(adas::ManeuverDirection::NONE, reset_at);
    lane_semantic_tracker.reset();
    lane_semantic = adas::LaneSemantic();
    intersection_logic = adas::IntersectionLogic();
    intersection_result = adas::IntersectionResult();
    warning_summary = adas::WarningSummary();
    current_lights.clear();
    task_launched = {};
    lane_captured_at = {};
    lane_source_frame = {{-1, -1}};
    last_detection_at = std::chrono::steady_clock::time_point();
    if (lane_running) {
      lane_future.wait();
      lane_running = false;
    }
  };
  while (true) {
    const auto frame_start = std::chrono::steady_clock::now();
    double capture_ms = 0.0;
    double compose_ms = 0.0;
    double render_ms = 0.0;
    const bool calibration_paused = calibration_target != CalibrationTarget::NONE &&
                                    !source_images[0].empty();
    std::array<cv::Mat, 4> frames;
    fresh_measurement.fill(false);
    bool ok = true;
    if (!calibration_paused) {
      ok = read_synchronized_group(streams, single_composite,
                                   source_composite, source_images);
    }
    for (int i = 0; i < 4; ++i) frames[i] = source_images[i].clone();
    capture_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - frame_start).count();
    if (!ok) {
      if (options.once) break;
      finish_lane_task();
      signal = adas::SignalResult();
      drive = adas::DriveResult();
      if (single_composite) streams[0].reset();
      else for (int i = 0; i < 4; ++i) streams[i].reset();
      source_frame_index = 0;
      signal_logic = adas::SignalLogic();
      drive_logic.reset();
      front_lane_detector.reset();
      rear_lane_detector.reset();
      lane = adas::LaneResult();
      rear_lane = adas::LaneResult();
      front_lane_target = adas::LaneResult();
      rear_lane_target = adas::LaneResult();
      front_neural_lane_tracker.reset();
      rear_neural_lane_tracker.reset();
      front_lane_missed_updates = rear_lane_missed_updates = 0;
      front_risk_estimator.reset();
      rear_risk_estimator.reset();
      front_lane_warning.reset();
      rear_lane_warning.reset();
      left_blind_monitor.reset();
      right_blind_monitor.reset();
      for (auto& tracker : trackers) tracker.reset();
      for (auto& view : detections) view.clear();
      continue;
    }
    if (!calibration_paused) {
    if (inference_running &&
        inference_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      InferenceResult result = inference_future.get();
      const auto tracking_now = std::chrono::steady_clock::now();
      if (tracking_now - result.captured_at < std::chrono::milliseconds(500)) {
        for (int i = 0; i < 4; ++i) {
          detections[i] = trackers[i].update(result.detections[i], result.captured_at, tracking_now);
          observed[i] = trackers[i].observations();
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
        current_lights = verify_colors ? result.verified_lights : result.detections[0];
        vehicle_state = vehicle_state_manager.snapshot(tracking_now);
        intersection_result = intersection_logic.update(current_lights, vehicle_state);
      }
      inference_running = false;
    }
    const auto tracking_now = std::chrono::steady_clock::now();
    for (int i = 0; i < 4; ++i) detections[i] = trackers[i].predict(tracking_now);
    // Run the full PC estimator on a worker so its 40-100 ms CPU pass cannot
    // stall video presentation. The worker alternates front/rear and always
    // consumes the newest available frame.
    if (lane_running &&
        lane_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      LaneTaskResult result = lane_future.get();
      lane_running = false;
      lane_task_uses_npu = false;
      last_lane_ms = result.elapsed_ms;
      const bool fresh_lane = std::chrono::steady_clock::now() - result.captured_at <
                              std::chrono::milliseconds(300);
      if (fresh_lane) {
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
        adas::LaneResult geometry = result.neural ?
            neural_tracker.update(measured) : stabilize_lane(
                measured, target, misses,
                lane_blocked_by_vehicle(detections[c], frames[c].cols, frames[c].rows));
        // Use the video timeline for LDW hold/clear times.  Wall-clock NPU
        // scheduling varies between replays of the same scene.
        target = monitor.update(geometry, result.media_at);
        if (c == 0) {
          lane_semantic = lane_semantic_tracker.update(
              result.markings, target, result.media_at);
        }
        lane_captured_at[c] = result.captured_at;
        lane_source_frame[c] = result.source_frame;
      } else ++stale_lane_drops;
    }
    const int selected_task = ufld_lane_detector && !options.separate ?
        adas::next_perception_task(task_launched,
                                   options.ufld_rear && options.rear_lane_enabled,
                                   perception_periods) : -1;
    const bool detector_pending = selected_task >= 0 ? selected_task == 0 :
        shown_frames - last_detector_start >= options.detect_every;
    if (detector_pending && !inference_running &&
        (!lane_running || !lane_task_uses_npu)) {
      last_detector_start = shown_frames;
      task_launched[0] = frame_start;
      if (options.separate) {
        const int camera = (shown_frames / options.detect_every) % 4;
        const auto measured_at = std::chrono::steady_clock::now();
        detections[camera] = trackers[camera].update(
            detector.detect(frames[camera], &npu_ms), measured_at, measured_at);
        observed[camera] = trackers[camera].observations();
        detection_seconds[camera] = source_frame_index / source_fps;
        fresh_measurement[camera] = true;
        last_detection_at = measured_at;
        if (camera == 0) {
          const auto color_start = std::chrono::steady_clock::now();
          const auto verified = adas::verify_light_colors(frames[0], observed[0], &hsv_checked, &hsv_rejected);
          hsv_ms = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-color_start).count();
          signal = signal_logic.update(verify_colors ? verified : observed[0], frames[0].cols, frames[0].rows);
          drive = drive_logic.update(signal);
          current_lights = verify_colors ? verified : observed[0];
          vehicle_state = vehicle_state_manager.snapshot(measured_at);
          intersection_result = intersection_logic.update(current_lights, vehicle_state);
        }
      } else {
        const bool fpga_composite = !options.legacy_mosaic;
        const auto compose_start = std::chrono::steady_clock::now();
        cv::Mat inference_image = fpga_composite
            ? make_fpga_composite(frames) : make_mosaic(frames);
        compose_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - compose_start).count();
        const auto captured_at = frame_start;
        const double source_seconds = source_frame_index / source_fps;
        const cv::Mat light_frame = frames[0].clone();
        inference_future = std::async(std::launch::async,
            [&detector, inference_image, captured_at, fpga_composite, source_seconds, light_frame]() {
              InferenceResult result;
              result.captured_at = captured_at;
              result.source_seconds = source_seconds;
              const std::vector<adas::Detection> combined =
                  detector.detect(inference_image, &result.npu_ms);
              result.detections = fpga_composite
                  ? split_fpga_composite(combined) : split_mosaic(combined);
              const auto color_start = std::chrono::steady_clock::now();
              result.verified_lights = adas::verify_light_colors(light_frame,result.detections[0],
                  &result.hsv_checked,&result.hsv_rejected);
              result.hsv_ms = std::chrono::duration<double,std::milli>(
                  std::chrono::steady_clock::now()-color_start).count();
              return result;
            });
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
    if (!lane_running) {
      int camera = -1;
      bool use_ufld_task = false;
      if (ufld_lane_detector) {
        // RK3568 has one NPU execution queue. Start UFLD only between YOLO
        // jobs; never let the two contexts queue behind one another.
        if (!inference_running && !detector_pending &&
            shown_frames - last_ufld_start >= options.ufld_every) {
          camera = selected_task >= 0 ? selected_task - 1 :
              (options.ufld_rear && options.rear_lane_enabled &&
               (++ufld_task_sequence % 3 == 0) ? 1 : 0);
          use_ufld_task = true;
          last_ufld_start = shown_frames;
        } else if (options.rear_lane_enabled && !options.ufld_rear &&
                   shown_frames % 12 == 7) {
          // The CULane model is forward-facing. Keep the calibrated OpenCV
          // estimator for the rear tele camera at a lower rate.
          camera = 1;
        }
      } else {
        // Front is the driving view and needs PC-like consecutive updates.
        // Give it three worker slots for every rear update.
        camera = (lane_task_sequence++ % 4 == 3) ? 1 : 0;
      }
      if (camera >= 0) {
        task_launched[camera + 1] = frame_start;
        cv::Mat lane_frame = frames[camera].clone();
        // Preserve the FPGA composite's native 960x540 front pixels for the
        // full-width UFLDv2 model. UI tiles remain 640x360 and receive scaled
        // lane geometry after inference.
        if (camera == 0 && single_composite &&
            source_composite.cols >= kFpgaCompositeW &&
            source_composite.rows >= kFpgaCompositeH) {
          lane_frame = source_composite(cv::Rect(0, 0, source_composite.cols / 2,
                                                 source_composite.rows / 2)).clone();
        }
        const auto task_captured_at = frame_start;
        const int task_source_frame = source_frame_index;
        const auto task_media_at = std::chrono::steady_clock::time_point(
            std::chrono::seconds(1)) +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(task_source_frame / source_fps));
        adas::UfldLaneDetector* ufld = ufld_lane_detector.get();
        lane_future = std::async(std::launch::async,
            [&front_lane_detector, &rear_lane_detector, ufld, lane_frame, camera,
             use_ufld_task, task_captured_at, task_media_at, task_source_frame]() {
              const auto started = std::chrono::steady_clock::now();
              LaneTaskResult result;
              result.camera = camera;
              result.neural = use_ufld_task;
              result.captured_at = task_captured_at;
              result.media_at = task_media_at;
              result.source_frame = task_source_frame;
              result.image_width = lane_frame.cols;
              result.image_height = lane_frame.rows;
              if (use_ufld_task) {
                result.lane = ufld->detect(lane_frame);
              } else {
                result.lane = camera == 0 ? front_lane_detector.detect(lane_frame)
                                          : rear_lane_detector.detect(lane_frame);
              }
              if (camera == 0) {
                result.markings = adas::observe_lane_markings(lane_frame, result.lane);
              }
              result.elapsed_ms = std::chrono::duration<double, std::milli>(
                  std::chrono::steady_clock::now() - started).count();
              return result;
            });
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
    if (last_detection_at == std::chrono::steady_clock::time_point() ||
        presentation_now - last_detection_at > std::chrono::milliseconds(800)) {
      signal_logic = adas::SignalLogic();
      drive_logic.reset();
      signal = adas::SignalResult();
      drive = adas::DriveResult();
      front_risk_estimator.reset();
      rear_risk_estimator.reset();
      left_blind_monitor.reset();
      right_blind_monitor.reset();
      current_lights.clear();
      intersection_logic = adas::IntersectionLogic();
      intersection_result = adas::IntersectionResult();
    }
    front_risk = front_risk_estimator.update(
        observed[0], frames[0].cols, frames[0].rows, &lane, fresh_measurement[0], detection_seconds[0]);
    rear_risk = rear_risk_estimator.update(
        observed[1], frames[1].cols, frames[1].rows, &rear_lane, fresh_measurement[1], detection_seconds[1]);
    left_blind = left_blind_monitor.update(
        observed[2], frames[2].cols, frames[2].rows, fresh_measurement[2]);
    right_blind = right_blind_monitor.update(
        observed[3], frames[3].cols, frames[3].rows, fresh_measurement[3]);
    }
    const auto advice_now = std::chrono::steady_clock::now();
    vehicle_state = vehicle_state_manager.snapshot(advice_now);
    const bool side_fresh = last_detection_at != std::chrono::steady_clock::time_point() &&
        advice_now - last_detection_at < std::chrono::milliseconds(800);
    lane_change_result = lane_change_fsm.update(
        vehicle_state, lane_semantic, left_blind.occupied, right_blind.occupied,
        side_fresh, near_lane_change_regulation(observed[0]), advice_now);
    warning_summary = warning_manager.update(
        front_risk.warning, rear_risk.warning, left_blind.occupied,
        right_blind.occupied, lane.departure, lane_change_result,
        intersection_result);
    const auto render_start = std::chrono::steady_clock::now();
    adas::draw_overlay(frames[0], detections[0], lane, signal, front_risk,
                       drive, display_fps, npu_ms, &lane_semantic);
    draw_lane_geometry(frames[1], rear_lane);
    draw_boxes(frames[1], detections[1], &rear_risk);
    const bool left_intent = vehicle_state.turn_valid &&
        vehicle_state.turn == adas::TurnSignal::LEFT;
    const bool right_intent = vehicle_state.turn_valid &&
        vehicle_state.turn == adas::TurnSignal::RIGHT;
    draw_boxes(frames[2], detections[2], nullptr, left_blind.track_id,
               left_blind.occupied && left_intent);
    draw_boxes(frames[3], detections[3], nullptr, right_blind.track_id,
               right_blind.occupied && right_intent);
    draw_rear_risk_overlay(frames[1], rear_risk);
    draw_blind_spot_overlay(frames[2], left_blind, left_intent, "LEFT");
    draw_blind_spot_overlay(frames[3], right_blind, right_intent, "RIGHT");
    if (rear_risk.warning) warning_banner(frames[1], "后车快速接近  TTC预警");
    if (left_blind.occupied)
      warning_banner(frames[2], "左侧盲区有目标");
    if (right_blind.occupied)
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
    const bool refresh_dashboard = shown_frames < 2 || shown_frames % 3 == 0;
    if (refresh_dashboard) {
      const bool health_ready = shown_frames >
          std::max(10, static_cast<int>(std::max(1.0, source_fps) * 2.0));
      draw_sidebar(canvas, display_fps, instant_fps, npu_ms, signal, detections, lane, rear_lane,
                   drive, front_risk, rear_risk, left_blind, right_blind, side_fresh,
                   health_ready,
                   last_lane_ms, lane_age_ms, vehicle_state, lane_semantic,
                   lane_change_result, intersection_result, verify_colors);
      draw_bottom_left(canvas, scene_index,
                       static_cast<int>(scene_paths.size()),
                       options.scene.substr(options.scene.find_last_of("/\\") + 1));
      caption(canvas, warning_summary.priority > 0 ? warning_summary.text : lane_change_result.reason,
          1404,1052, warning_summary.priority >= 70 ? cv::Scalar(80,80,245)
                                                    : cv::Scalar(215,188,130),.46);
      draw_progress_bar(canvas, source_frame_index, source_frames, calibration_paused);
    }
    render_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - render_start).count();
    const int snapshot_frame = options.snapshot_frame >= 0 ? options.snapshot_frame :
        (options.max_frames > 0 ? std::min(4, options.max_frames - 1) : 4);
    if (!options.snapshot.empty() && shown_frames == snapshot_frame) {
      cv::Mat snapshot = canvas;
      if (gl_presenter) {
        snapshot = canvas.clone();
        for (int i = 0; i < 4; ++i) draw_tile(snapshot, frames[i], i);
      }
      if (!cv::imwrite(options.snapshot, snapshot)) std::cerr << "Could not save snapshot\n";
    }
    if (!options.headless) {
      int key = -1;
      int click_x = -1, click_y = -1;
      int scene_delta = 0;
      if (gl_presenter) {
        gl_presenter->present(canvas, frames, kViewRects);
        key = gl_presenter->poll_input(&click_x, &click_y);
      } else {
        cv::imshow("RK3568 V7 | NVIDIA 4 VIEW", canvas);
        key = cv::waitKey(1) & 0xff;
      }
      if (key == 'q' || key == 27) break;
      if (key == 'm') {
        options.separate = !options.separate;
        for (auto& tracker : trackers) tracker.reset();
        for (auto& view : detections) view.clear();
        signal_logic = adas::SignalLogic();
        drive_logic.reset();
        signal = adas::SignalResult();
        drive = adas::DriveResult();
        front_risk_estimator.reset();
        rear_risk_estimator.reset();
        left_blind_monitor.reset();
        right_blind_monitor.reset();
      }
      if (key == '[') scene_delta = -1;
      if (key == ']') scene_delta = 1;
      if (click_x >= 0 && click_y >= 0) {
        const cv::Point click(click_x, click_y);
        const auto intent_now=std::chrono::steady_clock::now();
        if (kIntentLeft.contains(click)) {
          vehicle_state_manager.set_demo_turn(adas::TurnSignal::LEFT, intent_now);
          lane_change_fsm.request(adas::ManeuverDirection::LEFT, intent_now);
        }
        if (kIntentOff.contains(click)) {
          vehicle_state_manager.set_demo_turn(adas::TurnSignal::OFF, intent_now);
          lane_change_fsm.request(adas::ManeuverDirection::NONE, intent_now);
        }
        if (kIntentRight.contains(click)) {
          vehicle_state_manager.set_demo_turn(adas::TurnSignal::RIGHT, intent_now);
          lane_change_fsm.request(adas::ManeuverDirection::RIGHT, intent_now);
        }
        if (kSignalVerify.contains(click)) {
          verify_colors=!verify_colors;
          signal_logic=adas::SignalLogic(); drive_logic.reset();
          signal=adas::SignalResult(); drive=adas::DriveResult();
          std::cout << "HSV verification=" << verify_colors << std::endl;
        }
        if (kPrevSceneButton.contains(click)) scene_delta = -1;
        if (kNextSceneButton.contains(click)) scene_delta = 1;
      }
      if (key == 'c' || key == 'r') {
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
          for (auto& tracker : trackers) tracker.reset();
          for (auto& view : detections) view.clear();
          signal_logic = adas::SignalLogic();
          drive_logic.reset();
          front_risk_estimator.reset();
          rear_risk_estimator.reset();
          front_lane_warning.reset();
          rear_lane_warning.reset();
          left_blind_monitor.reset();
          right_blind_monitor.reset();
          lane = adas::LaneResult();
          rear_lane = adas::LaneResult();
          front_lane_target = adas::LaneResult();
          rear_lane_target = adas::LaneResult();
          front_neural_lane_tracker.reset();
          rear_neural_lane_tracker.reset();
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
                front_neural_lane_tracker.reset();
                front_lane_missed_updates = 0;
              } else {
                rear_lane_warning.set_reference_from_roi(calibration_points);
                rear_lane = adas::LaneResult();
                rear_lane_target = adas::LaneResult();
                rear_neural_lane_tracker.reset();
                rear_lane_missed_updates = 0;
              }
              calibration_target = CalibrationTarget::NONE;
            } else {
              calibration_points.clear();
            }
          }
        }
      }
      if (scene_delta != 0 && scene_paths.size() > 1) {
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
          source_fps = next_fps;
          source_frames = next_frames;
          single_composite = next_single_composite;
          source_frame_index = 0;
          shown_frames = 0;
          lane_task_sequence = 0;
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
          front_neural_lane_tracker.reset();
          rear_neural_lane_tracker.reset();
          front_lane_missed_updates = rear_lane_missed_updates = 0;
          front_risk_estimator.reset();
          rear_risk_estimator.reset();
          front_lane_warning.reset();
          rear_lane_warning.reset();
          left_blind_monitor.reset();
          right_blind_monitor.reset();
          for (auto& tracker : trackers) tracker.reset();
          for (auto& view : detections) view.clear();
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
    if (!calibration_paused) ++source_frame_index;
    if (options.max_frames > 0 && shown_frames >= options.max_frames) break;
    if (calibration_paused) {
      std::this_thread::sleep_for(std::chrono::milliseconds(33));
    } else if (!options.benchmark) {
      const double playback_fps = options.display_fps_limit > 0.0
          ? std::min(source_fps, options.display_fps_limit) : source_fps;
      const double period = 1.0 / playback_fps;
      const double spent = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - frame_start).count();
      if (spent < period) {
        std::this_thread::sleep_for(std::chrono::duration<double>(period - spent));
      } else {
        // Do not grab/drop source frames. The PC demo processes frames in
        // sequence; skipping here made its temporal lane tracker jump several
        // frames at a time. When RK3568 is slower than the 30 FPS source, play
        // the demo slower instead of corrupting the lane motion sequence.
      }
    }
    if (shown_frames % 30 == 0) std::cout << "shown=" << shown_frames
        << " source_frame=" << source_frame_index << " fps_1s=" << display_fps
        << " fps_instant=" << instant_fps
        << " npu_ms=" << npu_ms << " lanes=" << lane.valid << ',' << rear_lane.valid
        << " capture_ms=" << capture_ms << " lane_ms=" << last_lane_ms
        << " compose_ms=" << compose_ms << " render_ms=" << render_ms
        << " frame_ms=" << std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - frame_start).count()
        << " lane_age_ms=" << lane_age_ms[0] << ',' << lane_age_ms[1]
        << " lane_source_frame=" << lane_source_frame[0] << ',' << lane_source_frame[1]
        << " hsv=" << verify_colors << " hsv_checked=" << hsv_checked
        << " hsv_rejected=" << hsv_rejected << " hsv_ms=" << hsv_ms
        << " lane_change=" << static_cast<int>(lane_change_result.state)
        << " lane_marking=" << static_cast<int>(lane_semantic.left) << ','
        << static_cast<int>(lane_semantic.right)
        << " lane_marking_evidence=" << lane_semantic.left_evidence << ','
        << lane_semantic.right_evidence
        << " tlc_s=" << lane_semantic.tlc_s
        << " intersection=" << static_cast<int>(intersection_result.action)
        << " warning=" << static_cast<int>(warning_summary.primary)
        << " stale_lane_drops=" << stale_lane_drops
        << " lane_offset=" << lane.offset_ratio << ',' << rear_lane.offset_ratio
        << " lane_departure=" << lane.departure << ',' << rear_lane.departure
        << " departure_side=" << static_cast<int>(lane.departure_side)
        << " lane_identity=" << lane.identity_uncertain
        << " lane_reassignment=" << static_cast<int>(lane.reassignment_side)
        << " risk_target=" << front_risk.target << ',' << rear_risk.target
        << " risk_warning=" << front_risk.warning << ',' << rear_risk.warning
        << " distance_m=" << front_risk.distance_m << ',' << rear_risk.distance_m
        << " ttc_s=" << front_risk.ttc_s << ',' << rear_risk.ttc_s
        << " blind=" << left_blind.occupied << ',' << right_blind.occupied
        << std::endl;
  }
  finish_lane_task();
  if (inference_running) inference_future.wait();
  const double seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();
  std::cout << "Done. shown=" << shown_frames << " wall_fps="
            << (seconds > 0.0 ? shown_frames / seconds : 0.0) << '\n';
  return 0;
}
