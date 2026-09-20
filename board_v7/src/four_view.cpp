#include "adas/config.hpp"
#include "adas/byte_tracker.hpp"
#include "adas/drive_decision.hpp"
#include "adas/lane_detector.hpp"
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

namespace {

const int kTileW = 640;
const int kTileH = 360;
const int kHeaderH = 60;
const int kFooterH = 300;
const int kSideW = 640;
const int kCanvasW = 2 * kTileW + kSideW;
const int kCanvasH = 1080;
const int kFpgaViewW = 960;
const int kFpgaViewH = 540;
const int kFpgaCompositeW = 2 * kFpgaViewW;
const int kFpgaCompositeH = 2 * kFpgaViewH;
const char* kFiles[] = {"front.mp4", "rear.mp4", "left.mp4", "right.mp4"};
const char* kTitles[] = {"前视 / YOLO V7", "后视 / YOLO V7", "左侧 / YOLO V7", "右侧 / YOLO V7"};
const cv::Rect kPrevSceneButton(834, 936, 196, 48);
const cv::Rect kNextSceneButton(1048, 936, 196, 48);

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
  bool headless = false;
  bool benchmark = false;
  bool once = false;
  bool separate = false;
  bool dump_detections = false;
  bool opencv_display = false;
  bool software_decode = false;
  bool legacy_mosaic = false;
  bool ufld_rear = false;
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
    if (key == "--help") {
      std::cout << "--scene DIR --model FILE --detect-every N --cpu-threads N "
                   "--ufld-model FILE --ufld-every N --max-frames N --snapshot FILE --snapshot-frame N "
                   "--headless --benchmark --once "
                   "--separate --dump-detections --opencv --software-decode --legacy-mosaic --ufld-rear\n";
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
  std::array<std::vector<adas::Detection>, 4> detections;
  double npu_ms = 0.0;
  std::chrono::steady_clock::time_point captured_at{};
};

struct LaneTaskResult {
  int camera = 0;
  adas::LaneResult lane;
  bool neural = false;
  double elapsed_ms = 0.0;
  std::chrono::steady_clock::time_point captured_at{};
};

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

void draw_roi_status(cv::Mat& frame, const std::vector<cv::Point2f>& points,
                     const adas::LaneResult& lane) {
  if (points.size() != 4) return;
  std::vector<cv::Point> polygon;
  for (const auto& point : points)
    polygon.emplace_back(static_cast<int>(point.x * frame.cols),
                         static_cast<int>(point.y * frame.rows));
  if (!lane.valid) {
    cv::polylines(frame, polygon, true, cv::Scalar(0, 180, 255), 2, cv::LINE_AA);
  } else {
    // Once a lane is available, leave only small corner markers. A full
    // trapezoid is the calibration search area and can be mistaken for a
    // straight detected lane on curved roads.
    for (const cv::Point& point : polygon)
      cv::circle(frame, point, 3, cv::Scalar(80, 255, 180), cv::FILLED, cv::LINE_AA);
  }
  adas::ui::draw_text(frame, lane.valid ? "已标定 / 检测到车道" : "已标定 / 正在搜索车道",
                      cv::Point(12, frame.rows - 12), 15,
                      lane.valid ? cv::Scalar(80, 255, 180) : cv::Scalar(0, 180, 255));
}

void draw_lane_geometry(cv::Mat& frame, const adas::LaneResult& lane) {
  if (!lane.valid) return;
  cv::Mat layer = frame.clone();
  cv::fillPoly(layer, std::vector<std::vector<cv::Point>>(1, lane.polygon),
               lane.departure ? cv::Scalar(0, 70, 255) : cv::Scalar(20, 190, 70));
  cv::addWeighted(layer, 0.24, frame, 0.76, 0, frame);
  cv::polylines(frame, lane.left, false, cv::Scalar(40, 255, 80), 5, cv::LINE_AA);
  cv::polylines(frame, lane.right, false, cv::Scalar(40, 255, 80), 5, cv::LINE_AA);
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
  result.departure = std::abs(result.offset_ratio) > 0.12f;
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
  cv::rectangle(canvas, cv::Rect(x, y, w, h), cv::Scalar(28, 34, 40), cv::FILLED);
  cv::rectangle(canvas, cv::Rect(x, y, w, h), cv::Scalar(53, 65, 72), 1);
}

cv::Scalar box_color(int cls) {
  if (cls >= 7 && cls <= 10) return cv::Scalar(55, 75, 255);
  if (cls >= 11 && cls <= 14) return cv::Scalar(70, 230, 95);
  if (cls == 0 || cls == 1) return cv::Scalar(210, 100, 240);
  return cv::Scalar(90, 200, 255);
}

void draw_boxes(cv::Mat& frame, const std::vector<adas::Detection>& detections) {
  for (const auto& d : detections) {
    const cv::Scalar color = box_color(d.class_id);
    cv::rectangle(frame, d.box, color, 2);
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

void warning_banner(cv::Mat& frame, const std::string& text) {
  const cv::Rect area(8, frame.rows - 38, std::min(frame.cols - 16, 330), 30);
  cv::rectangle(frame, area, cv::Scalar(20, 35, 190), cv::FILLED);
  adas::ui::draw_text(frame, text, cv::Point(area.x + 8, area.y + 22),
                      16, cv::Scalar(255, 255, 255));
}

const cv::Rect kProgressBar(20, 1052, 1240, 14);

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
  caption(canvas, label.str(), 1010, 1043, cv::Scalar(175, 205, 215), 0.43);
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
  const int x = (index % 2) * kTileW;
  const int y = kHeaderH + (index / 2) * kTileH;
  cv::Mat tile;
  if (frame.cols == kTileW && frame.rows == kTileH) tile = frame;
  else cv::resize(frame, tile, cv::Size(kTileW, kTileH), 0, 0, cv::INTER_AREA);
  tile.copyTo(canvas(cv::Rect(x, y, kTileW, kTileH)));
  cv::rectangle(canvas, cv::Rect(x, y, kTileW, kTileH), cv::Scalar(52, 63, 70), 2);
  cv::rectangle(canvas, cv::Rect(x + 8, y + 7, 207, 25), cv::Scalar(14, 22, 29), cv::FILLED);
  caption(canvas, kTitles[index], x + 16, y + 25, cv::Scalar(235, 245, 250), 0.48);
}

void draw_tile_label(cv::Mat& frame, int index) {
  cv::rectangle(frame, cv::Rect(0, 0, kTileW, kTileH), cv::Scalar(52, 63, 70), 2);
  cv::rectangle(frame, cv::Rect(8, 7, 207, 25), cv::Scalar(14, 22, 29), cv::FILLED);
  caption(frame, kTitles[index], 16, 25, cv::Scalar(235, 245, 250), 0.48);
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
                       const adas::SignalResult& signal) {
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
      if (detection.class_id <= 1) {
        cv::circle(canvas, position, 5, cv::Scalar(220, 80, 220), cv::FILLED);
      } else {
        const int icon_width = 9 + cvRound(apparent * 5.0f);
        const int icon_height = 15 + cvRound(apparent * 8.0f);
        draw_top_car(canvas, position, icon_width, icon_height,
                     cv::Scalar(165, 172, 176), camera >= 2, false);
      }
    }
  }
  draw_top_car(canvas, ego, 18, 29, cv::Scalar(100, 230, 150), false, true);
}

void draw_sidebar(cv::Mat& canvas, double fps, double npu_ms, const adas::SignalResult& signal,
                  const std::array<std::vector<adas::Detection>, 4>& detections,
                  const adas::LaneResult& front_lane,
                  const adas::LaneResult& rear_lane, int frame_index,
                  const std::string& scene_name) {
  const int x = 2 * kTileW + 16;
  const int w = kSideW - 32;
  panel(canvas, x, kHeaderH + 12, w, 74);
  caption(canvas, "系统状态", x + 14, kHeaderH + 36, cv::Scalar(150, 190, 205), 0.48);
  caption(canvas, "NPU运行   CPU 2-3   四路摄像头", x + 14, kHeaderH + 65,
          cv::Scalar(100, 235, 175), 0.49);

  panel(canvas, x, kHeaderH + 98, w, 116);
  caption(canvas, "运行性能", x + 14, kHeaderH + 121, cv::Scalar(150, 190, 205), 0.48);
  std::ostringstream perf;
  perf << std::fixed << std::setprecision(1) << fps << " FPS（1秒）";
  caption(canvas, perf.str(), x + 14, kHeaderH + 169, cv::Scalar(240, 245, 250), 0.92);
  std::ostringstream npu;
  npu << "NPU " << std::fixed << std::setprecision(1) << npu_ms << " ms";
  caption(canvas, npu.str(), x + 285, kHeaderH + 168, cv::Scalar(100, 220, 255), 0.48);
  caption(canvas, "四路视频按帧同步", x + 14, kHeaderH + 197,
          cv::Scalar(130, 155, 170), 0.43);

  panel(canvas, x, kHeaderH + 226, w, 88);
  caption(canvas, "前方信号灯", x + 14, kHeaderH + 250, cv::Scalar(150, 190, 205), 0.48);
  const cv::Scalar sig_color = signal.state == adas::SignalState::RED ? cv::Scalar(70, 80, 255) :
      signal.state == adas::SignalState::GREEN ? cv::Scalar(80, 235, 100) : cv::Scalar(150, 170, 180);
  const char* signal_text = signal.state == adas::SignalState::RED ? "红灯" :
      signal.state == adas::SignalState::GREEN ? "绿灯" : "无";
  caption(canvas, signal_text, x + 14, kHeaderH + 294, sig_color, 0.95);

  panel(canvas, x, kHeaderH + 326, w, 226);
  caption(canvas, "四路检测结果", x + 14, kHeaderH + 352,
          cv::Scalar(150, 190, 205), 0.48);
  const char* camera_names[] = {"前视", "后视", "左侧", "右侧"};
  for (int i = 0; i < 4; ++i) {
    std::ostringstream row;
    row << camera_names[i] << "   " << detections[i].size() << " 个目标";
    caption(canvas, row.str(), x + 15, kHeaderH + 383 + i * 33,
            cv::Scalar(220, 235, 240), 0.51);
  }
  if (!detections[0].empty()) {
    const auto& d = detections[0].front();
    std::ostringstream row;
    row << "前视主要目标：" << d.name << ' ' << std::fixed << std::setprecision(2) << d.score;
    caption(canvas, row.str(), x + 15, kHeaderH + 531, box_color(d.class_id), 0.45);
  }

  panel(canvas, x, kHeaderH + 564, w, 120);
  caption(canvas, "数据状态", x + 14, kHeaderH + 590, cv::Scalar(150, 190, 205), 0.48);
  caption(canvas, "车速：--   路线：--", x + 14, kHeaderH + 622,
          cv::Scalar(220, 225, 235), 0.51);
  caption(canvas, "四路共用一次 NPU 推理", x + 14, kHeaderH + 650,
          cv::Scalar(150, 180, 195), 0.46);
  std::ostringstream clip;
  clip << scene_name.substr(0, 8) << "   帧 " << frame_index;
  caption(canvas, clip.str(), x + 14, kHeaderH + 675,
          cv::Scalar(130, 155, 170), 0.43);

  const int map_y = kHeaderH + 696;
  panel(canvas, x, map_y, w, 126);
  caption(canvas, "四路目标定位", x + 14, kHeaderH + 722,
          cv::Scalar(150, 190, 205), 0.48);
  caption(canvas, "ByteTrack 俯视示意", x + 14, kHeaderH + 765,
          cv::Scalar(235, 240, 245), 0.50);
  caption(canvas, "非真实世界坐标", x + 14, kHeaderH + 798,
          cv::Scalar(140, 165, 180), 0.43);
  draw_surround_map(canvas, cv::Rect(x + 196, map_y + 9, w - 210, 108), detections,
                    front_lane, rear_lane, signal);

  panel(canvas, x, kHeaderH + 834, w, 164);
  caption(canvas, "摄像头 / 模型", x + 14, kHeaderH + 862,
          cv::Scalar(150, 190, 205), 0.48);
  caption(canvas, "四路同步 30 FPS 视频", x + 14, kHeaderH + 902,
          cv::Scalar(235, 240, 245), 0.59);
  caption(canvas, "YOLOv8n P2  |  21类  |  RK3568 NPU", x + 14,
          kHeaderH + 937, cv::Scalar(160, 215, 220), 0.54);
  caption(canvas, "目标位置当前为图像坐标", x + 14,
          kHeaderH + 974, cv::Scalar(140, 165, 180), 0.48);
}

void draw_bottom_left(cv::Mat& canvas, double fps, double npu_ms,
                      const std::array<std::vector<adas::Detection>, 4>& detections,
                      int frame_index, bool separate, bool fpga_composite,
                      int scene_index,
                      int scene_count, const std::string& scene_name) {
  const int y = kHeaderH + 2 * kTileH + 12;
  panel(canvas, 14, y, 390, kFooterH - 26);
  panel(canvas, 417, y, 390, kFooterH - 26);
  panel(canvas, 820, y, 446, kFooterH - 26);
  caption(canvas, "当前检测", 28, y + 29,
          cv::Scalar(150, 190, 205), 0.51);
  const char* labels[] = {"前视", "后视", "左侧", "右侧"};
  for (int i = 0; i < 4; ++i) {
    std::ostringstream row;
    row << labels[i] << "：" << detections[i].size() << " 个目标";
    if (!detections[i].empty())
      row << "  主要 " << detections[i].front().name;
    caption(canvas, row.str(), 28, y + 68 + i * 38,
            cv::Scalar(220, 230, 235), 0.49);
  }
  caption(canvas, "标签随每次 NPU 推理刷新", 28, y + 239,
          cv::Scalar(130, 160, 175), 0.45);

  caption(canvas, "系统", 431, y + 29,
          cv::Scalar(150, 190, 205), 0.51);
  std::ostringstream rate;
  rate << "FPS（近1秒） " << std::fixed << std::setprecision(1) << fps;
  caption(canvas, rate.str(), 431, y + 69, cv::Scalar(120, 235, 175), 0.56);
  std::ostringstream npu;
  npu << "NPU耗时       " << std::fixed << std::setprecision(1) << npu_ms << " ms";
  caption(canvas, npu.str(), 431, y + 105, cv::Scalar(225, 235, 240), 0.53);
  caption(canvas, "CPU绑定          2,3", 431, y + 141,
          cv::Scalar(225, 235, 240), 0.53);
  std::ostringstream sync;
  sync << "源视频帧       " << frame_index;
  caption(canvas, sync.str(), 431, y + 177,
          cv::Scalar(225, 235, 240), 0.53);
  caption(canvas, separate ? "推理：分路轮询" :
          (fpga_composite ? "推理：FPGA 1080p 单帧" : "推理：四路拼接单次调用"),
          431, y + 225, cv::Scalar(130, 190, 220), 0.49);

  caption(canvas, "操作", 834, y + 29,
          cv::Scalar(150, 190, 205), 0.51);
  caption(canvas, "Q / ESC     关闭演示", 834, y + 75,
          cv::Scalar(230, 238, 242), 0.59);
  caption(canvas, "C / R          标定前视 / 后视车道", 834, y + 117,
          cv::Scalar(230, 238, 242), 0.55);
  cv::rectangle(canvas, kPrevSceneButton, cv::Scalar(38, 67, 88), cv::FILLED);
  cv::rectangle(canvas, kPrevSceneButton, cv::Scalar(95, 175, 215), 2);
  cv::rectangle(canvas, kNextSceneButton, cv::Scalar(38, 67, 88), cv::FILLED);
  cv::rectangle(canvas, kNextSceneButton, cv::Scalar(95, 175, 215), 2);
  caption(canvas, "<  上一场景", kPrevSceneButton.x + 24, kPrevSceneButton.y + 31,
          cv::Scalar(220, 240, 250), 0.52);
  caption(canvas, "下一场景  >", kNextSceneButton.x + 22, kNextSceneButton.y + 31,
          cv::Scalar(220, 240, 250), 0.52);
  std::ostringstream scene;
  scene << "场景 " << (scene_index + 1) << '/' << scene_count << "  "
        << scene_name.substr(0, 8) << "   按键 [ / ]";
  caption(canvas, scene.str(), 834, y + 226, cv::Scalar(150, 205, 215), 0.47);
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
  int front_lane_missed_updates = 0;
  int rear_lane_missed_updates = 0;
  adas::RiskResult front_risk;
  adas::RiskResult rear_risk;
  adas::BlindSpotResult left_blind;
  adas::BlindSpotResult right_blind;
  std::array<std::vector<adas::Detection>, 4> detections;
  std::array<bool, 4> fresh_measurement{{false, false, false, false}};
  std::future<InferenceResult> inference_future;
  bool inference_running = false;
  std::future<LaneTaskResult> lane_future;
  bool lane_running = false;
  bool lane_task_uses_npu = false;
  int lane_task_sequence = 0;
  int ufld_task_sequence = 0;
  int last_detector_start = -1000000;
  int last_ufld_start = -1000000;
  double npu_ms = 0.0;
  double display_fps = 0.0;
  std::deque<std::chrono::steady_clock::time_point> presentation_times;
  auto fps_refresh = std::chrono::steady_clock::now();
  double skip_debt = 0.0;
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
  cv::Mat canvas(kCanvasH, kCanvasW, CV_8UC3, cv::Scalar(19, 24, 29));
  cv::rectangle(canvas, cv::Rect(0, 0, kCanvasW, kHeaderH),
                cv::Scalar(29, 36, 43), cv::FILLED);
  caption(canvas, "ROADFUSION EDGE  |  NVIDIA PHYSICALAI  |  四路智能驾驶辅助",
          20, 31, cv::Scalar(235, 240, 245), 0.62);
  std::array<cv::Mat, 4> source_images;
  cv::Mat source_composite;
  double last_lane_ms = 0.0;
  const auto finish_lane_task = [&]() {
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
      if (single_composite) streams[0].reset();
      else for (int i = 0; i < 4; ++i) streams[i].reset();
      source_frame_index = 0;
      skip_debt = 0.0;
      signal_logic = adas::SignalLogic();
      drive_logic.reset();
      front_lane_detector.reset();
      rear_lane_detector.reset();
      lane = adas::LaneResult();
      rear_lane = adas::LaneResult();
      front_lane_target = adas::LaneResult();
      rear_lane_target = adas::LaneResult();
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
          fresh_measurement[i] = true;
        }
        npu_ms = result.npu_ms;
        signal = signal_logic.update(detections[0], frames[0].cols, frames[0].rows);
        drive = drive_logic.update(signal);
      }
      inference_running = false;
    }
    const auto tracking_now = std::chrono::steady_clock::now();
    for (int i = 0; i < 4; ++i) detections[i] = trackers[i].predict(tracking_now);
    const bool detector_pending =
        shown_frames - last_detector_start >= options.detect_every;
    if (detector_pending && !inference_running &&
        (!lane_running || !lane_task_uses_npu)) {
      last_detector_start = shown_frames;
      if (options.separate) {
        const int camera = (shown_frames / options.detect_every) % 4;
        const auto measured_at = std::chrono::steady_clock::now();
        detections[camera] = trackers[camera].update(
            detector.detect(frames[camera], &npu_ms), measured_at, measured_at);
        fresh_measurement[camera] = true;
        signal = signal_logic.update(detections[0], frames[0].cols, frames[0].rows);
      } else {
        const bool fpga_composite = !options.legacy_mosaic;
        const auto compose_start = std::chrono::steady_clock::now();
        cv::Mat inference_image = fpga_composite
            ? make_fpga_composite(frames) : make_mosaic(frames);
        compose_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - compose_start).count();
        const auto captured_at = std::chrono::steady_clock::now();
        inference_future = std::async(std::launch::async,
            [&detector, inference_image, captured_at, fpga_composite]() {
              InferenceResult result;
              result.captured_at = captured_at;
              const std::vector<adas::Detection> combined =
                  detector.detect(inference_image, &result.npu_ms);
              result.detections = fpga_composite
                  ? split_fpga_composite(combined) : split_mosaic(combined);
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
                              std::chrono::milliseconds(result.neural ? 450 : 250);
      if (fresh_lane) {
        if (result.camera == 0) {
          const adas::LaneResult measured = front_lane_warning.update(result.lane);
          front_lane_target = stabilize_lane(
              measured, front_lane_target, front_lane_missed_updates,
              lane_blocked_by_vehicle(detections[0], frames[0].cols, frames[0].rows));
        } else {
          const adas::LaneResult measured = rear_lane_warning.update(result.lane);
          rear_lane_target = stabilize_lane(
              measured, rear_lane_target, rear_lane_missed_updates,
              lane_blocked_by_vehicle(detections[1], frames[1].cols, frames[1].rows));
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
          // Front drives LDW/FCW and is more latency-sensitive. When rear lane
          // detection is enabled, reserve one of five jobs for it.
          camera = options.ufld_rear && (++ufld_task_sequence % 5 == 0) ? 1 : 0;
          use_ufld_task = true;
          last_ufld_start = shown_frames;
        } else if (!options.ufld_rear && shown_frames % 12 == 7) {
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
        cv::Mat lane_frame = frames[camera].clone();
        const auto lane_captured_at = std::chrono::steady_clock::now();
        adas::UfldLaneDetector* ufld = ufld_lane_detector.get();
        lane_future = std::async(std::launch::async,
            [&front_lane_detector, &rear_lane_detector, ufld, lane_frame, camera,
             use_ufld_task, lane_captured_at]() {
              const auto started = std::chrono::steady_clock::now();
              LaneTaskResult result;
              result.camera = camera;
              result.neural = use_ufld_task;
              result.captured_at = lane_captured_at;
              if (use_ufld_task) {
                result.lane = ufld->detect(lane_frame);
              } else {
                result.lane = camera == 0 ? front_lane_detector.detect(lane_frame)
                                          : rear_lane_detector.detect(lane_frame);
              }
              result.elapsed_ms = std::chrono::duration<double, std::milli>(
                  std::chrono::steady_clock::now() - started).count();
              return result;
            });
        lane_running = true;
        lane_task_uses_npu = use_ufld_task;
      }
    }
    lane = interpolate_lane(lane, front_lane_target, 0.95f);
    rear_lane = interpolate_lane(rear_lane, rear_lane_target, 0.82f);
    front_risk = front_risk_estimator.update(
        detections[0], frames[0].cols, frames[0].rows, &lane, fresh_measurement[0]);
    rear_risk = rear_risk_estimator.update(
        detections[1], frames[1].cols, frames[1].rows, &rear_lane, fresh_measurement[1]);
    left_blind = left_blind_monitor.update(
        detections[2], frames[2].cols, frames[2].rows, fresh_measurement[2]);
    right_blind = right_blind_monitor.update(
        detections[3], frames[3].cols, frames[3].rows, fresh_measurement[3]);
    }
    const auto render_start = std::chrono::steady_clock::now();
    adas::draw_overlay(frames[0], detections[0], lane, signal, front_risk,
                       drive, display_fps, npu_ms);
    draw_lane_geometry(frames[1], rear_lane);
    draw_roi_status(frames[0], front_roi_points, lane);
    draw_roi_status(frames[1], rear_roi_points, rear_lane);
    for (int i = 1; i < 4; ++i) draw_boxes(frames[i], detections[i]);
    if (rear_risk.warning) warning_banner(frames[1], "后车快速接近  TTC预警");
    if (left_blind.occupied)
      warning_banner(frames[2], "左侧盲区有目标");
    if (right_blind.occupied)
      warning_banner(frames[3], "右侧盲区有目标");
    if (calibration_target == CalibrationTarget::FRONT)
      draw_calibration(frames[0], calibration_points, "前视");
    if (calibration_target == CalibrationTarget::REAR)
      draw_calibration(frames[1], calibration_points, "后视");
    if (gl_presenter) {
      for (int i = 0; i < 4; ++i) draw_tile_label(frames[i], i);
    } else {
      for (int i = 0; i < 4; ++i) draw_tile(canvas, frames[i], i);
    }
    // Text rasterization is expensive on Cortex-A55. Camera textures and all
    // safety overlays still refresh every frame; dashboard text and timeline
    // refresh at one third of the video rate.
    const bool refresh_dashboard = shown_frames < 2 || shown_frames % 3 == 0;
    if (refresh_dashboard) {
      draw_sidebar(canvas, display_fps, npu_ms, signal, detections, lane, rear_lane,
                   source_frame_index,
                   options.scene.substr(options.scene.find_last_of("/\\") + 1));
      draw_bottom_left(canvas, display_fps, npu_ms, detections,
                       source_frame_index, options.separate,
                       !options.legacy_mosaic, scene_index,
                       static_cast<int>(scene_paths.size()),
                       options.scene.substr(options.scene.find_last_of("/\\") + 1));
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
        gl_presenter->present(canvas, frames, kHeaderH, kTileW, kTileH);
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
        if (kPrevSceneButton.contains(click)) scene_delta = -1;
        if (kNextSceneButton.contains(click)) scene_delta = 1;
      }
      if (key == 'c' || key == 'r') {
        calibration_target = key == 'c' ? CalibrationTarget::FRONT : CalibrationTarget::REAR;
        calibration_points.clear();
      }
      if (!calibration_paused && click_x >= 0 && click_y >= kProgressBar.y - 10 &&
          click_y <= kProgressBar.y + kProgressBar.height + 10) {
        const float ratio = std::max(0.0f, std::min(1.0f,
            (click_x - kProgressBar.x) / static_cast<float>(kProgressBar.width)));
        const int target = static_cast<int>(ratio * (source_frames - 1));
        bool seek_ok = streams[0].seek(target);
        if (!single_composite) {
          for (int i = 1; i < 4; ++i) seek_ok = streams[i].seek(target) && seek_ok;
        }
        if (seek_ok) {
          source_frame_index = target;
          skip_debt = 0.0;
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
          front_lane_missed_updates = rear_lane_missed_updates = 0;
        }
      }
      if (calibration_target != CalibrationTarget::NONE && click_x >= 0 && click_y >= kHeaderH) {
        const int camera_x = calibration_target == CalibrationTarget::FRONT ? 0 : kTileW;
        if (click_x >= camera_x && click_x < camera_x + kTileW && click_y < kHeaderH + kTileH) {
          calibration_points.emplace_back((click_x - camera_x) / static_cast<float>(kTileW),
                                          (click_y - kHeaderH) / static_cast<float>(kTileH));
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
                front_lane_missed_updates = 0;
              } else {
                rear_lane_warning.set_reference_from_roi(calibration_points);
                rear_lane = adas::LaneResult();
                rear_lane_target = adas::LaneResult();
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
          skip_debt = 0.0;
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
          front_lane_missed_updates = rear_lane_missed_updates = 0;
          front_risk_estimator.reset();
          rear_risk_estimator.reset();
          front_lane_warning.reset();
          rear_lane_warning.reset();
          left_blind_monitor.reset();
          right_blind_monitor.reset();
          for (auto& tracker : trackers) tracker.reset();
          for (auto& view : detections) view.clear();
          presentation_times.clear();
          display_fps = 0.0;
          std::cout << "Switched scene to " << options.scene << std::endl;
          continue;
        }
      }
    }
    const auto presented_at = std::chrono::steady_clock::now();
    presentation_times.push_back(presented_at);
    while (presentation_times.size() > 1 &&
           presented_at - presentation_times.front() > std::chrono::seconds(1))
      presentation_times.pop_front();
    if (presented_at - fps_refresh >= std::chrono::milliseconds(200)) {
      const double span = std::chrono::duration<double>(
          presented_at - presentation_times.front()).count();
      display_fps = span > 0.0 ? (presentation_times.size() - 1) / span : 0.0;
      fps_refresh = presented_at;
    }
    ++shown_frames;
    if (!calibration_paused) ++source_frame_index;
    if (options.max_frames > 0 && shown_frames >= options.max_frames) break;
    if (calibration_paused) {
      std::this_thread::sleep_for(std::chrono::milliseconds(33));
    } else if (!options.benchmark) {
      const double period = 1.0 / source_fps;
      const double spent = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - frame_start).count();
      if (spent < period) {
        std::this_thread::sleep_for(std::chrono::duration<double>(period - spent));
      } else {
        // Do not grab/drop source frames. The PC demo processes frames in
        // sequence; skipping here made its temporal lane tracker jump several
        // frames at a time. When RK3568 is slower than the 30 FPS source, play
        // the demo slower instead of corrupting the lane motion sequence.
        skip_debt = 0.0;
      }
    }
    if (shown_frames % 30 == 0) std::cout << "shown=" << shown_frames
        << " source_frame=" << source_frame_index << " fps_1s=" << display_fps
        << " npu_ms=" << npu_ms << " lanes=" << lane.valid << ',' << rear_lane.valid
        << " capture_ms=" << capture_ms << " lane_ms=" << last_lane_ms
        << " compose_ms=" << compose_ms << " render_ms=" << render_ms
        << " frame_ms=" << std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - frame_start).count()
        << " lane_offset=" << lane.offset_ratio << ',' << rear_lane.offset_ratio
        << " lane_departure=" << lane.departure << ',' << rear_lane.departure
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
