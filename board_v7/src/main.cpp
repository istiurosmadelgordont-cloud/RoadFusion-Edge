#include "adas/config.hpp"
#include "adas/drive_decision.hpp"
#include "adas/lane_detector.hpp"
#include "adas/overlay.hpp"
#include "adas/risk_estimator.hpp"
#include "adas/rknn_detector.hpp"
#include "adas/signal_logic.hpp"
#include "adas/text_renderer.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace {

struct Options {
  std::string model = "models/unified21_p2_v7_640_int8.rknn";
  std::string source = "0";
  std::string output;
  bool display = true;
  int max_frames = 0;
  int detection_interval = 2;
  int lane_interval = 2;
  int cpu_threads = 2;
  float confidence = 0.25f;
  bool dump_detections = false;
  std::string roi_file = "config/lane_roi.txt";
};

void usage(const char* program) {
  std::cout << "Usage: " << program << " [options]\n"
            << "  --model FILE          RKNN INT8 model\n"
            << "  --source FILE|INDEX   video/image/camera input\n"
            << "  --output FILE         save annotated video/image\n"
            << "  --headless            do not open a window\n"
            << "  --max-frames N        stop after N frames (0 = all)\n"
            << "  --detect-every N      NPU inference interval\n"
            << "  --lane-every N        CPU lane detection interval\n"
            << "  --cpu-threads N       OpenCV CPU worker threads\n"
            << "  --confidence FLOAT    detection threshold\n"
            << "  --dump-detections     print first-frame detections for verification\n"
            << "  --roi-file FILE       persistent four-point lane calibration\n"
            << "Keys: O open, P pause, C calibrate, D detection, -/+ size, Q quit\n";
}

bool parse(int argc, char** argv, Options& options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") { usage(argv[0]); return false; }
    if (arg == "--headless") { options.display = false; continue; }
    if (arg == "--dump-detections") { options.dump_detections = true; continue; }
    if (i + 1 >= argc) { std::cerr << "Missing value for " << arg << std::endl; return false; }
    const std::string value = argv[++i];
    if (arg == "--model") options.model = value;
    else if (arg == "--source") options.source = value;
    else if (arg == "--output") options.output = value;
    else if (arg == "--max-frames") options.max_frames = std::atoi(value.c_str());
    else if (arg == "--detect-every") options.detection_interval = std::max(1, std::atoi(value.c_str()));
    else if (arg == "--lane-every") options.lane_interval = std::max(1, std::atoi(value.c_str()));
    else if (arg == "--cpu-threads") options.cpu_threads = std::max(1, std::min(4, std::atoi(value.c_str())));
    else if (arg == "--confidence") options.confidence = std::atof(value.c_str());
    else if (arg == "--roi-file") options.roi_file = value;
    else { std::cerr << "Unknown option: " << arg << std::endl; return false; }
  }
  return true;
}

bool integer_string(const std::string& value) {
  return !value.empty() && std::find_if(value.begin(), value.end(), [](char c) { return c < '0' || c > '9'; }) == value.end();
}

struct CalibrationUi {
  adas::LaneDetector* detector = nullptr;
  std::vector<cv::Point2f> points;
  bool collecting = true;
  bool detection_enabled = false;
  std::string roi_file;
  cv::Rect video_rect;
  std::vector<cv::Rect> buttons;
  bool open_requested = false;
  bool pause_requested = false;
  bool calibrate_requested = false;
  bool detection_toggle_requested = false;
  bool resize_requested = false;
  bool resume_requested = false;
  int scale_index = 1;
};

bool save_roi(const std::string& path, const std::vector<cv::Point2f>& points) {
  if (points.size() != 4) return false;
  std::ofstream saved(path.c_str());
  if (!saved) return false;
  for (const cv::Point2f& point : points) saved << point.x << ' ' << point.y << '\n';
  return true;
}

bool valid_roi(const std::vector<cv::Point2f>& points) {
  if (points.size() != 4) return false;
  const float top_width = points[1].x - points[0].x;
  const float bottom_width = points[2].x - points[3].x;
  const float top_y = 0.5f * (points[0].y + points[1].y);
  const float bottom_y = 0.5f * (points[2].y + points[3].y);
  return top_width > 0.03f && bottom_width > 0.20f && top_width < bottom_width &&
         bottom_y - top_y > 0.15f;
}

std::vector<cv::Point2f> order_roi(const std::vector<cv::Point2f>& clicked) {
  std::vector<cv::Point2f> ordered = clicked;
  std::sort(ordered.begin(), ordered.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
    return a.y == b.y ? a.x < b.x : a.y < b.y;
  });
  cv::Point2f top_left = ordered[0].x < ordered[1].x ? ordered[0] : ordered[1];
  cv::Point2f top_right = ordered[0].x < ordered[1].x ? ordered[1] : ordered[0];
  cv::Point2f bottom_left = ordered[2].x < ordered[3].x ? ordered[2] : ordered[3];
  cv::Point2f bottom_right = ordered[2].x < ordered[3].x ? ordered[3] : ordered[2];
  return {top_left, top_right, bottom_right, bottom_left};
}

void mouse_callback(int event, int x, int y, int, void* userdata) {
  CalibrationUi* ui = static_cast<CalibrationUi*>(userdata);
  if (!ui || event != cv::EVENT_LBUTTONDOWN) return;
  const cv::Point click(x, y);
  for (size_t i = 0; i < ui->buttons.size(); ++i) {
    if (!ui->buttons[i].contains(click)) continue;
    if (i == 0) ui->open_requested = true;
    if (i == 1) ui->pause_requested = true;
    if (i == 2) ui->calibrate_requested = true;
    if (i == 3) ui->detection_toggle_requested = true;
    if (i == 4 && ui->scale_index > 0) { --ui->scale_index; ui->resize_requested = true; }
    if (i == 5 && ui->scale_index < 2) { ++ui->scale_index; ui->resize_requested = true; }
    return;
  }
  if (!ui->collecting || !ui->video_rect.contains(click)) return;
  const float nx = (x - ui->video_rect.x) / static_cast<float>(ui->video_rect.width);
  const float ny = (y - ui->video_rect.y) / static_cast<float>(ui->video_rect.height);
  ui->points.emplace_back(nx, ny);
  std::cout << "ROI point " << ui->points.size() << ": " << nx << "," << ny << std::endl;
  if (ui->points.size() == 4) {
    ui->points = order_roi(ui->points);
    if (!valid_roi(ui->points)) {
      ui->points.clear();
      std::cerr << "Invalid ROI shape; click the four lane trapezoid corners again." << std::endl;
      return;
    }
    ui->detector->set_roi(ui->points[0], ui->points[1], ui->points[2], ui->points[3]);
    if (save_roi(ui->roi_file, ui->points)) std::cout << "ROI saved to " << ui->roi_file << std::endl;
    else std::cerr << "Warning: cannot save ROI to " << ui->roi_file << std::endl;
    ui->collecting = false;
    ui->detection_enabled = true;
    ui->resume_requested = true;
    std::cout << "ROI calibrated; recognition enabled." << std::endl;
  }
}

bool is_image(const std::string& path) {
  const size_t dot = path.find_last_of('.');
  if (dot == std::string::npos) return false;
  std::string ext = path.substr(dot + 1);
  std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
  return ext == "jpg" || ext == "jpeg" || ext == "png" || ext == "bmp";
}

bool open_source(const std::string& source, cv::Mat& still, cv::VideoCapture& capture,
                 bool& single_image) {
  const bool next_is_image = is_image(source);
  if (next_is_image) {
    cv::Mat loaded = cv::imread(source);
    if (loaded.empty()) return false;
    capture.release();
    still = loaded;
  } else {
    cv::VideoCapture loaded;
    if (integer_string(source)) loaded.open(std::atoi(source.c_str()));
    else loaded.open(source);
    if (!loaded.isOpened()) return false;
    capture.release();
    capture = loaded;
    still.release();
  }
  single_image = next_is_image;
  return true;
}

std::string choose_video_file() {
  FILE* pipe = popen("zenity --file-selection --title='Open road video' "
                     "--file-filter='Video files | *.mp4 *.avi *.mkv *.mov *.m4v *.wmv' 2>/dev/null", "r");
  if (!pipe) return std::string();
  char buffer[1024];
  std::string path;
  while (std::fgets(buffer, sizeof(buffer), pipe)) path += buffer;
  pclose(pipe);
  while (!path.empty() && (path.back() == '\n' || path.back() == '\r')) path.pop_back();
  return path;
}

cv::Rect place_image(cv::Mat& canvas, const cv::Mat& source, const cv::Rect& area) {
  cv::rectangle(canvas, area, cv::Scalar(8, 11, 13), cv::FILLED);
  cv::rectangle(canvas, area, cv::Scalar(52, 62, 67), 1);
  if (source.empty()) return area;
  const double scale = std::min(area.width / static_cast<double>(source.cols),
                                area.height / static_cast<double>(source.rows));
  const cv::Size size(std::max(1, static_cast<int>(source.cols * scale)),
                      std::max(1, static_cast<int>(source.rows * scale)));
  cv::Mat resized;
  cv::resize(source, resized, size, 0, 0, cv::INTER_AREA);
  if (resized.channels() == 1) cv::cvtColor(resized, resized, cv::COLOR_GRAY2BGR);
  const cv::Rect target(area.x + (area.width - size.width) / 2,
                        area.y + (area.height - size.height) / 2, size.width, size.height);
  resized.copyTo(canvas(target));
  return target;
}

void draw_title(cv::Mat& canvas, const std::string& title, int x, int y) {
  cv::putText(canvas, title, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.52,
              cv::Scalar(120, 245, 225), 1, cv::LINE_AA);
}

cv::Mat compose_dashboard(const cv::Mat& frame, const adas::LaneResult& lane,
                          CalibrationUi& ui, bool paused, double fps,
                          const std::string& source_name) {
  const int widths[] = {1200, 1500, 1720};
  const int heights[] = {675, 844, 968};
  const int canvas_width = widths[ui.scale_index];
  const int canvas_height = heights[ui.scale_index];
  cv::Mat canvas(canvas_height, canvas_width, CV_8UC3, cv::Scalar(19, 23, 25));
  const int margin = std::max(8, canvas_width / 125);
  const int toolbar_height = std::max(58, canvas_height / 13);
  const int button_gap = 7;
  const int button_width = std::max(116, canvas_width * 93 / 1000);
  const int button_height = toolbar_height - 18;
  const char* labels[] = {"OPEN VIDEO [O]", paused ? "RESUME [P]" : "PAUSE [P]",
                          "CALIBRATE [C]", ui.detection_enabled ? "DETECT ON [D]" : "DETECT OFF [D]",
                          "SIZE - [-]", "SIZE + [+]"};
  ui.buttons.clear();
  int button_x = margin;
  for (int i = 0; i < 6; ++i) {
    const cv::Rect button(button_x, 9, button_width, button_height);
    ui.buttons.push_back(button);
    const bool active = (i == 2 && ui.collecting) || (i == 3 && ui.detection_enabled);
    cv::rectangle(canvas, button, active ? cv::Scalar(38, 122, 105) : cv::Scalar(26, 103, 158),
                  cv::FILLED, cv::LINE_AA);
    int baseline = 0;
    const cv::Size text = cv::getTextSize(labels[i], cv::FONT_HERSHEY_SIMPLEX, 0.43, 1, &baseline);
    cv::putText(canvas, labels[i], cv::Point(button.x + (button.width - text.width) / 2,
                button.y + (button.height + text.height) / 2), cv::FONT_HERSHEY_SIMPLEX,
                0.43, cv::Scalar(245, 250, 250), 1, cv::LINE_AA);
    button_x += button_width + button_gap;
  }
  std::ostringstream status;
  status << source_name << "   FPS " << std::fixed << std::setprecision(1) << fps;
  cv::putText(canvas, status.str(), cv::Point(button_x + 10, toolbar_height / 2 + 6),
              cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(115, 230, 240), 1, cv::LINE_AA);

  const int content_y = toolbar_height;
  const int title_height = 27;
  const int content_height = canvas_height - content_y - margin;
  const int left_width = static_cast<int>((canvas_width - margin * 3) * 0.70);
  const int right_x = margin * 2 + left_width;
  const int right_width = canvas_width - right_x - margin;
  draw_title(canvas, "ADAS MAIN VIEW", margin, content_y + 19);
  const cv::Rect main_area(margin, content_y + title_height, left_width,
                           content_height - title_height);
  ui.video_rect = place_image(canvas, frame, main_area);

  const int half_height = (content_height - title_height * 2 - margin) / 2;
  draw_title(canvas, "BIRD-EYE BINARY (IPM)", right_x, content_y + 19);
  const cv::Rect bird_area(right_x, content_y + title_height, right_width, half_height);
  place_image(canvas, lane.bird_eye, bird_area);
  const int curve_title_y = bird_area.y + bird_area.height + margin;
  draw_title(canvas, "SLIDING WINDOWS + QUADRATIC FIT", right_x, curve_title_y + 19);
  const cv::Rect curve_area(right_x, curve_title_y + title_height, right_width,
                            canvas_height - margin - curve_title_y - title_height);
  place_image(canvas, lane.curve_fit, curve_area);

  if (ui.collecting) {
    std::vector<cv::Point> shown_points;
    for (const cv::Point2f& point : ui.points) {
      shown_points.emplace_back(ui.video_rect.x + static_cast<int>(point.x * ui.video_rect.width),
                                ui.video_rect.y + static_cast<int>(point.y * ui.video_rect.height));
    }
    for (const cv::Point& point : shown_points) cv::circle(canvas, point, 7, cv::Scalar(0, 255, 255), cv::FILLED);
    if (shown_points.size() > 1) cv::polylines(canvas, shown_points, false, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::rectangle(canvas, cv::Rect(ui.video_rect.x + 12, ui.video_rect.y + 12,
                  std::min(620, ui.video_rect.width - 24), 42), cv::Scalar(16, 20, 22), cv::FILLED);
    cv::putText(canvas, "CALIBRATION: click 4 lane corners in any order; detection starts after point 4",
                cv::Point(ui.video_rect.x + 22, ui.video_rect.y + 40), cv::FONT_HERSHEY_SIMPLEX,
                0.48, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
  }
  return canvas;
}

bool load_roi(const std::string& path, adas::LaneDetector& detector,
              std::vector<cv::Point2f>* loaded_points = nullptr) {
  std::ifstream input(path.c_str());
  cv::Point2f points[4];
  if (!(input >> points[0].x >> points[0].y >> points[1].x >> points[1].y
              >> points[2].x >> points[2].y >> points[3].x >> points[3].y)) return false;
  detector.set_roi(points[0], points[1], points[2], points[3]);
  if (loaded_points) loaded_points->assign(points, points + 4);
  std::cout << "Loaded lane ROI: " << path << std::endl;
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  adas::ui::initialize_text("/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc");
  Options options;
  if (!parse(argc, argv, options)) return argc > 1 ? 1 : 0;
  cv::setNumThreads(options.cpu_threads);
  std::cout << "ARM CPU OpenCV threads: " << cv::getNumThreads() << std::endl;
  adas::DetectorConfig detector_config;
  detector_config.model_path = options.model;
  detector_config.confidence = options.confidence;
  adas::RknnDetector detector(detector_config);
  if (!detector.ready()) {
    std::cerr << "Detector init failed: " << detector.error() << std::endl;
    return 2;
  }
  std::cout << "RKNN runtime: " << detector.runtime_version() << std::endl;

  adas::LaneConfig lane_config;
  adas::LaneDetector lane_detector(lane_config);
  adas::SignalLogic signal_logic;
  adas::DriveConfig drive_config;
  adas::DriveDecisionLogic drive_logic(drive_config);
  adas::RiskEstimator risk_estimator;
  CalibrationUi calibration;
  calibration.detector = &lane_detector;
  calibration.roi_file = options.roi_file;
  const bool saved_roi_loaded = load_roi(options.roi_file, lane_detector, &calibration.points);

  cv::Mat still;
  cv::VideoCapture capture;
  bool single_image = false;
  if (!open_source(options.source, still, capture, single_image)) {
    std::cerr << "Cannot open source: " << options.source << std::endl;
    return 3;
  }

  if (options.display) {
    cv::namedWindow("RK3568 ADAS", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("RK3568 ADAS", mouse_callback, &calibration);
  }
  cv::VideoWriter writer;
  std::vector<adas::Detection> detections;
  adas::LaneResult lane;
  int frame_index = 0;
  double fps_smoothed = 0.0;
  double npu_ms = 0.0;
  double source_fps = single_image ? 0.0 : capture.get(cv::CAP_PROP_FPS);
  if (!single_image && (source_fps < 1.0 || source_fps > 120.0)) source_fps = 25.0;
  double skip_debt = 0.0;
  bool paused = false;
  bool pause_after_next_frame = options.display && !saved_roi_loaded;
  cv::Mat frame;
  calibration.collecting = options.display && !saved_roi_loaded;
  calibration.detection_enabled = !options.display || saved_roi_loaded;

  while (true) {
    const auto loop_start = std::chrono::steady_clock::now();
    if (!paused) {
      if (single_image) frame = still.clone();
      else if (!capture.read(frame)) {
        if (!options.display || !capture.set(cv::CAP_PROP_POS_FRAMES, 0) || !capture.read(frame)) break;
      }
      if (frame.empty()) break;
      if (frame_index == 0 || (frame_index + 1) % options.lane_interval == 0) {
        lane = lane_detector.detect(frame);
      }
      if (calibration.detection_enabled) {
        if (frame_index % options.detection_interval == 0) detections = detector.detect(frame, &npu_ms);
        if (options.dump_detections && frame_index == 0) {
          for (const adas::Detection& d : detections) {
            std::cout << "detection=" << d.name << " score=" << d.score << " box="
                      << d.box.x << "," << d.box.y << "," << d.box.width << ","
                      << d.box.height << std::endl;
          }
        }
        const adas::SignalResult signal = signal_logic.update(detections, frame.cols, frame.rows);
        const adas::DriveResult drive = drive_logic.update(signal);
        const adas::RiskResult risk = risk_estimator.update(detections, frame.cols, frame.rows);
        adas::draw_overlay(frame, detections, lane, signal, risk, drive, fps_smoothed, npu_ms);
      } else {
        detections.clear();
      }

      if (!options.output.empty()) {
        if (single_image) {
          cv::imwrite(options.output, frame);
        } else {
          if (!writer.isOpened()) {
            double source_fps = capture.get(cv::CAP_PROP_FPS);
            if (source_fps < 1.0 || source_fps > 120.0) source_fps = 25.0;
            writer.open(options.output, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), source_fps, frame.size());
            if (!writer.isOpened()) std::cerr << "Warning: cannot create output " << options.output << std::endl;
          }
          if (writer.isOpened()) writer.write(frame);
        }
      }
      ++frame_index;
      const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - loop_start).count();
      const double instant_fps = seconds > 0.0 ? 1.0 / seconds : 0.0;
      fps_smoothed = fps_smoothed == 0.0 ? instant_fps : fps_smoothed * 0.90 + instant_fps * 0.10;
      if (frame_index % 30 == 0) std::cout << "frames=" << frame_index << " fps=" << fps_smoothed << " npu_ms=" << npu_ms << " detections=" << detections.size() << std::endl;
      if (pause_after_next_frame) { paused = true; pause_after_next_frame = false; }
    }

    if (options.display) {
      const size_t slash = options.source.find_last_of("/\\");
      const std::string source_name = slash == std::string::npos ? options.source : options.source.substr(slash + 1);
      cv::Mat display_frame = compose_dashboard(frame, lane, calibration, paused, fps_smoothed, source_name);
      cv::imshow("RK3568 ADAS", display_frame);
      int wait_ms = single_image || paused ? 20 : 1;
      int skip_frames = 0;
      if (!single_image && !paused && source_fps > 0.0) {
        const double used_seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - loop_start).count();
        const double frame_period = 1.0 / source_fps;
        skip_debt += used_seconds / frame_period - 1.0;
        if (skip_debt < 0.0) {
          wait_ms = std::max(1, static_cast<int>(std::round(-skip_debt * frame_period * 1000.0)));
          skip_debt = 0.0;
        } else {
          skip_frames = static_cast<int>(std::floor(skip_debt));
          skip_debt -= skip_frames;
        }
        for (int i = 0; i < skip_frames; ++i) {
          if (!capture.grab()) break;
        }
      }
      const int key = cv::waitKey(wait_ms) & 0xff;
      if (key == 'q' || key == 27) break;
      if (key == 'p' || key == ' ') calibration.pause_requested = true;
      if (key == 'c') calibration.calibrate_requested = true;
      if (key == 'd') calibration.detection_toggle_requested = true;
      if (key == 'o') calibration.open_requested = true;
      if (key == '-' && calibration.scale_index > 0) { --calibration.scale_index; calibration.resize_requested = true; }
      if ((key == '+' || key == '=') && calibration.scale_index < 2) { ++calibration.scale_index; calibration.resize_requested = true; }

      if (calibration.pause_requested) {
        calibration.pause_requested = false;
        paused = !paused;
      }
      if (calibration.calibrate_requested) {
        calibration.calibrate_requested = false;
        calibration.points.clear();
        calibration.collecting = true;
        calibration.detection_enabled = false;
        paused = true;
        std::cout << "Click the four lane trapezoid corners in any order." << std::endl;
      }
      if (calibration.detection_toggle_requested) {
        calibration.detection_toggle_requested = false;
        if (!calibration.collecting) calibration.detection_enabled = !calibration.detection_enabled;
      }
      if (calibration.resume_requested) {
        calibration.resume_requested = false;
        paused = false;
        lane_detector.reset();
        signal_logic = adas::SignalLogic();
        drive_logic.reset();
        risk_estimator = adas::RiskEstimator();
      }
      calibration.resize_requested = false;
      if (calibration.open_requested) {
        calibration.open_requested = false;
        const std::string selected = choose_video_file();
        if (!selected.empty()) {
          if (open_source(selected, still, capture, single_image)) {
            options.source = selected;
            writer.release();
            detections.clear();
            lane = adas::LaneResult();
            lane_detector.reset();
            signal_logic = adas::SignalLogic();
            drive_logic.reset();
            risk_estimator = adas::RiskEstimator();
            frame_index = 0;
            fps_smoothed = 0.0;
            source_fps = single_image ? 0.0 : capture.get(cv::CAP_PROP_FPS);
            if (!single_image && (source_fps < 1.0 || source_fps > 120.0)) source_fps = 25.0;
            skip_debt = 0.0;
            paused = false;
            calibration.points.clear();
            calibration.collecting = true;
            calibration.detection_enabled = false;
            pause_after_next_frame = true;
            std::cout << "Opened source: " << selected << std::endl;
          } else {
            std::cerr << "Cannot open selected source: " << selected << std::endl;
          }
        }
      }
    }
    if (single_image && !options.display) break;
    if (options.max_frames > 0 && frame_index >= options.max_frames) break;
  }
  std::cout << "Done. frames=" << frame_index << " average_fps=" << fps_smoothed << std::endl;
  return 0;
}
