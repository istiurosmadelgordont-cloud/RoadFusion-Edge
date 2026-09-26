#include "adas/ufld_lane_detector.hpp"
#include "adas/lane_geometry_tracker.hpp"
#include "adas/warning_logic.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>

// Deterministic diagnostic: fixed video samples and media timestamps isolate
// perception/LDW from the live scheduler. Mirroring is a derived test, never
// evidence that the original recording contains a right lane change.
int main(int argc, char** argv) {
  if (argc != 5 && argc != 6) {
    std::cerr << "usage: lane_video_replay MODEL VIDEO CSV STRIDE [mirror]\n";
    return 2;
  }
  try {
    const int stride = std::stoi(argv[4]);
    const bool mirror = argc == 6;
    if (stride < 1 || (mirror && std::string(argv[5]) != "mirror"))
      throw std::runtime_error("invalid stride or mirror argument");
    cv::VideoCapture input(argv[2]);
    if (!input.isOpened()) throw std::runtime_error("cannot open video");
    const double fps = input.get(cv::CAP_PROP_FPS);
    if (!std::isfinite(fps) || fps <= 0) throw std::runtime_error("invalid video FPS");
    adas::UfldLaneDetector detector(argv[1]);
    if (!detector.ready()) throw std::runtime_error(detector.error());
    std::ofstream output(argv[3]);
    if (!output) throw std::runtime_error("cannot open CSV");
    output << "frame,seconds,mirrored,valid,partial,raw_offset,offset,identity,reassignment,departure,side\n";
    adas::NeuralLaneGeometryTracker tracker;
    adas::LaneDepartureMonitor monitor;
    const auto origin = std::chrono::steady_clock::time_point(std::chrono::seconds(1));
    cv::Mat frame;
    int index = 0, samples = 0;
    while (input.read(frame)) {
      if (index % stride == 0) {
        cv::resize(frame, frame, cv::Size(640, 360));
        if (mirror) cv::flip(frame, frame, 1);
        const auto raw = detector.detect(frame);
        const auto at = origin + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(index / fps));
        const auto lane = monitor.update(tracker.update(raw), at);
        output << index << ',' << index / fps << ',' << mirror << ',' << raw.valid
               << ',' << raw.partial << ',' << raw.offset_ratio << ',' << lane.offset_ratio
               << ',' << lane.identity_uncertain << ',' << static_cast<int>(lane.reassignment_side)
               << ',' << lane.departure << ',' << static_cast<int>(lane.departure_side) << '\n';
        ++samples;
      }
      ++index;
    }
    if (!samples || !output) throw std::runtime_error("empty video or CSV write failure");
    std::cout << "samples=" << samples << " mirrored=" << mirror << '\n';
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
