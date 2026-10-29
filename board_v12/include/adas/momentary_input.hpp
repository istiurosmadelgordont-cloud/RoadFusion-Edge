#pragma once
#include <mutex>
#include <vector>
#include <opencv2/core.hpp>

namespace adas {
// GLES 与 OpenCV 共用鼠标状态与点击队列；车辆运动状态由模拟控制器独立保存。
// 回调可能运行在窗口线程；快照和取消操作都在同一把锁下完成。
class MomentaryInput {
 public:
  struct Snapshot {
    bool down = false;
    cv::Point origin{-1,-1}, pointer{-1,-1}, click{-1,-1};
    std::vector<cv::Point> clicks;
  };
  void press(int x, int y) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.down = true; state_.origin = state_.pointer = state_.click = cv::Point(x,y);
    state_.clicks.push_back(state_.click);
  }
  void move(int x, int y, bool left_down) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.pointer = cv::Point(x,y);
    if (!left_down) state_.down = false; // 补救窗口外松开造成的丢失事件。
  }
  void release() {
    std::lock_guard<std::mutex> lock(mutex_); state_.down = false;
  }
  void cancel() {
    std::lock_guard<std::mutex> lock(mutex_); state_ = Snapshot();
  }
  Snapshot take() {
    std::lock_guard<std::mutex> lock(mutex_);
    const Snapshot result = state_;
    state_.click = cv::Point(-1,-1); state_.clicks.clear(); return result;
  }
 private:
  std::mutex mutex_;
  Snapshot state_;
};
} // namespace adas
