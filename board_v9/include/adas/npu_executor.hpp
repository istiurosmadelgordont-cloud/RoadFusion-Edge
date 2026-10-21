#pragma once
#include <condition_variable>
#include <deque>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <vector>

namespace adas {
// One active NPU job and at most one pending job. Both callers retain their
// future until consumed, so neither object nor lane jobs can form a backlog.
class NpuExecutor {
 public:
  explicit NpuExecutor(int workers=1) {
    if(workers<1 || workers>2)throw std::invalid_argument("NPU executor needs 1 or 2 CPU workers");
    try {for(int i=0;i<workers;++i)workers_.emplace_back([this]() { work(); });}
    catch(...) {
      {std::lock_guard<std::mutex> lock(mutex_);stopping_=true;}
      ready_.notify_all();for(auto& worker:workers_)worker.join();throw;
    }
  }
  ~NpuExecutor() {
    { std::lock_guard<std::mutex> lock(mutex_); stopping_=true; }
    ready_.notify_all();
    for(auto& worker:workers_)worker.join();
  }
  NpuExecutor(const NpuExecutor&)=delete;
  NpuExecutor& operator=(const NpuExecutor&)=delete;
  template<class F>
  std::future<typename std::result_of<F()>::type> submit(F function) {
    using R=typename std::result_of<F()>::type;
    auto task=std::make_shared<std::packaged_task<R()>>(std::move(function));
    auto future=task->get_future();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // A worker may not have removed the first submitted task yet.
      if(stopping_ || jobs_.size()>=2)throw std::runtime_error("NPU queue full or stopped");
      jobs_.push_back([task]() { (*task)(); });
    }
    ready_.notify_one();
    return future;
  }
 private:
  void work() {
    for(;;) {
      std::function<void()> job;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait(lock,[this]() { return stopping_ || !jobs_.empty(); });
        if(jobs_.empty())return;
        job=std::move(jobs_.front()); jobs_.pop_front();
      }
      job();
    }
  }
  std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<std::function<void()>> jobs_;
  bool stopping_=false;
  std::vector<std::thread> workers_;
};
}
