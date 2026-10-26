#include "adas/npu_executor.hpp"
#include "adas/npu_run_guard.hpp"
#include "adas/lane_freshness.hpp"
#include <atomic>
#include <cassert>
#include <iostream>

int main() {
  using namespace adas;
  auto base=LaneClock::now();
  auto at=[base](int ms){return base+std::chrono::milliseconds(ms);};
  assert(lane_result_displayable(at(0),at(240),at(380)));
  assert(!lane_decision_fresh(at(0),at(380)));
  assert(lane_decision_fresh(at(0),at(299)));
  assert(!lane_decision_fresh(at(0),at(300)));
  assert(!lane_result_displayable(at(0),at(300),at(380)));
  assert(!lane_result_displayable(at(0),at(200),at(1000)));
  assert(!lane_result_displayable(at(100),at(50),at(200)));
  assert(!lane_result_displayable(at(0),at(200),at(100)));
  std::promise<void> first_started,release;
  auto gate=release.get_future().share();
  std::atomic<int> active(0),latest(1),executions(0);
  {
    NpuExecutor worker;
    auto first=worker.submit([&](){
      assert(active.fetch_add(1)==0);first_started.set_value();gate.wait();
      ++executions;--active;return 7;
    });
    first_started.get_future().wait();
    auto next=worker.submit([&](){
      assert(active.fetch_add(1)==0);int frame=latest.load();++executions;--active;return frame;
    });
    latest=2;release.set_value();
    assert(first.get()==7 && next.get()==2 && executions==2 && active==0);
    auto failed=worker.submit([]()->int {throw std::runtime_error("expected");});
    bool caught=false;try{failed.get();}catch(const std::runtime_error&){caught=true;}
    assert(caught);
    assert(worker.submit([](){return 9;}).get()==9);
  }
  // Pending work is drained before references captured by jobs can go away.
  {NpuExecutor worker;worker.submit([&](){++executions;});}
  assert(executions==3);
  {
    NpuExecutor worker(2);
    std::promise<void> release_prepared;auto prepare_gate=release_prepared.get_future().share();
    std::atomic<int> prepared(0),on_npu(0);
    auto job=[&](){++prepared;prepare_gate.wait();NpuRunGuard lock(npu_run_mutex());assert(on_npu.fetch_add(1)==0);--on_npu;return 1;};
    auto first=worker.submit(job),second=worker.submit(job);
    while(prepared.load()<2)std::this_thread::yield();
    release_prepared.set_value();assert(first.get()+second.get()==2);
  }
  {
    // Nested MO work belongs to the same YOLO observation; a lane context
    // cannot enter after the inner crop releases but before the batch ends.
    NpuRunGuard observation(npu_run_mutex());
    NpuRunGuard crop(npu_run_mutex(),std::try_to_lock);
    assert(crop.owns_lock());
    std::promise<bool> attempted;
    std::atomic<int> phase(1);
    std::thread lane([&](){
      NpuRunGuard probe(npu_run_mutex(),std::try_to_lock);
      attempted.set_value(probe.owns_lock());
      assert(!probe.owns_lock());
      NpuRunGuard waiting(npu_run_mutex());
      assert(phase==3);
    });
    assert(!attempted.get_future().get());
    crop.unlock();phase=2;
    assert(observation.owns_lock());
    phase=3;observation.unlock();lane.join();
  }
  {
    // A second worker can already be waiting for the NPU. It must select the
    // camera's latest frame after admission, rather than before that wait.
    NpuExecutor worker(2);
    NpuRunGuard busy(npu_run_mutex());
    std::promise<void> waiting;std::atomic<int> camera_frame(1);
    auto queued=worker.submit([&](){
      waiting.set_value();
      NpuRunGuard admitted(npu_run_mutex());
      return camera_frame.load();
    });
    waiting.get_future().wait();camera_frame=2;busy.unlock();
    assert(queued.get()==2);
  }
  std::cout<<"NPU executor PASS: serialized jobs, latest input, exceptions, drain, display/decision expiry\n";
}
