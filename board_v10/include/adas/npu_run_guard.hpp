#pragma once
#include <mutex>
namespace adas {
// Contexts may prepare/decode in parallel, but one job owns the RK3568 NPU
// through inputs_set/run/outputs_get. Two-stage YOLO holds the same ownership
// through its MO crops: a long lane job must not split one lamp observation.
// MO can also run independently, so its local guard nests on the same thread.
inline std::recursive_mutex& npu_run_mutex() {static std::recursive_mutex mutex;return mutex;}
using NpuRunGuard=std::unique_lock<std::recursive_mutex>;
}
