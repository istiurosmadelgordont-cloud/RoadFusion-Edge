#pragma once
#include "adas/image_parameters.hpp"
#include <chrono>
#include <cstring>
#include <string>
#include <vector>

namespace adas {
// 事务状态机由控制线程推进；UI 只读取快照，不执行 ioctl 或等待 FPGA。
// 超时仍保留 busy：迟到回执有效，新的参数不能覆盖旧的在途请求。
class ImageTransaction {
 public:
  using Clock=std::chrono::steady_clock;
  enum class Phase { IDLE, QUEUED, WAITING, RECEIVED, APPLIED, SUBMITTED, UNCERTAIN };
  Phase phase=Phase::IDLE;
  bool available=false, ack_mode=false, busy=false, timed_out=false;
  __u32 cookie=0, flags=0;
  std::string message="请开始 PCIe 采集后再应用参数";
  std::vector<std::string> logs;
  fpga_image_submit submitted{};
  Clock::time_point began{};
  void log(const std::string& s) {
    message=s; logs.push_back(s); if(logs.size()>8)logs.erase(logs.begin());
  }
  bool queue(const fpga_image_submit& config) {
    if(!available || busy)return false;
    submitted={}; fpga_cfg_sanitize(config.words,submitted.words);
    busy=true; cookie=flags=0; timed_out=false; phase=Phase::QUEUED;
    log("参数已排队，等待控制线程发送"); return true;
  }
  void sent(Clock::time_point now) {
    began=now; phase=ack_mode?Phase::WAITING:Phase::SUBMITTED;
    busy=ack_mode;
    log(ack_mode?"参数已提交，等待 FPGA 回执":"参数已提交；单向接口不能确认硬件生效");
  }
  void submit_failed(const std::string& reason,bool uncertain) {
    busy=uncertain; phase=uncertain?Phase::UNCERTAIN:Phase::IDLE; log(reason);
  }
  void observe(const fpga_image_ack_status& status) {
    if(!busy || phase==Phase::QUEUED)return;
    if(!status.cookie || (cookie && cookie!=status.cookie) || status.reserved ||
       (status.flags & (FPGA_IMAGE_STOPPED|FPGA_IMAGE_DRAINING)) ||
       std::memcmp(status.words,submitted.words,sizeof(status.words))) {
      if(phase!=Phase::UNCERTAIN)log("回执编号/参数异常；保留事务，请退出采集后恢复");
      phase=Phase::UNCERTAIN; return;
    }
    // 一旦出现不一致不再认领另一个 cookie；停止/重开才可恢复。
    if(phase==Phase::UNCERTAIN)return;
    if(!cookie) { cookie=status.cookie; log("参数 #"+std::to_string(cookie)+" 等待 FPGA 接收"); }
    if((status.flags & FPGA_IMAGE_RECEIVED) && !(flags & FPGA_IMAGE_RECEIVED)) {
      phase=Phase::RECEIVED; log("参数 #"+std::to_string(cookie)+"：FPGA 已接收 RECEIVED");
    }
    flags|=status.flags;
    if((flags & (FPGA_IMAGE_RECEIVED|FPGA_IMAGE_APPLIED))==
       (FPGA_IMAGE_RECEIVED|FPGA_IMAGE_APPLIED)) {
      busy=false; phase=Phase::APPLIED;
      log("参数 #"+std::to_string(cookie)+"：已生效 APPLIED"+(timed_out?"（迟到回执）":""));
    }
  }
  void check_timeout(Clock::time_point now) {
    if(busy && phase!=Phase::QUEUED && !timed_out && now-began>=std::chrono::seconds(3)) {
      timed_out=true; log("确认超时，继续等待迟到回执；禁止覆盖，退出采集后恢复");
    }
  }
};
} // namespace adas

