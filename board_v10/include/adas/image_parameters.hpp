#pragma once
#include "adas/image_protocol.h"
#include <array>
#include <algorithm>

namespace adas {
// 与 pcie_v3 Qt 查看器使用完全相同的位定义、预设及 FPGA 钳位规则。
struct ImageParameters {
  std::array<bool,11> switches{};
  std::array<int,13> values{};
  int preset = 0;
  static const char* switch_name(int i) {
    static const char* names[]={"去雾 DCP","Gamma","CLAHE","高光保护","亮度降噪",
      "透射率滤波","天空保护","防光晕","阴影保护","原高光压制","原暗光增强"};
    return names[i];
  }
  static const char* value_name(int i) {
    static const char* names[]={"去雾下限","天空下限","去雾强度","降噪混合","降噪 sigma",
      "暗光阈值","高光阈值","CLAHE clip","CLAHE 强度","阴影增益","提亮限制","高光膝点","高光斜率"};
    return names[i];
  }
  static const char* preset_name(int i) {
    static const char* names[]={"全旁路","温和对比度 / 高光保护","背光提亮","雾天：仅去雾",
      "雾天：综合增强","夜间 / 隧道","轻度降噪","仅暗光增强","仅高光压制","暗光增强 + 高光压制"};
    return i>=0 && i<10 ? names[i] : "自定义";
  }
  static int bit(int i) { static const int bits[]={0,1,2,3,4,5,6,7,8,11,12}; return bits[i]; }
  static int minimum(int i) { return i==0 || i==1 || i==7 ? 1 : i==4 ? 16 : 0; }
  static int maximum(int i) {
    static const int values[]={255,255,255,256,48,255,255,65535,4,64,255,255,256};
    return values[i];
  }
  ImageParameters() { load_preset(0); }
  void load_preset(int index) {
    static const __u32 presets[][5]={
      {0x1c0,0x00f38034,0x00004100,0x02030190,0x0080c040},
      {0x1cc,0x00f38034,0x00004100,0x010100c8,0x0080c020},
      {0x1ce,0x00f38034,0x00004100,0x010100c8,0x0080c020},
      {0x1e1,0x00e0a060,0x00004100,0x02030190,0x0080c040},
      {0x1ff,0x00e0a060,0x00004080,0x010100c8,0x0080c020},
      {0x1de,0x00f38034,0x000040c0,0x010100c8,0x0080c020},
      {0x1d0,0x00f38034,0x00002080,0x010100c8,0x0080c020},
      {0x11c0,0x00f38034,0x00604100,0x02030190,0x0080c040},
      {0x09c0,0x00f38034,0xc0004100,0x02030190,0x0080c040},
      {0x19c0,0x00f38034,0xc0604100,0x02030190,0x0080c040}};
    if(index<0 || index>=10)return;
    preset=index; const auto* w=presets[index];
    for(int i=0;i<11;++i)switches[i]=(w[0] & (1U<<bit(i)))!=0;
    values={{int(w[1]&255),int((w[1]>>8)&255),int((w[1]>>16)&255),
      int(w[2]&511),int((w[2]>>9)&63),int((w[2]>>16)&255),int(w[2]>>24),
      int(w[3]&65535),int((w[3]>>16)&7),int((w[3]>>19)&127),
      int(w[4]&255),int((w[4]>>8)&255),int((w[4]>>16)&511)}};
  }
  void set_value(int i,int n) {
    n=std::max(minimum(i),std::min(maximum(i),n));
    if(i==4)n=n<24?16:n<40?32:48;
    values[i]=n; preset=-1;
    // 界面同步展示约束，不能让驱动悄悄把用户看到的天空下限抬高。
    if(i==0 || i==1)values[1]=std::max(values[0],values[1]);
  }
  fpga_image_submit words() const {
    fpga_image_submit result{}; __u32 raw[5]{};
    for(int i=0;i<11;++i)if(switches[i])raw[0]|=1U<<bit(i);
    raw[1]=values[0]|(__u32(values[1])<<8)|(__u32(values[2])<<16);
    raw[2]=values[3]|(__u32(values[4])<<9)|(__u32(values[5])<<16)|(__u32(values[6])<<24);
    raw[3]=values[7]|(__u32(values[8])<<16)|(__u32(values[9])<<19);
    raw[4]=values[10]|(__u32(values[11])<<8)|(__u32(values[12])<<16);
    fpga_cfg_sanitize(raw,result.words); return result;
  }
};
} // namespace adas

