#pragma once
#include <fstream>
#include <string>
#include <vector>

namespace adas {

// 输入设备和演示场景配置分开：PCIe 仍收实时帧，但可复用同一测试视频
// 的灯组 ROI、方向规则和 AVM 标定。只查找同名场景，不能随便套别的标定。
inline std::string scene_asset_path(const std::string& scene,
                                    const std::string& asset) {
  std::string directory = scene;
  while (directory.size() > 1 && directory.back() == '/') directory.pop_back();
  const std::string direct = directory + "/" + asset;
  std::vector<std::string> candidates{direct};
  if (!directory.empty() && directory.front() != '/') {
    // 新源码目录通常不包含视频和标定，允许读取板上已有的同名素材目录。
    // 显式绝对路径不走这些候选位置，避免覆盖用户指定的素材来源。
    for (const char* root : {"../", "../board_v8/", "../board_v7/"})
      candidates.push_back(std::string(root) + direct);
  }
  const auto slash = directory.find_last_of('/');
  const std::string name = directory.substr(slash == std::string::npos ? 0 : slash + 1);
  // 仓库随源码保存了灯组配置；没有整套 MP4 时，PCIe 仍能复用该配置。
  if (!name.empty() && !directory.empty() && directory.front() != '/')
    candidates.push_back("config/simulated_test_1_scenes/" + name + "/" + asset);
  for (const auto& path : candidates)
    if (std::ifstream(path.c_str()).good()) return path;
  return direct; // 保留请求路径，由加载器明确报告缺少文件。
}

} // namespace adas
