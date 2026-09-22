#pragma once

#include <cstdint>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace adas {

// RKNN accepts a uint32_t byte count. Keep a lower project limit as a guard
// against corrupt paths, special files and accidental multi-gigabyte inputs.
inline std::vector<unsigned char> read_model_file(
    const std::string& path, std::string* error = nullptr,
    std::uint64_t max_bytes = 512ULL * 1024ULL * 1024ULL) {
  std::ifstream file(path.c_str(), std::ios::binary | std::ios::ate);
  if (!file) {
    if (error) *error = "cannot open model: " + path;
    return {};
  }

  const std::streamoff end = file.tellg();
  if (end <= 0 || static_cast<std::uint64_t>(end) > max_bytes ||
      static_cast<std::uint64_t>(end) >
          std::numeric_limits<std::uint32_t>::max()) {
    if (error) *error = "model size is empty or outside the allowed range";
    return {};
  }

  file.seekg(0, std::ios::beg);
  std::vector<unsigned char> data(static_cast<std::size_t>(end));
  if (!file.read(reinterpret_cast<char*>(data.data()),
                 static_cast<std::streamsize>(end))) {
    if (error) *error = "cannot read complete model: " + path;
    return {};
  }
  return data;
}

}  // namespace adas
