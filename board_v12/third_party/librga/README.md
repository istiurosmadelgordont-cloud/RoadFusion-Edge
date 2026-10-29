# 项目私有 Rockchip librga

来源：https://github.com/airockchip/librga
下载地址：https://codeload.github.com/airockchip/librga/tar.gz/refs/heads/main
使用此前从官方仓库保存的快照；以本目录文件哈希固定内容，不在构建时拉取 main。
API 版本：1.10.6_[3]；Linux gcc-aarch64 静态库与同一快照的 include/ 配套。
上游许可为 Apache-2.0，原始头文件和 COPYING 保留。

板子系统库为 librga 1.3.1，而内核为 RGA multicore v1.2.26。
在该组合上，旧库将虚拟地址错误提交为 fd，真实转换返回 Invalid argument。
本目录只供 board_v12 构建和运行使用，不替换 /usr/lib 或 /lib 中的系统库，
不影响旧工程和 pcie_v4 查看器。headers 与 library 必须成套使用。
CMake 在 ARM64 默认选择本目录，采集的 RGA 调用静态链接到此版本。
OpenCV/RKNN 等系统组件仍可能间接加载系统旧库；不能只凭 ldd 判断采集采用的版本。
上游动态库依赖 GLIBC_2.29，板子是 GLIBC_2.28；使用同版静态库并在板子链接，
由板子工具链解析 libc/libm 符号，不升级板子 glibc。
若运行环境已经有匹配的系统 RGA，可显式 -DADAS_USE_BUNDLED_RGA=OFF。
此处只有 ARM64 库，普通 x86 主机不能使用这份二进制。
Library SHA256: 853159434f487f1bd852db4c1d45eeb4a6b1629ba4c93f2cb2fbd5b20f11ec67
Source archive SHA256: 3192ec5463d4d3d4ffa691e1e5fb7038e65cb78ec7981b5bc3251673ee684ab5
