# 兼容板子自带的 CMake 3.13.4；本工程使用的构建命令不要求 3.16。
cmake_minimum_required(VERSION 3.13)
project(pcie_viewer LANGUAGES CXX)

# 用户程序通过 Linux 设备文件访问驱动，不编译或包含内核驱动 C 文件。
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR "pcie_viewer must be built for Linux")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 旧版 CMake 使用 Qt 5，避免加载要求更高 CMake 版本的 Qt 6 配置。
# 显式指定主版本，也兼容尚未提供 QT_VERSION_MAJOR 的旧版 Qt 5。
if(CMAKE_VERSION VERSION_LESS "3.16")
    find_package(Qt5 REQUIRED COMPONENTS Widgets)
    set(PCIE_QT_WIDGETS_TARGET Qt5::Widgets)
else()
    find_package(Qt6 QUIET COMPONENTS Widgets)
    if(Qt6_FOUND)
        set(PCIE_QT_WIDGETS_TARGET Qt6::Widgets)
    else()
        find_package(Qt5 REQUIRED COMPONENTS Widgets)
        set(PCIE_QT_WIDGETS_TARGET Qt5::Widgets)
    endif()
endif()
find_package(Threads REQUIRED)

# 鲁班猫的旧系统 librga 与 multicore 内核不匹配。ARM64 默认使用
# 项目自带的配套静态库/头文件，不替换系统库，也不升级 glibc。
set(PCIE_RGA_DEFAULT OFF)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(PCIE_RGA_DEFAULT ON)
endif()
option(PCIE_ENABLE_RGA "Enable DMA-BUF/RGA preview on ARM64" ${PCIE_RGA_DEFAULT})

add_executable(pcie_viewer main.cpp)
target_link_libraries(pcie_viewer PRIVATE
    ${PCIE_QT_WIDGETS_TARGET}
    Threads::Threads
)
if(PCIE_ENABLE_RGA)
    if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
        message(FATAL_ERROR "Bundled RGA library is ARM64 only; use PCIE_ENABLE_RGA=OFF on x86")
    endif()
    set(PCIE_RGA_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/librga")
    if(NOT EXISTS "${PCIE_RGA_ROOT}/lib/librga.a" OR NOT EXISTS "${PCIE_RGA_ROOT}/include/rga/im2d.h")
        message(FATAL_ERROR "Copy the whole third_party/librga directory for RGA preview")
    endif()
    target_include_directories(pcie_viewer SYSTEM BEFORE PRIVATE "${PCIE_RGA_ROOT}/include")
    target_compile_definitions(pcie_viewer PRIVATE PCIE_HAVE_RGA=1)
    target_link_libraries(pcie_viewer PRIVATE "${PCIE_RGA_ROOT}/lib/librga.a" ${CMAKE_DL_LIBS} m)
else()
    target_compile_definitions(pcie_viewer PRIVATE PCIE_HAVE_RGA=0)
endif()
target_compile_options(pcie_viewer PRIVATE -Wall -Wextra -Wpedantic)
