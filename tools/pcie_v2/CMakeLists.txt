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

add_executable(pcie_viewer main.cpp)
target_link_libraries(pcie_viewer PRIVATE
    ${PCIE_QT_WIDGETS_TARGET}
    Threads::Threads
)
target_compile_options(pcie_viewer PRIVATE -Wall -Wextra -Wpedantic)
