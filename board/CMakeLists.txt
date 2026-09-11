cmake_minimum_required(VERSION 3.10)
project(rk3568_adas LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")

find_package(OpenCV REQUIRED)
find_library(RKNNRT_LIBRARY NAMES rknnrt PATHS /usr/lib /usr/local/lib)
if(NOT RKNNRT_LIBRARY)
  message(FATAL_ERROR "librknnrt.so was not found")
endif()

add_executable(adas_demo
  src/main.cpp
  src/rknn_detector.cpp
  src/lane_detector.cpp
  src/signal_logic.cpp
  src/risk_estimator.cpp
  src/overlay.cpp
)

target_include_directories(adas_demo PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/rknn
  ${OpenCV_INCLUDE_DIRS}
)
target_compile_options(adas_demo PRIVATE -O3 -Wall -Wextra)
target_link_libraries(adas_demo PRIVATE ${OpenCV_LIBS} ${RKNNRT_LIBRARY} pthread dl)

install(TARGETS adas_demo RUNTIME DESTINATION bin)
