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
  src/drive_decision.cpp
  src/risk_estimator.cpp
  src/overlay.cpp
)

add_executable(adas_four_view
  src/four_view.cpp
  src/byte_tracker.cpp
  src/rknn_detector.cpp
  src/signal_logic.cpp
  src/lane_detector.cpp
  src/drive_decision.cpp
  src/risk_estimator.cpp
  src/overlay.cpp
)

find_library(EGL_LIBRARY NAMES EGL)
find_library(GLES_LIBRARY NAMES GLESv2)
find_library(X11_LIBRARY NAMES X11)
find_library(RGA_LIBRARY NAMES rga)
if(NOT EGL_LIBRARY OR NOT GLES_LIBRARY OR NOT X11_LIBRARY OR NOT RGA_LIBRARY)
  message(FATAL_ERROR "Four-view demo needs EGL, GLESv2, X11 and RGA")
endif()
target_sources(adas_four_view PRIVATE src/gl_presenter.cpp)
target_compile_definitions(adas_four_view PRIVATE ADAS_HAVE_GLES=1)
target_link_libraries(adas_four_view PRIVATE ${EGL_LIBRARY} ${GLES_LIBRARY} ${X11_LIBRARY} ${RGA_LIBRARY})

target_include_directories(adas_demo PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/rknn
  ${OpenCV_INCLUDE_DIRS}
)
target_compile_options(adas_demo PRIVATE -O3 -Wall -Wextra)
target_link_libraries(adas_demo PRIVATE ${OpenCV_LIBS} ${RKNNRT_LIBRARY} pthread dl)

target_include_directories(adas_four_view PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/rknn
  ${OpenCV_INCLUDE_DIRS}
)
target_compile_options(adas_four_view PRIVATE -O3 -Wall -Wextra)
target_link_libraries(adas_four_view PRIVATE ${OpenCV_LIBS} ${RKNNRT_LIBRARY} pthread dl)

install(TARGETS adas_demo RUNTIME DESTINATION bin)
