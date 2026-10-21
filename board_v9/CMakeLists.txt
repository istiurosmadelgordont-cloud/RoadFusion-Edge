cmake_minimum_required(VERSION 3.10)
project(rk3568_adas LANGUAGES CXX)
find_package(OpenCV REQUIRED)

add_executable(npu_executor_regression EXCLUDE_FROM_ALL tests/npu_executor_regression.cpp)
target_include_directories(npu_executor_regression PRIVATE include)
target_compile_options(npu_executor_regression PRIVATE -UNDEBUG)
target_link_libraries(npu_executor_regression PRIVATE pthread)

add_executable(lane_visual_regression EXCLUDE_FROM_ALL tests/lane_visual_regression.cpp)
target_include_directories(lane_visual_regression PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_compile_options(lane_visual_regression PRIVATE -UNDEBUG)
target_link_libraries(lane_visual_regression PRIVATE opencv_core)

add_executable(lane_paint_regression EXCLUDE_FROM_ALL tests/lane_paint_regression.cpp)
target_include_directories(lane_paint_regression PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_compile_options(lane_paint_regression PRIVATE -UNDEBUG)
target_link_libraries(lane_paint_regression PRIVATE ${OpenCV_LIBS})

add_executable(light_verification_frames_regression EXCLUDE_FROM_ALL tests/light_verification_frames_regression.cpp)
target_include_directories(light_verification_frames_regression PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_compile_options(light_verification_frames_regression PRIVATE -UNDEBUG)
target_link_libraries(light_verification_frames_regression PRIVATE ${OpenCV_LIBS} pthread)

add_executable(lane_timeline_replay EXCLUDE_FROM_ALL tests/lane_timeline_replay.cpp)
target_include_directories(lane_timeline_replay PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(lane_timeline_replay PRIVATE ${OpenCV_LIBS})

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")

find_package(Freetype REQUIRED)
find_library(RKNNRT_LIBRARY NAMES rknnrt PATHS /usr/lib /usr/local/lib)
if(NOT RKNNRT_LIBRARY)
  message(FATAL_ERROR "librknnrt.so was not found")
endif()

add_executable(adas_demo
  src/main.cpp
  src/rknn_detector.cpp
  src/lamp_classifier.cpp
  src/lane_detector.cpp
  src/signal_logic.cpp
  src/drive_decision.cpp
  src/risk_estimator.cpp
  src/overlay.cpp
  src/text_renderer.cpp
)

add_executable(adas_four_view
  src/four_view.cpp
  src/byte_tracker.cpp
  src/warning_logic.cpp
  src/rknn_detector.cpp
  src/lamp_classifier.cpp
  src/signal_logic.cpp
  src/lane_detector.cpp
  src/drive_decision.cpp
  src/risk_estimator.cpp
  src/overlay.cpp
  src/text_renderer.cpp
  src/ufld_lane_detector.cpp
)

add_executable(ufld_benchmark src/ufld_benchmark.cpp)

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

# Use MPP NV12 metadata directly; videoconvert on this board is too slow.
find_path(GSTREAMER_INCLUDE_DIR gst/gst.h PATH_SUFFIXES gstreamer-1.0 REQUIRED)
find_path(GLIB_INCLUDE_DIR glib.h PATH_SUFFIXES glib-2.0 REQUIRED)
find_path(GLIB_CONFIG_INCLUDE_DIR glibconfig.h
  PATHS /usr/lib/aarch64-linux-gnu/glib-2.0/include /usr/lib/glib-2.0/include REQUIRED)
foreach(GST_LIB gstapp-1.0 gstvideo-1.0 gstreamer-1.0 gobject-2.0 glib-2.0)
  find_library(GST_${GST_LIB}_LIBRARY NAMES ${GST_LIB} REQUIRED)
  target_link_libraries(adas_four_view PRIVATE ${GST_${GST_LIB}_LIBRARY})
endforeach()
target_include_directories(adas_four_view PRIVATE
  ${GSTREAMER_INCLUDE_DIR} ${GLIB_INCLUDE_DIR} ${GLIB_CONFIG_INCLUDE_DIR})

target_include_directories(adas_demo PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/rknn
  ${OpenCV_INCLUDE_DIRS}
  ${FREETYPE_INCLUDE_DIRS}
)
target_compile_options(adas_demo PRIVATE -O3 -Wall -Wextra)
target_link_libraries(adas_demo PRIVATE ${OpenCV_LIBS} ${RKNNRT_LIBRARY} ${FREETYPE_LIBRARIES} pthread dl)

target_include_directories(adas_four_view PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/rknn
  ${OpenCV_INCLUDE_DIRS}
  ${FREETYPE_INCLUDE_DIRS}
)
target_compile_options(adas_four_view PRIVATE -O3 -Wall -Wextra)
target_link_libraries(adas_four_view PRIVATE ${OpenCV_LIBS} ${RKNNRT_LIBRARY} ${FREETYPE_LIBRARIES} pthread dl)
target_include_directories(ufld_benchmark PRIVATE include third_party/rknn ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ufld_benchmark PRIVATE ${OpenCV_LIBS} ${RKNNRT_LIBRARY} pthread dl)

install(TARGETS adas_demo RUNTIME DESTINATION bin)

add_executable(perception_regression EXCLUDE_FROM_ALL
  tests/perception_regression.cpp src/warning_logic.cpp src/signal_logic.cpp
  src/drive_decision.cpp src/risk_estimator.cpp src/byte_tracker.cpp)
target_include_directories(perception_regression PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(perception_regression PRIVATE ${OpenCV_LIBS})
# Release builds must still execute the regression assertions.
target_compile_options(perception_regression PRIVATE -UNDEBUG)

add_executable(overlay_regression EXCLUDE_FROM_ALL
  tests/overlay_regression.cpp src/overlay.cpp src/text_renderer.cpp)
target_include_directories(overlay_regression PRIVATE
  include ${OpenCV_INCLUDE_DIRS} ${FREETYPE_INCLUDE_DIRS})
target_link_libraries(overlay_regression PRIVATE ${OpenCV_LIBS} ${FREETYPE_LIBRARIES})
target_compile_options(overlay_regression PRIVATE -UNDEBUG)

add_executable(ufld_decode_regression EXCLUDE_FROM_ALL tests/ufld_decode_regression.cpp)
target_include_directories(ufld_decode_regression PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ufld_decode_regression PRIVATE ${OpenCV_LIBS})
target_compile_options(ufld_decode_regression PRIVATE -UNDEBUG)

add_executable(lane_video_replay EXCLUDE_FROM_ALL
  tests/lane_video_replay.cpp src/ufld_lane_detector.cpp src/warning_logic.cpp)
target_include_directories(lane_video_replay PRIVATE include third_party/rknn ${OpenCV_INCLUDE_DIRS})
target_link_libraries(lane_video_replay PRIVATE ${OpenCV_LIBS} ${RKNNRT_LIBRARY} ${RGA_LIBRARY} pthread dl)

add_executable(preview_resize_regression EXCLUDE_FROM_ALL tests/preview_resize_regression.cpp)
target_include_directories(preview_resize_regression PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(preview_resize_regression PRIVATE ${OpenCV_LIBS})
target_compile_options(preview_resize_regression PRIVATE -O3)
target_link_libraries(preview_resize_regression PRIVATE pthread)

add_executable(avm_regression EXCLUDE_FROM_ALL tests/avm_regression.cpp)
target_include_directories(avm_regression PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(avm_regression PRIVATE ${OpenCV_LIBS})
target_compile_options(avm_regression PRIVATE -UNDEBUG)

add_executable(ui_upload_regression EXCLUDE_FROM_ALL tests/ui_upload_regression.cpp)

add_executable(lane_kalman_regression EXCLUDE_FROM_ALL tests/lane_kalman_regression.cpp)
target_include_directories(lane_kalman_regression PRIVATE include)
target_compile_options(lane_kalman_regression PRIVATE -UNDEBUG)

add_executable(lane_outer_style_regression EXCLUDE_FROM_ALL tests/lane_outer_style_regression.cpp)
target_include_directories(lane_outer_style_regression PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(lane_outer_style_regression PRIVATE ${OpenCV_LIBS})
target_compile_options(lane_outer_style_regression PRIVATE -UNDEBUG)
target_include_directories(ui_upload_regression PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ui_upload_regression PRIVATE ${OpenCV_LIBS})
target_compile_options(ui_upload_regression PRIVATE -UNDEBUG)

# 检查 PCIe/回放共用的用途车道、按钮按住/松开和预警入口；不访问设备或模型。
# 测试包含主程序源码，未调用的硬件执行路径由链接器移除。
add_executable(input_mode_regression EXCLUDE_FROM_ALL
  tests/input_mode_regression.cpp src/text_renderer.cpp)
target_include_directories(input_mode_regression PRIVATE
  include third_party/rknn ${OpenCV_INCLUDE_DIRS} ${FREETYPE_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIR} ${GLIB_INCLUDE_DIR} ${GLIB_CONFIG_INCLUDE_DIR})
target_compile_definitions(input_mode_regression PRIVATE ADAS_HAVE_GLES=1)
target_compile_options(input_mode_regression PRIVATE -UNDEBUG -ffunction-sections -fdata-sections)
set_property(TARGET input_mode_regression APPEND_STRING PROPERTY LINK_FLAGS " -Wl,--gc-sections")
target_link_libraries(input_mode_regression PRIVATE ${OpenCV_LIBS} ${FREETYPE_LIBRARIES} pthread)
foreach(GST_LIB gstapp-1.0 gstvideo-1.0 gstreamer-1.0 gobject-2.0 glib-2.0)
  target_link_libraries(input_mode_regression PRIVATE ${GST_${GST_LIB}_LIBRARY})
endforeach()
