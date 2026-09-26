#pragma once

#include <array>
#include <opencv2/core.hpp>
#include <string>

#include <EGL/egl.h>
#include <GLES2/gl2.h>
#include <X11/Xlib.h>

namespace adas {

class GlPresenter {
 public:
  GlPresenter(int width, int height);
  ~GlPresenter();
  GlPresenter(const GlPresenter&) = delete;
  GlPresenter& operator=(const GlPresenter&) = delete;

  void present(const cv::Mat& ui, const std::array<cv::Mat, 4>& cameras,
               const std::array<cv::Rect, 4>& view_rects);
  int poll_input(int* click_x, int* click_y);
  const std::string& renderer() const { return renderer_; }

 private:
  void draw_texture(GLuint texture, const cv::Mat& image,
                    int x, int y, int width, int height);
  void init_texture(GLuint& texture, int width, int height);
  void cleanup();

  int width_ = 0;
  int height_ = 0;
  Display* x_display_ = nullptr;
  Window window_ = 0;
  Colormap colormap_ = 0;
  Atom wm_delete_window_ = None;
  Atom net_wm_state_ = None;
  Atom net_wm_state_fullscreen_ = None;
  bool fullscreen_ = true;
  EGLDisplay egl_display_ = EGL_NO_DISPLAY;
  EGLContext egl_context_ = EGL_NO_CONTEXT;
  EGLSurface egl_surface_ = EGL_NO_SURFACE;
  GLuint program_ = 0;
  GLuint ui_texture_ = 0;
  std::array<GLuint, 4> camera_textures_{{0, 0, 0, 0}};
  GLint pos_location_ = -1;
  GLint uv_location_ = -1;
  std::string renderer_;

  void set_fullscreen(bool enabled);
};

}  // namespace adas
