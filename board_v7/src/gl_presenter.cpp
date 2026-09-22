#include "adas/gl_presenter.hpp"

#include <stdexcept>
#include <string>

#include <X11/Xatom.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>

namespace adas {
namespace {

#define GL_ENTRY_LIST(X) \
  X(ActiveTexture, ACTIVETEXTURE) X(AttachShader, ATTACHSHADER) \
  X(BindTexture, BINDTEXTURE) X(Clear, CLEAR) X(ClearColor, CLEARCOLOR) \
  X(CompileShader, COMPILESHADER) X(CreateProgram, CREATEPROGRAM) \
  X(CreateShader, CREATESHADER) X(DeleteProgram, DELETEPROGRAM) \
  X(DeleteShader, DELETESHADER) X(DeleteTextures, DELETETEXTURES) \
  X(Disable, DISABLE) X(DrawArrays, DRAWARRAYS) \
  X(EnableVertexAttribArray, ENABLEVERTEXATTRIBARRAY) \
  X(GenTextures, GENTEXTURES) X(GetAttribLocation, GETATTRIBLOCATION) \
  X(GetError, GETERROR) X(GetProgramiv, GETPROGRAMIV) \
  X(GetShaderInfoLog, GETSHADERINFOLOG) X(GetShaderiv, GETSHADERIV) \
  X(GetString, GETSTRING) X(GetUniformLocation, GETUNIFORMLOCATION) \
  X(LinkProgram, LINKPROGRAM) X(PixelStorei, PIXELSTOREI) \
  X(ShaderSource, SHADERSOURCE) X(TexImage2D, TEXIMAGE2D) \
  X(TexParameteri, TEXPARAMETERI) X(TexSubImage2D, TEXSUBIMAGE2D) \
  X(Uniform1i, UNIFORM1I) X(UseProgram, USEPROGRAM) \
  X(VertexAttribPointer, VERTEXATTRIBPOINTER) X(Viewport, VIEWPORT)

#define DECLARE_GL(name, upper) static PFNGL##upper##PROC p_gl##name = nullptr;
GL_ENTRY_LIST(DECLARE_GL)
#undef DECLARE_GL

void load_gl_entries() {
#define LOAD_GL(name, upper) \
  p_gl##name = reinterpret_cast<PFNGL##upper##PROC>(eglGetProcAddress("gl" #name)); \
  if (!p_gl##name) throw std::runtime_error("Missing GLES entry gl" #name);
  GL_ENTRY_LIST(LOAD_GL)
#undef LOAD_GL
}

#define glActiveTexture p_glActiveTexture
#define glAttachShader p_glAttachShader
#define glBindTexture p_glBindTexture
#define glClear p_glClear
#define glClearColor p_glClearColor
#define glCompileShader p_glCompileShader
#define glCreateProgram p_glCreateProgram
#define glCreateShader p_glCreateShader
#define glDeleteProgram p_glDeleteProgram
#define glDeleteShader p_glDeleteShader
#define glDeleteTextures p_glDeleteTextures
#define glDisable p_glDisable
#define glDrawArrays p_glDrawArrays
#define glEnableVertexAttribArray p_glEnableVertexAttribArray
#define glGenTextures p_glGenTextures
#define glGetAttribLocation p_glGetAttribLocation
#define glGetError p_glGetError
#define glGetProgramiv p_glGetProgramiv
#define glGetShaderInfoLog p_glGetShaderInfoLog
#define glGetShaderiv p_glGetShaderiv
#define glGetString p_glGetString
#define glGetUniformLocation p_glGetUniformLocation
#define glLinkProgram p_glLinkProgram
#define glPixelStorei p_glPixelStorei
#define glShaderSource p_glShaderSource
#define glTexImage2D p_glTexImage2D
#define glTexParameteri p_glTexParameteri
#define glTexSubImage2D p_glTexSubImage2D
#define glUniform1i p_glUniform1i
#define glUseProgram p_glUseProgram
#define glVertexAttribPointer p_glVertexAttribPointer
#define glViewport p_glViewport

GLuint compile_shader(GLenum type, const char* source) {
  const GLuint shader = glCreateShader(type);
  if (!shader) {
    throw std::runtime_error("glCreateShader failed, error=" +
                             std::to_string(static_cast<unsigned>(glGetError())));
  }
  glShaderSource(shader, 1, &source, nullptr);
  glCompileShader(shader);
  GLint ok = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
  if (!ok) {
    char log[1024] = {};
    glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
    glDeleteShader(shader);
    throw std::runtime_error(std::string("GLES shader: ") + log);
  }
  return shader;
}

}  // namespace

GlPresenter::GlPresenter(int width, int height) : width_(width), height_(height) {
  try {
    x_display_ = XOpenDisplay(nullptr);
    if (!x_display_) throw std::runtime_error("XOpenDisplay failed");
    egl_display_ = eglGetDisplay(reinterpret_cast<EGLNativeDisplayType>(x_display_));
    if (egl_display_ == EGL_NO_DISPLAY || !eglInitialize(egl_display_, nullptr, nullptr))
      throw std::runtime_error("eglInitialize failed");
    if (!eglBindAPI(EGL_OPENGL_ES_API)) throw std::runtime_error("eglBindAPI failed");
    const EGLint attributes[] = {
        EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
        EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8,
        EGL_NONE};
    EGLConfig config = nullptr;
    EGLint count = 0;
    if (!eglChooseConfig(egl_display_, attributes, &config, 1, &count) || count < 1)
      throw std::runtime_error("No GLES2 X11 config");
    EGLint visual_id = 0;
    eglGetConfigAttrib(egl_display_, config, EGL_NATIVE_VISUAL_ID, &visual_id);
    XVisualInfo* visual_info = nullptr;
    if (visual_id) {
      XVisualInfo lookup{};
      lookup.visualid = static_cast<VisualID>(visual_id);
      int matches = 0;
      visual_info = XGetVisualInfo(x_display_, VisualIDMask, &lookup, &matches);
    }
    Visual* visual = visual_info ? visual_info->visual : DefaultVisual(x_display_, DefaultScreen(x_display_));
    const int depth = visual_info ? visual_info->depth : DefaultDepth(x_display_, DefaultScreen(x_display_));
    const Window root = RootWindow(x_display_, DefaultScreen(x_display_));
    colormap_ = XCreateColormap(x_display_, root, visual, AllocNone);
    XSetWindowAttributes window_attrs{};
    window_attrs.colormap = colormap_;
    window_attrs.event_mask = ExposureMask | KeyPressMask | ButtonPressMask |
                              Button1MotionMask | StructureNotifyMask;
    window_attrs.override_redirect = False;
    window_ = XCreateWindow(x_display_, root, 0, 0, width_, height_, 0, depth,
                            InputOutput, visual,
                            CWColormap | CWEventMask, &window_attrs);
    if (visual_info) XFree(visual_info);
    if (!window_) throw std::runtime_error("XCreateWindow failed");
    XStoreName(x_display_, window_, "RK3568 V7 | NVIDIA 4 VIEW");

    // Let the desktop window manager own the window.  The previous
    // override_redirect window bypassed the WM, so Alt+Tab could not switch
    // away and Esc/q were easily lost when keyboard focus changed.
    wm_delete_window_ = XInternAtom(x_display_, "WM_DELETE_WINDOW", False);
    XSetWMProtocols(x_display_, window_, &wm_delete_window_, 1);
    net_wm_state_ = XInternAtom(x_display_, "_NET_WM_STATE", False);
    net_wm_state_fullscreen_ =
        XInternAtom(x_display_, "_NET_WM_STATE_FULLSCREEN", False);
    XChangeProperty(x_display_, window_, net_wm_state_, XA_ATOM, 32,
                    PropModeReplace,
                    reinterpret_cast<unsigned char*>(&net_wm_state_fullscreen_),
                    1);

    XMapWindow(x_display_, window_);
    XFlush(x_display_);
    egl_surface_ = eglCreateWindowSurface(egl_display_, config,
                                           static_cast<EGLNativeWindowType>(window_), nullptr);
    if (egl_surface_ == EGL_NO_SURFACE) throw std::runtime_error("eglCreateWindowSurface failed");
    const EGLint context_attrs[] = {EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE};
    egl_context_ = eglCreateContext(egl_display_, config, EGL_NO_CONTEXT, context_attrs);
    if (egl_context_ == EGL_NO_CONTEXT ||
        !eglMakeCurrent(egl_display_, egl_surface_, egl_surface_, egl_context_))
      throw std::runtime_error("GLES context failed");
    load_gl_entries();

    const char* vertex_source =
        "#version 100\n"
        "attribute vec2 aPos;\n"
        "attribute vec2 aUV;\n"
        "varying vec2 vUV;\n"
        "void main() { gl_Position = vec4(aPos, 0.0, 1.0); vUV = aUV; }\n";
    const char* fragment_source =
        "#version 100\n"
        "precision mediump float;\n"
        "varying vec2 vUV;\n"
        "uniform sampler2D uTex;\n"
        "void main() { vec4 c = texture2D(uTex, vUV); gl_FragColor = vec4(c.b, c.g, c.r, 1.0); }\n";
    const GLuint vertex = compile_shader(GL_VERTEX_SHADER, vertex_source);
    const GLuint fragment = compile_shader(GL_FRAGMENT_SHADER, fragment_source);
    program_ = glCreateProgram();
    glAttachShader(program_, vertex);
    glAttachShader(program_, fragment);
    glLinkProgram(program_);
    glDeleteShader(vertex);
    glDeleteShader(fragment);
    GLint linked = 0;
    glGetProgramiv(program_, GL_LINK_STATUS, &linked);
    if (!linked) throw std::runtime_error("GLES program link failed");
    pos_location_ = glGetAttribLocation(program_, "aPos");
    uv_location_ = glGetAttribLocation(program_, "aUV");
    glUseProgram(program_);
    glUniform1i(glGetUniformLocation(program_, "uTex"), 0);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glDisable(GL_DEPTH_TEST);
    init_texture(ui_texture_, width_, height_);
    for (GLuint& texture : camera_textures_) init_texture(texture, 640, 360);
    const GLubyte* name = glGetString(GL_RENDERER);
    renderer_ = name ? reinterpret_cast<const char*>(name) : "unknown";
  } catch (...) {
    cleanup();
    throw;
  }
}

GlPresenter::~GlPresenter() { cleanup(); }

void GlPresenter::init_texture(GLuint& texture, int width, int height) {
  glGenTextures(1, &texture);
  glBindTexture(GL_TEXTURE_2D, texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0,
               GL_RGB, GL_UNSIGNED_BYTE, nullptr);
}

void GlPresenter::draw_texture(GLuint texture, const cv::Mat& image,
                               int x, int y, int width, int height) {
  if (image.empty() || image.type() != CV_8UC3 || !image.isContinuous())
    throw std::runtime_error("GLES input must be contiguous BGR8");
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, texture);
  glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, image.cols, image.rows,
                  GL_RGB, GL_UNSIGNED_BYTE, image.data);
  glViewport(x, height_ - y - height, width, height);
  const GLfloat vertices[] = {
      -1.0f, -1.0f, 0.0f, 1.0f,
       1.0f, -1.0f, 1.0f, 1.0f,
      -1.0f,  1.0f, 0.0f, 0.0f,
       1.0f,  1.0f, 1.0f, 0.0f};
  glVertexAttribPointer(pos_location_, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), vertices);
  glVertexAttribPointer(uv_location_, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), vertices + 2);
  glEnableVertexAttribArray(pos_location_);
  glEnableVertexAttribArray(uv_location_);
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
}

void GlPresenter::present(const cv::Mat& ui, const std::array<cv::Mat, 4>& cameras,
                          const std::array<cv::Rect, 4>& view_rects) {
  glViewport(0, 0, width_, height_);
  glClearColor(0.07f, 0.09f, 0.11f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);
  draw_texture(ui_texture_, ui, 0, 0, width_, height_);
  for (int i = 0; i < 4; ++i)
    draw_texture(camera_textures_[i], cameras[i],
                 view_rects[i].x, view_rects[i].y,
                 view_rects[i].width, view_rects[i].height);
  if (!eglSwapBuffers(egl_display_, egl_surface_))
    throw std::runtime_error("eglSwapBuffers failed");
}

int GlPresenter::poll_input(int* click_x, int* click_y) {
  if (click_x) *click_x = -1;
  if (click_y) *click_y = -1;
  while (XPending(x_display_)) {
    XEvent event{};
    XNextEvent(x_display_, &event);
    if (event.type == ClientMessage &&
        static_cast<Atom>(event.xclient.data.l[0]) == wm_delete_window_) {
      return 27;
    }
    if (event.type == KeyPress) {
      const KeySym key = XLookupKeysym(&event.xkey, 0);
      if (key == XK_Escape) return 27;
      if (key == XK_q || key == XK_Q) return 'q';
      if (key == XK_F11) {
        set_fullscreen(!fullscreen_);
        continue;
      }
      if (key == XK_m || key == XK_M) return 'm';
      if (key == XK_c || key == XK_C) return 'c';
      if (key == XK_r || key == XK_R) return 'r';
      if (key == XK_bracketleft) return '[';
      if (key == XK_bracketright) return ']';
    }
    if (event.type == ButtonPress && event.xbutton.button == Button1) {
      if (click_x) *click_x = event.xbutton.x;
      if (click_y) *click_y = event.xbutton.y;
    }
    if (event.type == MotionNotify && (event.xmotion.state & Button1Mask)) {
      if (click_x) *click_x = event.xmotion.x;
      if (click_y) *click_y = event.xmotion.y;
    }
  }
  return -1;
}

void GlPresenter::set_fullscreen(bool enabled) {
  if (!x_display_ || !window_ || net_wm_state_ == None ||
      net_wm_state_fullscreen_ == None || fullscreen_ == enabled) {
    return;
  }

  XEvent event{};
  event.type = ClientMessage;
  event.xclient.window = window_;
  event.xclient.message_type = net_wm_state_;
  event.xclient.format = 32;
  event.xclient.data.l[0] = enabled ? 1 : 0;  // add/remove
  event.xclient.data.l[1] = net_wm_state_fullscreen_;
  event.xclient.data.l[2] = None;
  event.xclient.data.l[3] = 1;  // normal application request
  XSendEvent(x_display_, DefaultRootWindow(x_display_), False,
             SubstructureRedirectMask | SubstructureNotifyMask, &event);
  XFlush(x_display_);
  fullscreen_ = enabled;
}

void GlPresenter::cleanup() {
  if (egl_display_ != EGL_NO_DISPLAY && egl_context_ != EGL_NO_CONTEXT) {
    eglMakeCurrent(egl_display_, egl_surface_, egl_surface_, egl_context_);
    if (ui_texture_) glDeleteTextures(1, &ui_texture_);
    for (GLuint& texture : camera_textures_) if (texture) glDeleteTextures(1, &texture);
    if (program_) glDeleteProgram(program_);
  }
  if (egl_display_ != EGL_NO_DISPLAY) {
    eglMakeCurrent(egl_display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    if (egl_context_ != EGL_NO_CONTEXT) eglDestroyContext(egl_display_, egl_context_);
    if (egl_surface_ != EGL_NO_SURFACE) eglDestroySurface(egl_display_, egl_surface_);
    eglTerminate(egl_display_);
  }
  if (x_display_) {
    if (window_) XDestroyWindow(x_display_, window_);
    if (colormap_) XFreeColormap(x_display_, colormap_);
    XCloseDisplay(x_display_);
  }
}

}  // namespace adas
