#include "mujoco_simulation/component/camera/camera_renderer.hpp"

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <mutex>
#include <utility>

#include "mujoco_simulation/common/logging.hpp"
#include "mujoco_simulation/common/macro.hpp"

namespace mujoco_simulation {

bool CameraRenderer::ensure_glfw_initialized() {
  static std::mutex mutex;
  static bool initialized = false;

  std::lock_guard<std::mutex> lock(mutex);
  if (initialized) {
    return true;
  }
  if (glfwInit() == GLFW_FALSE) {
    return false;
  }
  initialized = true;
  return true;
}

CameraRenderer::CameraRenderer() : CameraRenderer(CameraRendererConfig{}) {}

CameraRenderer::CameraRenderer(CameraRendererConfig config) : config_(config) {}

CameraRenderer::~CameraRenderer() { UNUSED(shutdown()); }

bool CameraRenderer::initialize(const mjModel& model) {
  if (initialized_) {
    return true;
  }

  if (!ensure_gl_context()) {
    return false;
  }

  render_data_ = mj_makeData(&model);
  if (render_data_ == nullptr) {
    release_current_context();
    LOG_ERROR << "failed to allocate render data.";
    return false;
  }

  mjv_defaultScene(&scene_);
  mjv_defaultOption(&option_);
  mjr_defaultContext(&render_context_);
  option_.flags[mjVIS_RANGEFINDER] = 0;
  for (int group = 0; group < mjNGROUP; ++group) {
    option_.sitegroup[group] = 0;
  }

  mjv_makeScene(&model, &scene_, config_.max_scene_geometries);
  mjr_makeContext(&model, &render_context_, mjFONTSCALE_150);
  mjr_setBuffer(mjFB_OFFSCREEN, &render_context_);
  initialized_ = true;
  release_current_context();
  return true;
}

bool CameraRenderer::shutdown() {
  if (gl_context_.backend == OffscreenGlBackend::Glfw && gl_context_.window != nullptr) {
    glfwMakeContextCurrent(gl_context_.window);
  } else if (gl_context_.backend == OffscreenGlBackend::Egl &&
             gl_context_.egl_display != EGL_NO_DISPLAY &&
             gl_context_.egl_context != EGL_NO_CONTEXT &&
             gl_context_.egl_surface != EGL_NO_SURFACE) {
    UNUSED(eglMakeCurrent(static_cast<::EGLDisplay>(gl_context_.egl_display),
                          static_cast<::EGLSurface>(gl_context_.egl_surface),
                          static_cast<::EGLSurface>(gl_context_.egl_surface),
                          static_cast<::EGLContext>(gl_context_.egl_context)));
  }
  if (initialized_) {
    mjv_freeScene(&scene_);
    mjr_freeContext(&render_context_);
    initialized_ = false;
  }
  if (render_data_ != nullptr) {
    mj_deleteData(render_data_);
    render_data_ = nullptr;
  }
  if (gl_context_.window != nullptr) {
    glfwDestroyWindow(gl_context_.window);
    gl_context_.window = nullptr;
  }
  if (gl_context_.egl_display != EGL_NO_DISPLAY) {
    eglMakeCurrent(static_cast<::EGLDisplay>(gl_context_.egl_display), EGL_NO_SURFACE,
                   EGL_NO_SURFACE, EGL_NO_CONTEXT);
    if (gl_context_.egl_surface != EGL_NO_SURFACE) {
      eglDestroySurface(static_cast<::EGLDisplay>(gl_context_.egl_display),
                        static_cast<::EGLSurface>(gl_context_.egl_surface));
      gl_context_.egl_surface = EGL_NO_SURFACE;
    }
    if (gl_context_.egl_context != EGL_NO_CONTEXT) {
      eglDestroyContext(static_cast<::EGLDisplay>(gl_context_.egl_display),
                        static_cast<::EGLContext>(gl_context_.egl_context));
      gl_context_.egl_context = EGL_NO_CONTEXT;
    }
    eglTerminate(static_cast<::EGLDisplay>(gl_context_.egl_display));
    gl_context_.egl_display = EGL_NO_DISPLAY;
  }
  offscreen_width_ = 0;
  offscreen_height_ = 0;
  gl_context_.backend = OffscreenGlBackend::None;
  release_current_context();
  return true;
}

bool CameraRenderer::copy_simulation_data(const mjModel& model, const mjData& source) {
  if (!initialize(model)) {
    return false;
  }
  if (gl_context_.backend == OffscreenGlBackend::Glfw && gl_context_.window != nullptr) {
    glfwMakeContextCurrent(gl_context_.window);
  } else if (gl_context_.backend == OffscreenGlBackend::Egl) {
    if (!eglMakeCurrent(static_cast<::EGLDisplay>(gl_context_.egl_display),
                        static_cast<::EGLSurface>(gl_context_.egl_surface),
                        static_cast<::EGLSurface>(gl_context_.egl_surface),
                        static_cast<::EGLContext>(gl_context_.egl_context))) {
      LOG_ERROR << "failed to make EGL context current.";
      return false;
    }
  }
  if (mj_copyData(render_data_, &model, &source) == nullptr) {
    release_current_context();
    LOG_ERROR << "failed to copy MuJoCo simulation data.";
    return false;
  }
  release_current_context();
  return true;
}

bool CameraRenderer::render(const mjModel& model, const CameraConfig& spec, std::uint64_t sequence,
                            std::uint64_t timestamp,
                            std::shared_ptr<const CameraRenderState>& out) {
  if (!initialize(model)) {
    return false;
  }

  int camera_id = -1;
  double fovy_degrees = 0.0;
  if (!ensure_camera_binding(model, spec, camera_id, fovy_degrees)) {
    return false;
  }
  if (!ensure_offscreen_capacity(spec.width, spec.height)) {
    return false;
  }

  if (gl_context_.backend == OffscreenGlBackend::Glfw && gl_context_.window != nullptr) {
    glfwMakeContextCurrent(gl_context_.window);
  } else if (gl_context_.backend == OffscreenGlBackend::Egl) {
    if (!eglMakeCurrent(static_cast<::EGLDisplay>(gl_context_.egl_display),
                        static_cast<::EGLSurface>(gl_context_.egl_surface),
                        static_cast<::EGLSurface>(gl_context_.egl_surface),
                        static_cast<::EGLContext>(gl_context_.egl_context))) {
      LOG_ERROR << "failed to make EGL context current.";
      return false;
    }
  }
  mjr_setBuffer(mjFB_OFFSCREEN, &render_context_);

  mjvCamera camera{};
  mjv_defaultCamera(&camera);
  camera.type = mjCAMERA_FIXED;
  camera.fixedcamid = camera_id;

  const mjrRect viewport{0, 0, spec.width, spec.height};
  mjv_updateScene(&model, render_data_, &option_, nullptr, &camera, mjCAT_ALL, &scene_);
  mjr_render(viewport, &scene_, &render_context_);

  auto state = std::make_shared<CameraRenderState>();
  state->sequence = sequence;
  state->timestamp = timestamp;
  state->frame_id = spec.frame_id.empty() ? spec.name : spec.frame_id;
  state->optical_frame_id = spec.optical_frame_id.empty() ? state->frame_id : spec.optical_frame_id;
  state->intrinsics = compute_intrinsics(fovy_degrees, static_cast<std::uint32_t>(spec.width),
                                         static_cast<std::uint32_t>(spec.height));

  std::vector<std::uint8_t> rgb_buffer;
  std::vector<float> depth_buffer;
  unsigned char* rgb_ptr = nullptr;
  float* depth_ptr = nullptr;

  if (spec.enable_rgb) {
    rgb_buffer.assign(
        static_cast<std::size_t>(spec.width) * static_cast<std::size_t>(spec.height) * 3U, 0U);
    rgb_ptr = rgb_buffer.data();
  }
  if (spec.enable_depth) {
    depth_buffer.assign(
        static_cast<std::size_t>(spec.width) * static_cast<std::size_t>(spec.height), 0.0F);
    depth_ptr = depth_buffer.data();
  }
  mjr_readPixels(rgb_ptr, depth_ptr, viewport, &render_context_);
  release_current_context();

  if (spec.enable_rgb) {
    state->color.width = static_cast<std::uint32_t>(spec.width);
    state->color.height = static_cast<std::uint32_t>(spec.height);
    state->color.step = static_cast<std::uint32_t>(spec.width * 3);
    state->color.data.resize(rgb_buffer.size());
    flip_rgb(rgb_buffer, state->color.width, state->color.height, state->color.data);
  }

  if (spec.enable_depth) {
    state->depth.width = static_cast<std::uint32_t>(spec.width);
    state->depth.height = static_cast<std::uint32_t>(spec.height);
    state->depth.data.resize(depth_buffer.size());
    convert_and_flip_depth(model, depth_buffer, state->depth.width, state->depth.height,
                           state->depth.data);
  }

  out = std::static_pointer_cast<const CameraRenderState>(state);
  return true;
}

bool CameraRenderer::ensure_gl_context() {
  if (gl_context_.backend == OffscreenGlBackend::Glfw && gl_context_.window != nullptr) {
    glfwMakeContextCurrent(gl_context_.window);
    return true;
  }
  if (gl_context_.backend == OffscreenGlBackend::Egl && gl_context_.egl_display != EGL_NO_DISPLAY &&
      gl_context_.egl_context != EGL_NO_CONTEXT && gl_context_.egl_surface != EGL_NO_SURFACE) {
    if (!eglMakeCurrent(static_cast<::EGLDisplay>(gl_context_.egl_display),
                        static_cast<::EGLSurface>(gl_context_.egl_surface),
                        static_cast<::EGLSurface>(gl_context_.egl_surface),
                        static_cast<::EGLContext>(gl_context_.egl_context))) {
      LOG_ERROR << "failed to restore EGL context.";
      return false;
    }
    return true;
  }

  if (config_.allow_glfw_backend && ensure_glfw_initialized()) {
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
    gl_context_.window = glfwCreateWindow(1, 1, "", nullptr, nullptr);
    if (gl_context_.window != nullptr) {
      gl_context_.backend = OffscreenGlBackend::Glfw;
      glfwMakeContextCurrent(gl_context_.window);
      return true;
    }
  }

  if (!config_.allow_egl_backend) {
    LOG_ERROR << "no offscreen rendering backend is allowed.";
    return false;
  }
  return initialize_egl_context();
}

bool CameraRenderer::ensure_offscreen_capacity(int width, int height) {
  if (width <= 0 || height <= 0) {
    LOG_ERROR << "offscreen width and height must be positive.";
    return false;
  }
  if (width <= offscreen_width_ && height <= offscreen_height_) {
    return true;
  }

  if (gl_context_.backend == OffscreenGlBackend::Glfw && gl_context_.window != nullptr) {
    glfwMakeContextCurrent(gl_context_.window);
  } else if (gl_context_.backend == OffscreenGlBackend::Egl) {
    if (!eglMakeCurrent(static_cast<::EGLDisplay>(gl_context_.egl_display),
                        static_cast<::EGLSurface>(gl_context_.egl_surface),
                        static_cast<::EGLSurface>(gl_context_.egl_surface),
                        static_cast<::EGLContext>(gl_context_.egl_context))) {
      LOG_ERROR << "failed to make EGL context current.";
      return false;
    }
  }
  offscreen_width_ = std::max(offscreen_width_, width);
  offscreen_height_ = std::max(offscreen_height_, height);
  mjr_resizeOffscreen(offscreen_width_, offscreen_height_, &render_context_);
  mjr_setBuffer(mjFB_OFFSCREEN, &render_context_);
  release_current_context();
  return true;
}

bool CameraRenderer::ensure_camera_binding(const mjModel& model, const CameraConfig& spec,
                                           int& camera_id, double& fovy_degrees) {
  if (spec.name.empty()) {
    LOG_ERROR << "camera component name must not be empty.";
    return false;
  }
  if (spec.camera_name.empty()) {
    LOG_ERROR << "camera name must not be empty.";
    return false;
  }
  if (spec.width <= 0 || spec.height <= 0) {
    LOG_ERROR << "camera width and height must be positive.";
    return false;
  }
  if (!spec.enable_rgb && !spec.enable_depth) {
    LOG_ERROR << "camera must enable rgb or depth output.";
    return false;
  }

  const int id = mj_name2id(&model, mjOBJ_CAMERA, spec.camera_name.c_str());
  if (id < 0) {
    LOG_ERROR << "camera was not found in model.";
    return false;
  }

  camera_id = id;
  fovy_degrees = static_cast<double>(model.cam_fovy[id]);
  return true;
}

CameraRenderIntrinsics CameraRenderer::compute_intrinsics(double fovy_degrees, std::uint32_t width,
                                                          std::uint32_t height) const {
  CameraRenderIntrinsics intrinsics;
  if (width == 0 || height == 0) {
    return intrinsics;
  }

  const double aspect = static_cast<double>(width) / static_cast<double>(height);
  const double fovy_radians = fovy_degrees * M_PI / 180.0;
  intrinsics.fy = static_cast<double>(height) / (2.0 * std::tan(fovy_radians / 2.0));
  const double fovx_radians = 2.0 * std::atan(aspect * std::tan(fovy_radians / 2.0));
  intrinsics.fx = static_cast<double>(width) / (2.0 * std::tan(fovx_radians / 2.0));
  intrinsics.cx = (static_cast<double>(width) - 1.0) / 2.0;
  intrinsics.cy = (static_cast<double>(height) - 1.0) / 2.0;
  intrinsics.k = {intrinsics.fx, 0.0, intrinsics.cx, 0.0, intrinsics.fy,
                  intrinsics.cy, 0.0, 0.0,           1.0};
  intrinsics.p = {intrinsics.fx, 0.0, intrinsics.cx, 0.0, 0.0, intrinsics.fy,
                  intrinsics.cy, 0.0, 0.0,           0.0, 1.0, 0.0};
  return intrinsics;
}

void CameraRenderer::flip_rgb(const std::vector<std::uint8_t>& source, std::uint32_t width,
                              std::uint32_t height, std::vector<std::uint8_t>& dest) const {
  const std::size_t row_size = static_cast<std::size_t>(width) * 3U;
  for (std::uint32_t row = 0; row < height; ++row) {
    const std::size_t src_offset = static_cast<std::size_t>(row) * row_size;
    const std::size_t dst_offset = static_cast<std::size_t>(height - 1U - row) * row_size;
    std::memcpy(dest.data() + dst_offset, source.data() + src_offset, row_size);
  }
}

void CameraRenderer::convert_and_flip_depth(const mjModel& model, const std::vector<float>& source,
                                            std::uint32_t width, std::uint32_t height,
                                            std::vector<float>& dest) const {
  const float near = static_cast<float>(model.vis.map.znear * model.stat.extent);
  const float far = static_cast<float>(model.vis.map.zfar * model.stat.extent);
  const float depth_scale = 1.0F - near / far;

  for (std::uint32_t row = 0; row < height; ++row) {
    for (std::uint32_t column = 0; column < width; ++column) {
      const std::size_t src_index = static_cast<std::size_t>(row) * width + column;
      const std::size_t dst_index = static_cast<std::size_t>(height - 1U - row) * width + column;
      const float normalized = source[src_index];
      dest[dst_index] = near / (1.0F - normalized * depth_scale);
    }
  }
}

bool CameraRenderer::initialize_egl_context() {
  gl_context_.egl_display =
      eglGetPlatformDisplay(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr);
  if (gl_context_.egl_display == EGL_NO_DISPLAY) {
    gl_context_.egl_display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
  }
  if (gl_context_.egl_display == EGL_NO_DISPLAY) {
    LOG_ERROR << "failed to acquire EGL display.";
    return false;
  }

  EGLint major = 0;
  EGLint minor = 0;
  if (!eglInitialize(static_cast<::EGLDisplay>(gl_context_.egl_display), &major, &minor)) {
    gl_context_.egl_display = EGL_NO_DISPLAY;
    LOG_ERROR << "failed to initialize EGL.";
    return false;
  }

  const EGLint config_attribs[] = {EGL_SURFACE_TYPE,
                                   EGL_PBUFFER_BIT,
                                   EGL_RED_SIZE,
                                   8,
                                   EGL_GREEN_SIZE,
                                   8,
                                   EGL_BLUE_SIZE,
                                   8,
                                   EGL_ALPHA_SIZE,
                                   8,
                                   EGL_DEPTH_SIZE,
                                   24,
                                   EGL_RENDERABLE_TYPE,
                                   EGL_OPENGL_BIT,
                                   EGL_NONE};
  EGLConfig egl_config = nullptr;
  EGLint num_configs = 0;
  if (!eglChooseConfig(static_cast<::EGLDisplay>(gl_context_.egl_display), config_attribs,
                       &egl_config, 1, &num_configs) ||
      num_configs == 0) {
    eglTerminate(static_cast<::EGLDisplay>(gl_context_.egl_display));
    gl_context_.egl_display = EGL_NO_DISPLAY;
    LOG_ERROR << "failed to choose EGL config.";
    return false;
  }

  if (!eglBindAPI(EGL_OPENGL_API)) {
    eglTerminate(static_cast<::EGLDisplay>(gl_context_.egl_display));
    gl_context_.egl_display = EGL_NO_DISPLAY;
    LOG_ERROR << "failed to bind EGL OpenGL API.";
    return false;
  }

  gl_context_.egl_context = eglCreateContext(static_cast<::EGLDisplay>(gl_context_.egl_display),
                                             egl_config, EGL_NO_CONTEXT, nullptr);
  if (gl_context_.egl_context == EGL_NO_CONTEXT) {
    eglTerminate(static_cast<::EGLDisplay>(gl_context_.egl_display));
    gl_context_.egl_display = EGL_NO_DISPLAY;
    LOG_ERROR << "failed to create EGL context.";
    return false;
  }

  const EGLint pbuffer_attribs[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
  gl_context_.egl_surface = eglCreatePbufferSurface(
      static_cast<::EGLDisplay>(gl_context_.egl_display), egl_config, pbuffer_attribs);
  if (gl_context_.egl_surface == EGL_NO_SURFACE) {
    eglDestroyContext(static_cast<::EGLDisplay>(gl_context_.egl_display),
                      static_cast<::EGLContext>(gl_context_.egl_context));
    gl_context_.egl_context = EGL_NO_CONTEXT;
    eglTerminate(static_cast<::EGLDisplay>(gl_context_.egl_display));
    gl_context_.egl_display = EGL_NO_DISPLAY;
    LOG_ERROR << "failed to create EGL pbuffer surface.";
    return false;
  }

  if (!eglMakeCurrent(static_cast<::EGLDisplay>(gl_context_.egl_display),
                      static_cast<::EGLSurface>(gl_context_.egl_surface),
                      static_cast<::EGLSurface>(gl_context_.egl_surface),
                      static_cast<::EGLContext>(gl_context_.egl_context))) {
    eglDestroySurface(static_cast<::EGLDisplay>(gl_context_.egl_display),
                      static_cast<::EGLSurface>(gl_context_.egl_surface));
    gl_context_.egl_surface = EGL_NO_SURFACE;
    eglDestroyContext(static_cast<::EGLDisplay>(gl_context_.egl_display),
                      static_cast<::EGLContext>(gl_context_.egl_context));
    gl_context_.egl_context = EGL_NO_CONTEXT;
    eglTerminate(static_cast<::EGLDisplay>(gl_context_.egl_display));
    gl_context_.egl_display = EGL_NO_DISPLAY;
    LOG_ERROR << "failed to activate EGL context.";
    return false;
  }

  gl_context_.backend = OffscreenGlBackend::Egl;
  return true;
}

void CameraRenderer::release_current_context() {
  if (gl_context_.backend == OffscreenGlBackend::Glfw && gl_context_.window != nullptr) {
    glfwMakeContextCurrent(nullptr);
    return;
  }
  if (gl_context_.backend == OffscreenGlBackend::Egl && gl_context_.egl_display != EGL_NO_DISPLAY) {
    eglMakeCurrent(static_cast<::EGLDisplay>(gl_context_.egl_display), EGL_NO_SURFACE,
                   EGL_NO_SURFACE, EGL_NO_CONTEXT);
  }
}

}  // namespace mujoco_simulation
