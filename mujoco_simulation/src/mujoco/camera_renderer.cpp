#include "mujoco_simulation/mujoco/camera_renderer.hpp"

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <mutex>
#include <utility>

#include "common/logging.hpp"
#include "mujoco_simulation/common/macro.hpp"

namespace mujoco_simulation {
namespace {

constexpr int kHiddenContextWidth = 1;
constexpr int kHiddenContextHeight = 1;
constexpr int kColorChannelCount = 3;
constexpr int kEglColorBits = 8;
constexpr int kEglDepthBits = 24;
constexpr int kFontScale = mjFONTSCALE_150;
constexpr double kPi = 3.14159265358979323846;

template <typename Callback> class ScopeExit {
public:
  explicit ScopeExit(Callback callback) : callback_(std::move(callback)) {}
  ~ScopeExit() { callback_(); }

  ScopeExit(const ScopeExit &) = delete;
  ScopeExit &operator=(const ScopeExit &) = delete;

private:
  Callback callback_;
};

template <typename Callback> ScopeExit(Callback) -> ScopeExit<Callback>;

bool pixel_count_for(int width, int height, std::size_t &pixel_count) {
  if (width <= 0 || height <= 0) {
    return false;
  }

  const auto unsigned_width = static_cast<std::size_t>(width);
  const auto unsigned_height = static_cast<std::size_t>(height);
  if (unsigned_width >
      std::numeric_limits<std::size_t>::max() / unsigned_height) {
    return false;
  }

  pixel_count = unsigned_width * unsigned_height;
  return true;
}

} // namespace

CameraRenderer::CameraRenderer() : CameraRenderer(CameraRendererConfig{}) {}

CameraRenderer::CameraRenderer(CameraRendererConfig config) : config_(config) {}

CameraRenderer::~CameraRenderer() { UNUSED(release()); }

bool CameraRenderer::initialize(const mjContext &context) {
  if (initialized_) {
    return true;
  }
  if (!context.valid()) {
    LOG_ERROR << "camera renderer requires a valid MuJoCo context.";
    return false;
  }
  if (config_.max_scene_geometries <= 0) {
    LOG_ERROR << "max_scene_geometries must be positive.";
    return false;
  }
  if (!create_context()) {
    return false;
  }

  bool succeeded = false;
  if (activate_context()) {
    {
      const ScopeExit deactivate([this] { deactivate_context(); });
      succeeded = create_render_resources(context);
    }
  }

  if (!succeeded) {
    UNUSED(release());
  }
  return succeeded;
}

bool CameraRenderer::release() {
  const bool has_context = gl_context_.backend != OffscreenGlBackend::None;
  const bool context_is_active = has_context && activate_context();
  if ((initialized_ || render_data_ != nullptr) && !context_is_active) {
    LOG_ERROR << "failed to activate offscreen context during release.";
  }

  destroy_render_resources();
  if (context_is_active) {
    deactivate_context();
  }
  destroy_context();
  offscreen_width_ = 0;
  offscreen_height_ = 0;
  return true;
}

bool CameraRenderer::copy_simulation_data(const mjContext &context) {
  if (!context.valid()) {
    LOG_ERROR << "camera renderer requires a valid MuJoCo context.";
    return false;
  }
  if (!initialize(context)) {
    return false;
  }
  if (!activate_context()) {
    return false;
  }
  const ScopeExit deactivate([this] { deactivate_context(); });

  if (render_data_ == nullptr ||
      mj_copyData(render_data_, context.model, context.data) == nullptr) {
    LOG_ERROR << "failed to copy MuJoCo simulation data.";
    return false;
  }
  return true;
}

bool CameraRenderer::render(const mjContext &context, const CameraConfig &spec,
                            std::uint64_t sequence, std::uint64_t timestamp,
                            std::shared_ptr<const CameraRenderState> &out) {
  if (!context.valid()) {
    LOG_ERROR << "camera renderer requires a valid MuJoCo context.";
    return false;
  }
  if (!initialize(context)) {
    return false;
  }

  int camera_id = -1;
  double fovy_degrees = 0.0;
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
  camera_id = mj_name2id(context.model, mjOBJ_CAMERA, spec.camera_name.c_str());
  if (camera_id < 0) {
    LOG_ERROR << "camera was not found in model.";
    return false;
  }
  fovy_degrees = static_cast<double>(context.model->cam_fovy[camera_id]);
  if (!std::isfinite(fovy_degrees) || fovy_degrees <= 0.0 ||
      fovy_degrees >= 180.0) {
    LOG_ERROR << "camera field of view must be in the range (0, 180) degrees.";
    return false;
  }
  if (!activate_context()) {
    return false;
  }
  const ScopeExit deactivate([this] { deactivate_context(); });
  if (!resize_offscreen_buffer(spec.width, spec.height)) {
    return false;
  }
  if (render_data_ == nullptr) {
    LOG_ERROR << "render data is not initialized.";
    return false;
  }

  std::size_t pixel_count = 0;
  if (!pixel_count_for(spec.width, spec.height, pixel_count) ||
      pixel_count >
          std::numeric_limits<std::size_t>::max() / kColorChannelCount) {
    LOG_ERROR << "camera image dimensions exceed the supported pixel capacity.";
    return false;
  }

  mjr_setBuffer(mjFB_OFFSCREEN, &render_context_);

  mjvCamera camera{};
  mjv_defaultCamera(&camera);
  camera.type = mjCAMERA_FIXED;
  camera.fixedcamid = camera_id;

  const mjrRect viewport{0, 0, spec.width, spec.height};
  mjv_updateScene(context.model, render_data_, &option_, nullptr, &camera,
                  mjCAT_ALL, &scene_);
  mjr_render(viewport, &scene_, &render_context_);

  auto state = std::make_shared<CameraRenderState>();
  state->sequence = sequence;
  state->timestamp = timestamp;
  state->frame_id = spec.frame_id.empty() ? spec.name : spec.frame_id;
  state->optical_frame_id =
      spec.optical_frame_id.empty() ? state->frame_id : spec.optical_frame_id;
  state->intrinsics =
      compute_intrinsics(fovy_degrees, static_cast<std::uint32_t>(spec.width),
                         static_cast<std::uint32_t>(spec.height));

  std::vector<std::uint8_t> rgb_buffer;
  std::vector<float> depth_buffer;
  unsigned char *rgb_ptr = nullptr;
  float *depth_ptr = nullptr;

  if (spec.enable_rgb) {
    rgb_buffer.assign(pixel_count * kColorChannelCount, 0U);
    rgb_ptr = rgb_buffer.data();
  }
  if (spec.enable_depth) {
    depth_buffer.assign(pixel_count, 0.0F);
    depth_ptr = depth_buffer.data();
  }
  mjr_readPixels(rgb_ptr, depth_ptr, viewport, &render_context_);

  if (spec.enable_rgb) {
    state->color.width = static_cast<std::uint32_t>(spec.width);
    state->color.height = static_cast<std::uint32_t>(spec.height);
    state->color.step =
        static_cast<std::uint32_t>(spec.width * kColorChannelCount);
    state->color.data.resize(rgb_buffer.size());
    transform_rgb(rgb_buffer, state->color.width, state->color.height,
                  state->color.data);
  }

  if (spec.enable_depth) {
    state->depth.width = static_cast<std::uint32_t>(spec.width);
    state->depth.height = static_cast<std::uint32_t>(spec.height);
    state->depth.data.resize(depth_buffer.size());
    if (!transform_depth(context, depth_buffer, state->depth.width,
                         state->depth.height, state->depth.data)) {
      return false;
    }
  }

  out = std::static_pointer_cast<const CameraRenderState>(state);
  return true;
}

bool CameraRenderer::create_context() {
  if (gl_context_.backend != OffscreenGlBackend::None) {
    return true;
  }

  if (config_.allow_glfw_backend && create_glfw_context()) {
    return true;
  }

  if (!config_.allow_egl_backend) {
    LOG_ERROR << "no usable offscreen rendering backend is available.";
    return false;
  }
  return create_egl_context();
}

bool CameraRenderer::create_glfw_context() {
  static std::mutex mutex;
  static bool glfw_initialized = false;
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (!glfw_initialized) {
      if (glfwInit() == GLFW_FALSE) {
        LOG_WARNING << "failed to initialize GLFW; trying EGL.";
        return false;
      }
      glfw_initialized = true;
    }
  }

  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
  gl_context_.window = glfwCreateWindow(
      kHiddenContextWidth, kHiddenContextHeight, "", nullptr, nullptr);
  if (gl_context_.window == nullptr) {
    LOG_WARNING << "failed to create hidden GLFW OpenGL context; trying EGL.";
    return false;
  }
  gl_context_.backend = OffscreenGlBackend::Glfw;
  return true;
}

bool CameraRenderer::activate_context() {
  if (gl_context_.backend == OffscreenGlBackend::Glfw &&
      gl_context_.window != nullptr) {
    glfwMakeContextCurrent(gl_context_.window);
    return true;
  }
  if (gl_context_.backend == OffscreenGlBackend::Egl &&
      gl_context_.egl_display != EGL_NO_DISPLAY &&
      gl_context_.egl_context != EGL_NO_CONTEXT &&
      gl_context_.egl_surface != EGL_NO_SURFACE) {
    if (eglMakeCurrent(static_cast<::EGLDisplay>(gl_context_.egl_display),
                       static_cast<::EGLSurface>(gl_context_.egl_surface),
                       static_cast<::EGLSurface>(gl_context_.egl_surface),
                       static_cast<::EGLContext>(gl_context_.egl_context))) {
      return true;
    }
    LOG_ERROR << "failed to activate EGL context.";
    return false;
  }

  LOG_ERROR << "offscreen OpenGL context is not available.";
  return false;
}

bool CameraRenderer::resize_offscreen_buffer(int width, int height) {
  std::size_t pixel_count = 0;
  if (!pixel_count_for(width, height, pixel_count)) {
    LOG_ERROR << "offscreen image dimensions must be positive and not overflow "
                 "pixel capacity.";
    return false;
  }
  UNUSED(pixel_count);
  if (width <= offscreen_width_ && height <= offscreen_height_) {
    return true;
  }
  const int new_width = std::max(offscreen_width_, width);
  const int new_height = std::max(offscreen_height_, height);
  mjr_resizeOffscreen(new_width, new_height, &render_context_);
  mjr_setBuffer(mjFB_OFFSCREEN, &render_context_);
  offscreen_width_ = new_width;
  offscreen_height_ = new_height;
  return true;
}

bool CameraRenderer::create_render_resources(const mjContext &context) {
  render_data_ = mj_makeData(context.model);
  if (render_data_ == nullptr) {
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

  mjv_makeScene(context.model, &scene_, config_.max_scene_geometries);
  mjr_makeContext(context.model, &render_context_, kFontScale);
  mjr_setBuffer(mjFB_OFFSCREEN, &render_context_);
  initialized_ = true;
  return true;
}

CameraRenderIntrinsics
CameraRenderer::compute_intrinsics(double fovy_degrees, std::uint32_t width,
                                   std::uint32_t height) const {
  CameraRenderIntrinsics intrinsics;
  if (width == 0 || height == 0) {
    return intrinsics;
  }

  const double aspect =
      static_cast<double>(width) / static_cast<double>(height);
  const double fovy_radians = fovy_degrees * kPi / 180.0;
  intrinsics.fy =
      static_cast<double>(height) / (2.0 * std::tan(fovy_radians / 2.0));
  const double fovx_radians =
      2.0 * std::atan(aspect * std::tan(fovy_radians / 2.0));
  intrinsics.fx =
      static_cast<double>(width) / (2.0 * std::tan(fovx_radians / 2.0));
  intrinsics.cx = (static_cast<double>(width) - 1.0) / 2.0;
  intrinsics.cy = (static_cast<double>(height) - 1.0) / 2.0;
  intrinsics.k = {intrinsics.fx, 0.0, intrinsics.cx, 0.0, intrinsics.fy,
                  intrinsics.cy, 0.0, 0.0,           1.0};
  intrinsics.p = {intrinsics.fx, 0.0, intrinsics.cx, 0.0, 0.0, intrinsics.fy,
                  intrinsics.cy, 0.0, 0.0,           0.0, 1.0, 0.0};
  return intrinsics;
}

void CameraRenderer::transform_rgb(const std::vector<std::uint8_t> &source,
                                   std::uint32_t width, std::uint32_t height,
                                   std::vector<std::uint8_t> &dest) const {
  const std::size_t row_size =
      static_cast<std::size_t>(width) * kColorChannelCount;
  for (std::uint32_t row = 0; row < height; ++row) {
    const std::size_t src_offset = static_cast<std::size_t>(row) * row_size;
    const std::size_t dst_offset =
        static_cast<std::size_t>(height - 1U - row) * row_size;
    std::memcpy(dest.data() + dst_offset, source.data() + src_offset, row_size);
  }
}

bool CameraRenderer::transform_depth(const mjContext &context,
                                     const std::vector<float> &source,
                                     std::uint32_t width, std::uint32_t height,
                                     std::vector<float> &dest) const {
  const float near = static_cast<float>(context.model->vis.map.znear *
                                        context.model->stat.extent);
  const float far = static_cast<float>(context.model->vis.map.zfar *
                                       context.model->stat.extent);
  if (!std::isfinite(near) || !std::isfinite(far) || near <= 0.0F ||
      far <= near) {
    LOG_ERROR << "MuJoCo camera depth range must satisfy 0 < near < far.";
    return false;
  }
  if (source.size() != dest.size() ||
      source.size() != static_cast<std::size_t>(width) * height) {
    LOG_ERROR << "camera depth buffer has an unexpected size.";
    return false;
  }

  const float depth_scale = 1.0F - near / far;
  for (std::uint32_t row = 0; row < height; ++row) {
    for (std::uint32_t column = 0; column < width; ++column) {
      const std::size_t src_index =
          static_cast<std::size_t>(row) * width + column;
      const std::size_t dst_index =
          static_cast<std::size_t>(height - 1U - row) * width + column;
      const float denominator = 1.0F - source[src_index] * depth_scale;
      if (!std::isfinite(denominator) || denominator <= 0.0F) {
        LOG_ERROR << "camera depth buffer contains an invalid normalized depth "
                     "value.";
        return false;
      }
      dest[dst_index] = near / denominator;
    }
  }
  return true;
}

bool CameraRenderer::create_egl_context() {
  gl_context_.egl_display = eglGetPlatformDisplay(EGL_PLATFORM_SURFACELESS_MESA,
                                                  EGL_DEFAULT_DISPLAY, nullptr);
  if (gl_context_.egl_display == EGL_NO_DISPLAY) {
    gl_context_.egl_display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
  }
  if (gl_context_.egl_display == EGL_NO_DISPLAY) {
    LOG_ERROR << "failed to acquire EGL display.";
    return false;
  }

  EGLint major = 0;
  EGLint minor = 0;
  if (!eglInitialize(static_cast<::EGLDisplay>(gl_context_.egl_display), &major,
                     &minor)) {
    LOG_ERROR << "failed to initialize EGL.";
    destroy_egl_context();
    return false;
  }

  const EGLint config_attribs[] = {
      EGL_SURFACE_TYPE,    EGL_PBUFFER_BIT, EGL_RED_SIZE,   kEglColorBits,
      EGL_GREEN_SIZE,      kEglColorBits,   EGL_BLUE_SIZE,  kEglColorBits,
      EGL_ALPHA_SIZE,      kEglColorBits,   EGL_DEPTH_SIZE, kEglDepthBits,
      EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,  EGL_NONE};
  EGLConfig egl_config = nullptr;
  EGLint num_configs = 0;
  if (!eglChooseConfig(static_cast<::EGLDisplay>(gl_context_.egl_display),
                       config_attribs, &egl_config, 1, &num_configs) ||
      num_configs == 0) {
    LOG_ERROR << "failed to choose EGL config.";
    destroy_egl_context();
    return false;
  }
  if (!eglBindAPI(EGL_OPENGL_API)) {
    LOG_ERROR << "failed to bind EGL OpenGL API.";
    destroy_egl_context();
    return false;
  }

  gl_context_.egl_context =
      eglCreateContext(static_cast<::EGLDisplay>(gl_context_.egl_display),
                       egl_config, EGL_NO_CONTEXT, nullptr);
  if (gl_context_.egl_context == EGL_NO_CONTEXT) {
    LOG_ERROR << "failed to create EGL context.";
    destroy_egl_context();
    return false;
  }

  const EGLint pbuffer_attribs[] = {EGL_WIDTH, kHiddenContextWidth, EGL_HEIGHT,
                                    kHiddenContextHeight, EGL_NONE};
  gl_context_.egl_surface = eglCreatePbufferSurface(
      static_cast<::EGLDisplay>(gl_context_.egl_display), egl_config,
      pbuffer_attribs);
  if (gl_context_.egl_surface == EGL_NO_SURFACE) {
    LOG_ERROR << "failed to create EGL pbuffer surface.";
    destroy_egl_context();
    return false;
  }

  gl_context_.backend = OffscreenGlBackend::Egl;
  return true;
}

void CameraRenderer::deactivate_context() {
  if (gl_context_.backend == OffscreenGlBackend::Glfw &&
      gl_context_.window != nullptr) {
    glfwMakeContextCurrent(nullptr);
  } else if (gl_context_.backend == OffscreenGlBackend::Egl &&
             gl_context_.egl_display != EGL_NO_DISPLAY) {
    UNUSED(eglMakeCurrent(static_cast<::EGLDisplay>(gl_context_.egl_display),
                          EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT));
  }
}

void CameraRenderer::destroy_context() {
  destroy_glfw_context();
  destroy_egl_context();
  gl_context_ = OffscreenGlContext{};
}

void CameraRenderer::destroy_render_resources() {
  if (initialized_) {
    mjv_freeScene(&scene_);
    mjr_freeContext(&render_context_);
    initialized_ = false;
  }
  if (render_data_ != nullptr) {
    mj_deleteData(render_data_);
    render_data_ = nullptr;
  }
  mjv_defaultScene(&scene_);
  mjv_defaultOption(&option_);
  mjr_defaultContext(&render_context_);
}

void CameraRenderer::destroy_glfw_context() {
  if (gl_context_.window != nullptr) {
    glfwDestroyWindow(gl_context_.window);
    gl_context_.window = nullptr;
  }
}

void CameraRenderer::destroy_egl_context() {
  if (gl_context_.egl_display == EGL_NO_DISPLAY) {
    return;
  }

  const auto display = static_cast<::EGLDisplay>(gl_context_.egl_display);
  UNUSED(
      eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT));
  if (gl_context_.egl_surface != EGL_NO_SURFACE) {
    UNUSED(eglDestroySurface(
        display, static_cast<::EGLSurface>(gl_context_.egl_surface)));
    gl_context_.egl_surface = EGL_NO_SURFACE;
  }
  if (gl_context_.egl_context != EGL_NO_CONTEXT) {
    UNUSED(eglDestroyContext(
        display, static_cast<::EGLContext>(gl_context_.egl_context)));
    gl_context_.egl_context = EGL_NO_CONTEXT;
  }
  UNUSED(eglTerminate(display));
  gl_context_.egl_display = EGL_NO_DISPLAY;
}

} // namespace mujoco_simulation
