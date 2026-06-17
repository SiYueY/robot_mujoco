#include "mujoco_simulation/component/camera/camera_renderer.hpp"

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <mutex>
#include <utility>

namespace mujoco_simulation {
namespace {

bool ensure_glfw_initialized() {
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

}  // namespace

CameraRenderer::CameraRenderer() : CameraRenderer(CameraRendererConfig{}) {}

CameraRenderer::CameraRenderer(CameraRendererConfig config) : config_(config) {}

CameraRenderer::~CameraRenderer() { (void)shutdown(); }

Status CameraRenderer::initialize(const mjModel& model) {
  if (initialized_) {
    return Status::Ok();
  }

  Status status = ensure_gl_context();
  if (!status.ok()) {
    return status;
  }

  render_data_ = mj_makeData(&model);
  if (render_data_ == nullptr) {
    release_current_context();
    return Status::render_failed(
        "CameraRenderer failed to allocate MuJoCo render mjData for offscreen rendering.");
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
  return Status::Ok();
}

Status CameraRenderer::shutdown() {
  if (gl_context_.backend == OffscreenGlBackend::Glfw && gl_context_.window != nullptr) {
    glfwMakeContextCurrent(gl_context_.window);
  } else if (gl_context_.backend == OffscreenGlBackend::Egl &&
             gl_context_.egl_display != EGL_NO_DISPLAY &&
             gl_context_.egl_context != EGL_NO_CONTEXT &&
             gl_context_.egl_surface != EGL_NO_SURFACE) {
    (void)eglMakeCurrent(static_cast<::EGLDisplay>(gl_context_.egl_display),
                         static_cast<::EGLSurface>(gl_context_.egl_surface),
                         static_cast<::EGLSurface>(gl_context_.egl_surface),
                         static_cast<::EGLContext>(gl_context_.egl_context));
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
  return Status::Ok();
}

Status CameraRenderer::copy_simulation_data(const mjModel& model, const mjData& source) {
  Status status = initialize(model);
  if (!status.ok()) {
    return status;
  }
  if (gl_context_.backend == OffscreenGlBackend::Glfw && gl_context_.window != nullptr) {
    glfwMakeContextCurrent(gl_context_.window);
  } else if (gl_context_.backend == OffscreenGlBackend::Egl) {
    if (!eglMakeCurrent(static_cast<::EGLDisplay>(gl_context_.egl_display),
                        static_cast<::EGLSurface>(gl_context_.egl_surface),
                        static_cast<::EGLSurface>(gl_context_.egl_surface),
                        static_cast<::EGLContext>(gl_context_.egl_context))) {
      return Status::render_failed(
          "CameraRenderer failed to activate the EGL context before copying simulation data.");
    }
  }
  if (mj_copyData(render_data_, &model, &source) == nullptr) {
    release_current_context();
    return Status::render_failed(
        "CameraRenderer failed to copy MuJoCo simulation data into the render context.");
  }
  release_current_context();
  return Status::Ok();
}

Result<std::shared_ptr<const CameraSample>> CameraRenderer::render(const mjModel& model,
                                                                   const CameraConfig& spec,
                                                                   std::uint64_t sequence,
                                                                   std::uint64_t timestamp_ns) {
  Status status = initialize(model);
  if (!status.ok()) {
    return status;
  }

  int camera_id = -1;
  double fovy_degrees = 0.0;
  status = ensure_camera_binding(model, spec, &camera_id, &fovy_degrees);
  if (!status.ok()) {
    return status;
  }
  status = ensure_offscreen_capacity(spec.width, spec.height);
  if (!status.ok()) {
    return status;
  }

  if (gl_context_.backend == OffscreenGlBackend::Glfw && gl_context_.window != nullptr) {
    glfwMakeContextCurrent(gl_context_.window);
  } else if (gl_context_.backend == OffscreenGlBackend::Egl) {
    if (!eglMakeCurrent(static_cast<::EGLDisplay>(gl_context_.egl_display),
                        static_cast<::EGLSurface>(gl_context_.egl_surface),
                        static_cast<::EGLSurface>(gl_context_.egl_surface),
                        static_cast<::EGLContext>(gl_context_.egl_context))) {
      return Status::render_failed(
          "CameraRenderer failed to activate the EGL context before rendering camera '" +
          spec.common.name + "'.");
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

  auto sample = std::make_shared<CameraSample>();
  sample->sequence = sequence;
  sample->timestamp_ns = timestamp_ns;
  sample->frame_id = spec.common.frame_id.empty() ? spec.common.name : spec.common.frame_id;
  sample->optical_frame_id =
      spec.optical_frame_id.empty() ? sample->frame_id : spec.optical_frame_id;
  sample->intrinsics = compute_intrinsics(fovy_degrees, static_cast<std::uint32_t>(spec.width),
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
    sample->color = ImageBuffer{};
    sample->color->width = static_cast<std::uint32_t>(spec.width);
    sample->color->height = static_cast<std::uint32_t>(spec.height);
    sample->color->step = static_cast<std::uint32_t>(spec.width * 3);
    sample->color->format = PixelFormat::Rgb8;
    sample->color->data.resize(rgb_buffer.size());
    flip_rgb(rgb_buffer, sample->color->width, sample->color->height, &sample->color->data);
  }

  if (spec.enable_depth) {
    sample->depth = DepthBuffer{};
    sample->depth->width = static_cast<std::uint32_t>(spec.width);
    sample->depth->height = static_cast<std::uint32_t>(spec.height);
    sample->depth->format = DepthFormat::Float32Meters;
    sample->depth->data.resize(depth_buffer.size());
    convert_and_flip_depth(model, depth_buffer, sample->depth->width, sample->depth->height,
                           &sample->depth->data);
  }

  return std::static_pointer_cast<const CameraSample>(sample);
}

Status CameraRenderer::ensure_gl_context() {
  if (gl_context_.backend == OffscreenGlBackend::Glfw && gl_context_.window != nullptr) {
    glfwMakeContextCurrent(gl_context_.window);
    return Status::Ok();
  }
  if (gl_context_.backend == OffscreenGlBackend::Egl && gl_context_.egl_display != EGL_NO_DISPLAY &&
      gl_context_.egl_context != EGL_NO_CONTEXT && gl_context_.egl_surface != EGL_NO_SURFACE) {
    if (!eglMakeCurrent(static_cast<::EGLDisplay>(gl_context_.egl_display),
                        static_cast<::EGLSurface>(gl_context_.egl_surface),
                        static_cast<::EGLSurface>(gl_context_.egl_surface),
                        static_cast<::EGLContext>(gl_context_.egl_context))) {
      return Status::render_failed("CameraRenderer failed to reactivate the existing EGL context.");
    }
    return Status::Ok();
  }

  if (config_.allow_glfw_backend && ensure_glfw_initialized()) {
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
    gl_context_.window = glfwCreateWindow(1, 1, "", nullptr, nullptr);
    if (gl_context_.window != nullptr) {
      gl_context_.backend = OffscreenGlBackend::Glfw;
      glfwMakeContextCurrent(gl_context_.window);
      return Status::Ok();
    }
  }

  if (!config_.allow_egl_backend) {
    return Status::render_failed(
        "CameraRenderer could not create a hidden GLFW context and EGL fallback is disabled.");
  }
  return initialize_egl_context();
}

Status CameraRenderer::ensure_offscreen_capacity(int width, int height) {
  if (width <= 0 || height <= 0) {
    return Status::invalid_argument("Camera offscreen viewport must be positive.");
  }
  if (width <= offscreen_width_ && height <= offscreen_height_) {
    return Status::Ok();
  }

  if (gl_context_.backend == OffscreenGlBackend::Glfw && gl_context_.window != nullptr) {
    glfwMakeContextCurrent(gl_context_.window);
  } else if (gl_context_.backend == OffscreenGlBackend::Egl) {
    if (!eglMakeCurrent(static_cast<::EGLDisplay>(gl_context_.egl_display),
                        static_cast<::EGLSurface>(gl_context_.egl_surface),
                        static_cast<::EGLSurface>(gl_context_.egl_surface),
                        static_cast<::EGLContext>(gl_context_.egl_context))) {
      return Status::render_failed(
          "CameraRenderer failed to activate the EGL context before resizing the offscreen "
          "buffer.");
    }
  }
  offscreen_width_ = std::max(offscreen_width_, width);
  offscreen_height_ = std::max(offscreen_height_, height);
  mjr_resizeOffscreen(offscreen_width_, offscreen_height_, &render_context_);
  mjr_setBuffer(mjFB_OFFSCREEN, &render_context_);
  release_current_context();
  return Status::Ok();
}

Status CameraRenderer::ensure_camera_binding(const mjModel& model, const CameraConfig& spec,
                                             int* camera_id, double* fovy_degrees) {
  if (camera_id == nullptr || fovy_degrees == nullptr) {
    return Status::invalid_argument("Camera binding output pointers must not be null.");
  }
  if (spec.common.name.empty()) {
    return Status::invalid_argument("Camera name must not be empty.");
  }
  if (spec.camera_name.empty()) {
    return Status::invalid_argument("MuJoCo camera name must not be empty for camera '" +
                                    spec.common.name + "'.");
  }
  if (spec.width <= 0 || spec.height <= 0) {
    return Status::invalid_argument("Camera '" + spec.common.name +
                                    "' requires positive width and height.");
  }
  if (!spec.enable_rgb && !spec.enable_depth) {
    return Status::invalid_argument("Camera '" + spec.common.name +
                                    "' must enable rgb or depth output.");
  }

  const int id = mj_name2id(&model, mjOBJ_CAMERA, spec.camera_name.c_str());
  if (id < 0) {
    return Status::binding_failed("Camera '" + spec.common.name +
                                  "' could not bind to MuJoCo camera '" + spec.camera_name + "'.");
  }

  *camera_id = id;
  *fovy_degrees = static_cast<double>(model.cam_fovy[id]);
  return Status::Ok();
}

CameraIntrinsics CameraRenderer::compute_intrinsics(double fovy_degrees, std::uint32_t width,
                                                    std::uint32_t height) const {
  CameraIntrinsics intrinsics;
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
                              std::uint32_t height, std::vector<std::uint8_t>* dest) const {
  if (dest == nullptr) {
    return;
  }
  const std::size_t row_size = static_cast<std::size_t>(width) * 3U;
  for (std::uint32_t row = 0; row < height; ++row) {
    const std::size_t src_offset = static_cast<std::size_t>(row) * row_size;
    const std::size_t dst_offset = static_cast<std::size_t>(height - 1U - row) * row_size;
    std::memcpy(dest->data() + dst_offset, source.data() + src_offset, row_size);
  }
}

void CameraRenderer::convert_and_flip_depth(const mjModel& model, const std::vector<float>& source,
                                            std::uint32_t width, std::uint32_t height,
                                            std::vector<float>* dest) const {
  if (dest == nullptr) {
    return;
  }

  const float near = static_cast<float>(model.vis.map.znear * model.stat.extent);
  const float far = static_cast<float>(model.vis.map.zfar * model.stat.extent);
  const float depth_scale = 1.0F - near / far;

  for (std::uint32_t row = 0; row < height; ++row) {
    for (std::uint32_t column = 0; column < width; ++column) {
      const std::size_t src_index = static_cast<std::size_t>(row) * width + column;
      const std::size_t dst_index = static_cast<std::size_t>(height - 1U - row) * width + column;
      const float normalized = source[src_index];
      (*dest)[dst_index] = near / (1.0F - normalized * depth_scale);
    }
  }
}

Status CameraRenderer::initialize_egl_context() {
  gl_context_.egl_display =
      eglGetPlatformDisplay(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr);
  if (gl_context_.egl_display == EGL_NO_DISPLAY) {
    gl_context_.egl_display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
  }
  if (gl_context_.egl_display == EGL_NO_DISPLAY) {
    return Status::render_failed("CameraRenderer failed to acquire an EGL display.");
  }

  EGLint major = 0;
  EGLint minor = 0;
  if (!eglInitialize(static_cast<::EGLDisplay>(gl_context_.egl_display), &major, &minor)) {
    gl_context_.egl_display = EGL_NO_DISPLAY;
    return Status::render_failed("CameraRenderer failed to initialize EGL.");
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
    return Status::render_failed("CameraRenderer failed to choose a compatible EGL config.");
  }

  if (!eglBindAPI(EGL_OPENGL_API)) {
    eglTerminate(static_cast<::EGLDisplay>(gl_context_.egl_display));
    gl_context_.egl_display = EGL_NO_DISPLAY;
    return Status::render_failed("CameraRenderer failed to bind the EGL OpenGL API.");
  }

  gl_context_.egl_context = eglCreateContext(static_cast<::EGLDisplay>(gl_context_.egl_display),
                                             egl_config, EGL_NO_CONTEXT, nullptr);
  if (gl_context_.egl_context == EGL_NO_CONTEXT) {
    eglTerminate(static_cast<::EGLDisplay>(gl_context_.egl_display));
    gl_context_.egl_display = EGL_NO_DISPLAY;
    return Status::render_failed("CameraRenderer failed to create an EGL context.");
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
    return Status::render_failed("CameraRenderer failed to create an EGL pbuffer surface.");
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
    return Status::render_failed("CameraRenderer failed to activate the EGL context.");
  }

  gl_context_.backend = OffscreenGlBackend::Egl;
  return Status::Ok();
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
