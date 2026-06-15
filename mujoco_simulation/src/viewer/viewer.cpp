#include "mujoco_simulation/viewer/viewer.hpp"

#include <cstring>
#include <exception>

#include "glfw_adapter.h"
#include "simulate.h"

namespace mujoco_simulation {
namespace {

void delete_simulate(mujoco::Simulate* simulate) { delete simulate; }

}  // namespace

Viewer::Viewer() : simulate_(nullptr, delete_simulate) {}

Viewer::~Viewer() { stop(); }

bool Viewer::start(mjModel* model, mjData* data, const std::string& displayed_filename,
                   std::string& error_message) {
  if (model == nullptr || data == nullptr) {
    error_message = "Viewer requires non-null MuJoCo model and data.";
    return false;
  }
  if (render_thread_.joinable()) {
    error_message.clear();
    return true;
  }

  try {
    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&visual_options_);
    mjv_defaultPerturb(&perturb_);

    render_thread_ = std::thread([this, model, data, displayed_filename]() {
      try {
        auto simulate =
            SimulateHandle(new mujoco::Simulate(std::make_unique<mujoco::GlfwAdapter>(), &camera_,
                                                &visual_options_, &perturb_, true),
                           delete_simulate);
        simulate->exitrequest.store(false);

        {
          const ::mujoco::MutexLock simulate_lock(simulate->mtx);
          simulate->mnew_ = model;
          simulate->dnew_ = data;
          std::strncpy(simulate->filename, displayed_filename.c_str(),
                       sizeof(simulate->filename) - 1);
          simulate->filename[sizeof(simulate->filename) - 1] = '\0';
          simulate->loadrequest = 1;
        }

        {
          std::lock_guard<std::mutex> lock(mutex_);
          simulate_ = std::move(simulate);
        }

        simulate_->RenderLoop();
      } catch (const std::exception& exc) {
        (void)exc;
      }
    });
  } catch (const std::exception& exc) {
    if (render_thread_.joinable()) {
      render_thread_.join();
    }
    error_message = std::string("Failed to start MuJoCo viewer: ") + exc.what();
    return false;
  }

  error_message.clear();
  return true;
}

void Viewer::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (simulate_ != nullptr) {
      simulate_->exitrequest.store(true);
    }
  }
  if (render_thread_.joinable()) {
    render_thread_.join();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  simulate_.reset();
}

bool Viewer::sync(bool state_only, std::string& error_message) {
  std::lock_guard<std::mutex> state_lock(mutex_);
  if (simulate_ == nullptr) {
    error_message = "Viewer is not initialized.";
    return false;
  }

  try {
    std::unique_lock<std::recursive_mutex> simulate_lock(simulate_->mtx);
    if (simulate_->exitrequest.load()) {
      error_message = "Viewer is stopping.";
      return false;
    }
    simulate_->Sync(state_only);
  } catch (const std::exception& exc) {
    error_message = std::string("Failed to sync MuJoCo viewer: ") + exc.what();
    return false;
  }

  error_message.clear();
  return true;
}

bool Viewer::is_running() const { return render_thread_.joinable(); }

mjvScene* Viewer::scene() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (simulate_ == nullptr) {
    return nullptr;
  }
  return &simulate_->scn;
}

mjrContext* Viewer::render_context() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (simulate_ == nullptr || simulate_->platform_ui == nullptr) {
    return nullptr;
  }
  return &simulate_->platform_ui->mjr_context();
}

}  // namespace mujoco_simulation
