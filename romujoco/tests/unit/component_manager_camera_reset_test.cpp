#include <mujoco/mujoco.h>

#include <iostream>
#include <map>

#include "component/component_manager.hpp"
#include "test_support.hpp"

namespace {

bool check(bool value, const char* message) {
    if (!value) {
        std::cerr << message << '\n';
    }
    return value;
}

class RetainingCameraRenderService final : public romujoco::CameraRenderService {
public:
    explicit RetainingCameraRenderService(bool replace_pending_batch)
    : replace_pending_batch_(replace_pending_batch) {}

    bool initialize(const romujoco::SimulationConfig&, const mjModel*) override {
        return true;
    }
    romujoco::CameraRenderSubmitResult submit(
        const romujoco::CameraRenderBatchRequest& request,
        romujoco::CameraRenderTicket& ticket) override {
        ticket = {request.generation, request.sequence};
        latest_ticket_ = ticket;
        return replace_pending_batch_
                   ? romujoco::CameraRenderSubmitResult::ReplacedPendingBatch
                   : romujoco::CameraRenderSubmitResult::Accepted;
    }
    romujoco::CameraRenderWaitStatus wait(
        const romujoco::CameraRenderTicket& ticket, std::chrono::milliseconds) override {
        return query(ticket);
    }
    romujoco::CameraRenderWaitStatus query(
        const romujoco::CameraRenderTicket& ticket) const override {
        return completed_.count(key(ticket)) != 0U
                   ? romujoco::CameraRenderWaitStatus::Completed
                   : romujoco::CameraRenderWaitStatus::Timeout;
    }
    bool read_batch_result(
        const romujoco::CameraRenderTicket& ticket,
        romujoco::CameraRenderBatchResult& result) override {
        const auto found = completed_.find(key(ticket));
        if (found == completed_.end()) return false;
        result = found->second;
        ++read_count_;
        return true;
    }
    bool reset() override { return true; }
    bool shutdown() override { return true; }

    void complete_latest(romujoco::CameraId id, std::uint64_t sequence) {
        romujoco::CameraRenderBatchResult result;
        result.ticket = latest_ticket_;
        result.status = romujoco::CameraBatchStatus::Completed;
        result.all_succeeded = true;
        result.cameras.push_back(
            {id,
             romujoco::CameraTaskStatus::Completed,
             {},
             latest_ticket_.generation,
             latest_ticket_.sequence,
             0,
             0.0,
             sequence,
             0,
             ""});
        completed_[key(latest_ticket_)] = std::move(result);
    }
    std::size_t read_count() const noexcept { return read_count_; }

private:
    static std::pair<std::uint64_t, std::uint64_t> key(
        const romujoco::CameraRenderTicket& ticket) {
        return {ticket.generation, ticket.sequence};
    }

    romujoco::CameraRenderTicket latest_ticket_{};
    std::map<std::pair<std::uint64_t, std::uint64_t>, romujoco::CameraRenderBatchResult>
        completed_;
    std::size_t read_count_{0};
    bool replace_pending_batch_{false};
};

}  // namespace

int main() {
    romujoco_test::TemporaryFile model_file(
        "mujoco_component_manager_camera_reset_test.xml");
    if (!check(
            model_file.write(R"(<mujoco><worldbody>
  <camera name="test_camera" pos="0 -2 0.5" xyaxes="1 0 0 0 0 1"/>
</worldbody></mujoco>)"),
            "failed to write MuJoCo model")) {
        return 1;
    }
    char error[1024] = {};
    mjModel* model = mj_loadXML(model_file.path().c_str(), nullptr, error, sizeof(error));
    if (!check(model != nullptr, error)) return 1;
    mjData* data = mj_makeData(model);
    if (!check(data != nullptr, "failed to allocate MuJoCo data")) {
        mj_deleteModel(model);
        return 1;
    }
    mj_forward(model, data);
    romujoco::mjContext context(model, data);

    romujoco::CameraConfig camera;
    camera.id = 2;
    camera.name = "camera";
    camera.frame_id = "camera_frame";
    camera.camera_name = "test_camera";
    camera.optical_frame_id = "camera_optical";
    camera.width = 32;
    camera.height = 24;
    camera.period = 0.01;
    romujoco::ComponentConfigList components{camera};
    // ComponentManager must retain a ticket when a submission replaces a
    // pending batch; this is a successful latest-only submission, not an error.
    RetainingCameraRenderService service(true);
    romujoco::ComponentManager manager;
    if (!check(
            manager.init(context, components, service) && manager.update(context),
            "failed to submit initial camera batch")) {
        context.clear();
        return 1;
    }
    service.complete_latest(camera.id, 1);
    if (!check(manager.update(context), "failed to consume initial camera batch")) {
        context.clear();
        return 1;
    }
    romujoco::RobotState before_reset;
    if (!check(
            manager.read_state(context, before_reset) && before_reset.cameras != nullptr &&
                before_reset.cameras->size() == 1U && (*before_reset.cameras)[0]->id == camera.id,
            "initial camera state was not published")) {
        context.clear();
        return 1;
    }
    const std::size_t reads_before_reset = service.read_count();
    if (!check(
            manager.reset(context) && manager.update(context),
            "failed to reset component manager")) {
        context.clear();
        return 1;
    }
    romujoco::RobotState after_reset;
    const bool no_old_result_consumed = service.read_count() == reads_before_reset;
    const bool no_old_camera_published =
        manager.read_state(context, after_reset) && after_reset.cameras == nullptr;
    context.clear();
    return no_old_result_consumed && no_old_camera_published ? 0 : 1;
}
