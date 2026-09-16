#include <atomic>
#include <cstdint>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "buffer/state_buffer.hpp"
#include "component/component_id_resolver.hpp"

namespace {
bool check(bool value, const char* message) {
    if (!value) std::cerr << message << '\n';
    return value;
}

template <typename State>
romujoco::StateSnapshots<State> make_states(std::initializer_list<std::size_t> ids) {
    auto states = std::make_shared<std::vector<romujoco::StateSnapshot<State>>>();
    states->reserve(ids.size());
    for (const std::size_t id : ids) {
        auto state = std::make_shared<State>();
        state->id = id;
        states->push_back(std::move(state));
    }
    return std::static_pointer_cast<const std::vector<romujoco::StateSnapshot<State>>>(
        states);
}

bool check_sparse_indices() {
    romujoco::StateBuffer buffer;
    romujoco::ComponentConfigList components;
    for (const std::size_t id : {0U, 2U, 255U}) {
        romujoco::JointInfo joint;
        joint.id = id;
        components.push_back(std::move(joint));
        romujoco::MecanumMobileBaseInfo mobile_base;
        mobile_base.common.id = id;
        components.push_back(std::move(mobile_base));
        romujoco::ImuInfo imu;
        imu.id = id;
        components.push_back(std::move(imu));
        romujoco::CameraConfig camera;
        camera.id = id;
        components.push_back(std::move(camera));
        romujoco::LidarInfo lidar;
        lidar.id = id;
        components.push_back(std::move(lidar));
    }
    if (!check(
            buffer.configure(romujoco::ComponentIdResolver::create(components)),
            "failed to configure sparse state indices")) {
        return false;
    }
    if (!check(
            !buffer.configure(romujoco::ComponentIdResolver::create(components)),
            "re-configure was accepted")) {
        return false;
    }

    auto snapshot = std::make_shared<romujoco::RobotState>();
    snapshot->joints = make_states<romujoco::JointState>({0, 2, 255});
    snapshot->mobile_bases = make_states<romujoco::MobileBaseState>({0, 2, 255});
    snapshot->imus = make_states<romujoco::ImuState>({0, 2, 255});
    snapshot->cameras = make_states<romujoco::CameraState>({0, 2, 255});
    snapshot->lidars = make_states<romujoco::LidarState>({0, 2, 255});
    if (!check(buffer.write(std::move(snapshot)), "failed to publish sparse state snapshot"))
        return false;

    romujoco::JointState joint;
    joint.id = 255;
    romujoco::MobileBaseState mobile_base;
    mobile_base.id = 255;
    romujoco::ImuState imu;
    imu.id = 255;
    romujoco::CameraState camera;
    camera.id = 255;
    romujoco::LidarState lidar;
    lidar.id = 255;
    if (!check(
            buffer.read(joint) && joint.id == 255 && buffer.read(mobile_base) &&
                mobile_base.id == 255 && buffer.read(imu) && imu.id == 255 && buffer.read(camera) &&
                camera.id == 255 && buffer.read(lidar) && lidar.id == 255,
            "sparse state index did not resolve a configured ID")) {
        return false;
    }

    joint.id = 1;
    if (!check(!buffer.read(joint), "unconfigured state ID was accepted")) return false;
    joint.id = 256;
    if (!check(!buffer.read(joint), "out-of-range state ID was accepted")) return false;

    auto mismatched = std::make_shared<romujoco::RobotState>();
    mismatched->joints = make_states<romujoco::JointState>({2, 0, 255});
    if (!check(!buffer.write(std::move(mismatched)), "mismatched state order was accepted"))
        return false;

    buffer.shutdown();
    joint.id = 0;
    return check(!buffer.read(joint), "cleared state buffer retained an index or snapshot");
}

bool check_plural_reads() {
    romujoco::StateBuffer buffer;
    romujoco::ComponentConfigList components;
    for (const std::size_t id : {0U, 2U, 255U}) {
        romujoco::JointInfo joint;
        joint.id = id;
        components.push_back(std::move(joint));
        romujoco::MecanumMobileBaseInfo mobile_base;
        mobile_base.common.id = id;
        components.push_back(std::move(mobile_base));
        romujoco::ImuInfo imu;
        imu.id = id;
        components.push_back(std::move(imu));
        romujoco::CameraConfig camera;
        camera.id = id;
        components.push_back(std::move(camera));
        romujoco::LidarInfo lidar;
        lidar.id = id;
        components.push_back(std::move(lidar));
    }
    if (!check(
            buffer.configure(romujoco::ComponentIdResolver::create(components)),
            "failed to configure plural-read state indices")) {
        return false;
    }

    const romujoco::JointStates sentinel_joints =
        make_states<romujoco::JointState>({7});
    const romujoco::MobileBaseStates sentinel_mobile_bases =
        make_states<romujoco::MobileBaseState>({7});
    const romujoco::ImuStates sentinel_imus =
        make_states<romujoco::ImuState>({7});
    const romujoco::CameraStates sentinel_cameras =
        make_states<romujoco::CameraState>({7});
    const romujoco::LidarStates sentinel_lidars =
        make_states<romujoco::LidarState>({7});

    romujoco::JointStates joints = sentinel_joints;
    romujoco::MobileBaseStates mobile_bases = sentinel_mobile_bases;
    romujoco::ImuStates imus = sentinel_imus;
    romujoco::CameraStates cameras = sentinel_cameras;
    romujoco::LidarStates lidars = sentinel_lidars;
    if (!check(
            !buffer.read(joints) && joints == sentinel_joints && !buffer.read(mobile_bases) &&
                mobile_bases == sentinel_mobile_bases && !buffer.read(imus) &&
                imus == sentinel_imus && !buffer.read(cameras) && cameras == sentinel_cameras &&
                !buffer.read(lidars) && lidars == sentinel_lidars,
            "plural read succeeded or modified output before a snapshot was published")) {
        return false;
    }

    auto snapshot = std::make_shared<romujoco::RobotState>();
    snapshot->joints = make_states<romujoco::JointState>({0, 2, 255});
    snapshot->mobile_bases = make_states<romujoco::MobileBaseState>({0, 2, 255});
    snapshot->imus = make_states<romujoco::ImuState>({0, 2, 255});
    snapshot->cameras = make_states<romujoco::CameraState>({0, 2, 255});
    snapshot->lidars = make_states<romujoco::LidarState>({0, 2, 255});
    const romujoco::JointStates published_joints = snapshot->joints;
    const romujoco::MobileBaseStates published_mobile_bases = snapshot->mobile_bases;
    const romujoco::ImuStates published_imus = snapshot->imus;
    const romujoco::CameraStates published_cameras = snapshot->cameras;
    const romujoco::LidarStates published_lidars = snapshot->lidars;
    if (!check(buffer.write(std::move(snapshot)), "failed to publish plural-read snapshot")) {
        return false;
    }

    if (!check(
            buffer.read(joints) && joints == published_joints && buffer.read(mobile_bases) &&
                mobile_bases == published_mobile_bases && buffer.read(imus) &&
                imus == published_imus && buffer.read(cameras) && cameras == published_cameras &&
                buffer.read(lidars) && lidars == published_lidars,
            "plural read did not return the published snapshot")) {
        return false;
    }

    buffer.shutdown();
    return check(
        !buffer.read(joints) && !buffer.read(mobile_bases) && !buffer.read(imus) &&
            !buffer.read(cameras) && !buffer.read(lidars),
        "plural read succeeded after shutdown");
}
}  // namespace

int main() {
    if (!check_sparse_indices()) return 1;
    if (!check_plural_reads()) return 1;
    romujoco::StateBuffer buffer;
    if (!check(
            buffer.configure(romujoco::ComponentIdResolver::create({})),
            "failed to configure empty state index")) {
        return 1;
    }
    std::atomic<bool> finished{false};
    std::atomic<bool> valid{true};
    std::thread writer([&] {
        for (std::uint64_t sequence = 1; sequence <= 10000U; ++sequence) {
            auto state = std::make_shared<romujoco::RobotState>();
            state->sequence = sequence;
            state->step = sequence;
            buffer.write(std::move(state));
        }
        finished.store(true);
    });
    std::thread reader([&] {
        std::uint64_t previous = 0;
        while (!finished.load()) {
            const auto state = buffer.read();
            if (state != nullptr && state->sequence < previous) valid.store(false);
            if (state != nullptr) previous = state->sequence;
        }
    });
    writer.join();
    reader.join();
    const auto final_state = buffer.read();
    return check(valid.load(), "state sequence regressed under concurrent access") &&
                   check(
                       final_state != nullptr && final_state->sequence == 10000U,
                       "final state was not published")
               ? 0
               : 1;
}
