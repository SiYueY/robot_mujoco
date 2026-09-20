#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "romujoco/component/lidar.hpp"

#include "component/component.hpp"

namespace romujoco {

class LidarComponent : public SimulationComponent {
public:
    explicit LidarComponent(LidarInfo info);

    bool init(const SimulationContext& context) override;
    bool reset(const SimulationContext& context) override;
    bool advance(const SimulationContext& context) override;
    bool update(const SimulationContext& context) override;

    bool read_laser_scan_state(std::shared_ptr<const LaserScanState>& state) const;
    bool read_point_cloud_state(std::shared_ptr<const PointCloudState>& state) const;

    const LidarInfo& info() const noexcept { return info_; }
    bool is_initialized() const noexcept { return initialized_; }

public:
    using SharedPtr = std::shared_ptr<LidarComponent>;
    using UniquePtr = std::unique_ptr<LidarComponent>;
    using WeakPtr = std::weak_ptr<LidarComponent>;

private:
    LidarInfo info_;
    int site_id_{-1};
    int body_id_{-1};
    std::vector<mjtNum> local_directions_;
    std::vector<mjtNum> world_directions_;
    std::vector<mjtNum> distances_;
    std::vector<mjtByte> geom_groups_;
    std::shared_ptr<const LaserScanState> laser_scan_state_;
    std::shared_ptr<const PointCloudState> point_cloud_state_;
    std::uint64_t sequence_{0};
    bool initialized_{false};
};

}  // namespace romujoco
