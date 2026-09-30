#pragma once

#include <cstdint>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "romujoco/component/lidar.hpp"

#include "component/component.hpp"

namespace romujoco {

class LidarComponent : public SimulationComponent {
public:
    explicit LidarComponent(LidarInfo info);
    ~LidarComponent() override;

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
    void scan(const mjModel* model, mjData* data, std::uint64_t sequence,
              std::shared_ptr<const LaserScanState>& laser,
              std::shared_ptr<const PointCloudState>& cloud);
    void run_worker();
    void stop_worker();

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
    const mjModel* model_{nullptr};
    std::unique_ptr<mjData, MjDataDeleter> worker_data_;
    std::vector<mjtNum> job_state_;
    std::thread worker_;
    std::mutex worker_mutex_;
    std::condition_variable worker_ready_;
    bool worker_stop_{false};
    bool worker_busy_{false};
    bool job_pending_{false};
    std::uint64_t job_sequence_{0};
    std::shared_ptr<const LaserScanState> completed_laser_;
    std::shared_ptr<const PointCloudState> completed_cloud_;
};

}  // namespace romujoco
