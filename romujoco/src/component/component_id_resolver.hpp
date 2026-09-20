#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "romujoco/config/simulation_config.hpp"

namespace romujoco {
class ComponentIdResolver {
public:
    using Indices = std::vector<std::size_t>;
    static constexpr std::size_t no_index = std::numeric_limits<std::size_t>::max();
    static std::shared_ptr<const ComponentIdResolver> create(
        const ComponentConfigList& components) {
        auto result = std::make_shared<ComponentIdResolver>();
        std::vector<std::size_t> joints, grippers, mobile_bases, imus, cameras, laser_scans,
            point_clouds;
        for (const ComponentConfig& component : components) {
            if (const auto* value = std::get_if<JointInfo>(&component)) joints.push_back(value->id);
            if (const auto* value = std::get_if<GripperInfo>(&component))
                grippers.push_back(value->id);
            if (const auto* value = std::get_if<MecanumMobileBaseInfo>(&component))
                mobile_bases.push_back(value->common.id);
            if (const auto* value = std::get_if<SwerveMobileBaseInfo>(&component))
                mobile_bases.push_back(value->common.id);
            if (const auto* value = std::get_if<ImuInfo>(&component)) imus.push_back(value->id);
            if (const auto* value = std::get_if<CameraConfig>(&component))
                cameras.push_back(value->id);
            if (const auto* value = std::get_if<LidarInfo>(&component))
                (value->output == LidarOutput::LaserScan ? laser_scans : point_clouds)
                    .push_back(value->id);
        }
        if (!make(joints, result->joints_) || !make(grippers, result->grippers_) ||
            !make(mobile_bases, result->mobile_bases_) || !make(imus, result->imus_) ||
            !make(cameras, result->cameras_) || !make(laser_scans, result->laser_scans_) ||
            !make(point_clouds, result->point_clouds_))
            return {};
        return result;
    }
    const Indices& joints() const noexcept { return joints_; }
    const Indices& grippers() const noexcept { return grippers_; }
    const Indices& mobile_bases() const noexcept { return mobile_bases_; }
    const Indices& imus() const noexcept { return imus_; }
    const Indices& cameras() const noexcept { return cameras_; }
    const Indices& laser_scans() const noexcept { return laser_scans_; }
    const Indices& point_clouds() const noexcept { return point_clouds_; }

private:
    static bool make(std::vector<std::size_t>& ids, Indices& indices) {
        std::sort(ids.begin(), ids.end());
        if (!ids.empty() &&
            (ids.back() > 255U || std::adjacent_find(ids.begin(), ids.end()) != ids.end()))
            return false;
        indices.assign(ids.empty() ? 0U : ids.back() + 1U, no_index);
        for (std::size_t index = 0; index < ids.size(); ++index) indices[ids[index]] = index;
        return true;
    }
    Indices joints_, grippers_, mobile_bases_, imus_, cameras_, laser_scans_, point_clouds_;
};
}  // namespace romujoco
