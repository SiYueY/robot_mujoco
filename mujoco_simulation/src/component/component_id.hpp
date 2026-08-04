#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "mujoco_simulation/config/simulation_config.hpp"

namespace mujoco_simulation {

class ComponentIdResolver {
public:
    using Indices = std::vector<std::size_t>;
    static constexpr std::size_t no_index = std::numeric_limits<std::size_t>::max();

    static std::shared_ptr<const ComponentIdResolver> create(
        const ComponentConfigList& components) {
        auto index = std::make_shared<ComponentIdResolver>();
        std::vector<std::size_t> joints, mobile_bases, imus, cameras, lidars;
        for (const ComponentConfig& component : components) {
            if (const auto* value = std::get_if<JointInfo>(&component)) joints.push_back(value->id);
            if (const auto* value = std::get_if<MobileBaseInfo>(&component))
                mobile_bases.push_back(value->id);
            if (const auto* value = std::get_if<ImuInfo>(&component)) imus.push_back(value->id);
            if (const auto* value = std::get_if<CameraConfig>(&component))
                cameras.push_back(value->id);
            if (const auto* value = std::get_if<LidarInfo>(&component)) lidars.push_back(value->id);
        }
        if (!create(joints, index->joints_) || !create(mobile_bases, index->mobile_bases_) ||
            !create(imus, index->imus_) || !create(cameras, index->cameras_) ||
            !create(lidars, index->lidars_))
            return {};
        return index;
    }

    const Indices& joints() const noexcept { return joints_; }
    const Indices& mobile_bases() const noexcept { return mobile_bases_; }
    const Indices& imus() const noexcept { return imus_; }
    const Indices& cameras() const noexcept { return cameras_; }
    const Indices& lidars() const noexcept { return lidars_; }

private:
    static bool create(std::vector<std::size_t>& ids, Indices& indices) {
        std::sort(ids.begin(), ids.end());
        if (!ids.empty() &&
            (ids.back() > 255U || std::adjacent_find(ids.begin(), ids.end()) != ids.end()))
            return false;
        indices.assign(ids.empty() ? 0U : ids.back() + 1U, no_index);
        for (std::size_t position = 0; position < ids.size(); ++position)
            indices[ids[position]] = position;
        return true;
    }

    Indices joints_, mobile_bases_, imus_, cameras_, lidars_;
};

}  // namespace mujoco_simulation
