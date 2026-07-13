#include <array>
#include <vector>

namespace mujoco_simulation {

using Vector3d = std::array<double, 3>;
using Vector4d = std::array<double, 4>;
using Vector9d = std::array<double, 9>;
using Vector12d = std::array<double, 12>;
using VectorXd = std::vector<double>;

using Quaterniond = Vector4d;

}  // namespace mujoco_simulation
