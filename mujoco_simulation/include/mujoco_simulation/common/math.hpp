#include <array>
#include <vector>

namespace mujoco_simulation {

// Vector
using Vector3d = std::array<double, 3>;
using Vector4d = std::array<double, 4>;
using Vector9d = std::array<double, 9>;
using Vector12d = std::array<double, 12>;
using VectorXd = std::vector<double>;

// Matrix
using Matrix3d = std::array<std::array<double, 3>, 3>;
using Matrix4d = std::array<std::array<double, 4>, 4>;
using Matrix9d = std::array<std::array<double, 9>, 9>;
using Matrix12d = std::array<std::array<double, 12>, 12>;
using MatrixXd = std::vector<std::vector<double>>;

// Quaternion
using Quaterniond = Vector4d;

}  // namespace mujoco_simulation
