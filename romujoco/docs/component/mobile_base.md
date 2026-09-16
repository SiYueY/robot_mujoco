# MobileBase

`MobileBase` provides a chassis-independent planar command and simulation
ground-truth state.  It does not provide wheel odometry or ROS messages.

## Public contract

`MobileBaseCommand` contains a `ComponentId` and `PlanarTwist`:

- `linear_x`: base-frame +X velocity
- `linear_y`: base-frame +Y velocity
- `angular_z`: base-frame +Z angular velocity

All values must be finite.  A chassis must reject a non-zero unsupported
component; it must not silently discard it.

`MobileBaseState` contains a timestamp, world-frame `Pose3d`, and base-frame
`Twist3d`.  Quaternions use `{w, x, y, z}` order.

## Configurations

The common configuration is `MobileBaseCommonInfo`.  `base_body_name` must
identify a body with exactly one free joint; the free joint is discovered from
the body rather than configured separately.

- `MecanumMobileBaseInfo` is supported only with `execution_mode = Kinematic`.
  Its implementation writes the authoritative free-joint pose and velocity.
- `SwerveMobileBaseInfo` is supported only with `execution_mode = Dynamic`.
  Every module has unique steering/drive joints and actuators.  Steering uses a
  MuJoCo position actuator and drive uses a velocity actuator.

XML selects the concrete config at parsing time:

```xml
<mobile_base id="20" name="base" type="swerve" execution="dynamic"
             base_body="base_link" period="0.01">
  <module name="front" x="0.3" y="-0.2" radius="0.05"
          steering_joint="front_steer" steering_actuator="front_position"
          drive_joint="front_drive" drive_actuator="front_velocity"/>
</mobile_base>
```

Swerve inverse kinematics chooses the shortest steering path, reverses drive
direction when that path exceeds π/2, and holds steering at zero drive speed.
Dynamic chassis state is read from MuJoCo after stepping; it is never derived
from swerve forward kinematics.
