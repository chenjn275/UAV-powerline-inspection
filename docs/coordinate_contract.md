# Coordinate and time contract

This contract is the interface boundary for the first closed-loop prototype. A later ROS node may publish these values, but it must preserve the same frame and timestamp semantics.

## Frames

```text
map -> odom -> base_link -> lidar_link
                         \-> camera_color_optical_frame
```

- `map` is fixed for one launch and is the frame of the known line model and static obstacles.
- `odom` is the continuous local estimate. For the no-loop-closure baseline, `map` and `odom` may be aligned at startup; drift must remain explicit.
- `base_link` uses ROS FLU conventions. `lidar_link` is the rigid LiDAR frame.
- `camera_color_optical_frame` uses the ROS optical convention: right, down, forward.
- Each transform has exactly one publisher. A map correction must update the task-model adapter; the controller continues in the continuous local frame until the correction is handled.

## Point and image timing

For a LiDAR point measured at time `t`:

```text
p_world(t) = T_world_base(t) * T_base_lidar * p_lidar
```

If a point cloud is already expressed in `map` or `world`, the adapter must not apply the pose transform a second time. Camera projection uses the image acquisition time. Points from another time must be motion-compensated before projection. `ApproximateTime` is a queueing policy, not a clock-synchronization proof.

Every published task, observation and trajectory record carries:

- `frame_id`;
- acquisition or generation timestamp;
- model/trajectory version;
- validity deadline or stale flag.

## PX4 boundary

The bridge must measure, in a dedicated test, the local-world axis mapping, origin translation, initial yaw difference and estimator-reset behavior. A local LIO axis is not labelled East/North without an independent heading reference. PX4 setpoints and external-odometry messages are accepted only after that alignment test passes.
