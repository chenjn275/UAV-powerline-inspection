#!/usr/bin/env python3
"""Move a visible drone through a transmission-line inspection demonstration.

This is a visualization and task-state demo.  It does not publish PX4
setpoints and it does not claim flight dynamics or sensor accuracy.
"""

from __future__ import annotations

import math
import time

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, Quaternion
from rclpy.node import Node
from std_msgs.msg import String


class InspectionGazeboDemo(Node):
    def __init__(self) -> None:
        super().__init__("inspection_gazebo_demo")
        self.declare_parameter("duration_s", 45.0)
        self._duration = max(5.0, float(self.get_parameter("duration_s").value))
        # Gazebo ROS exposes this service at different names depending on the
        # plugin composition. Keep both forms so the scene works with the
        # stock Humble launch file and with a namespaced Gazebo instance.
        self._set_state_clients = tuple(
            self.create_client(SetEntityState, service_name)
            for service_name in ("/gazebo/set_entity_state", "/set_entity_state")
        )
        self._status_pub = self.create_publisher(String, "/inspection/gazebo/status", 10)
        self._coverage_pub = self.create_publisher(String, "/inspection/gazebo/coverage", 10)
        self._start = time.monotonic()
        self._last_token = -1
        self._done = False
        self._pending_states = {}
        self._timer = self.create_timer(0.1, self._tick)

    @staticmethod
    def _pose(x: float, y: float, z: float, yaw: float = 0.0) -> Pose:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation = Quaternion(
            z=math.sin(yaw * 0.5),
            w=math.cos(yaw * 0.5),
        )
        return pose

    def _set_entity(self, name: str, pose: Pose) -> None:
        client = next((item for item in self._set_state_clients if item.service_is_ready()), None)
        if client is None:
            return
        pending = self._pending_states.get(name)
        if pending is not None and not pending.done():
            # Do not queue multiple pose updates for the same model.  A
            # backlog of out-of-order SetEntityState requests makes Gazebo's
            # renderer appear to flicker while the aircraft is moving.
            return
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = name
        request.state.pose = pose
        request.state.reference_frame = "world"
        future = client.call_async(request)
        self._pending_states[name] = future

        def clear_pending(done_future) -> None:
            if self._pending_states.get(name) is done_future:
                self._pending_states.pop(name, None)

        future.add_done_callback(clear_pending)

    def _publish(self, status: str, coverage: float) -> None:
        status_msg = String()
        status_msg.data = status
        self._status_pub.publish(status_msg)
        coverage_msg = String()
        coverage_msg.data = f"coverage_ratio={coverage:.3f} completed_cells={round(25 * coverage)}/25"
        self._coverage_pub.publish(coverage_msg)

    def _tick(self) -> None:
        if not any(client.service_is_ready() for client in self._set_state_clients):
            self._publish("WAITING_FOR_GAZEBO_STATE_SERVICE", 0.0)
            return

        elapsed = time.monotonic() - self._start
        progress = min(1.0, elapsed / self._duration)
        # Inspect the near tower from bottom to top while orbiting it.  This
        # matches the task-book operation: vertical pole inspection, rather
        # than a straight flight along one conductor.
        angle = 2.0 * math.pi * progress * 2.0
        yaw = angle + math.pi / 2.0
        x = 0.0 + 3.8 * math.cos(angle)
        y = 0.0 + 3.8 * math.sin(angle)
        z = 1.5 + 8.0 * progress
        self._set_entity("inspection_drone", self._pose(x, y, z, yaw))
        # The depth camera and lidar are separate sensor models so they can
        # publish through gazebo_ros. Keep their mounting offsets synchronized
        # with the visible aircraft during the reference inspection.
        self._set_entity(
            "inspection_d435i",
            self._pose(x + 0.35 * math.cos(yaw), y + 0.35 * math.sin(yaw), z - 0.18, yaw),
        )
        self._set_entity("inspection_mid360", self._pose(x, y, z + 0.18, yaw))

        token = min(24, int(progress * 25.0))
        while self._last_token < token:
            self._last_token += 1
            token_angle = 2.0 * math.pi * self._last_token / 24.0
            token_x = 3.8 * math.cos(token_angle)
            token_y = 3.8 * math.sin(token_angle)
            token_z = 1.5 + 8.0 * self._last_token / 24.0
            # Covered cells appear as green spheres on the tower inspection ring.
            self._set_entity(
                f"coverage_{self._last_token:02d}",
                self._pose(token_x, token_y, token_z),
            )

        if progress >= 1.0:
            if not self._done:
                self._done = True
                self._publish("COMPLETE inspection_demo_finished", 1.0)
            return
        self._publish(
            f"SCANNING tower_height={z:.1f}m/9.5m progress={progress:.3f}",
            progress,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InspectionGazeboDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
