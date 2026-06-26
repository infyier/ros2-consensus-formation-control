import csv
import math
import os
import time
from datetime import datetime
from functools import partial

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def quat_to_yaw(q) -> float:

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class ConsensusNode(Node):
    NUM_ROBOTS = 3
    ROBOT_IDS = [1, 2, 3]

    def __init__(self):
        super().__init__("consensus_node")

        self.declare_parameter("robot_id", 1)
        self.declare_parameter("max_v", 1.0)
        self.declare_parameter("max_w", 1.5)
        self.declare_parameter("log_dir", os.path.expanduser("~/ugv_formation_logs"))

        self.robot_id = (
            self.get_parameter("robot_id").get_parameter_value().integer_value
        )
        self.max_v = self.get_parameter("max_v").get_parameter_value().double_value
        self.max_w = self.get_parameter("max_w").get_parameter_value().double_value
        self.log_dir = self.get_parameter("log_dir").get_parameter_value().string_value

        self.delta_phi = 2.0 * math.pi / self.NUM_ROBOTS

        self.states: dict[int, tuple[float, float, float]] = {}

        ns = f"ugv{self.robot_id}"
        self.cmd_pub = self.create_publisher(Twist, f"/{ns}/cmd_vel", 10)

        for rid in self.ROBOT_IDS:
            self.create_subscription(
                Odometry,
                f"/ugv{rid}/odom",
                partial(self._odom_cb, rid),
                10,
            )

        self.control_timer = self.create_timer(0.1, self._control_loop)

        self._csv_file, self._csv_writer = self._init_logger()

        self.get_logger().info(
            f"[ugv{self.robot_id}] Cyclic pursuit controller started."
        )

    def _odom_cb(self, robot_id: int, msg: Odometry) -> None:

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.states[robot_id] = (x, y, yaw)

    def _control_loop(self):
        if len(self.states) < self.NUM_ROBOTS:
            return

        x, y, yaw = self.states[self.robot_id]
        leader_id = (self.robot_id % self.NUM_ROBOTS) + 1
        lx, ly, _ = self.states[leader_id]

        TARGET_RADIUS = 5.0
        TARGET_SPACING = (2.0 * math.pi) / self.NUM_ROBOTS

        # velocity: maintaining 120 degree spacing
        my_phase = math.atan2(y, x)
        leader_phase = math.atan2(ly, lx)

        # calculate angular distance
        phase_diff = (leader_phase - my_phase) % (2.0 * math.pi)
        spacing_error = phase_diff - TARGET_SPACING

        # speed up if falling behind, slow down if too close
        # base speed is 0.5 m/s
        target_v = 0.5 + (0.8 * spacing_error)
        target_v = max(0.1, min(target_v, self.max_v))  # Clamp before kinematic calc

        # strict kinematic steering
        current_radius = math.hypot(x, y)
        radial_error = TARGET_RADIUS - current_radius

        # the exact angular velocity needed to drive in a circle of TARGET_RADIUS
        # at our current speed target_v
        kinematic_w = target_v / TARGET_RADIUS

        # tangent angle of the circle at our current position
        tangent_angle = math.atan2(x, -y)

        # how far is our current nose pointing away from the perfect tangent
        heading_error = wrap(tangent_angle - yaw)

        # steering command = Base curve + Heading correction + Radial pull
        # if we are outside the circle (radial_error < 0), we turn sharper (add to w)
        # if we are inside the circle (radial_error > 0), we turn wider (subtract from w)
        target_w = kinematic_w + (1.2 * heading_error) - (0.4 * radial_error)

        # smoothing and publishing
        alpha = 0.2
        v = (alpha * target_v) + ((1 - alpha) * getattr(self, "prev_v", 0.5))
        w = (alpha * target_w) + ((1 - alpha) * getattr(self, "prev_w", kinematic_w))

        self.prev_v, self.prev_w = v, w

        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(max(-self.max_w, min(w, self.max_w)))
        self.cmd_pub.publish(twist)

        # logging
        distance = math.hypot(lx - x, ly - y)
        r = math.hypot(x, y)
        self.get_logger().info(
            f"UGV{self.robot_id} -> UGV{leader_id} | "
            f"dist={distance:.2f} | r={r:.2f} | r_err={radial_error:.2f}"
        )

    def _init_logger(self):
        os.makedirs(self.log_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join(self.log_dir, f"ugv{self.robot_id}_{stamp}.csv")
        f = open(filepath, "w", newline="")
        writer = csv.writer(f)
        writer.writerow(
            [
                "time_s",
                "x",
                "y",
                "yaw_rad",
                "distance_to_leader",
                "desired_heading",
                "v_cmd_ms",
                "w_cmd_rads",
                "distance_error",
                "unused",
                "heading_error",
            ]
        )
        self.get_logger().info(f"[ugv{self.robot_id}] Logging to {filepath}")
        return f, writer

    def _log_row(
        self, x, y, yaw, radius, phase, v, w, radial_err, spacing_err, heading_err
    ) -> None:
        self._csv_writer.writerow(
            [
                f"{time.time():.4f}",
                f"{x:.4f}",
                f"{y:.4f}",
                f"{yaw:.4f}",
                f"{radius:.4f}",
                f"{phase:.4f}",
                f"{v:.4f}",
                f"{w:.4f}",
                f"{radial_err:.4f}",
                f"{spacing_err:.4f}",
                f"{heading_err:.4f}",
            ]
        )

        self._csv_file.flush()

    def destroy_node(self):
        self._csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ConsensusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
