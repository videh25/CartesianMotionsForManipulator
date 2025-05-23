import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
import time

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState


class SinusoidalTrajectoryTester(Node):
    def __init__(self):
        super().__init__("sinusoidal_trajectory_tester")

        # ==== Configuration ====
        self.joint_name = "elbow_joint"  #  Change this to match your setup
        self.trajectory_duration = 6.0  # seconds
        self.publish_rate = 50  # Hz
        self.amplitude = 3.14  # rad/s (velocity)
        self.frequency = 1.0  # Hz (sinusoid)

        self.controller_ns = "/joint_trajectory_controller"
        self.traj_pub = self.create_publisher(
            JointTrajectory, f"{self.controller_ns}/joint_trajectory", 10
        )
        self.state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            f"{self.controller_ns}/controller_state",
            self.state_callback,
            10,
        )

        self.timestamps = []
        self.desired_velocities = []
        self.actual_velocities = []

        self.send_trajectory()
        self.start_time = time.time()

    def send_trajectory(self):
        self.get_logger().info("Preparing sinusoidal trajectory...")

        num_points = int(self.trajectory_duration * self.publish_rate)
        times = np.linspace(0, self.trajectory_duration, num_points)
        velocities = self.amplitude * np.sin(2 * np.pi * self.frequency * times)
        positions = -0.0 / 2 + np.cumsum(velocities) * (1 / self.publish_rate)
        # initial position of joint here

        msg = JointTrajectory()
        msg.joint_names = [self.joint_name]

        for i, t in enumerate(times):
            pt = JointTrajectoryPoint()
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t % 1) * 1e9)
            pt.positions = [positions[i]]
            pt.velocities = [velocities[i]]
            msg.points.append(pt)

        self.get_logger().info("Publishing trajectory...")
        self.traj_pub.publish(msg)

    def state_callback(self, msg: JointTrajectoryControllerState):
        try:
            idx = msg.joint_names.index(self.joint_name)
        except ValueError:
            self.get_logger().error(
                f"Joint '{self.joint_name}' not found in state message."
            )
            return

        now = time.time()
        elapsed = now - self.start_time

        if elapsed > self.trajectory_duration + 1.0:
            self.plot_and_exit()
            return

        self.timestamps.append(elapsed)
        self.actual_velocities.append(msg.feedback.velocities[idx])
        self.desired_velocities.append(msg.reference.velocities[idx])

    def plot_and_exit(self):
        self.get_logger().info("Plotting results...")

        plt.figure(figsize=(10, 5))
        plt.plot(
            self.timestamps[: len(self.desired_velocities)],
            self.desired_velocities,
            label="Desired Velocity",
            linestyle="--",
        )
        plt.plot(
            self.timestamps[: len(self.actual_velocities)],
            self.actual_velocities,
            label="Actual Velocity",
        )
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (rad/s)")
        plt.title(f"Velocity Tracking for '{self.joint_name}'")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SinusoidalTrajectoryTester()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
