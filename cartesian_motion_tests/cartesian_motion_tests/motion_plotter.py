import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from tf2_ros import TransformListener, Buffer

import numpy as np
import matplotlib.pyplot as plt
from threading import Thread, Lock
import time


class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__("trajectory_plotter")

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Controller state subscriber
        self.create_subscription(
            JointTrajectoryControllerState,
            "/joint_trajectory_controller/controller_state",
            self.controller_state_callback,
            10,
        )

        # Data storage
        self.lock = Lock()
        self.tf_times = []
        self.tf_speeds = []
        self.last_tf = None
        self.last_tf_time = None

        self.joint_names = None
        self.joint_times = []
        self.joint_actual = []
        self.joint_commanded = []

        # Spin in background thread
        Thread(target=self.spin_thread, daemon=True).start()

    def spin_thread(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            self.read_tf()

    def read_tf(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                "base_link", "tool0", rclpy.time.Time()
            )
        except:
            return

        t = trans.header.stamp.sec + trans.header.stamp.nanosec * 1e-9
        pos = trans.transform.translation
        pos_np = np.array([pos.x, pos.y, pos.z])

        with self.lock:
            if self.last_tf is not None:
                dt = t - self.last_tf_time
                if dt > 0:
                    speed = np.linalg.norm(pos_np - self.last_tf) / dt
                    self.tf_times.append(t)
                    self.tf_speeds.append(speed)

            self.last_tf = pos_np
            self.last_tf_time = t

    def controller_state_callback(self, msg):
        t = self.get_clock().now().seconds_nanoseconds()
        timestamp = t[0] + t[1] * 1e-9

        with self.lock:
            if self.joint_names is None:
                self.joint_names = msg.joint_names

            self.joint_times.append(timestamp)
            # self.joint_actual.append(list(msg.feedback.positions))
            self.joint_actual.append(list(msg.feedback.velocities))
            # self.joint_commanded.append(list(msg.reference.positions))

    def plot(self):
        with self.lock:
            # TF speed
            plt.figure()
            plt.plot(self.tf_times, self.tf_speeds)
            plt.title("Tool Linear Speed (base_link -> tool0) @ ee_speed @ 0.04 m/s")
            plt.xlabel("Time [s]")
            plt.ylabel("Speed [m/s]")
            plt.grid(True)

            # Joint angles - all in one figure with subplots
            if not self.joint_times or not self.joint_actual:
                print("No joint data recorded.")
                return

            actual = np.array(self.joint_actual)
            commanded = np.array(self.joint_commanded)
            times = np.array(self.joint_times)

            num_joints = len(self.joint_names)
            fig, axs = plt.subplots(
                num_joints, 1, sharex=True, figsize=(10, 2.5 * num_joints)
            )
            fig.suptitle("Joint Velocities @ ee_speed = 0.04 m/s")

            for i, name in enumerate(self.joint_names):
                ax = axs[i]
                ax.plot(times, actual[:, i], label="Actual Velocities")
                # ax.plot(times, commanded[:, i], label="Commanded", linestyle="--")
                ax.set_ylabel(f"{name}\n[rad/s]")
                ax.grid(True)
                ax.legend(loc="upper right")

            axs[-1].set_xlabel("Time [s]")
            plt.tight_layout(rect=[0, 0, 1, 0.97])
            plt.show()


def main():
    rclpy.init()
    node = TrajectoryPlotter()

    try:
        print("Recording... Press Ctrl+C to stop and show plots.")
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping and plotting...")
        node.plot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
