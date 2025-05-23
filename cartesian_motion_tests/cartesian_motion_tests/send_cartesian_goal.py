import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray
from builtin_interfaces.msg import Time
import numpy as np

from cartesian_motion_interfaces.action import CartesianTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from time import sleep


class CartesianClient(Node):
    def __init__(self):
        super().__init__("cartesian_client")
        self._action_client = ActionClient(
            self, CartesianTrajectory, "/cartesian_motion"
        )
        self.joint_angles_publisher = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

    def send_goal(self, waypoints, max_speed, max_acc):
        goal_msg = CartesianTrajectory.Goal()

        # Fill in the PoseArray
        goal_msg.waypoints.header.stamp = self.get_clock().now().to_msg()
        goal_msg.waypoints.header.frame_id = "base_link"  # change if needed
        goal_msg.waypoints.poses = waypoints

        goal_msg.maximum_speed = max_speed
        goal_msg.maximum_acceleration = max_acc

        self.get_logger().info("Waiting for server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Sending goal...")
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: success = {result.success}")
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Feedback: {feedback_msg.feedback.feedback}")


def make_pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return pose


def main(args=None):
    rclpy.init(args=args)

    client = CartesianClient()

    # === EDIT WAYPOINTS HERE ===
    target_positions = [  # Tested ik to start the following trajectory
        0.0,
        -np.pi / 2,
        0.0,
        -np.pi / 2,
        -np.pi / 2,
        0.0,
    ]
    target_positions = [  # Tested ik to start the following trajectory
        0.000,
        -2.155,
        1.504,
        -2.553,
        -1.558,
        0.000,
    ]
    poses = [
        # make_pose(0.168, 0.635, 0.856, -0.5, 0.5, -0.5, 0.5),
        # make_pose(0.168, 0.085, 0.856, -0.5, 0.5, -0.5, 0.5),
        # make_pose(0.168, 0.085, 0.756, -0.5, 0.5, -0.5, 0.5),
        # make_pose(0.168, 0.635, 0.756, -0.5, 0.5, -0.5, 0.5),
        #
        make_pose(0.168, 0.135, 0.856, -0.5, 0.5, -0.5, 0.5),
        make_pose(0.168, 0.135, 0.556, -0.5, 0.5, -0.5, 0.5),
        # make_pose(0.168, 0.435, 0.756, -0.5, 0.5, -0.5, 0.5),
        #
        # make_pose(0.202, 0.165, 1.106, -0.488, 0.481, -0.512, 0.518),
        # make_pose(0.202, 0.165, 0.606, -0.488, 0.481, -0.512, 0.518),
    ]

    max_speed = 0.04  # m/s
    max_acc = 0.01  # m/s²

    traj_msg = JointTrajectory()
    traj_msg.joint_names = client.joint_names
    point = JointTrajectoryPoint()
    point.positions = target_positions
    point.time_from_start.sec = 2  # reach target in 2 seconds
    traj_msg.points.append(point)
    client.joint_angles_publisher.publish(traj_msg)
    print("Sleeping for 10 seconds after publishing initial angles")
    sleep(10)

    client.send_goal(poses, max_speed, max_acc)
    rclpy.spin(client)


if __name__ == "__main__":
    main()
