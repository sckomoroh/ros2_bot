#!/bin/python3

import rclpy
import time

from action_msgs.msg import GoalStatus
from rclpy.node import Node
from lifecycle_msgs.srv import GetState
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from enum import Enum


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class Navigator(Node):
    ANGLES_ORIENTATIONS = {
        0: {"z": 0.0, "w": 1.0},
        90: {"z": 0.7071068, "w": 0.7071068},
        180: {"z": 1.0, "w": 0.0},
        -90: {"z": -0.7071068, "w": 0.7071068},
    }

    def __init__(self):
        super().__init__("robot_navigator")

        self.goal_poses = []
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def wait_for_navigation(self):
        self.get_logger().info("Waiting for amcl")
        self.__wait_for_node_activate("amcl")

        self.get_logger().info("Waiting for bt_navigator")
        self.__wait_for_node_activate("bt_navigator")

    def add_pose(self, x: float, y: float, angle: int):
        self.get_logger().info(f"Add pose: X: {x} Y: {y} angle:{angle}")
        self.__create_pose(x, y, angle)

    def start_navigation(self):
        for goal_pose in self.goal_poses:
            self.get_logger().info(
                f"Navigate to pose: X:{goal_pose.pose.position.x} Y:{goal_pose.pose.position.y}"
            )

            self.__go_to_pose(goal_pose)

            i = 0
            while not self.__isTaskComplete():
                i = i + 1
                if self.feedback and i % 5 == 0:
                    print(
                        f"Distance remaining: {self.feedback.distance_remaining:.2f} m"
                    )

            result = self.__getResult()

            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("Goal succeeded")
            elif result == TaskResult.CANCELED:
                self.get_logger().warn("Goal was canceled")
                self.__clean_navigation()
                return
            elif result == TaskResult.FAILED:
                self.get_logger().error("Goal failed")
                self.__clean_navigation()
                print("Goal failed!")
            else:
                self.get_logger().error("Goal has an invalid return status")

    def __getResult(self):
        if not self.status:
            return TaskResult.UNKNOWN

        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            return TaskResult.UNKNOWN

    def __isTaskComplete(self):
        if not self.result_future:
            # task was cancelled or completed
            return True

        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                return True
        else:
            return False

        return True

    def __go_to_pose(self, goal_pose: PoseStamped):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg, self.__feedbackCallback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error(
                f"Goal to {goal_pose.pose.position.x:.02f}  {goal_pose.pose.position.y:.02f} was rejected!"
            )
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def __feedbackCallback(self, msg):
        self.feedback = msg.feedback

    def __clean_navigation(self):
        self.goal_poses.clear()

    def __create_pose(self, x, y, angle):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = self.ANGLES_ORIENTATIONS[angle]["z"]
        goal_pose.pose.orientation.w = self.ANGLES_ORIENTATIONS[angle]["w"]
        self.goal_poses.append(goal_pose)

    def __wait_for_node_activate(self, node_name: str):
        node_service = f"{node_name}/get_state"
        client = self.create_client(GetState, node_service)
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"{node_service} service not available, waiting...")

        req = GetState.Request()
        state = "unknown"
        while state != "active":
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
            time.sleep(2)


def main():
    rclpy.init()

    nav = Navigator()

    print("Wait for navigation")
    nav.wait_for_navigation()

    nav.add_pose(1.0, 0.0, 90)
    nav.add_pose(1.0, 0.5, 180)
    nav.add_pose(0.0, 0.5, 90)
    nav.add_pose(0.0, 1.0, 0)
    nav.add_pose(1.0, 1.0, 90)
    nav.add_pose(1.0, 1.5, 180)
    nav.add_pose(0.0, 1.5, 90)

    nav.start_navigation()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
