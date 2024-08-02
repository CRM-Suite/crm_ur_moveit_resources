#!/usr/bin/env python3
"""
A script to move the robot arm 5 cm in the positive Z direction using the MoveItPy API.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy

from geometry_msgs.msg import PoseStamped


def plan_and_execute(robot, planning_component, logger):
    """Helper function to plan and execute a motion."""
    # Plan to goal
    logger.info("Planning trajectory")
    plan_result = planning_component.plan()

    # Execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(1.0)


def main():
    rclpy.init()
    logger = get_logger("moveit_py.z_axis_move")

    # Instantiate MoveItPy instance and get planning component
    robot = MoveItPy(node_name="moveit_py")
    planning_component = robot.get_planning_component("manipulator")
    logger.info("MoveItPy instance created")

    # Set start state to current state
    planning_component.set_start_state_to_current_state()

    # Get current pose and set the goal pose 5 cm above the current position
    current_pose = planning_component.get_current_pose().pose
    logger.info(f"Current Pose: {current_pose}")

    target_pose = PoseStamped()
    target_pose.header.frame_id = "base_link"
    target_pose.pose = current_pose
    target_pose.pose.position.z += 0.05

    # Set the goal pose
    planning_component.set_goal_state(pose_stamped_msg=target_pose, pose_link="tool0")

    # Plan and execute the motion
    plan_and_execute(robot, planning_component, logger)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
