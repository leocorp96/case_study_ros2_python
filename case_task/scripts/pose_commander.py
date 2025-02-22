#! /usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint
from builtin_interfaces.msg import Duration

from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveItPoseCommander(Node):
    def __init__(self):
        super().__init__('moveit_pose_commander')
        # Action Client for MoveIt2
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self.screw_pub = self.create_publisher(Float64MultiArray, '/irb_eff_group_controller/commands', 10)
        self.joint_publisher = self.create_publisher(
            JointTrajectory,
            "/irb_arm_w_screw_group_controller/joint_trajectory",
            10
        )
        self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "screwdriver-screwdriver_tcp_dummy"]

    def send_joint_command(self, positions, duration=2.0):
        """Send a joint trajectory command to the robot."""
        if len(self.joint_names) != len(positions):
            self.get_logger().error("Mismatch between joint names and positions!")
            return

        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        traj_msg.points.append(point)

        self.get_logger().info(f"Sending joint command: {positions}")
        self.joint_publisher.publish(traj_msg)
    
    def perform_screw_op(self, unscrew=True):
        start_t = time.time()
        msg = Float64MultiArray()
        val = 0.5 if (unscrew) else -0.5
        self.get_logger().info("Beginning Screw Operation!")
        while ((time.time() - start_t) < 2.0): #spin for 2 seconds
            try:
                msg.data = [val]
                self.screw_pub.publish(msg)
            except Exception as e:
                self.get_logger().error("Screw Operation failed!")
                return False
        msg.data[0] = 0.0
        self.screw_pub.publish(msg)
        self.get_logger().info("Screw Operation Completed!")
        return True
    
    def spin_until_future_complete_with_timeout(self, node, future, timeout_sec=15.0):
        """Spin until the future completes or times out."""
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            rclpy.spin_once(node, timeout_sec=0.1)
        return future.done()

    def move_to_pose(self, target_pose: PoseStamped, group_name="irb_arm_w_screw_group", ee_link="screwdriver_tcp_dummy"):
        """Send a pose goal to MoveIt for execution."""
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Move Group Action server not available!")
            return

        # Create a motion plan request
        motion_plan = MotionPlanRequest()
        motion_plan.group_name = group_name

        # Define Position Constraint
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = target_pose.header.frame_id
        position_constraint.link_name = ee_link
        position_constraint.target_point_offset.x = target_pose.pose.position.x
        position_constraint.target_point_offset.y = target_pose.pose.position.y
        position_constraint.target_point_offset.z = target_pose.pose.position.z
        position_constraint.weight = 1.0

        # Define Orientation Constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = target_pose.header.frame_id
        orientation_constraint.link_name = ee_link
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.01
        orientation_constraint.absolute_y_axis_tolerance = 0.01
        orientation_constraint.absolute_z_axis_tolerance = 0.01
        orientation_constraint.weight = 1.0

        # Add constraints
        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        motion_plan.goal_constraints.append(constraints)

        max_attempts = 3
        attempt = 0

        while attempt < max_attempts:
            attempt += 1
            self.get_logger().info(f"Attempt {attempt}/{max_attempts}: Sending pose goal...")

            # Create Action Goal
            move_action_goal = MoveGroup.Goal()
            move_action_goal.request = motion_plan

            # Send goal to MoveIt
            self.get_logger().info(f"Sending pose goal: {target_pose.pose}")
            future_goal = self.move_group_client.send_goal_async(move_action_goal)
            success = self.spin_until_future_complete_with_timeout(self, future_goal, timeout_sec=15.0)
            #rclpy.spin_until_future_complete(self, future_goal)

            if success:
                self.get_logger().info("Motion plan received successfully!")
                goal_handle = future_goal.result()
                if not goal_handle.accepted:
                    self.get_logger().error("Motion planning request was rejected!")
                    return False

                self.get_logger().info("Motion plan accepted, waiting for execution...")

                # Wait for result
                future_result = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, future_result)

                # Check if goal succeeded
                result = future_result.result()
                if result:
                    self.get_logger().info("Motion execution completed successfully!")
                    time.sleep(1.5) #sleep for 1.5 seconds
                    if(self.perform_screw_op()):
                        time.sleep(1.5) #sleep for 1.5 seconds
                    return True
                else:
                    self.get_logger().error("Motion execution failed!")
                    continue
            else:
                self.get_logger().error("Motion planning request timed out!")
        self.get_logger().error("All attempts failed. Aborting motion.")
        return False 

    def return_to_home(self):
        self.send_joint_command(positions=[0.0, -1.091, 1.0771, 0.0, 0.0, 0.0, 0.0])

def main(args=None):
    rclpy.init(args=args)
    node = MoveItPoseCommander()

    # Define a Pose Goal
    target_pose = PoseStamped()
    target_pose.header.frame_id = "world"
    target_pose.pose.position.x = 1.717251709
    target_pose.pose.position.y = -0.482763214
    target_pose.pose.position.z = 1.300887573
    target_pose.pose.orientation.x = 0.707102
    target_pose.pose.orientation.y = 0.00343985
    target_pose.pose.orientation.z = 0.707094
    target_pose.pose.orientation.w = 0.00349319

    # Send the pose goal
    node.move_to_pose(target_pose)

    node.get_logger().info("Motion Complete!")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
