#! /usr/bin/env python3

import os, json, time
import sys
import yaml
import rclpy
import numpy as np
import rclpy.logging
from scripts.utils import ScrewData, ScrewDrive, ScrewPose, ScrewType

# message libraries
from geometry_msgs.msg import PoseStamped, Pose

# moveit_py
from moveit.planning import MoveItPy
#from moveit.core.robot_state import RobotState

# config file libraries
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

from scripts.pose_commander import MoveItPoseCommander
from rclpy.node import Node

class RobotLogic(Node):
    def __init__(self, robot_name:str, package_name:str):
        super().__init__("robot_logic_node")
    
    # Function to check if file is valid
    def _is_valid_json(self, file_path: str) -> bool:
        """Check if the file exists, is not empty, and contains valid JSON."""
        if not os.path.exists(file_path):
            print(f"Error: File '{file_path}' does not exist.")
            return False
        
        if os.path.getsize(file_path) == 0:
            print(f"Error: File '{file_path}' is empty.")
            return False

        try:
            with open(file_path, "r") as file:
                json.load(file)  # Try parsing JSON
            return True
        except json.JSONDecodeError:
            print(f"Error: File '{file_path}' contains invalid JSON.")
            return False
        
    def getScrewPoses(self, filename:str)-> ScrewData:
        file_path=os.path.join(
                    get_package_share_directory("case_task"),
                    "resource",
                    f"{filename}.json",
                )
        if(self._is_valid_json(file_path=file_path)):
            with open(file_path, "r") as file:
                data = json.load(file)

            
            if "screws" not in data or "screw_type" not in data["screws"] or "poses" not in data["screws"]:
                print(f"Error: Missing required keys in JSON structure.")
                return None
            else:
                # Parse screw type details
                screw_type_data = ScrewType(
                    drive=ScrewDrive(
                        type=data["screws"]["screw_type"]["drive"]["type"],
                        size=data["screws"]["screw_type"]["drive"]["size"]
                    ),
                    length=data["screws"]["screw_type"]["length"],
                    turns=data["screws"]["screw_type"]["turns"]
                )
                # Parse screw poses
                screw_poses = [
                    ScrewPose(
                        position=pose["position"],
                        orientation=pose["orientation"]
                    )
                    for pose in data["screws"]["poses"]
                ]

                # Create structured ScrewData object
                screw_data = ScrewData(
                    screw_type=screw_type_data,
                    poses=screw_poses
                )
                return screw_data

def main(args=None):
    rclpy.init(args=args)  
    rclpy.logging.get_logger('Robot Logic').info("Starting Screw Task !")
    pose_node = MoveItPoseCommander()

    rl_node = RobotLogic(robot_name='irb6640_205', package_name='case_task')
    screw_data:ScrewData = rl_node.getScrewPoses('screw_poses')
    rclpy.logging.get_logger('Robot Logic').info("Detected Screw with following specifications: ")
    rclpy.logging.get_logger('Robot Logic').info("Type: %s - Size: %s" % 
                        (screw_data.screw_type.drive.type, screw_data.screw_type.drive.size))
    rclpy.logging.get_logger('Robot Logic').info("Length: %s - Turns: %s" % 
                        (screw_data.screw_type.length, screw_data.screw_type.turns))
    
    # Define a Pose Goal
    target_pose = PoseStamped()
    target_pose.header.frame_id = "world"
    for sc_pose in screw_data.poses:
        #print(sc_pose)
        rclpy.logging.get_logger('Robot Logic').info("Moving to: ")
        rclpy.logging.get_logger('Robot Logic').info("\tPosition: ")
        rclpy.logging.get_logger('Robot Logic').info("\t\tx: %s" % sc_pose.position[0])
        rclpy.logging.get_logger('Robot Logic').info("\t\ty: %s" % sc_pose.position[1])
        rclpy.logging.get_logger('Robot Logic').info("\t\tz: %s" % sc_pose.position[2])
        rclpy.logging.get_logger('Robot Logic').info("\tOrientation: ")
        rclpy.logging.get_logger('Robot Logic').info("\t\tw: %s" % sc_pose.orientation[0])
        rclpy.logging.get_logger('Robot Logic').info("\t\tx: %s" % sc_pose.orientation[1])
        rclpy.logging.get_logger('Robot Logic').info("\t\ty: %s" % sc_pose.orientation[2])
        rclpy.logging.get_logger('Robot Logic').info("\t\tz: %s" % sc_pose.orientation[3])

        target_pose.pose.position.x = sc_pose.position[0]
        target_pose.pose.position.y = sc_pose.position[1]
        target_pose.pose.position.z = sc_pose.position[2]
        target_pose.pose.orientation.x = sc_pose.orientation[0]
        target_pose.pose.orientation.y = sc_pose.orientation[1]
        target_pose.pose.orientation.z = sc_pose.orientation[2]
        target_pose.pose.orientation.w = sc_pose.orientation[3]

        # Send the pose goal
        pose_node.move_to_pose(target_pose)
        time.sleep(3.0) #Wait for sometime before going to next pose

    pose_node.return_to_home()
    pose_node.get_logger().info("All Motions Completed!")
    pose_node.destroy_node()

    rclpy.logging.get_logger('Robot Logic').info("Screw Task - Ended!")
    rclpy.spin(rl_node)
    rclpy.shutdown()     

if __name__ == '__main__':
    main()
