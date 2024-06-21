#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
from typing import Dict

import actionlib
import rospy
import yaml
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from std_msgs.msg import Empty
from std_msgs.msg import String

ENV_LOCATION_YAML = 'LOCATION_YAML'
TOPIC_UNITY_CMD = '/unity_whill_cmd'
TOPIC_UNITY_RESPONSE = '/unity_whill_response'


def read_yaml(path: str) -> Dict[str, Pose]:
    """Read yaml file and convert file contents to a dictionary that has a Pose type object as value.

    Sample content
    --------------
    P0:
        position:
            x: 0.0
            y: 0.0
            z: 0.0
        orientation:
            x: 0.0
            y: 0.0
            z: 0.0
            w: 1.0
    --------------

    Args:
        path (str): Path to the yaml file.

    Returns:
        Dict[str, Pose]: Dictionary with string as key and Pose as value.

    """
    with open(path, 'r') as file:
        data = yaml.safe_load(file)
    for key in data.keys():
        data[key] = Pose(
            position=Point(**data[key]['position']),
            orientation=Quaternion(**data[key]['orientation'])
        )
    return data


class UnityCommandHandler:
    """Listens for commands from Unity and convert it to a PoseStamped type object."""

    def __init__(self) -> None:
        """Constructor."""
        self.__move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.__goal = MoveBaseGoal()
        self.__goal.target_pose.header.frame_id = "map"
        self.__pose_dict = read_yaml(os.environ[ENV_LOCATION_YAML])
        rospy.Subscriber(TOPIC_UNITY_CMD, String, self.callback)
        self.__response_pub = rospy.Publisher(TOPIC_UNITY_RESPONSE, Empty, queue_size=10)

    def callback(self, data: String) -> None:
        """Callback function."""
        if data.data not in self.__pose_dict.keys():
            rospy.logerr(f'Invalid location: {data.data}')
            return
        else:
            rospy.loginfo(f'Moving to goal with ID: {data.data}')
        # Add info to PoseStamped message.
        self.__goal.target_pose.header.stamp = rospy.Time.now()
        self.__goal.target_pose.pose = self.__pose_dict[data.data]
        self.__move_base_client.send_goal(self.__goal)
        result = self.__move_base_client.wait_for_result()
        if not result:
            rospy.logerr('Failed to reach goal')
            return
        rospy.loginfo('Confirmed goal reach, sending response to Unity')
        self.__response_pub.publish(Empty())


def main() -> None:
    """Main function."""
    rospy.init_node('unity_command_handler')
    UnityCommandHandler()
    rospy.spin()


if __name__ == '__main__':
    main()
