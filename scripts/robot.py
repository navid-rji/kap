#!/usr/bin/env python3

import moveit_commander
from moveit_commander import MoveGroupCommander

moveit_commander.roscpp_initialize([])
group_name = "panda_arm"

move_group = MoveGroupCommander(group_name)

current_pose = move_group.get_current_pose().pose

pose = {
    "position": {
        "x": current_pose.position.x,
        "y": current_pose.position.y,
        "z": current_pose.position.z,
    },
    "orientation": {
        "x": current_pose.orientation.x,
        "y": current_pose.orientation.y,
        "z": current_pose.orientation.z,
        "w": current_pose.orientation.w,
    },
}

print(pose)