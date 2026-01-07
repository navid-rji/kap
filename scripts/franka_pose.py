#!/usr/bin/env python3

import rospy
import numpy as np
import tf.transformations as tft
from franka_msgs.msg import FrankaState

def state_cb(msg: FrankaState):
    # O_T_EE is column-major, length 16
    T = np.array(msg.O_T_EE, dtype=np.float64).reshape((4, 4), order='F')

    x, y, z = T[0, 3], T[1, 3], T[2, 3]

    T44 = np.eye(4)
    T44[0:3, 0:3] = T[0:3, 0:3]
    qx, qy, qz, qw = tft.quaternion_from_matrix(T44)

    rospy.loginfo_throttle(
        0.5,
        "EE pose | pos [m]: (%.4f, %.4f, %.4f) | quat: (%.4f, %.4f, %.4f, %.4f)",
        x, y, z, qx, qy, qz, qw
    )

def main():
    rospy.init_node("fr3_pose_test")
    rospy.Subscriber(
        "/franka_state_controller/franka_states",
        FrankaState,
        state_cb,
        queue_size=1,
        tcp_nodelay=True,
    )
    rospy.loginfo("Listening to /franka_state_controller/franka_states …")
    rospy.spin()

if __name__ == "__main__":
    main()
