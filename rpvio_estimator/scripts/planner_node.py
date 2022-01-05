"""
This is the python version of planner_node.cpp
"""
#!/usr/bin/env python
import message_filters
import rospy
from rospy.rostime import Time
from std_msgs.msg import String
from geometry_msgs.msg import Point, Quaternion
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from message_filters import Subscriber
from message_filters import TimeSynchronizer

from scipy.spatial.transform.rotation import Rotation

import numpy as np

goal = np.array([25.0, -5.0, 5.0]).reshape((3, 1))
goal_ = np.array([25.0, -5.0, 5.0, 1.0]).reshape((4, 1))

# Read IMU-Camera fixed transformation

def vertices_and_odom_callback(vertices_msg, odometry_msg):
    trans = odometry_msg.pose.pose.position
    rot = odometry_msg.pose.pose.orientation
    t_i = np.array([trans.x, trans.y, trans.z]).reshape((3, 1))
    r_i = Rotation.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()

    Ti = np.zeros((4, 4))
    Ti[:2, :2] = r_i
    Ti[:2, 3:] = t_i

    local_goal = r_i @ goal + t_i # TODO: transform the goal to local frame
    line_points = []
    for i in range(10):
       pt = (i/9) @ local_goal
       way_pt = Point(pt.x(), pt.y(), pt.z())
       line_points.append(way_pt)


    # Create cuboids
    # Create a marker and visualize straight line path
    # Compute the STOMP trajectories
    # Visualize the initial STOMP trajectories
    # Optimize the STOMP trajectories
    # Visualize the optimized STOMP trajectories

def run():
    rospy.init_node('pyplanner', anonymous=True)

    vertices_sub = message_filters.Subscriber("/rpvio_mapper/frame_cloud", PointCloud)
    odometry_sub = message_filters.Subscriber("/vins_estimator/odometry", Odometry)
    ts = TimeSynchronizer([vertices_sub, odometry_sub], 20)
    ts.registerCallback(vertices_and_odom_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    run()
