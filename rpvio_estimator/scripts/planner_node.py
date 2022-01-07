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
from planner import Planner

def vertices_and_odom_callback(vertices_msg, odometry_msg):
    planner = Planner(vertices_msg, odometry_msg)
    planner.compute_paths()
    planner.publish_paths()

def register_pub_sub():
    rospy.init_node('pyplanner', anonymous=True)

    vertices_sub = message_filters.Subscriber("/rpvio_mapper/frame_cloud", PointCloud)
    odometry_sub = message_filters.Subscriber("/vins_estimator/odometry", Odometry)
    ts = TimeSynchronizer([vertices_sub, odometry_sub], 20)
    ts.registerCallback(vertices_and_odom_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    register_pub_sub()
