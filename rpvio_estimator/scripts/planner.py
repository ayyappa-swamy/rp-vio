#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform.rotation import Rotation

import numpy as np

import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

from box_world import BoxWorld

class Planner:
    num_of_paths = 3000 #number of paths
    num_of_way_points = 75 #number of waypoints in each waypoint
    tic = np.zeros(3)
    ric = np.eye(3)
    ti = np.zeros(3)
    ri = np.eye(3)
    global_goal = np.array([25.0, -5.0, -5.0])

    def __init__(self, vertices_msg, odometry_msg):
        self.vertices_msg = vertices_msg
        self.odometry_msg = odometry_msg
        
        self.read_cam_imu_transform()
        self.parse_odometry_msg(odometry_msg)

        self.map = BoxWorld(vertices_msg)

        self.local_goal = self.world2cam(self.global_goal)
        print("Local goal is: ")
        print(self.local_goal)

        self.register_publishers()

    def register_publishers(self):
        self.local_goal_pub = rospy.Publisher("local_goal", PointCloud)
        self.local_stomp_pub = rospy.Publisher("gaussian_paths", MarkerArray)
        self.feasible_path_pub = rospy.Publisher("feasible_path", PointCloud)
        print("Registered publishers")
    
    def read_cam_imu_transform(self):
        fs = cv2.FileStorage("../../../rpvio_sim_config.yaml", cv2.FILE_STORAGE_READ)
        self.ric = np.array(fs.getNode("extrinsicRotation").mat())
        self.tic =  np.array(fs.getNode("extrinsicTranslation").mat())
        
        print("Read cam imu transform")
        print("tic: ")
        print(self.tic)
        print("ric: ")
        print(self.ric)

    def parse_odometry_msg(self, odometry_msg):
        trans = odometry_msg.pose.pose.position
        rot = odometry_msg.pose.pose.orientation
        self.ti = np.array([trans.x, trans.y, trans.z]).reshape((3, 1))
        self.ri = Rotation.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()

    def world2cam(self, world_vector):
        world_vector = world_vector.reshape((3, 1))
        local_vector = self.ri.T * world_vector - self.ri.T * self.ti
        cam_vector = self.ric.T * local_vector - self.ric.T * self.tic
        return cam_vector.flatten()

    def compute_paths(self):
        self.compute_stomp_paths()

    def compute_stomp_paths(self):
        num_goal = self.num_of_paths
        num = self.num_of_way_points

        x_init =  0.0
        y_init =  0.0
        z_init =  0.0

        x_des_traj_init = x_init
        y_des_traj_init = y_init
        z_des_traj_init = z_init

        ############################################################## Hyperparameters
        t_fin = 5

        ######################################## noise sampling

        ########### Random samples for batch initialization of heading angles
        A = np.diff(np.diff(np.identity(num), axis = 0), axis = 0)
        # A = np.diff(np.identity(prob.num), axis =0 )
        # print(A.shape)
        temp_1 = np.zeros(num)
        temp_2 = np.zeros(num)
        temp_3 = np.zeros(num)
        temp_4 = np.zeros(num)

        temp_1[0] = 1.0
        temp_2[0] = -2
        temp_2[1] = 1
        temp_3[-1] = -2
        temp_3[-2] = 1

        temp_4[-1] = 1.0

        A_mat = -np.vstack(( temp_1, temp_2, A, temp_3, temp_4   ))

        x_fin = self.local_goal[0] 
        y_fin = self.local_goal[1]
        z_fin = self.local_goal[2] 

        t_interp = np.linspace(0, t_fin, num)
        x_interp = x_des_traj_init + ((x_fin-x_des_traj_init)/t_fin) * t_interp
        y_interp = y_des_traj_init + ((y_fin-y_des_traj_init)/t_fin) * t_interp
        z_interp = z_des_traj_init + ((z_fin-z_des_traj_init)/t_fin) * t_interp

        # A_mat = A
        # print(temp_1.shape)
        # print(A_mat.shape)
        R = np.dot(A_mat.T, A_mat)
        mu = np.zeros(num)
        cov = np.linalg.pinv(R)

        # print(R.shape)
        ################# Gaussian Trajectory Sampling
        eps_kx = np.random.multivariate_normal(mu, 0.03*cov, (num_goal, ))
        eps_ky = np.random.multivariate_normal(mu, 0.03*cov, (num_goal, ))
        eps_kz = np.random.multivariate_normal(mu, 0.03*cov, (num_goal, ))

        self.x_samples = x_interp+eps_kx
        self.y_samples = y_interp+eps_ky
        self.z_samples = z_interp+eps_kz

    def publish_paths(self):
        self.publish_local_goal()
        self.publish_local_stomp_paths()

    def publish_local_goal(self):
        goal_pc = PointCloud()
        goal_pc.header = self.vertices_msg.header
        goal_pc.points.append(self.to_ros_point(self.local_goal))
        self.local_goal_pub.publish(goal_pc)
    
    def publish_local_stomp_paths(self):
        ma = MarkerArray()
        
        optimal_line_strip = Marker()
        is_optimal_colliding = True
        max_sdf_cost = -100000000

        # Add a line set marker for each path
        for p in range(self.x_samples.shape[0]):
            line_strip = Marker()
            line_strip.header = self.vertices_msg.header
            
            line_strip.pose.orientation.w = 1.0

            line_strip.id = p+2
            line_strip.type = Marker.LINE_STRIP

            line_strip.scale.x = 0.03

            line_strip.color.r = 1.0
            line_strip.color.a = 0.7

            is_colliding = False
            traj_cost = 0.0

            for w in range(self.x_samples.shape[1]):
                line_pt = Point()
                line_pt.x = self.x_samples[p, w]
                line_pt.y = self.y_samples[p, w]
                line_pt.z = self.z_samples[p, w]
                
                line_strip.points.append(line_pt)
                point_cost = self.map.get_point_cost(self.from_ros_point(line_pt))
                if point_cost < 2.0:
                    is_colliding = True

                traj_cost += point_cost

            if (traj_cost > max_sdf_cost) and not is_colliding:
                max_sdf_cost = traj_cost
                is_optimal_colliding = False
                optimal_line_strip = line_strip

            ma.markers.append(line_strip)

        if not is_optimal_colliding:
            optimal_line_strip.color.r = 0.0
            optimal_line_strip.color.g = 1.0
            optimal_line_strip.color.b = 0.0
            optimal_line_strip.color.x = 0.06
            optimal_line_strip.color.a = 1.0

            ma.markers.append(optimal_line_strip)

            feasible_cloud = PointCloud()
            feasible_cloud.header = self.vertices_msg.header

            for point in optimal_line_strip.points:
                pt = self.from_ros_point(point)
                w_pt = self.world2cam(pt)
                w_point = self.to_ros_point(w_pt.flatten())

                feasible_cloud.points.push_back(w_point)

            self.feasible_path_pub.publish(feasible_cloud)

        self.local_stomp_pub.publish(ma)

    def from_ros_point(self, ros_point):
        return np.array([ros_point.x, ros_point.y, ros_point.z]) 

    def to_ros_point(self, point):
        ros_point = Point()
        ros_point.x = point[0]
        ros_point.y = point[1]
        ros_point.z = point[2]

        return ros_point