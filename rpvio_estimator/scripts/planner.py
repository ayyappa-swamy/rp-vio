#!/usr/bin/env python
from os import pipe
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud

import airsim
import cvxpy
import numpy as np

import signal
import sys

from box_world import BoxWorld

class Planner:
    self.num_of_waypoints = 100
    self.num_of_paths = 500
    def __init__(self, vertices, odometry):
        self.vertices = vertices
        self.current_pose = self.compute_pose_from_odometry(odometry)
        self.map = self.create_map(vertices)
        self.goal = np.zeros((3, 1))
        self.init_paths = np.zeros((num_of_paths, num_of_waypoints))

    def compute_pose_from_odometry(self, odometry):
        pose = np.eye(4)
        return pose

    def create_map(self, vertices):
        map = BoxWorld(vertices, odometry)
        return map

    def plan_paths(self):
        num_goal = 3000 ####### batch size
        num = 75

        x_init =  30.0
        y_init =  -4.0
        z_init =  0.0
        yaw_init = 0.0

        x_des_traj_init = x_init
        y_des_traj_init = y_init
        z_des_traj_init = z_init
        yaw_des_traj_init = yaw_init

        vx_des = 1.0
        vy_des = -0.40
        vz_des = -0.2
        vyaw_des = -np.pi/200

        ############################################################## Hyperparameters
        t_fin = 75

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

        x_fin = x_des_traj_init+vx_des*t_fin
        y_fin = y_des_traj_init+vy_des*t_fin
        z_fin = z_des_traj_init+vz_des*t_fin
        yaw_fin = yaw_des_traj_init+vyaw_des*t_fin

        t_interp = np.linspace(0, t_fin, num)
        x_interp = x_des_traj_init + ((x_fin-x_des_traj_init)/t_fin) * t_interp
        y_interp = y_des_traj_init + ((y_fin-y_des_traj_init)/t_fin) * t_interp
        z_interp = z_des_traj_init + ((z_fin-z_des_traj_init)/t_fin) * t_interp
        yaw_interp = yaw_des_traj_init + ((yaw_fin-yaw_des_traj_init)/t_fin) * t_interp

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
        eps_kyaw = np.random.multivariate_normal(mu, 0.01*cov, (num_goal, ))

        x_samples = x_interp+eps_kx
        y_samples = y_interp+eps_ky
        z_samples = z_interp+eps_kz
        yaw_samples = yaw_interp+0.0*eps_kyaw

    def visualize_paths(self):
        pass

    def execute(self):
        pass

    def init_paths(self):
        pass

    def optimize_paths(self):
        pass
