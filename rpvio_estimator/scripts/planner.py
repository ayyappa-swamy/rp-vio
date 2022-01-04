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

class Box:
    def Box(self, vertices):
        pass

class BoxWorld:
    def BoxWorld(self, vertices):
        pass

class Planner:
    def Planner(self, vertices, odometry):
        pass

    def create_map(self):
        pass
    
    def compute_goal(self):
        pass

    def plan_paths(self):
        pass

    def visualize_paths(self):
        pass

    def execute(self):
        pass

    def init_paths(self):
        pass

    def optimize_paths(self):
        pass
