#pragma once

/**
 * To subscribe to
 *  poses
 *  plane segment params
 * 
 * Maintain header wise 
 *  poses
 *  plane segment params
 * 
 **/

// first things first
/**
 * simple log from mapper node
 * subscribe to point_cloud and color them based on plane id and publish coloured point cloud
 * subscribe to header-wise pose
 * subscribe to header-wise mask_cloud
 * (
 * Write a common callback using time synchronizer
 * in the callback, 
 *  add all the messages to a buffer
 *  maintain current and previous plane ids
 *  maintain plane id vs plane params (first measurement)
 * 
 * Also publish all the plane clouds (), using mask clouds in the buffer
 * When a plane goes out of view,
 *  Optimize all the planes, update the map
 *  
 **/
#include "parameters.h"

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <random>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "eigenmvn.h"

using namespace std;
using namespace Eigen;

ros::Publisher pub_paths;

map<double, vector<Vector4d>> plane_measurements;

MatrixXd diff(MatrixXd input)
{
  MatrixXd output;
  output = MatrixXd::Zero(input.rows()-1, input.cols());

  output = input.block(1, 0, input.rows()-1, input.cols()) - input.block(0, 0, input.rows()-1, input.cols());

  return output;
}