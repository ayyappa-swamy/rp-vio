#pragma once

/**
 * To subscribe to
 *  mask cloud
 *  poses
 *  feature points (/current_cloud)
 * 
 * Maintain header wise 
 *  images or mask_clouds
 *  poses
 * 
 * Maintain window wise
 *  sparse feature points (/current_cloud)
 * 
 * Algorithm:
 *  on mask_cloud
 *      hash w.r.t header
 *  on pose
 *      hash w.r.t header
 *  on feature cloud
 *      optimize
 *      reconstruct
 *      maintain plane id tracks
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

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

using namespace std;
using namespace Eigen;

ros::Publisher pub_plane_cloud;

/**
 * Implements vector1.dot(vector2) == 0 constraint
 * residual size: 1
 **/
struct OrthogonalConstraint {
    template <typename T>
    bool operator()(
        const T* const vector_array1,
        const T* const vector_array2,
        T* residual
    ) const {
        T v1[3];
        v1[0] = vector_array1[0];
        v1[1] = vector_array1[1];
        v1[2] = T(0);
        
        T v2[3];
        v2[0] = vector_array2[0];
        v2[1] = vector_array2[1];
        v2[2] = T(0);

        residual[0] = T(100) * ceres::DotProduct(v1, v2);
        return true;
    }
};

/**
 * Implements vector1.cross(vector2) == zero_vec(tor constraint
 * residual size: 3
 **/
struct ParallelConstraint {
    template <typename T>
    bool operator()(
        const T* const vector_array1,
        const T* const vector_array2,
        T* residual
    ) const {
        T cross[3];
        T v1[3];
        v1[0] = vector_array1[0];
        v1[1] = vector_array1[1];
        v1[2] = T(0);
        
        T v2[3];
        v2[0] = vector_array2[0];
        v2[1] = vector_array2[1];
        v2[2] = T(0);

        ceres::CrossProduct(v1, v2, cross);

        residual[0] = T(100) * cross[0];
        residual[1] = T(100) * cross[1];
        residual[2] = T(100) * cross[2];
        return true;
    }
};

/**
 * Implements local parameterization of unit normal
 **/
struct UnitNormalParameterization {
    template <typename T>
    bool operator()(
        const T* x, const T* delta, T* x_plus_delta
    ) const {
        Eigen::Map<const Eigen::Matrix<T, 2, 1>> x_(x);
        Eigen::Map<const Eigen::Matrix<T, 2, 1>> delta_(delta);
        Eigen::Map<Eigen::Matrix<T, 2, 1>> x_plus_delta_(x_plus_delta);

        x_plus_delta_ = x_ + delta_;
        x_plus_delta_.normalize();

        return true;
    }
};