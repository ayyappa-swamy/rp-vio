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
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/line_descriptor.hpp>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "utility/color_ids.h"

using namespace std;
using namespace Eigen;

ros::Publisher pub_plane_cloud;
ros::Publisher marker_pub;
ros::Publisher ellipse_pub;
ros::Publisher frame_pub;
ros::Publisher frame_pub2;
ros::Publisher masked_im_pub;

map<double, vector<Vector4d>> plane_measurements;

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
        Eigen::Matrix<T, 3, 1> v1;
        v1 << vector_array1[0], vector_array1[1], vector_array1[2];
        
        Eigen::Matrix<T, 3, 1> v2;
        v2 << vector_array2[0], vector_array2[1], vector_array2[2];

        residual[0] = T(100) * ceres::DotProduct(v1.normalized().data(), v2.normalized().data());
        return true;
    }
};


/**
 * Implements vector1.dot(vector2) == 0 constraint
 * residual size: 1
 **/
struct OrthogonalConstraintQuat {
    template <typename T>
    bool operator()(
        const T* const vector_array1,
        const T* const vector_array2,
        T* residual
    ) const {
        Eigen::Quaternion<T> v1(vector_array1[3], vector_array1[0], vector_array1[1], vector_array1[2]);
        v1.normalize();

        Eigen::Quaternion<T> v2(vector_array2[3], vector_array2[0], vector_array2[1], vector_array2[2]);
        v2.normalize();

        residual[0] = T(100) * v1.vec().normalized().dot(v2.vec().normalized());
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
        Eigen::Matrix<T, 3, 1> v1;
        v1 << vector_array1[0], vector_array1[1], vector_array1[2];
        
        Eigen::Matrix<T, 3, 1> v2;
        v2 << vector_array2[0], vector_array2[1], vector_array2[2];

        ceres::CrossProduct(v1.normalized().data(), v2.normalized().data(), cross);

        residual[0] = T(100) * cross[0];
        residual[1] = T(100) * cross[1];
        residual[2] = T(100) * cross[2];
        return true;
    }
};

/**
 * Implements horizontal normal constraint a.k.a vertical plane
 * residual size: 1
 **/
struct VerticalPlaneConstraint {
    template <typename T>
    bool operator()(
        const T* const vector_array1,
        T* residual
    ) const {        
        Eigen::Quaternion<T> v1(vector_array1);
        residual[0] = T(100) * v1.normalized().vec()(2);
        return true;
    }
};

// Plane measurement Constraint
// residual size: 4
struct PlaneMeasurementConstraint {
    PlaneMeasurementConstraint(Quaterniond &plane_meas): plane_meas_(plane_meas) {}
    
    template <typename T>
    bool operator()( 
        const T* const plane, T* residual
    ) const {
        Eigen::Quaternion<T> plane_quat(plane[3], plane[0], plane[1], plane[2]);
        plane_quat.normalize();
        Eigen::Quaternion<T> meas_quat = plane_meas_.cast<T>();;
        meas_quat.normalize();
        
        // residual[0] = meas_quat.x() - plane_quat.x();
        // residual[1] = meas_quat.y() - plane_quat.y();
        // residual[2] = meas_quat.z() - plane_quat.z();
        // residual[3] = meas_quat.w() - plane_quat.w();

        residual[0] = T(100) * (meas_quat.coeffs().normalized() - plane_quat.coeffs().normalized()).norm();
        return true;
    }
    
    Quaterniond plane_meas_;
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

void publish_plane_cloud(
    vector<sensor_msgs::PointCloudConstPtr> &mask_clouds,
    vector<nav_msgs::OdometryConstPtr> &odometry_msgs,
    map<int, Vector4d> plane_params
){
    sensor_msgs::PointCloud plane_cloud;
    sensor_msgs::ChannelFloat32 colors;
    colors.name = "rgb";

    map<int, Isometry3d> plane2world_isos;
    map<int, Vector4d> plane_seg_params; // min_z, max_z, min_y, max_y
        
    vector<int> cur_pids;
    for (map<int, Vector4d>::iterator it = plane_params.begin(); it != plane_params.end(); ++it){
        Vector4d pp = it->second;

        Vector3d normal;
        normal << pp(0), pp(1), pp(2);
        double d = -pp(3)/normal.norm();
        normal.normalize();
        
        if (fabs(d) < 100) {
            cur_pids.push_back(it->first);
            
            // Find nearest point on the plane (as center)
            Vector3d center = d * normal;

            Vector3d x_axis;
            x_axis << 1.0, 0.0, 0.0;

            // Compute the rotation of x-axis w.r.t normal
            double cos_theta = x_axis.dot(normal);
            double sin_theta = std::sqrt(1.0 - cos_theta * cos_theta);

            Isometry3d plane2world;
            plane2world.translation() = center;
            plane2world.linear() <<
                cos_theta, -sin_theta, 0.0,
                sin_theta, cos_theta, 0.0,
                0.0, 0.0, 1.0; 
            
            plane2world_isos[it->first] = plane2world;
            plane_seg_params[it->first] = Vector4d::Zero();
        }
        // ROS_INFO("---------------CURRENT PIDs----------------ID = %d --------------------", it->first);
    }

    /**
     * Plan:
     * 
     * Compute isometry for each plane frist
     * 
     * For each point:
     *  Transform the world point to its plane coord sys
     *  Check if its on border
     *  Update plane segment params
     *  Compute border points from plane segment params
     *  Transform border points to world sys
     *  Create line strip from border points and publish
     **/

    for(int qi = mask_clouds.size()-1; qi < mask_clouds.size(); qi++) {
        // publish plane cloud
        sensor_msgs::PointCloudConstPtr mask_cloud = mask_clouds[qi];
        nav_msgs::OdometryConstPtr odometry_msg = odometry_msgs[qi];
        plane_cloud.header = mask_cloud->header;

        ROS_DEBUG("-----------Computing FIXED transforms--------------");
        // Retrieve pose from odometry message
        Isometry3d Tic;
        Tic.linear() = RIC[0];
        Tic.translation() = TIC[0];
        ROS_DEBUG("-----------Computed FIXED transforms DONE--------------");

        ROS_DEBUG("-----------Computing odometry transforms--------------");

        Vector3d trans;
        trans <<
            odometry_msg->pose.pose.position.x,
            odometry_msg->pose.pose.position.y,
            odometry_msg->pose.pose.position.z;

        double quat_x = odometry_msg->pose.pose.orientation.x;
        double quat_y = odometry_msg->pose.pose.orientation.y;
        double quat_z = odometry_msg->pose.pose.orientation.z;
        double quat_w = odometry_msg->pose.pose.orientation.w;
        Quaterniond quat(quat_w, quat_x, quat_y, quat_z);

        Isometry3d Ti;
        Ti.linear() = quat.normalized().toRotationMatrix();
        Ti.translation() = trans;
        
        ROS_DEBUG("-----------Computed odometry transforms DONE--------------");

        map<int, int> pid2HEX;
        pid2HEX[39] = 0x0000FF;// cv::Scalar(255,0,0);
        pid2HEX[66] = 0xFF00FF;// cv::Scalar(255,0,255);
        pid2HEX[91] = 0x00FF00;// cv::Scalar(0,255,0);
        pid2HEX[130] = 0xFF0000;// cv::Scalar(0,0,255);
        pid2HEX[162] = 0x00FFFF;// cv::Scalar(255,255,0);
        pid2HEX[175] = 0xFFFF00;// cv::Scalar(0,255,255);

        // Back-project all mask points using odometry
        ROS_DEBUG("=============Back projecting mask points to planes DONE=============");
        for (unsigned int i = 0; i < mask_cloud->points.size(); i++) {
            int ppid = mask_cloud->channels[1].values[i];
            if (find(cur_pids.begin(), cur_pids.end(), ppid) != cur_pids.end()) {
                Vector4d pp = plane_params[ppid];

                Vector3d normal;
                normal << pp(0), pp(1), pp(2);
                double d = -pp(3)/normal.norm();
                normal.normalize();

                // if (fabs(d) < 100) {
                    double lambda = 0.0;
                    double lambda2 = 0.0;

                    geometry_msgs::Point32 m;
                    m = mask_cloud->points[i];
                    Vector3d c_point(m.x, m.y, m.z);

                    Vector4d pp_ci = (Ti * Tic).matrix().transpose() * pp.normalized();

                    Vector3d normal_ci;
                    normal_ci << pp_ci(0), pp_ci(1), pp_ci(2);
                    double d_ci = -pp_ci(3)/normal_ci.norm();
                    normal_ci.normalize();
                    
                    // Vector3d ray;
                    // ray << c_point(0), c_point(1), c_point(2);
                    // ray.normalize();

                    // if (fabs(normal_ci.dot(ray)) < 0.6)
                    //     continue;

                    Vector4d pp2 = pp.normalized();
                    pp2.head<3>() *= -1;
                    Vector4d pp_ci2 = (Ti * Tic).matrix().transpose() * pp2.normalized();

                    Vector3d normal_ci2;
                    normal_ci2 << pp_ci2(0), pp_ci2(1), pp_ci2(2);
                    double d_ci2 = -pp_ci2(3)/normal_ci2.norm();
                    normal_ci2.normalize();

                    lambda2 = -fabs(d_ci2) / (normal_ci2.dot(c_point));
                    lambda = -fabs(d_ci) / (normal_ci.dot(c_point));

                    if ((lambda2 * c_point)(2) < 0) {
                        c_point = lambda * c_point;
                                            
                    } else {
                        c_point = lambda2 * c_point;
                    }

                    // Vector3d ray;
                    // ray << c_point(0), c_point(1), c_point(2);
                    // ray.normalize();
                    Vector3d principal_ray;
                    principal_ray << 0.0, 0.0, 1.0;

                    if ((fabs(normal_ci.dot(principal_ray)) < 0.2) || (fabs(normal_ci2.dot(principal_ray)) < 0.2))
                        continue;
                    
                    // Transform c_point (current camera) to imu (current IMU)
                    Vector3d i_point = Tic.rotation() * c_point + Tic.translation();

                    // Transform current imu point to world imu
                    Vector3d i0_point = (Ti.rotation() * i_point) + Ti.translation();

                    Vector3d w_pts_i = i0_point;

                    geometry_msgs::Point32 p;
                    p.x = w_pts_i(0);
                    p.y = w_pts_i(1);
                    p.z = w_pts_i(2);
                    plane_cloud.points.push_back(p);
                    
                    int rgb = mask_cloud->channels[0].values[i];
                    // int rgb = pid2HEX[ppid];
                    float float_rgb = *reinterpret_cast<float*>(&rgb);
                    colors.values.push_back(float_rgb);

                    // Check if it's on border
                    Vector3d p_pt = plane2world_isos[ppid].inverse() * w_pts_i;
                    Vector4d borders = plane_seg_params[ppid];

                    if (p_pt(1) < borders(0)) // min y
                        plane_seg_params[ppid](0) = p_pt(1);
                    else if (p_pt(1) > borders(1)) // max y
                        plane_seg_params[ppid](1) = p_pt(1);
                    else if (p_pt(2) < borders(2)) // min z
                        plane_seg_params[ppid](2) = p_pt(2);
                    else if (p_pt(2) > borders(3)) // max z
                        plane_seg_params[ppid](3) = p_pt(2);
                // }
            }
        }
    }
    // publish current point cloud
    plane_cloud.channels.push_back(colors);
    pub_plane_cloud.publish(plane_cloud);

    map<int, vector<Vector3d>> plane_border_points;
    visualization_msgs::Marker line_list;

    line_list.header = plane_cloud.header;
    // line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;

    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    // LINE_LIST markers use only the x component of scale, for the line width
    line_list.scale.x = 0.5;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    sensor_msgs::PointCloud frame_cloud;
    sensor_msgs::ChannelFloat32 p_ids;

    // Compute border points for each plane
    for (map<int, Vector4d>::iterator it = plane_seg_params.begin(); it != plane_seg_params.end(); ++it){
        vector<geometry_msgs::Point> b_pts;
        
        double y_min = it->second(0);
        double y_max = it->second(1);
        double z_min = it->second(2);
        double z_max = it->second(3);

        // 0: (y_min, z_min)
        Vector3d bottom_left;
        bottom_left << 0, y_min, z_min;
        Vector3d bottom_left_w = plane2world_isos[it->first] * bottom_left;
        geometry_msgs::Point bl_pt;
        bl_pt.x = bottom_left_w(0);
        bl_pt.y = bottom_left_w(1);
        bl_pt.z = bottom_left_w(2);
        b_pts.push_back(bl_pt);

        // 1: (y_min, z_max)
        Vector3d top_left;
        top_left << 0, y_min, z_max;
        Vector3d top_left_w = plane2world_isos[it->first] * top_left;
        geometry_msgs::Point tl_pt;
        tl_pt.x = top_left_w(0);
        tl_pt.y = top_left_w(1);
        tl_pt.z = top_left_w(2);
        b_pts.push_back(tl_pt);

        // 2: (y_max, z_max)
        Vector3d top_right;
        top_right << 0, y_max, z_max;
        Vector3d top_right_w = plane2world_isos[it->first] * top_right;
        geometry_msgs::Point tr_pt;
        tr_pt.x = top_right_w(0);
        tr_pt.y = top_right_w(1);
        tr_pt.z = top_right_w(2);
        b_pts.push_back(tr_pt);

        // 3: (y_max, z_min)
        Vector3d bottom_right;
        bottom_right << 0, y_max, z_min;
        Vector3d bottom_right_w = plane2world_isos[it->first] * bottom_right;
        geometry_msgs::Point br_pt;
        br_pt.x = bottom_right_w(0);
        br_pt.y = bottom_right_w(1);
        br_pt.z = bottom_right_w(2);
        b_pts.push_back(br_pt);

        // 0 -> 1
        line_list.points.push_back(b_pts[0]);
        line_list.points.push_back(b_pts[1]);

        // 1 -> 2
        line_list.points.push_back(b_pts[1]);
        line_list.points.push_back(b_pts[2]);

        // 2 -> 3
        line_list.points.push_back(b_pts[2]);
        line_list.points.push_back(b_pts[3]);

        // 3 -> 0
        line_list.points.push_back(b_pts[3]);
        line_list.points.push_back(b_pts[0]);

        // 0: (y_min, z_min)
        geometry_msgs::Point32 bl_pt32;
        bl_pt32.x = bl_pt.x;
        bl_pt32.y = bl_pt.y;
        bl_pt32.z = bl_pt.z;
        frame_cloud.points.push_back(bl_pt32);
        p_ids.values.push_back(it->first);

        // 1: (y_min, z_max)
        geometry_msgs::Point32 tl_pt32;
        tl_pt32.x = tl_pt.x;
        tl_pt32.y = tl_pt.y;
        tl_pt32.z = tl_pt.z;
        frame_cloud.points.push_back(tl_pt32);
        p_ids.values.push_back(it->first);

        // 2: (y_max, z_max)
        geometry_msgs::Point32 tr_pt32;
        tr_pt32.x = tr_pt.x;
        tr_pt32.y = tr_pt.y;
        tr_pt32.z = tr_pt.z;
        frame_cloud.points.push_back(tr_pt32);
        p_ids.values.push_back(it->first);

        // 3: (y_max, z_min)
        geometry_msgs::Point32 br_pt32;
        br_pt32.x = br_pt.x;
        br_pt32.y = br_pt.y;
        br_pt32.z = br_pt.z;
        frame_cloud.points.push_back(br_pt32);
        p_ids.values.push_back(it->first);
    }
    frame_cloud.channels.push_back(p_ids);

    marker_pub.publish(line_list);
    frame_pub.publish(frame_cloud);
}

void draw_quad(cv::Mat &image, cv::Mat mask_image, int plane_id)
{
    cv::Mat plane_mask;

    unsigned long hex = id2color(plane_id);
    int r = ((hex >> 16) & 0xFF);
    int g = ((hex >> 8) & 0xFF);
    int b = ((hex) & 0xFF);

    cv::Scalar id_color(r, g, b);
    cv::inRange(mask_image, id_color, id_color, plane_mask);

    cv::Mat dilated;
    cv::dilate(plane_mask, dilated, cv::Mat(), cv::Point(-1, -1), 5);
    cv::erode(dilated, dilated, cv::Mat(), cv::Point(-1, -1), 7);
    ROS_INFO("Dilated the plane mask with id %d", plane_id);

    // find contours
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours( dilated, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );
    // std::cout << "largest contour has " << contours[0].size() << "points" << std::endl;
    ROS_INFO("Found %d contours", (int)contours.size());

    if (contours.size() > 0)
    {
        ROS_INFO("Largest contour has %d points", (int)contours[0].size());

        // simplify contours
        double epsilon = 0.01*cv::arcLength(contours[0], true);
        vector<cv::Point> approx_contour;
        cv::approxPolyDP(contours[0], approx_contour, epsilon, true);
        // std::cout << "simplified contour has" << approx_contour.size() << "points" << std::endl;
        ROS_INFO("Simplified contour has %d points", (int)approx_contour.size());
        
        if (approx_contour.size() >= 4) {
            // Draw the simplified contour
            vector<vector<cv::Point> > approx_contours;
            approx_contours.push_back(approx_contour);
            ROS_INFO("Drawing the contour");
            cv::drawContours(image, approx_contours, 0, id_color, cv::FILLED);
        }
    }
}

void draw_quads(cv::Mat &image, cv::Mat mask_image, vector<int> plane_ids)
{
    for (int i = 0; i < plane_ids.size(); i++)
    {
        draw_quad(image, mask_image, plane_ids[i]);
    }
}

void compute_cuboid_vertices(Vector4d plane_params, vector<Vector3d> points, sensor_msgs::PointCloud &vertex_cloud)
{
    Vector3d normal = plane_params.head<3>().normalized();
    Vector3d vertical(0, 1, 0);
    Vector3d horizontal = normal.cross(vertical);

    Vector3d min_n_pt;
    Vector3d min_h_pt;
    Vector3d min_v_pt;
    double min_n_d = 10000;
    double min_h_d = 10000;
    double min_v_d = 10000;

    Vector3d max_n_pt;
    Vector3d max_h_pt;
    Vector3d max_v_pt;
    double max_n_d = -10000;
    double max_h_d = -10000;
    double max_v_d = -10000;

    for (int i = 0; i < points.size(); i++)
    {
        Vector3d point = points[i];

        if (plane_params.dot(point.homogeneous()) > 1)
            continue;

        double nd = normal.dot(point);
        double hd = horizontal.dot(point);
        double vd = vertical.dot(point);

        if (nd < min_n_d)
            min_n_pt = point;
        else if (nd > max_n_d)
            max_n_pt = point;

        if (hd < min_h_d)
            min_h_pt = point;
        else if (hd > max_h_d)
            max_h_pt = point;

        if (vd < min_v_d)
            min_v_pt = point;
        else if (vd > max_v_d)
            max_v_pt = point;
    }

    // Compute the 8 vertices bounding points
    // pt 1: (max x, min y, max z)
    geometry_msgs::Point32 pt1;
    pt1.x = max_h_pt.x();
    pt1.y = min_v_pt.y();
    pt1.z = max_n_pt.z();
    vertex_cloud.points.push_back(pt1);
    
    // pt 2: (max x, min y, min z)
    geometry_msgs::Point32 pt2;
    pt2.x = max_h_pt.x();
    pt2.y = min_v_pt.y();
    pt2.z = min_n_pt.z();
    vertex_cloud.points.push_back(pt2);
       
    // pt 3: (min x, min y, min z)
    geometry_msgs::Point32 pt3;
    pt3.x = min_h_pt.x();
    pt3.y = min_v_pt.y();
    pt3.z = min_n_pt.z();
    vertex_cloud.points.push_back(pt3);
    
    // pt 4: (min x, min y, max z)
    geometry_msgs::Point32 pt4;
    pt4.x = min_h_pt.x();
    pt4.y = min_v_pt.y();
    pt4.z = max_n_pt.z();
    vertex_cloud.points.push_back(pt4);
    
    // // pt 5: (max x, max y, max z)
    // geometry_msgs::Point32 pt5(max_h_pt.x(), max_v_pt.y(), max_n_pt.z());
    // vertex_cloud.points.push_back(pt5);
    
    // // pt 6: (max x, max y, min z)
    // geometry_msgs::Point32 pt6(max_h_pt.x(), max_v_pt.y(), min_n_pt.z());
    // vertex_cloud.points.push_back(pt6);
    
    // // pt 7: (min x, max y, min z)
    // geometry_msgs::Point32 pt7(min_h_pt.x(), max_v_pt.y(), min_n_pt.z());
    // vertex_cloud.points.push_back(pt7);
    
    // // pt 8: (min x, max y, max z)
    // geometry_msgs::Point32 pt8(min_h_pt.x(), max_v_pt.y(), max_n_pt.z());
    // vertex_cloud.points.push_back(pt8);
}