//
// Created by user on 2/9/22.
//

#ifndef RPVIO_ESTIMATOR_OPTICALFLOWPROPAGATOR_H
#define RPVIO_ESTIMATOR_OPTICALFLOWPROPAGATOR_H

#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>

struct Frame {
    sensor_msgs::PointCloudConstPtr pcd = nullptr;
    nav_msgs::OdometryConstPtr odom = nullptr;
    cv::Mat img;
    int frame_id = -1;

    Frame() {
        this->frame_id = -1;
    }
};

struct ProcessedFrame : Frame {
    cv::Mat plane_mask;
};

class OpticalFlowPropagator {
public:
    OpticalFlowPropagator();

    ~OpticalFlowPropagator();

    ProcessedFrame source_frame;

    void reset(const ProcessedFrame &f);

    cv::Mat propagate_farneback(const cv::Mat &rgb_img);

};


#endif //RPVIO_ESTIMATOR_OPTICALFLOWPROPAGATOR_H
