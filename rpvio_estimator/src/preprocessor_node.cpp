#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>


#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include "OpticalFlowPropagator.h"

OpticalFlowPropagator propagator;


std::condition_variable con_frame_buf;
std::condition_variable con_propagator;

std::mutex m_frame_buf;
std::mutex m_propagator;
std::queue <Frame> frame_buf;
int current_frame_id = 0;


int processing_frame_id;

void publish_processed(const ProcessedFrame &f) {
    // TODO: publish
}


// thread: propagate `processed_frame` and publish
[[noreturn]] void propagate_and_publish() {
    Frame f;
    while (true) {
        m_frame_buf.lock();
        std::unique_lock <std::mutex> lk(m_propagator);
        con_propagator.wait(lk, [] { return propagator.source_frame.frame_id != -1 and !frame_buf.empty(); });

        while (!frame_buf.empty() and frame_buf.front().frame_id < propagator.source_frame.frame_id) {
            frame_buf.pop();
        }

        if (!frame_buf.empty()) {
            f = frame_buf.front();
            frame_buf.pop();
            m_frame_buf.unlock();
        } else {
            lk.unlock();
            m_frame_buf.unlock();
            continue;
        }

        ProcessedFrame processed_f;
        if (f.frame_id == propagator.source_frame.frame_id) {
            processed_f = propagator.source_frame;
        } else {
            processed_f.frame_id = f.frame_id;
            processed_f.img = f.img;
            processed_f.odom = f.odom;
            processed_f.pcd = f.pcd;
            processed_f.plane_mask = propagator.propagate_farneback(f.img);
        }
        lk.unlock();

        publish_processed(processed_f);
    }

}


// thread: process frames
[[noreturn]] void process() {
    while (true) {
        Frame f;
        std::unique_lock <std::mutex> lk(m_frame_buf);
        con_frame_buf.wait(lk, [] { return !frame_buf.empty(); });
        f = frame_buf.front();
        lk.unlock();

        ProcessedFrame processed_f;
        // TODO: run plannercnn on f and save it to processed_f

        m_propagator.lock();
        propagator.reset(processed_f);
        m_propagator.unlock();
        con_propagator.notify_one();
    }
}

void mapping_callback(
        const sensor_msgs::PointCloudConstPtr &features_msg,
        const nav_msgs::OdometryConstPtr &odometry_msg,
        const sensor_msgs::ImageConstPtr &img_msg
) {
    Frame f;
    f.frame_id = ++current_frame_id;
    f.pcd = features_msg;
    f.odom = odometry_msg;
    f.img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;

    m_frame_buf.lock();
    frame_buf.push(f);
    m_frame_buf.unlock();
    con_frame_buf.notify_one();

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "rpvio_preprocessor");


}
