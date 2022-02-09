#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include "OpticalFlowPropagator.h"

OpticalFlowPropagator propagator;


std::condition_variable con_frame_buf;
std::condition_variable con_propagator;

std::mutex m_frame_buf;
std::mutex m_propagator;
std::mutex m_sent_frame_id;
std::queue<Frame> frame_buf;
int current_frame_id = -1;
int sent_frame_id = -2;

Frame current_frame;


void publish_processed(const ProcessedFrame &f) {
    // TODO: publish
    ROS_INFO("Publishing frame %d", f.frame_id);
}

cv::Mat run_plannercnn(const cv::Mat img) {
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    return img;
}


// thread: propagate `processed_frame` and publish
[[noreturn]] void propagate_and_publish() {
    ROS_INFO("propagate started %d", std::this_thread::get_id());
    Frame f;
    while (true) {

        std::unique_lock<std::mutex> lk(m_propagator);
        con_propagator.wait(lk, [&] {

            m_frame_buf.lock();
            m_sent_frame_id.lock();

            while (!frame_buf.empty() and frame_buf.front().frame_id < propagator.source_frame.frame_id) {
                frame_buf.pop();
            }

            bool ok = propagator.source_frame.frame_id != -1 and !frame_buf.empty();

            ok &= ((propagator.source_frame.frame_id == sent_frame_id) or (frame_buf.front().frame_id < sent_frame_id));
            if (ok) {
                f = frame_buf.front();
                frame_buf.pop();
            }

            m_sent_frame_id.unlock();
            m_frame_buf.unlock();
            return ok;
        });


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
    ROS_INFO("PLANERCNN started %d", std::this_thread::get_id());
    Frame f;
    while (true) {
        if (f.frame_id != -1) {
            ROS_INFO("Sending frame %d", f.frame_id);

            ProcessedFrame processed_f(f);
            processed_f.plane_mask = run_plannercnn(f.img);

            ROS_INFO("Got frame %d", processed_f.frame_id);

            m_propagator.lock();
            propagator.reset(processed_f);
            m_propagator.unlock();
            con_propagator.notify_one();
        }
        std::unique_lock<std::mutex> lk(m_frame_buf);
        con_frame_buf.wait(lk, [] { return !frame_buf.empty(); });
        f = current_frame;
        lk.unlock();

        m_sent_frame_id.lock();
        sent_frame_id = f.frame_id;
        m_sent_frame_id.unlock();
        con_propagator.notify_one();

    }
}

void preprocessing_callback(
//        const sensor_msgs::PointCloudConstPtr &features_msg,
//        const nav_msgs::OdometryConstPtr &odometry_msg,
        const sensor_msgs::ImageConstPtr &img_msg
) {
    Frame f;
    f.frame_id = ++current_frame_id;
    f.pcd = nullptr;
    f.odom = nullptr;
    f.img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;

    m_frame_buf.lock();
    frame_buf.push(f);
    m_frame_buf.unlock();
    current_frame = f;
    con_frame_buf.notify_one();

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "rpvio_preprocessor");
    ros::NodeHandle n;

    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
//    message_filters::Subscriber<sensor_msgs::PointCloud> sub_point_cloud(n, "/point_cloud", 100);
//    message_filters::Subscriber<nav_msgs::Odometry> sub_odometry(n, "/odometry", 100);
    ros::Subscriber sub = n.subscribe("/image", 10, preprocessing_callback);
//    message_filters::Subscriber<sensor_msgs::Image> sub_image(n, "/image", 10);

    std::thread plannercnn_process(process), propagate_process(propagate_and_publish);
    ros::spin();
}
