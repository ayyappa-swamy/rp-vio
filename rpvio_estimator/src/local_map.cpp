#include "local_map.h"

LocalMap::LocalMap(sensor_msgs::PointCloud features, nav_msgs::Odometry odometry, sensor_msgs::Image mask)
{
    features_msg = features;
    odometry_msg = odometry;
    mask_msg = mask;

    ROS_INFO("Received features, odometry and mask");
}

void LocalMap::process_odometry()
{
    Tic.linear() = RIC[0];
    Tic.translation() = TIC[0];
    ROS_INFO("Computed imu-camera transform");

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

    Ti.linear() = quat.normalized().toRotationMatrix();
    Ti.translation() = trans;

    ROS_INFO("Computed world to local transform");
}

void LocalMap::cluster_points()
{
    ROS_INFO("Point cloud has %d channels", (int)features_msg->channels.size());

    cv_bridge::CvImagePtr mask_ptr = cv_bridge::toCvCopy(mask_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat mask_img = mask_ptr->image;

    for (int fid = 0; fid < features_msg->points.size(); fid++)
    {
        int feature_id = (int)features_msg->channels[2].values[fid];

        // Compute plane id
        Vector3d fpoint;
        geometry_msgs::Point32 p = features_msg->points[fid];
        fpoint << p.x, p.y, p.z;

        int u = (int)features_msg->channels[0].values[fid];
        int v = (int)features_msg->channels[1].values[fid];

        Vector3d lpoint = world2local * fpoint;
        Eigen::Matrix3d K;
        K << FOCAL_LENGTH, 0, COL/2,
            0, FOCAL_LENGTH, ROW/2,
            0, 0, 1;

        Vector3d pt = K * lpoint;
        pt /= pt[2];

        u = (int)pt.x();
        v = (int)pt.y();
        
        int plane_id = get_plane_id(u, v, mask_img);

        if ((plane_id != 0) && (plane_id != 39))// Ignore sky and ground points
        {
            if (! mPlaneFeatureIds.count(plane_id))
            {
                Plane new_plane;
                new_plane.plane_id = plane_id;
                
                mPlanes[plane_id] = new_plane;
            }
            mPlaneFeatures[plane_id].feature_ids.insert(feature_id);

            PlaneFeature new_plane_feature;
            new_plane_feature.plane_id = plane_id;
            mPlaneFeatures[feature_id] = new_plane_feature;
        }
    }

    ROS_INFO("Found %d planes", (int)mPlanes.size());
}