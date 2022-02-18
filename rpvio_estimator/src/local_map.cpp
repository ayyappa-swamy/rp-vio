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
            if (! mPlaneFeatures.count(plane_id))
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

void LocalMap::fit_cuboids()
{
    for (auto& iter_plane: mPlanes)
    {   
        int plane_id = iter_plane.first;
        ROS_INFO("Number of features in plane id %d are %d", iter_plane.first, (int)iter_plane.second.feature_ids.size());

        vector<Vector3d> plane_points;
        for (auto feature_id: iter_plane.second.feature_ids)
        {
            Vector3d w_pt = mPlaneFeatures[feature_id].point;
            Vector3d c_pt = Tic.inverse() * (Ti.inverse() * w_pt);
            
            plane_points.push_back(c_pt);
        }

        Vector3d normal;
        Vector4d params;
        params = fit_vertical_plane_ransac(plane_points, plane_id);
        normal = params.head<3>();

        fit_cuboid_to_points(params, plane_points, cuboid_vertices);
    }
}

void publish_clusters(ros::Publisher clusters_pub)
{
    pcl::PointCloud<pcl::PointXYZRGB> clusters_pcd;
    
    // For each segmented plane
    for (auto& iter_plane: mPlanes)
    {   
        int plane_id = iter_plane.first;
        ROS_INFO("Number of features in plane id %d are %d", iter_plane.first, (int)iter_plane.second.feature_ids.size());
        
        // Compute color of this plane cluster
        unsigned long hex = id2color(plane_id);
        int r = ((hex >> 16) & 0xFF);
        int g = ((hex >> 8) & 0xFF);
        int b = ((hex) & 0xFF);

        // Create a coloured point cloud
        for (auto feature_id: iter_plane.second.feature_ids)
        {
            Vector3d w_pt = mPlaneFeatures[feature_id].point;
            pcl::PointXYZRGB pt;
            pt.x = w_pt.x();
            pt.y = w_pt.y();
            pt.z = w_pt.z();
            pt.r = b;
            pt.g = g;
            pt.b = r;
            clusters_pcd.points.push_back(pt);
        }
    }

    // Convert to ROS PointCloud2 and publish
    sensor_msgs::PointCloud2 clusters_cloud;
    pcl::toROSMsg(clusters_pcd, clusters_cloud);
    clusters_cloud.header = features_msg->header;
    clusters_pub.publish(clusters_cloud);
}