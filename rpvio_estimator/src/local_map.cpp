#include "local_map.h"

LocalMap::LocalMap(sensor_msgs::PointCloudConstPtr features, nav_msgs::OdometryConstPtr odometry, sensor_msgs::ImageConstPtr mask)
{
    features_msg = features;
    odometry_msg = odometry;
    mask_msg = mask;

    process_odometry();
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
    
    world2local = Tic.inverse() * Ti.inverse();

    ROS_INFO("Computed world to local transform");
    
    // set colour id to 0
    color_index[(unsigned long)0] = 0;
}

void LocalMap::cluster_points()
{
    ROS_INFO("Point cloud has %d channels and %d points", (int)features_msg->channels.size(), (int)features_msg->points.size());

    cv_bridge::CvImagePtr mask_ptr = cv_bridge::toCvCopy(mask_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat mask_img = mask_ptr->image;
    
    // Compute world to local transform
    Isometry3d world2local = Tic.inverse() * Ti.inverse(); 

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
        //Vector3d cpoint(lpoint.x(), 0.0, lpoint.z());
        //if (cpoint.norm() > 50)
        //    continue;

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
            if (! mPlanes.count(plane_id))
            {
                Plane new_plane;
                new_plane.plane_id = plane_id;
                
                mPlanes[plane_id] = new_plane;
            }
            mPlanes[plane_id].feature_ids.insert(feature_id);

            PlaneFeature new_plane_feature;
            new_plane_feature.plane_id = plane_id;
            new_plane_feature.point = fpoint;
            mPlaneFeatures[feature_id] = new_plane_feature;
        }
    }

    ROS_INFO("Found %d planes", (int)mPlanes.size());
}

void LocalMap::publish_cuboids(ros::Publisher cuboids_pub)
{
        visualization_msgs::Marker line_list;
        line_list.header = odometry_msg->header;

        // line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;

        line_list.id = 2;
        line_list.type = visualization_msgs::Marker::LINE_LIST;

        // LINE_LIST markers use only the x component of scale, for the line width
        line_list.scale.x = 0.08;

        // Line list is green
        // if (fabs(normal[0]) > fabs(normal[2]))
        line_list.color.r = 1.0;
        // else
            // line_list.color.b = 1.0;
        line_list.color.a = 1.0;


    //For each segmented plane
    for (auto& iter_plane: mPlanes)
    {   
        int plane_id = iter_plane.first;
        
        // Compute color of this plane cluster
        unsigned long hex = id2color(plane_id);
        int r = ((hex >> 16) & 0xFF);
        int g = ((hex >> 8) & 0xFF);
        int b = ((hex) & 0xFF);

        pcl::PointCloud<pcl::PointXYZRGB> plane_pcd;
        pcl::PointCloud<pcl::PointXYZRGB> filtered_plane_pcd;
        
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
            plane_pcd.points.push_back(pt);
        }
        
        // Create the filtering object
        plane_pcd.is_dense = false;
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
        ror.setInputCloud (plane_pcd.makeShared());
        ror.setRadiusSearch (3);
        ror.setMinNeighborsInRadius (2);
        ror.setKeepOrganized (true);
        ror.filter (filtered_plane_pcd);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(filtered_plane_pcd, filtered_plane_pcd, indices);
        
        ROS_INFO("Before filtering: %d, After filtering: %d", (int)plane_pcd.points.size(), (int)filtered_plane_pcd.points.size());
        std::cout << "Before" << std::endl;
        std::cout << plane_pcd << std::endl;
        std::cout << "After" << std::endl;
        std::cout << filtered_plane_pcd << std::endl;

        if (filtered_plane_pcd.points.size() < 5)
            continue;
        
        vector<Vector3d> plane_points;
        for (int pid = 0; pid < filtered_plane_pcd.points.size(); pid++)
        {   
            pcl::PointXYZRGB pt = filtered_plane_pcd.points[pid];
            Vector3d w_pt(pt.x, pt.y, pt.z);
            Vector3d c_pt = Tic.inverse() * (Ti.inverse() * w_pt);
                
            plane_points.push_back(c_pt);
        }

        Vector4d params = fit_vertical_plane_ransac(plane_points);
        ROS_INFO("Plane params %lf, %lf, %lf, %lf", params[0], params[1], params[2], params[3]);

        vector<geometry_msgs::Point> vertices;
        fit_cuboid_to_points(params, plane_points, vertices);
        create_cuboid_frame(vertices, line_list, (Ti * Tic));
    }

    cuboids_pub.publish(line_list);
}

void LocalMap::publish_clusters(ros::Publisher clusters_pub)
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

        pcl::PointCloud<pcl::PointXYZRGB> plane_pcd;
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
            plane_pcd.points.push_back(pt);
        }

        // Create the filtering object
        plane_pcd.is_dense = false;
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
        ror.setInputCloud (plane_pcd.makeShared());
        ror.setRadiusSearch (3);
        ror.setMinNeighborsInRadius (2);
        ror.setKeepOrganized (true);
        ror.filter (plane_pcd);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(plane_pcd, plane_pcd, indices);
        
        if (plane_pcd.points.size() < 5)
            continue;
        clusters_pcd += plane_pcd;
    }

    // Convert to ROS PointCloud2 and publish
    sensor_msgs::PointCloud2 clusters_cloud;
    pcl::toROSMsg(clusters_pcd, clusters_cloud);
    clusters_cloud.header = features_msg->header;
    clusters_pub.publish(clusters_cloud);
}
