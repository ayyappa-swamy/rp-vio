#include "mapper.h"

void mapping_callback(
    const sensor_msgs::PointCloudConstPtr &features_msg,
    const nav_msgs::OdometryConstPtr &odometry_msg,
    const sensor_msgs::ImageConstPtr &img_msg,
    const sensor_msgs::ImageConstPtr &mask_msg
)
{
    // Step 1: Cluster all the feature points based on their plane ids
    cv_bridge::CvImagePtr mask_ptr = cv_bridge::toCvCopy(mask_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat mask_img = mask_ptr->image;
    
    cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = img_ptr->image;
    ROS_INFO("Received point cloud with %d points", (int)features_msg->points.size());
    ROS_INFO("Received image with dimensions (%d, %d)", img.rows, img.cols);

    map<int, vector<Vector3d>> points_map = cluster_plane_features(features_msg, mask_img);
    ROS_INFO("Clustered the feature points based on %d planes", (int)points_map.size());

    sensor_msgs::PointCloud frame_cloud;
    frame_cloud.header = features_msg->header;
    sensor_msgs::PointCloud cent_cloud;
    cent_cloud.header = features_msg->header;

    vector<int> plane_ids;

    Isometry3d Tic;
    Tic.linear() = RIC[0];
    Tic.translation() = TIC[0];

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

    visualization_msgs::MarkerArray ma;

    Vector3d vertical(0, 1, 0);

    if (points_map.size() == 0)
        return;
        
    // ROS_INFO("Drawing quads for %d planes", plane_ids.size());
    // draw_quads(img, mask_img, plane_ids);
    // cv::Mat gray_img;
    // cv::cvtColor(img, gray_img, CV_BGR2GRAY);
    vector<Vector3d> vps;
    map<int, Vector3d> normals_map = draw_vp_lines(img, mask_img, vps);    

    // Print number of features per plane
    for (auto const& fpp: points_map)
    {
        ROS_INFO("Number of features in plane id %d are %d", fpp.first, (int)fpp.second.size());

        vector<Vector3d> plane_points;
        for (int i = 0; i < (int)fpp.second.size(); i++)
        {
            Vector3d c_pt = Tic.inverse() * (Ti.inverse() * fpp.second[i]);

            // if (c_pt.norm() <= 20)
                plane_points.push_back(c_pt);
        }

        // if (plane_points.size() < 5)
        //     continue;

        MatrixXd pts_mat(plane_points.size(), 4);
        vector<geometry_msgs::Point> vertices;

        Vector3d centroid(0.0, 0.0, 0.0);
        Vector3d normal = normals_map[fpp.first];

        double d = 0.0;

        for (int i = 0; i < (int)plane_points.size(); i++)
        {
            Vector3d c_pt = plane_points[i];

            geometry_msgs::Point32 p;
            p.x = (double)c_pt[0];
            p.y = (double)c_pt[1];
            p.z = (double)c_pt[2];

            centroid += c_pt;
            frame_cloud.points.push_back(p);

            Vector3d pt_ = c_pt;
            // pt_[2] = 1.0;

            d += normal.dot(c_pt);

            pts_mat.row(i) = pt_.homogeneous().transpose();
        }

        centroid /= plane_points.size();
        geometry_msgs::Point32 c;
        c.x = (double)centroid[0];
        c.y = (double)centroid[1];
        c.z = (double)centroid[2];
        cent_cloud.points.push_back(c);
        
        d /= plane_points.size();

        // find svd
        Vector4d params;
        params << normal[0], normal[1], normal[2], d;
        // Eigen::JacobiSVD<MatrixXd> pt_svd(pts_mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
        // params = pt_svd.matrixV().col(pt_svd.matrixV().cols() - 1);
        
        // ROS_INFO("Plane params are : %g, %g, %g, %g", params[0], params[1], params[2], params[3]);
        // Vector3d normal = params.head<3>();
        // // normal[2] = 0.0;

        Vector3d plane_dir = (normal.cross(vps[1])).normalized();

        if ((normal.norm() > 0.001) && (params[3] > 0.001))
        {
            // double d = params[3] / normal.norm();
            // normal.normalize();

            // // Find nearest point to origin
            // double lambda = -d / powf(normal.norm(), 2.0);

            // Vector3d point = lambda * normal;

            // Vector4d normed_params(normal[0], normal[1], normal[2], d);

            // if (fabs(normal.dot(vertical)) < 0.5)
            // {
            //     // ROS_INFO("Normalized plane params are : %g, %g, %g, %g", normal[0], normal[1], normal[2], d);
            //     // ROS_INFO("Nearest point is : %g, %g, %g", point[0], point[1], point[2]);
                
            //     if (fit_cuboid_to_point_cloud(normed_params, plane_points, vertices))
            //         create_cuboid_frame(vertices, line_list);
            // }

            visualization_msgs::Marker line_list;
            line_list.header = features_msg->header;

            // line_list.action = visualization_msgs::Marker::ADD;
            line_list.pose.orientation.w = 1.0;

            line_list.id = fpp.first;
            line_list.type = visualization_msgs::Marker::LINE_LIST;

            // LINE_LIST markers use only the x component of scale, for the line width
            line_list.scale.x = 0.1;

            // Line list is green
            if (normal[0] > normal[2])
                line_list.color.r = 1.0;
            else
                line_list.color.b = 1.0;
            line_list.color.a = 1.0;

            create_centroid_frame(centroid, plane_dir, vps[1], line_list);
            ma.markers.push_back(line_list);
        }
        
        plane_ids.push_back(fpp.first);
    }

    ROS_INFO("Publising marked image");
    std_msgs::Header img_header;
    img_header = img_msg->header;
    sensor_msgs::ImagePtr marked_image_msg = cv_bridge::CvImage(img_header, sensor_msgs::image_encodings::BGR8, img).toImageMsg();
    
    // Publish raw images with marked quads
    masked_im_pub.publish(marked_image_msg);

    // Process a particular plane id
    frame_pub.publish(frame_cloud);
    cent_pub.publish(cent_cloud);
    sensor_msgs::PointCloud2 frame_cloud2;
    sensor_msgs::convertPointCloudToPointCloud2(frame_cloud, frame_cloud2);
    frame_pub2.publish(frame_cloud2);
    ma_pub.publish(ma);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_mapper");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);
    load_color_palette(COLOR_PALETTE_PATH);

    // Register all subscribers
    message_filters::Subscriber<sensor_msgs::PointCloud> sub_point_cloud(n, "/vins_estimator/point_cloud", 1000);
    message_filters::Subscriber<nav_msgs::Odometry> sub_odometry(n, "/vins_estimator/odometry", 1000);
    message_filters::Subscriber<sensor_msgs::Image> sub_image(n, "/image", 100);
    message_filters::Subscriber<sensor_msgs::Image> sub_mask(n, "/mask", 100);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), sub_point_cloud, sub_odometry, sub_image, sub_mask);

    // message_filters::TimeSynchronizer<sensor_msgs::PointCloud, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image> sync(
    //     sub_point_cloud,
    //     sub_odometry,
    //     sub_image,
    //     sub_mask,
    //     2000
    // );
    sync.registerCallback(boost::bind(&mapping_callback, _1, _2, _3, _4));

    // Register all publishers
    // Publish coloured point cloud
    // Publish 3D plane segments (line list or marker array)

    frame_pub = n.advertise<sensor_msgs::PointCloud>("frame_cloud", 100);
    cent_pub = n.advertise<sensor_msgs::PointCloud>("centroid_cloud", 100);
    frame_pub2 = n.advertise<sensor_msgs::PointCloud2>("frame_cloud2", 100);
    masked_im_pub = n.advertise<sensor_msgs::Image>("masked_image", 10);
    marker_pub = n.advertise<visualization_msgs::Marker>("cuboids", 100);
    ma_pub = n.advertise<visualization_msgs::MarkerArray>("centroid_segs", 100);
    
    ros::spin();
    
    return 0;
}
