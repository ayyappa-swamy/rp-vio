#include "mapper.h"

sensor_msgs::PointField getFieldWithName(string name)
{
    sensor_msgs::PointField pf;
    pf.name = name;

    return pf;
}

void process_messages()
{
    while(true)
    {
        unique_lock<mutex> lck{sm_mutex};
        sm_cond.wait(lck, []{ return !sm_queue.empty();});

        SubMessages sub_msg;

        sub_msg = sm_queue.front();
        sm_queue.pop();
        lck.unlock();

        sensor_msgs::PointCloudConstPtr features_msg = sub_msg.features_msg;
        nav_msgs::OdometryConstPtr odometry_msg = sub_msg.odometry_msg;
        sensor_msgs::ImageConstPtr mask_msg = sub_msg.mask_msg;

         
        Vector3d ggoal(25.0, -5.0, 2.5); 

        //ROS_INFO("Image message timestamp %f", img_msg->header.stamp.toSec());
        //ROS_INFO("Features message timestamp %f", features_msg->header.stamp.toSec());
        //ROS_INFO("Mask message timestamp %f\n", mask_msg->header.stamp.toSec());

        // Step 1: Cluster all the feature points based on their plane ids
        cv_bridge::CvImagePtr mask_ptr = cv_bridge::toCvCopy(mask_msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat raw_mask_img = mask_ptr->image;
        
        // cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        // cv::Mat img = img_ptr->image;
        ROS_INFO("Received point cloud with %d points", (int)features_msg->points.size());
        //ROS_INFO("Received image with dimensions (%d, %d)", img.rows, img.cols);

        sensor_msgs::PointCloud frame_cloud;
        frame_cloud.header = features_msg->header;
        sensor_msgs::ChannelFloat32 plane_id_ch;
        sensor_msgs::PointCloud2 test_cloud;
        sensor_msgs::PointCloud2 test_cloud2;
        pcl::PointCloud<pcl::PointXYZRGB> test_pcd;
        pcl::PointCloud<pcl::PointXYZRGB> test_pcd2;

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

        // cv::Mat mask_img = processMaskSegments(raw_mask_img);
        cv::Mat mask_img = raw_mask_img;
        update_global_point_cloud(features_msg, mask_img, Tic.inverse() * Ti.inverse());

        ROS_INFO("Clustered the feature points based on %d planes", (int)mPlaneFeatureIds.size());

        Vector3d lgoal = Tic.inverse() * (Ti.inverse() * ggoal);
        lgoal[1] = 0.0;
        sensor_msgs::PointCloud goal_pcd;

        visualization_msgs::MarkerArray ma;

        Vector3d vertical(0, 1, 0);

        if (mPlaneFeatureIds.size() == 0)
            return;
            
        // ROS_INFO("Drawing quads for %d planes", plane_ids.size());
        // draw_quads(img, mask_img, plane_ids);
        // cv::Mat gray_img;
        // cv::cvtColor(img, gray_img, CV_BGR2GRAY);

        // vector<Vector3d> vps;
        // vector<Vector3d> vp_normals1;
        // map<int, Vector3d> normals_map = draw_vp_lines(img, mask_img, vps, vp_normals1);

        vector<Vector3d> vp_normals;
        // Vector3d x_axis(1, 0, 0);
        // Vector3d y_axis(0, 1, 0);
        // Vector3d lx_axis = (Tic.linear().inverse() * (Ti.linear().inverse() * y_axis).normalized()).normalized();
        // Vector3d ly_axis = lx_axis.cross(vertical).normalized();
        // vp_normals.push_back(lx_axis);
        // vp_normals.push_back(ly_axis);

        // write_normal_error(vp_normals1, vp_normals);

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

        auto tpcl_start = std::chrono::high_resolution_clock::now();
        // Print number of features per plane
        for (auto& sFeatureIds: mPlaneFeatureIds)
        {   
            int plane_id = sFeatureIds.first;
            ROS_INFO("Number of features in plane id %d are %d", sFeatureIds.first, (int)sFeatureIds.second.feature_ids.size());

            vector<Vector3d> plane_points;
            
            pcl::PointCloud<pcl::PointXYZRGB> plane_pcd;
            pcl::PointCloud<pcl::PointXYZRGB> fplane_pcd;
            for (auto feature_id: sFeatureIds.second.feature_ids)
            {
                Vector3d w_pt = mFeatures[feature_id].point;

                Vector3d c_pt = Tic.inverse() * (Ti.inverse() * w_pt);
                
                Vector3d t_pt(c_pt[0], 0.0, c_pt[2]);
                if (mFeatures[feature_id].measurement_count >= 0)
                {
                    // unsigned long hex = id2color(plane_id);
                    int r = 255;//((hex >> 16) & 0xFF);
                    int g = 255;//((hex >> 8) & 0xFF);
                    int b = 0;//((hex) & 0xFF);

                    pcl::PointXYZRGB pt;
                    pt.x = w_pt.x();
                    pt.y = w_pt.y();
                    pt.z = w_pt.z();
                    pt.r = b;
                    pt.g = g;
                    pt.b = r;
                    plane_pcd.points.push_back(pt);
                }
            }

            // Create the filtering object
            pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
            ror.setInputCloud (plane_pcd.makeShared());
            ror.setRadiusSearch (2);
            ror.setMinNeighborsInRadius (5);
            ror.setKeepOrganized (true);
            ror.filter (fplane_pcd);
            
            std::cout << "Before filtering: " << std::endl;
            std::cout << plane_pcd << endl;
            ROS_INFO("Before filtering: %d; After filterig: %d", (int)plane_pcd.points.size(), (int)fplane_pcd.points.size());
            std::cout << "After filtering: " << std::endl;
            std::cout << fplane_pcd << endl;

            if (plane_pcd.points.size() < 5)
                continue;

            for (int pid = 0; pid < plane_pcd.points.size(); pid++)
            {   
                pcl::PointXYZRGB pt = plane_pcd.points[pid];
                Vector3d w_pt(pt.x, pt.y, pt.z);

                Vector3d c_pt = Tic.inverse() * (Ti.inverse() * w_pt);
                
                // Vector3d t_pt(c_pt[0], 0.0, c_pt[2]);
                // if ((t_pt.norm() <= 35) && (c_pt.norm() > 2) && (mFeatures[feature_id].measurement_count > 5))
                // {
                    plane_points.push_back(c_pt);
                // }
            }

            test_pcd += plane_pcd;

            MatrixXd pts_mat(plane_points.size(), 4);
            vector<geometry_msgs::Point> vertices;

            Vector3d normal;// = normals_map[plane_id];

            // double d = 0.0;
            // for (int i = 0; i < (int)plane_points.size(); i++)
            // {
            //     Vector3d c_pt = plane_points[i];

            //     geometry_msgs::Point32 p;
            //     p.x = (double)c_pt[0];
            //     p.y = (double)c_pt[1];
            //     p.z = (double)c_pt[2];

            //     Vector3d pt_ = c_pt;
            //     // pt_[2] = 1.0;

            //     d += -normal.dot(c_pt);

            //     pts_mat.row(i) = pt_.homogeneous().transpose();
            // }
            
            // d /= plane_points.size();

            Vector4d params;
            // params << normal[0], normal[1], normal[2], d;

            params = fit_vertical_plane_ransac(plane_points, plane_id);
            normal = params.head<3>();

            if ((normal.norm() > 0.001) && (fabs(params[3]) > 0.001))
            {
                Vector4d normed_params(normal[0], normal[1], normal[2], params[3]);

                if (fabs(normal.dot(vertical)) < 0.5)
                {             
                    if (fit_cuboid_to_point_cloud(normed_params, plane_points, vertices, vp_normals))
                    {
                        create_cuboid_frame(vertices, line_list, (Ti * Tic));
                        //write_estimated_normal_error(normal , vp_normals /*ground truth*/);

                        for (int vid = 0; vid < vertices.size(); vid++)
                        {
                            frame_cloud.points.push_back(pointToPoint32(vertices[vid]));
                            plane_id_ch.values.push_back(plane_id);
                        }
                    }
                    else {
                        ROS_INFO("Not plotting cuboid %d with params %f %f %f %f", plane_id, normed_params[0], normed_params[1], normed_params[2], normed_params[3]); 
                    }
                }
            }
            
            plane_ids.push_back(plane_id);
        }

        marker_pub.publish(line_list);

        pcl::toROSMsg(test_pcd, test_cloud);
        test_cloud.header = features_msg->header;
        frame_pub2.publish(test_cloud);

        ROS_INFO("Publising marked image");
        std_msgs::Header img_header;
        img_header = mask_msg->header;
        sensor_msgs::ImagePtr marked_image_msg = cv_bridge::CvImage(img_header, sensor_msgs::image_encodings::BGR8, mask_img).toImageMsg();
        
        // Publish raw images with marked quads
        masked_im_pub.publish(marked_image_msg);

        // Process a particular plane id
        frame_cloud.channels.push_back(plane_id_ch);
        frame_pub.publish(frame_cloud);
        // sensor_msgs::PointCloud2 frame_cloud2;
        // sensor_msgs::convertPointCloudToPointCloud2(frame_cloud, frame_cloud2);
        // frame_pub2.publish(frame_cloud2);
        // ma_pub.publish(ma);
        
        goal_pcd.header = features_msg->header;
        goal_pcd.points.push_back(toGeomPoint32(lgoal));
        lgoal_pub.publish(goal_pcd);

            auto tpcl_end = std::chrono::high_resolution_clock::now();
            double elapsed_timepcl_ms = std::chrono::duration<double, std::milli>(tpcl_end-tpcl_start).count();
            ROS_INFO("time taken for mapping process is %g ms", elapsed_timepcl_ms);
    }
}

void mapping_callback(
    const sensor_msgs::PointCloudConstPtr &features_msg,
    const nav_msgs::OdometryConstPtr &odometry_msg,
    // const sensor_msgs::ImageConstPtr &img_msg,
    const sensor_msgs::ImageConstPtr &mask_msg
)
{
    SubMessages sub_msgs;
    sub_msgs.features_msg = features_msg;
    sub_msgs.odometry_msg = odometry_msg;
    // sub_msgs.img_msg = img_msg;
    sub_msgs.mask_msg = mask_msg;

    unique_lock<mutex> lck{sm_mutex};
    // sm_mutex.lock();
    sm_queue.push(sub_msgs);
    // sm_mutex.unlock();
    lck.unlock();
    sm_cond.notify_one();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_mapper");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);
    load_color_palette(COLOR_PALETTE_PATH);

    // Register all subscribers
    message_filters::Subscriber<sensor_msgs::PointCloud> sub_point_cloud(n, MAPPER_POINT_CLOUD_TOPIC, 5);
    message_filters::Subscriber<nav_msgs::Odometry> sub_odometry(n, MAPPER_ODOMETRY_TOPIC, 5);
    message_filters::Subscriber<sensor_msgs::Image> sub_mask(n, MAPPER_MASK_TOPIC, 5);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud, nav_msgs::Odometry, sensor_msgs::Image> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), sub_point_cloud, sub_odometry, sub_mask);

    // message_filters::TimeSynchronizer<sensor_msgs::PointCloud, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image> sync(
    //     sub_point_cloud,
    //     sub_odometry,
    //     sub_image,
    //     sub_mask,
    //     2000
    // );
    sync.registerCallback(boost::bind(&mapping_callback, _1, _2, _3));

    // Register all publishers
    // Publish coloured point cloud
    // Publish 3D plane segments (line list or marker array)

    frame_pub = n.advertise<sensor_msgs::PointCloud>("frame_cloud", 5);
    // cent_pub = n.advertise<sensor_msgs::PointCloud>("centroid_cloud", 5);
    lgoal_pub = n.advertise<sensor_msgs::PointCloud>("local_goal", 5);
    frame_pub2 = n.advertise<sensor_msgs::PointCloud2>("frame_cloud2", 5);
    masked_im_pub = n.advertise<sensor_msgs::Image>("masked_image", 5);
    marker_pub = n.advertise<visualization_msgs::Marker>("cuboids", 5);
    // ma_pub = n.advertise<visualization_msgs::MarkerArray>("centroid_segs", 100);
    
    thread processing_thread{process_messages};
    ros::spin();    
}