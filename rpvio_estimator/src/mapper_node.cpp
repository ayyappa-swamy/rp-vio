#include "mapper.h"

map<int, Vector4d> plane_params;
vector<sensor_msgs::PointCloudConstPtr> mask_clouds;
vector<nav_msgs::OdometryConstPtr> odometry_msgs;
map<int, vector<Vector3d>> reg_points;

void optimize_plane_params(
    map<int, Vector4d> &plane_params
) {
    // plane ids: 91, 39, 175, 130
    /**
     * Layout of scene with plane ids
     *       ______91______
     *      /              \ 
     *     |   162(floor)   |
     *    39     66(ceil)   175
     *     |                |
     *      \______130_____/
     **/
    vector<pair<int, int>> PARALLEL_PAIRS;
    vector<pair<int, int>> ORTHO_PAIRS;

    PARALLEL_PAIRS.push_back(make_pair(91, 130));
    PARALLEL_PAIRS.push_back(make_pair(39, 75));

    ORTHO_PAIRS.push_back(make_pair(91, 39));
    ORTHO_PAIRS.push_back(make_pair(91, 175));
    ORTHO_PAIRS.push_back(make_pair(39, 130));
    ORTHO_PAIRS.push_back(make_pair(130, 175));

    vector<int> cur_pids;
    for (map<int, Vector4d>::iterator it = plane_params.begin(); it != plane_params.end(); ++it){
        cur_pids.push_back(it->first);
    }

    ceres::Problem problem;

    // for(auto &p : plane_params)
    // {
    //     ceres::LocalParameterization *local_n_parameterization = new ceres::AutoDiffLocalParameterization<UnitNormalParameterization, 2, 2>;
    //     problem.AddParameterBlock(p.second.head<2>().data(), 2, local_n_parameterization);
    // }

    for(auto &p : plane_params)
    {
        ceres::LocalParameterization *local_n_parameterization = new ceres::EigenQuaternionParameterization();
        problem.AddParameterBlock(p.second.data(), 4, local_n_parameterization);
    }

    // Vector4d z_axis;
    // z_axis << 0, 0, 1, 0;
    
    // // ceres::LocalParameterization *local_n_parameterization = new ceres::HomogeneousVectorParameterization(3);
    // problem.AddParameterBlock(z_axis.data(), 4, new ceres::EigenQuaternionParameterization());
    // problem.SetParameterBlockConstant(z_axis.data());

    // for(auto &p : plane_params)
    // {
    //     problem.AddResidualBlock(
    //         new ceres::AutoDiffCostFunction<OrthogonalConstraint, 1, 4, 4>(
    //             new OrthogonalConstraint()
    //         ),
    //         NULL,
    //         p.second.data(),     
    //         z_axis.data()
    //     );        
    // }
    // for(auto &p : plane_params)
    // {
    //     problem.AddResidualBlock(
    //         new ceres::AutoDiffCostFunction<VerticalPlaneConstraint, 1, 4>(
    //             new VerticalPlaneConstraint()
    //         ),
    //         NULL,
    //         p.second.data()
    //     );        
    // }

    // Add Plane measurement constraint
    // for(auto &p: plane_measurements)
    // {
    //     for (int mid = 1; mid < p.second.size(); mid++) {
    //         Vector4d v = p.second[mid];
    //         v.normalize();
    //         Quaterniond quat(v[3], v[0], v[1], v[2]);
    //         problem.AddResidualBlock(
    //             new ceres::AutoDiffCostFunction<PlaneMeasurementConstraint, 1, 4>(
    //                 new PlaneMeasurementConstraint(quat)
    //             ),
    //             NULL,
    //             plane_params[p.first].data()
    //         );
    //     }
    // }

    // for (int opair_idx = 0; opair_idx < ORTHO_PAIRS.size(); opair_idx++) {
    //     int plane_id1 = ORTHO_PAIRS[opair_idx].first;
    //     int plane_id2 = ORTHO_PAIRS[opair_idx].second;
        
    //     if (
    //         (find(cur_pids.begin(), cur_pids.end(), plane_id1) != cur_pids.end()) &&
    //         (find(cur_pids.begin(), cur_pids.end(), plane_id2) != cur_pids.end())
    //     ) {
    //         // Add Orthogonal Constraint
    //         problem.AddResidualBlock(
    //             new ceres::AutoDiffCostFunction<OrthogonalConstraintQuat, 1, 4, 4>(
    //                 new OrthogonalConstraintQuat()
    //             ),
    //             NULL,
    //             plane_params[plane_id1].data(),     
    //             plane_params[plane_id2].data()
    //         );
    //     }
    // }

    // for (int ppair_idx = 0; ppair_idx < PARALLEL_PAIRS.size(); ppair_idx++) {
    //     int plane_id1 = PARALLEL_PAIRS[ppair_idx].first;
    //     int plane_id2 = PARALLEL_PAIRS[ppair_idx].second;
        
    //     if (
    //         (find(cur_pids.begin(), cur_pids.end(), plane_id1) != cur_pids.end()) &&
    //         (find(cur_pids.begin(), cur_pids.end(), plane_id2) != cur_pids.end())
    //     ) {
    //         // ceres::LocalParameterization *local_n_parameterization1 = new ceres::HomogeneousVectorParameterization(4);
    //         // ceres::LocalParameterization *local_n_parameterization2 = new ceres::HomogeneousVectorParameterization(4);
    //         // problem.AddParameterBlock(plane_params[plane_id1].data(), 3, local_n_parameterization1);
    //         // problem.AddParameterBlock(plane_params[plane_id2].data(), 3, local_n_parameterization2);
            
    //         // Add Parallel Constraint
    //         problem.AddResidualBlock(
    //             new ceres::AutoDiffCostFunction<ParallelConstraint, 3, 4, 4>(
    //                 new ParallelConstraint()
    //             ),
    //             NULL,
    //             plane_params[plane_id1].data(),     
    //             plane_params[plane_id2].data()
    //         );
    //     }
    // }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 100;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;
}

double z_min = 0;

void history_callback(
    const sensor_msgs::PointCloudConstPtr &features_msg
)
{
    // map<int, vector<Vector3d>> reg_points;
    visualization_msgs::Marker line_list;

    // Loop through all feature points
    for(int fi = 0; fi < features_msg->points.size(); fi++) {
        Vector3d fpoint;
        geometry_msgs::Point32 p = features_msg->points[fi];
        fpoint << p.x, p.y, p.z;

        int plane_id = (int)features_msg->channels[0].values[fi];

        reg_points[plane_id].push_back(fpoint);
    }


    line_list.header = features_msg->header;
    // line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;

    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    // LINE_LIST markers use only the x component of scale, for the line width
    line_list.scale.x = 0.5;

    // Line list is green
    line_list.color.g = 1.0;
    line_list.color.a = 1.0;

    sensor_msgs::PointCloud frame_cloud;
    sensor_msgs::ChannelFloat32 p_ids;

    // Fit planes ==> create map planeid vs. params
    for (auto const& fpp: reg_points) {
        MatrixXd pts_mat(fpp.second.size(), 3);
        MatrixXd plane_residuals(fpp.second.size(), 1);
    
        for (int pti = 0; pti < fpp.second.size(); pti++) {
            pts_mat.row(pti) = fpp.second[pti].normalized().transpose();
        }

        // find svd
        Vector3d params;
        Eigen::JacobiSVD<MatrixXd> pt_svd(pts_mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
        params = pt_svd.matrixV().col(pt_svd.matrixV().cols() - 1);
        params.normalize();

        // Vector4d info = covariance_matrix(pts_mat).diagonal().tail(4);
        Vector3d pn;
        pn << params(0), params(1), 0;
        double mag = pn.norm();
        pn.normalize();
        
        Vector4d params_;
        params_ << pn(0), pn(1), pn(2), params(2)/mag;

        Vector3d params_dir;
        params_dir << params_(0), params_(1), params_(3);
        params_dir.normalize();

        plane_residuals = (pts_mat * params_dir).col(0);
        double variance = 0.0;

        for (int pti = 0; pti < fpp.second.size(); pti++) {
            if (fabs(plane_residuals(pti)) > variance)
                variance = fabs(plane_residuals(pti));
        }

        Vector3d point_sum = Vector3d::Zero();
        
        double z_max = -1000;

        for (int pti = 0; pti < fpp.second.size(); pti++) {
            // if (fpp.second[pti].norm() > 200)
            //     continue;
            
            point_sum += fpp.second[pti];

            // if (fpp.second[pti].z() < z_min)
            //     z_min = fpp.second[pti].z();

            if (fpp.second[pti].z() > z_max)
                z_max = fpp.second[pti].z();
        }
        
        // Find mid point
        Vector3d mid_point = point_sum / fpp.second.size();
        ROS_INFO("variance for mid_point (%d): (%f, %f, %f) is %f", fpp.first, mid_point.x(), mid_point.y(), mid_point.z(), variance);

        Vector3d far_point = mid_point;
        double max_distance = 0.0;
        // Find the farthest point from mid_point on ground
        for (int pti = 0; pti < fpp.second.size(); pti++) {
            Vector3d cur_point = fpp.second[pti];
            double cur_distance = (cur_point.head<2>() - mid_point.head<2>()).norm();

            if ((cur_distance > max_distance) && (cur_distance < 100)) {
                far_point = cur_point;
                max_distance = cur_distance;
            }
        }
        ROS_INFO("far_point (%d): (%f, %f, %f)", fpp.first, far_point.x(), far_point.y(), far_point.z());

        Vector3d x_axis(1, 0, 0);
        Vector3d y_axis(0, 1, 0);

        double x_angle = fabs((mid_point - far_point).normalized().dot(x_axis));
        double y_angle = fabs((mid_point - far_point).normalized().dot(y_axis));

        Vector3d dir = (x_angle > y_angle) ? x_axis : y_axis;

        vector<geometry_msgs::Point> b_pts;

        // 0: (y_min, z_min)
        Vector3d bottom_left;
        bottom_left << mid_point.x(), mid_point.y(), z_min;
        bottom_left = bottom_left - (max_distance * dir);
        geometry_msgs::Point bl_pt;
        bl_pt.x = bottom_left(0);
        bl_pt.y = bottom_left(1);
        bl_pt.z = bottom_left(2);
        b_pts.push_back(bl_pt);

        // 1: (y_min, z_max)
        Vector3d top_left;
        top_left << mid_point.x(), mid_point.y(), z_max+5;
        top_left = top_left - (max_distance * dir);
        geometry_msgs::Point tl_pt;
        tl_pt.x = top_left(0);
        tl_pt.y = top_left(1);
        tl_pt.z = top_left(2);
        b_pts.push_back(tl_pt);

        // 2: (y_max, z_max)
        Vector3d top_right;
        top_right << mid_point.x(), mid_point.y(), z_max+5;
        top_right = top_right + (max_distance * dir);
        geometry_msgs::Point tr_pt;
        tr_pt.x = top_right(0);
        tr_pt.y = top_right(1);
        tr_pt.z = top_right(2);
        b_pts.push_back(tr_pt);

        // 3: (y_max, z_min)
        Vector3d bottom_right;
        bottom_right << mid_point.x(), mid_point.y(), z_min;
        bottom_right = bottom_right + (max_distance * dir);
        geometry_msgs::Point br_pt;
        br_pt.x = bottom_right(0);
        br_pt.y = bottom_right(1);
        br_pt.z = bottom_right(2);
        b_pts.push_back(br_pt);

        double area = 0.0;
        area = (bottom_left - top_left).norm() * (bottom_left - bottom_right).norm();

        if (area < 30)
        {
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
            p_ids.values.push_back(fpp.first);

            // 1: (y_min, z_max)
            geometry_msgs::Point32 tl_pt32;
            tl_pt32.x = tl_pt.x;
            tl_pt32.y = tl_pt.y;
            tl_pt32.z = tl_pt.z;
            frame_cloud.points.push_back(tl_pt32);
            p_ids.values.push_back(fpp.first);

            // 2: (y_max, z_max)
            geometry_msgs::Point32 tr_pt32;
            tr_pt32.x = tr_pt.x;
            tr_pt32.y = tr_pt.y;
            tr_pt32.z = tr_pt.z;
            frame_cloud.points.push_back(tr_pt32);
            p_ids.values.push_back(fpp.first);

            // 3: (y_max, z_min)
            geometry_msgs::Point32 br_pt32;
            br_pt32.x = br_pt.x;
            br_pt32.y = br_pt.y;
            br_pt32.z = br_pt.z;
            frame_cloud.points.push_back(br_pt32);
            p_ids.values.push_back(fpp.first);
        }
    }
    
    marker_pub.publish(line_list);
    frame_cloud.channels.push_back(p_ids);
    frame_pub.publish(frame_cloud);
}

void sync_callback(
    const sensor_msgs::PointCloudConstPtr &features_msg,
    const sensor_msgs::PointCloudConstPtr &mask_cloud,
    const nav_msgs::OdometryConstPtr &odometry_msg
)
{
    ROS_DEBUG("************ MAPPER TIME SYNC CALL BACK ************");
    map<int, vector<Vector3d>> reg_points;
    
    // Loop through all feature points
    for(int fi = 0; fi < features_msg->points.size(); fi++) {
        Vector3d fpoint;
        geometry_msgs::Point32 p = features_msg->points[fi];
        fpoint << p.x, p.y, 1;

        int plane_id = (int)features_msg->channels[0].values[fi];

        reg_points[plane_id].push_back(fpoint);
    }

    // Fit planes ==> create map planeid vs. params
    for (auto const& fpp: reg_points) {
        // if (
        //     (fpp.first == 162) ||
        //     (fpp.first == 66)
        // )
        //     continue;

        MatrixXd pts_mat(fpp.second.size(), 3);
    
        for (int pti = 0; pti < fpp.second.size(); pti++) {
            pts_mat.row(pti) = fpp.second[pti].transpose();
        }

        // find svd
        Vector3d params;
        Eigen::JacobiSVD<MatrixXd> pt_svd(pts_mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
        params = pt_svd.matrixV().col(pt_svd.matrixV().cols() - 1);
        params.normalize();

        // Vector4d info = covariance_matrix(pts_mat).diagonal().tail(4);
        Vector3d pn;
        pn << params(0), params(1), 0;
        double mag = pn.norm();
        pn.normalize();

        Vector3d x_axis(1, 0, 0);
        Vector3d y_axis(0, 1, 0);

        double x_angle = fabs(pn.dot(x_axis));
        double y_angle = fabs(pn.dot(y_axis));

        pn = (x_angle > y_angle) ? x_axis : y_axis;
        
        Vector4d params_;
        params_ << pn(0), pn(1), pn(2), params(2)/mag;

        if (fabs(params_(3)) < 100.0) { // To filter bad measurements 
            // store plane params
            plane_measurements[fpp.first].push_back(params_.normalized());

            if(plane_params.find(fpp.first) == plane_params.end())
                plane_params[fpp.first] = params_.normalized();

            // ROS_INFO("Info is: ia=%f, ib=%f, ic=%f, id=%f", info(0), info(1), info(2), info(3));
        }
    }

    map<int, double> init_depths;    
    for (auto& p: plane_params) {
        init_depths[p.first] = p.second(3)/p.second.head<3>().norm();
    }
    
    // optimize_plane_params(plane_params);

    for (auto& p: plane_params) {
        p.second.normalize();
        Vector3d n = p.second.head<3>();
        // n.normalize();
        // p.second(3) = init_depths[p.first];
        p.second(3) /= n.norm();
        
        p.second.head<3>().normalize();
        ROS_INFO("ID=%d, a=%f, b=%f, c=%f, d=%f", p.first, p.second(0), p.second(1), p.second(2), p.second(3));
    }

    mask_clouds.push_back(mask_cloud);
    odometry_msgs.push_back(odometry_msg);

    publish_plane_cloud(mask_clouds, odometry_msgs, plane_params);

    ROS_DEBUG("**************************************");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rpvio_mapper");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);

    // message_filters::Subscriber<sensor_msgs::PointCloud> sub_feature_cloud(n, "/rpvio_estimator/history_cloud", 2);
    // message_filters::Subscriber<sensor_msgs::PointCloud> sub_mask_cloud(n, "/rpvio_feature_tracker/mask_cloud", 1);
    // message_filters::Subscriber<nav_msgs::Odometry> sub_odometry(n, "/rpvio_estimator/odometry", 2);

    // message_filters::TimeSynchronizer<sensor_msgs::PointCloud, sensor_msgs::PointCloud, nav_msgs::Odometry> sync(
    //     sub_feature_cloud,
    //     sub_mask_cloud,
    //     sub_odometry,
    //     5
    // );
    // sync.registerCallback(boost::bind(&sync_callback, _1, _2, _3));

    ros::Subscriber sub_history_cloud = n.subscribe("/rpvio_estimator/point_cloud", 100, history_callback);

    pub_plane_cloud = n.advertise<sensor_msgs::PointCloud>("plane_cloud", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    frame_pub = n.advertise<sensor_msgs::PointCloud>("frame_cloud", 10);

    ros::spin();

    return 0;
}
