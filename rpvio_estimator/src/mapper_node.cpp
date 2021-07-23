#include "mapper.h"

map<int, Vector4d> plane_params;
vector<sensor_msgs::PointCloudConstPtr> mask_clouds;
vector<nav_msgs::OdometryConstPtr> odometry_msgs;

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
        if (
            (fpp.first == 162) ||
            (fpp.first == 66)
        )
            continue;

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
        
        Vector4d params_;
        params_ << pn(0), pn(1), pn(2), params(2)/mag;

        if (fabs(params_(3)) < 70.0) { // To filter bad measurements 
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

    message_filters::Subscriber<sensor_msgs::PointCloud> sub_feature_cloud(n, "/rpvio_estimator/point_cloud", 2);
    message_filters::Subscriber<sensor_msgs::PointCloud> sub_mask_cloud(n, "/rpvio_feature_tracker/mask_cloud", 1);
    message_filters::Subscriber<nav_msgs::Odometry> sub_odometry(n, "/rpvio_estimator/odometry", 2);

    message_filters::TimeSynchronizer<sensor_msgs::PointCloud, sensor_msgs::PointCloud, nav_msgs::Odometry> sync(
        sub_feature_cloud,
        sub_mask_cloud,
        sub_odometry,
        5
    );
    sync.registerCallback(boost::bind(&sync_callback, _1, _2, _3));

    pub_plane_cloud = n.advertise<sensor_msgs::PointCloud>("plane_cloud", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    frame_pub = n.advertise<sensor_msgs::PointCloud>("frame_cloud", 1);

    ros::spin();

    return 0;
}
