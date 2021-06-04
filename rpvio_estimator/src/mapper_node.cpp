#include "mapper.h"

void optimize_plane_params(
    map<int, Vector4d> &plane_params
) {
    // plane ids: 91, 39, 175, 130
    /**
     * Layout of scene with plane ids
     *       _____91_______
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

    for(auto &p : plane_params)
    {
        ceres::LocalParameterization *local_n_parameterization = new ceres::AutoDiffLocalParameterization<UnitNormalParameterization, 2, 2>;
        problem.AddParameterBlock(p.second.head<2>().data(), 2, local_n_parameterization);
    }

    // Vector3d z_axis;
    // z_axis << 0, 0, 1;
    
    // // ceres::LocalParameterization *local_n_parameterization = new ceres::HomogeneousVectorParameterization(3);
    // problem.AddParameterBlock(z_axis.data(), 3);
    // problem.SetParameterBlockConstant(z_axis.data());

    // for(auto &p : plane_params)
    // {
    //     problem.AddResidualBlock(
    //         new ceres::AutoDiffCostFunction<OrthogonalConstraint, 1, 3, 3>(
    //             new OrthogonalConstraint()
    //         ),
    //         NULL,
    //         p.second.data(),     
    //         z_axis.data()
    //     );        
    // }

    for (int opair_idx = 0; opair_idx < ORTHO_PAIRS.size(); opair_idx++) {
        int plane_id1 = ORTHO_PAIRS[opair_idx].first;
        int plane_id2 = ORTHO_PAIRS[opair_idx].second;
        
        if (
            (find(cur_pids.begin(), cur_pids.end(), plane_id1) != cur_pids.end()) &&
            (find(cur_pids.begin(), cur_pids.end(), plane_id2) != cur_pids.end())
        ) {
            // ceres::LocalParameterization *local_n_parameterization1 = new ceres::HomogeneousVectorParameterization(4);
            // ceres::LocalParameterization *local_n_parameterization2 = new ceres::HomogeneousVectorParameterization(4);
            // problem.AddParameterBlock(plane_params[plane_id1].data(), 3, local_n_parameterization1);
            // problem.AddParameterBlock(plane_params[plane_id2].data(), 3, local_n_parameterization2);

            // Add Orthogonal Constraint
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<OrthogonalConstraint, 1, 2, 2>(
                    new OrthogonalConstraint()
                ),
                NULL,
                plane_params[plane_id1].head<2>().data(),     
                plane_params[plane_id2].head<2>().data()
            );
        }
    }

    for (int ppair_idx = 0; ppair_idx < PARALLEL_PAIRS.size(); ppair_idx++) {
        int plane_id1 = PARALLEL_PAIRS[ppair_idx].first;
        int plane_id2 = PARALLEL_PAIRS[ppair_idx].second;
        
        if (
            (find(cur_pids.begin(), cur_pids.end(), plane_id1) != cur_pids.end()) &&
            (find(cur_pids.begin(), cur_pids.end(), plane_id2) != cur_pids.end())
        ) {
            // ceres::LocalParameterization *local_n_parameterization1 = new ceres::HomogeneousVectorParameterization(4);
            // ceres::LocalParameterization *local_n_parameterization2 = new ceres::HomogeneousVectorParameterization(4);
            // problem.AddParameterBlock(plane_params[plane_id1].data(), 3, local_n_parameterization1);
            // problem.AddParameterBlock(plane_params[plane_id2].data(), 3, local_n_parameterization2);
            
            // Add Parallel Constraint
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<ParallelConstraint, 3, 2, 2>(
                    new ParallelConstraint()
                ),
                NULL,
                plane_params[plane_id1].head<2>().data(),     
                plane_params[plane_id2].head<2>().data()
            );
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 100;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;
}

void publish_plane_cloud(
    const sensor_msgs::PointCloudConstPtr &mask_cloud,
    const nav_msgs::OdometryConstPtr &odometry_msg,
    map<int, Vector4d> plane_params
){
    vector<int> cur_pids;
    for (map<int, Vector4d>::iterator it = plane_params.begin(); it != plane_params.end(); ++it){
        cur_pids.push_back(it->first);
        // ROS_INFO("---------------CURRENT PIDs----------------ID = %d --------------------", it->first);
    }

    ROS_DEBUG("-----------Computing FIXED transforms--------------");
    // Retrieve pose from odometry message
    Isometry3d Tic;
    Tic.linear() = RIC[0];
    Tic.translation() = TIC[0];
    ROS_DEBUG("-----------Computed FIXED transforms DONE--------------");

    ROS_DEBUG("-----------Computing odometry transforms--------------");

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
    
    ROS_DEBUG("-----------Computed odometry transforms DONE--------------");

    sensor_msgs::PointCloud plane_cloud;
    plane_cloud.header = mask_cloud->header;
    
    sensor_msgs::ChannelFloat32 colors;
    colors.name = "rgb";

    map<int, int> pid2HEX;
    pid2HEX[39] = 0x0000FF;// cv::Scalar(255,0,0);
    pid2HEX[66] = 0xFF00FF;// cv::Scalar(255,0,255);
    pid2HEX[91] = 0x00FF00;// cv::Scalar(0,255,0);
    pid2HEX[130] = 0xFF0000;// cv::Scalar(0,0,255);
    pid2HEX[162] = 0x00FFFF;// cv::Scalar(255,255,0);
    pid2HEX[175] = 0xFFFF00;// cv::Scalar(0,255,255);

    // Back-project all mask points using odometry
    ROS_DEBUG("=============Back projecting mask points to planes DONE=============");
    for (unsigned int i = 0; i < mask_cloud->points.size(); i++) {
        int ppid = mask_cloud->channels[1].values[i];
        if (find(cur_pids.begin(), cur_pids.end(), ppid) != cur_pids.end()) {
            Vector4d pp = plane_params[ppid];

            Vector3d normal;
            normal << pp(0), pp(1), pp(2);
            normal.normalize();
            double d = -pp(3);

            if (fabs(d) < 100) {
                double lambda = 0.0;
                double lambda2 = 0.0;

                geometry_msgs::Point32 m;
                m = mask_cloud->points[i];
                Vector3d c_point(m.x, m.y, m.z);

                Vector4d pp_ci = (Ti * Tic).matrix().transpose() * pp;

                Vector3d normal_ci;
                normal_ci << pp_ci(0), pp_ci(1), pp_ci(2);
                normal_ci.normalize();

                Vector4d pp2 = pp;
                pp2.head<3>() *= -1;
                Vector4d pp_ci2 = (Ti * Tic).matrix().transpose() * pp2;

                Vector3d normal_ci2;
                normal_ci2 << pp_ci2(0), pp_ci2(1), pp_ci2(2);
                normal_ci2.normalize();

                lambda2 = -fabs(pp_ci2(3)) / (normal_ci2.dot(c_point));
                lambda = -fabs(pp_ci(3)) / (normal_ci.dot(c_point));

                if ((lambda2 * c_point)(2) < 0) {
                    c_point = lambda * c_point;
                } else {
                    c_point = lambda2 * c_point;
                }
                
                // Transform c_point (current camera) to imu (current IMU)
                Vector3d i_point = Tic.rotation() * c_point + Tic.translation();

                // Transform current imu point to world imu
                Vector3d i0_point = (Ti.rotation() * i_point) + Ti.translation();

                Vector3d w_pts_i = i0_point;

                geometry_msgs::Point32 p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                plane_cloud.points.push_back(p);
                
                // int rgb = mask_cloud->channels[0].values[i];
                int rgb = pid2HEX[ppid];
                float float_rgb = *reinterpret_cast<float*>(&rgb);
                colors.values.push_back(float_rgb);
            }
        }
    }

    // publish current point cloud
    plane_cloud.channels.push_back(colors);
    pub_plane_cloud.publish(plane_cloud);
}

void sync_callback(
    const sensor_msgs::PointCloudConstPtr &features_msg,
    const sensor_msgs::PointCloudConstPtr &mask_cloud,
    const nav_msgs::OdometryConstPtr &odometry_msg
)
{
    ROS_DEBUG("************ MAPPER TIME SYNC CALL BACK ************");
    map<int, Vector4d> plane_params;
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
            (fpp.first == 162) &&
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

        // Vector4d info = covariance_matrix(pts_mat).diagonal().tail(4);

        Vector3d pn;
        pn << params(0), params(1), 0;
        double mag = pn.norm();
        pn.normalize();
        
        Vector4d params_;
        params_ << pn(0), pn(1), pn(2), params(2)/mag;

        // store plane params
        plane_params[fpp.first] = params_;

        // ROS_INFO("Info is: ia=%f, ib=%f, ic=%f, id=%f", info(0), info(1), info(2), info(3));
    }

    map<int, Vector3d> init_normals;    
    for (auto& p: plane_params) {
        init_normals[p.first] = p.second.head<3>().normalized();
    }
    
    // optimize plane params
    optimize_plane_params(plane_params);

    for (auto& p: plane_params) {
        Vector3d n = p.second.head<3>();
        n.normalize();
        
        if (acos(n.dot(init_normals[p.first])) < 0)
            n = -1 * n;
        
        p.second.head<3>() = n;
        ROS_INFO("ID=%d, a=%f, b=%f, c=%f, d=%f", p.first, p.second(0), p.second(1), p.second(2), p.second(3));
    }

    // publish plane cloud
    publish_plane_cloud(mask_cloud, odometry_msg, plane_params);
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

    ros::spin();

    return 0;
}
