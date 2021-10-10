#include "mapper.h"

map<int, Vector4d> plane_params;
vector<sensor_msgs::PointCloudConstPtr> mask_clouds;
vector<nav_msgs::OdometryConstPtr> odometry_msgs;
// map<int, vector<Vector3d>> reg_points;

map<int, Vector4d> gt_params;

void read_gt_plane_params()
{
    std::string GT_FILE_PATH = RPVIO_GT_PATH + "gt_params.txt";
    ifstream gt_file(GT_FILE_PATH);
    std::string line;
	
	if (gt_file.is_open())
	{	
		while(getline(gt_file, line))
		{
			stringstream line_stream(line);
			int pid;
            double a, b, c, d;

			line_stream >> pid >> a >> b >> c >> d;

            gt_params[pid] = Vector4d(a, b, c, d);
		}
	}
	else cout << "Unable to open ground truth file: " << RPVIO_GT_PATH << endl;

	cout << "Number of planes are " << to_string(gt_params.size()) << endl;

    gt_file.close();
}

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
    map<int, vector<Vector3d>> reg_points;
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
    line_list.scale.x = 0.3;

    // Line list is green
    line_list.color.g = 1.0;
    line_list.color.a = 1.0;

    sensor_msgs::PointCloud frame_cloud;
    sensor_msgs::ChannelFloat32 p_ids;

    // Fit planes ==> create map planeid vs. params
    for (auto const& fpp: reg_points) {
        MatrixXd pts_mat(fpp.second.size(), 3);
        // MatrixXd plane_residuals(fpp.second.size(), 1);
    
        for (int pti = 0; pti < fpp.second.size(); pti++) {
            pts_mat.row(pti) = fpp.second[pti].homogeneous().transpose();
        }

        // find svd
        Vector4d params;
        Eigen::JacobiSVD<MatrixXd> pt_svd(pts_mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
        params = pt_svd.matrixV().col(pt_svd.matrixV().cols() - 1);
        params.normalize();

        // // Vector4d info = covariance_matrix(pts_mat).diagonal().tail(4);
        Vector3d pn;
        pn << params(0), params(1), 0;
        double mag = pn.norm();
        pn.normalize();
        
        // Vector4d params_;
        // params_ << pn(0), pn(1), pn(2), params(2)/mag;

        // Vector3d params_dir;
        // params_dir << params_(0), params_(1), params_(3);
        // params_dir.normalize();

        // plane_residuals = (pts_mat * params_dir).col(0);
        // double variance = 0.0;

        // for (int pti = 0; pti < fpp.second.size(); pti++) {
        //     if (fabs(plane_residuals(pti)) > variance)
        //         variance = fabs(plane_residuals(pti));
        // }

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
        // ROS_INFO("variance for mid_point (%d): (%f, %f, %f) is %f", fpp.first, mid_point.x(), mid_point.y(), mid_point.z(), variance);

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
        // Vector3d dir = (far_point - mid_point).normalized();

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
        top_left << mid_point.x(), mid_point.y(), z_max;
        top_left = top_left - (max_distance * dir);
        geometry_msgs::Point tl_pt;
        tl_pt.x = top_left(0);
        tl_pt.y = top_left(1);
        tl_pt.z = top_left(2);
        b_pts.push_back(tl_pt);

        // 2: (y_max, z_max)
        Vector3d top_right;
        top_right << mid_point.x(), mid_point.y(), z_max;
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

        if (max_distance*2 < 5)
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

void history_callback2(
    const sensor_msgs::PointCloudConstPtr &features_msg,
    const nav_msgs::OdometryConstPtr &odometry_msg
)
{
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

    map<int, vector<Vector3d>> reg_points;
    visualization_msgs::Marker line_list;
    visualization_msgs::MarkerArray ma;

    std::string GT_FILE_PATH = RPVIO_GT_PATH + "error_params.txt";
    ofstream errors_file(GT_FILE_PATH, std::fstream::out | std::fstream::app);    

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
    line_list.scale.x = 0.3;

    // Line list is green
    line_list.color.g = 1.0;
    line_list.color.a = 1.0;

    sensor_msgs::PointCloud frame_cloud;
    sensor_msgs::ChannelFloat32 p_ids;

    // Fit planes ==> create map planeid vs. params
    for (auto const& fpp: reg_points) {
        MatrixXd pts_mat(fpp.second.size(), 4);
        Vector3d point_sum = Vector3d::Zero();
        Vector3d mid_point = Vector3d::Zero();
    
        for (int pti = 0; pti < fpp.second.size(); pti++) {
            Vector3d pt = fpp.second[pti];
            // pt.z = 0.0;
            pts_mat.row(pti) = pt.homogeneous().transpose();
            pt[2] = 0.0;
            point_sum += pt;
        }
        mid_point = point_sum / fpp.second.size();

        // find svd
        Vector4d params;
        Eigen::JacobiSVD<MatrixXd> pt_svd(pts_mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
        params = pt_svd.matrixV().col(pt_svd.matrixV().cols() - 1);
        params.normalize();

        // // Vector4d info = covariance_matrix(pts_mat).diagonal().tail(4);
        Vector3d pn;
        pn << params(0), params(1), params(2);
        double mag = pn.norm();
        pn.normalize();

        // align the plane normal with vanishing point directions
        Vector3d x_axis(1, 0, 0);
        Vector3d y_axis(0, 1, 0);
        Vector3d z_axis(0, 0, 1);

        double x_angle = pn.dot(x_axis);
        double y_angle = pn.dot(y_axis);

        Vector3d normal_dir = (x_angle > y_angle) ? x_axis : y_axis;
        Vector3d plane_dir = (x_angle > y_angle) ? y_axis : x_axis;

        Vector4d params_;
        params_ << normal_dir(0), normal_dir(1), normal_dir(2), params(3)/mag;

        Vector3d camera_normal = quat * z_axis + trans;
        camera_normal[2] = 0.0;
        camera_normal.normalize();


        auto gtit = gt_params.find(fpp.first);
        if (gtit != gt_params.end()){
            double gt_d = gt_params[fpp.first][3];
            double error = fabs(fabs(params_[3]) - fabs(gt_d));
            
            Vector3d gt_normal = gt_params[fpp.first].head<3>();
            gt_normal[2] = 0.0;
            gt_normal.normalize();
            double viewing_angle = acos(camera_normal.dot(gt_normal));
            // if (error < 6)
            if (
                (fpp.second.size() > 20) &&
                (fabs(params_[3]) < 10) &&
                ((pts_mat * params).norm() < 1)
            ) {
                errors_file << to_string(error) << " " << to_string(viewing_angle) << std::endl;
            }
        }

        // Find the mean point-plane distance
        MatrixXd pp_distances = (pts_mat * params_);
        double pp_mean = pp_distances.mean();

        // Find the variance of point-plane distance
        MatrixXd mean_vec = pp_mean * MatrixXd::Ones(pp_distances.rows(), 1);
        double pp_variance = (pp_distances - mean_vec).squaredNorm() / pts_mat.rows();

        // Find the variance of the breadth
        MatrixXd variance_mat = (pts_mat.block(0, 0, pts_mat.rows(), 3).rowwise() - mid_point.transpose()) * plane_dir;
        variance_mat.cwiseAbs();
        double breadth_variance = min(variance_mat.maxCoeff(), 10.0);

        vector<geometry_msgs::Point> end_pts;

        // 0
        Vector3d bottom_left;
        bottom_left << mid_point.x(), mid_point.y(), 0.0;
        bottom_left = bottom_left - (breadth_variance * plane_dir);
        geometry_msgs::Point bl_pt;
        bl_pt.x = bottom_left(0);
        bl_pt.y = bottom_left(1);
        bl_pt.z = bottom_left(2);
        end_pts.push_back(bl_pt);

        // 1
        Vector3d bottom_right;
        bottom_right << mid_point.x(), mid_point.y(), 0.0;
        bottom_right = bottom_right + (breadth_variance * plane_dir);
        geometry_msgs::Point br_pt;
        br_pt.x = bottom_right(0);
        br_pt.y = bottom_right(1);
        br_pt.z = bottom_right(2);
        end_pts.push_back(br_pt);

        // 0 -> 1
        line_list.points.push_back(end_pts[0]);
        line_list.points.push_back(end_pts[1]);

        visualization_msgs::Marker ellipse;
        ellipse.scale.x = min(max(pp_variance * 3, 0.1), 0.9);
        ellipse.scale.y = min(max(breadth_variance * 2, 0.1), 10.0);
        ellipse.scale.z = 0.05;

        ellipse.type = visualization_msgs::Marker::SPHERE;
        ellipse.color.a = 1.0;
        ellipse.color.b = 1.0;
        ellipse.header = features_msg->header;
        ellipse.id = fpp.first;

        Vector4d ellipse_major = plane_dir.homogeneous().normalized();

        ellipse.pose.position.x = mid_point.x();
        ellipse.pose.position.y = mid_point.y();
        ellipse.pose.position.z = mid_point.z();
        ellipse.pose.orientation.x = ellipse_major(0);
        ellipse.pose.orientation.y = ellipse_major(1);
        ellipse.pose.orientation.z = ellipse_major(2);
        ellipse.pose.orientation.w = ellipse_major(3);

        ma.markers.push_back(ellipse);
    }

    marker_pub.publish(line_list);
    ellipse_pub.publish(ma);
    errors_file.close();
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

    // ros::Subscriber sub_history_cloud = n.subscribe("/rpvio_estimator/point_cloud", 10, history_callback2);

    message_filters::Subscriber<sensor_msgs::PointCloud> sub_point_cloud(n, "/rpvio_estimator/point_cloud", 10);
    message_filters::Subscriber<nav_msgs::Odometry> sub_odometry(n, "/rpvio_estimator/odometry", 10);

    message_filters::TimeSynchronizer<sensor_msgs::PointCloud, nav_msgs::Odometry> sync(
        sub_point_cloud,
        sub_odometry,
        5
    );
    sync.registerCallback(boost::bind(&history_callback2, _1, _2));

    pub_plane_cloud = n.advertise<sensor_msgs::PointCloud>("plane_cloud", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 5);
    frame_pub = n.advertise<sensor_msgs::PointCloud>("frame_cloud", 5);
    ellipse_pub = n.advertise<visualization_msgs::MarkerArray>("covar_markers", 10);

    read_gt_plane_params();

    ros::spin();

    return 0;
}
