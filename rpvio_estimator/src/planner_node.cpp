#include "planner.h"

map<int, Vector4d> plane_params;
vector<sensor_msgs::PointCloudConstPtr> mask_clouds;
vector<nav_msgs::OdometryConstPtr> odometry_msgs;
// map<int, vector<Vector3d>> reg_points;

map<int, Vector4d> gt_params;

void current_state_callback(
    const sensor_msgs::PointCloudConstPtr &frames_msg,
    const nav_msgs::OdometryConstPtr &odometry_msg
)
{
    int num_of_planes = (int)frames_msg->points.size()/4;
    ROS_INFO("Received frames message with %d planes", num_of_planes);
    if (num_of_planes < 1)
        return;

    // From the points, compute the camera facing normal
    // First find the normal
    // Then find the 'd' parameter
    // Then find the nearest point on the plane to current camera position
    // camera facing normal direction is given by (cam_position - nearest_point)
    // use the camera facing normal direction as the normal value for the plane
    

    vector<CuboidObject> cuboids;
    // Create cuboids
    for (unsigned int i = 0; i < frames_msg->points.size(); i += 4)
    {
        int p_id = frames_msg->channels[0].values[i];

        // Compute plane segment parameters center, normal, breadth and width
        geometry_msgs::Point32 bl_pt = frames_msg->points[i];
        geometry_msgs::Point32 tl_pt = frames_msg->points[i + 1];
        geometry_msgs::Point32 tr_pt = frames_msg->points[i + 2];
        geometry_msgs::Point32 br_pt = frames_msg->points[i + 3];

        Point bl(bl_pt.x, bl_pt.y, bl_pt.z);
        Point tl(tl_pt.x, tl_pt.y, tl_pt.z);
        Point tr(tr_pt.x, tr_pt.y, tr_pt.z);
        Point br(br_pt.x, br_pt.y, br_pt.z);

        FloatingPoint breadth = (br - bl).norm();
        FloatingPoint height = (tr - br).norm();
        Point center = (bl + tl + tr + br) / 4.0;
        Point normal = ((tl - bl).normalized().cross((br - bl).normalized())).normalized();

        CuboidObject cuboid = CuboidObject(center, normal, breadth /*breadth*/, 1 /*width*/, height /*height*/, Color::Gray(), p_id);
        cuboids.push_back(cuboid);
    }

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
    Matrix3d rot = quat.matrix();

    // Each sampled trajectory is visualized as a line_strip
    // Create a marker array object that holds all line_strips
    visualization_msgs::MarkerArray ma;

    // Sample many trajectories for a given odometry
    int num_goal = 50;
    int num = 20;

    double x_init = 2.0;
    double y_init = 0.0;
    double z_init = 0.0;

    double x_des_traj_init = x_init;
    double y_des_traj_init = y_init;
    double z_des_traj_init = z_init;

    double vx_des = 1.0;
    double vy_des = 0.7;
    double vz_des = -0.2;

    // ################################# Hyperparameters
    double t_fin = 5;

    // ################################# noise sampling

    // ########### Random samples for batch initialization of heading angles
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_int_distribution<uint64_t> dis;

    MatrixXd identity = MatrixXd::Identity(num, num);
    MatrixXd A = diff(diff(identity));

    std::cout << "A matrix size is " << to_string(A.rows()) << ", " << to_string(A.cols()) << std::endl;
    
    MatrixXd temp_1 = MatrixXd::Zero(1, num);
    MatrixXd temp_2 = MatrixXd::Zero(1, num);
    MatrixXd temp_3 = MatrixXd::Zero(1, num);
    MatrixXd temp_4 = MatrixXd::Zero(1, num);

    temp_1(0, 0) = 1.0;
    temp_2(0, 0) = -2;
    temp_2(0, 1) = 1;
    temp_3(0, num-1) = -2;
    temp_3(0, num-2) = 1;
    temp_4(0, num-1) = 1.0;

    MatrixXd A_mat = MatrixXd::Zero(num+2, num);
    A_mat.row(0) = temp_1;
    A_mat.row(1) = temp_2;
    A_mat.block(2, 0, A.rows(), A.cols()) = A;
    A_mat.row(A.rows()+2) = temp_3;
    A_mat.row(A.rows()+3) = temp_4;

    A_mat = -A_mat;
    std::cout << "A_mat matrix size is " << to_string(A_mat.rows()) << ", " << to_string(A_mat.cols()) << std::endl;

    MatrixXd R = A_mat.transpose() * A_mat;
    std::cout << "R matrix size is " << to_string(R.rows()) << ", " << to_string(R.cols()) << std::endl;

    MatrixXd mu = MatrixXd::Zero(num, 1);
    MatrixXd cov = 0.03 * R.inverse();

    Eigen::EigenMultivariateNormal<double> *normX_solver = new Eigen::EigenMultivariateNormal<double>(mu, cov, true, dis(gen));
    Eigen::EigenMultivariateNormal<double> *normY_solver = new Eigen::EigenMultivariateNormal<double>(mu, cov, true, dis(gen));
    Eigen::EigenMultivariateNormal<double> *normZ_solver = new Eigen::EigenMultivariateNormal<double>(mu, 0.5*cov, true, dis(gen));

    // ################# Gaussian Trajectory Sampling
    MatrixXd eps_kx = normX_solver->samples(num_goal).transpose();
    MatrixXd eps_ky = normY_solver->samples(num_goal).transpose();
    MatrixXd eps_kz = normZ_solver->samples(num_goal).transpose();

    double x_fin = 20.0;
    double y_fin = -10.0;
    double z_fin = 0.0;

    VectorXd t_interp = VectorXd::LinSpaced(num, 0, t_fin);
    VectorXd x_interp = (x_des_traj_init + ((x_fin-x_des_traj_init)/t_fin) * t_interp.array()).matrix();
    VectorXd y_interp = (y_des_traj_init + ((y_fin-y_des_traj_init)/t_fin) * t_interp.array()).matrix();
    VectorXd z_interp = (z_des_traj_init + ((z_fin-z_des_traj_init)/t_fin) * t_interp.array()).matrix();

    MatrixXd x_samples(eps_kx.rows(), eps_kx.cols());
    MatrixXd y_samples(eps_kx.rows(), eps_kx.cols());
    MatrixXd z_samples(eps_kx.rows(), eps_kx.cols());
    
    x_samples = eps_kx;
    x_samples.rowwise() += x_interp.transpose();
    y_samples = eps_ky;
    y_samples.rowwise() += y_interp.transpose();
    z_samples = eps_kz;
    z_samples.rowwise() += z_interp.transpose();

    // z_samples.rowwise() = y_interp.transpose() + eps_kz.rowwise();
    std::cout << "samples matrix size is " << to_string(x_samples.rows()) << ", " << to_string(x_samples.cols()) << std::endl;

    // std::ofstream file_samples("path_samples.txt");

    // Iterate over each sampled path
    visualization_msgs::Marker optimal_mmd_line_strip;
    visualization_msgs::Marker optimal_sdf_line_strip;
    double min_mmd_cost = 100000;
    double max_sdf_cost = -100000;
    
    for (int i = 0; i < x_samples.rows(); i++)
    {   
        visualization_msgs::Marker line_strip;
        
        line_strip.header = odometry_msg->header;
        // line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;

        line_strip.id = i;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        // LINE_STRIP markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.03;

        line_strip.color.a = 0.7;

        bool is_colliding = false;
        // Iterate over each point in the sampled path

        double trajectory_mmd_cost = 0.0;
        double trajectory_sdf_cost = 0.0;
        for (int j = 0; j < x_samples.cols(); j++)
        {   
            // Add point to the line strip
            geometry_msgs::Point line_pt;
            Vector3d line_pt_w;
            line_pt_w << x_samples(i, j), y_samples(i, j), z_samples(i, j);

            line_pt_w = (rot * line_pt_w) + trans;

            for (int oi = 0; oi < cuboids.size(); oi++) {
                double sdf_value = max(fabs(cuboids[oi].getDistanceToPoint(line_pt_w)) - 1.0, 0.0);
                double mmd_cost = getMMDcost(sdf_value) - 20.0;

                trajectory_mmd_cost += mmd_cost;
                trajectory_sdf_cost += sdf_value;
            }

            line_pt.x = line_pt_w.x();
            line_pt.y = line_pt_w.y();
            line_pt.z = line_pt_w.z();
            line_strip.points.push_back(line_pt);
        }

        line_strip.color.r = 1.0;
        line_strip.color.g = 0.0;
        line_strip.color.b = 0.0;
            
        if (trajectory_mmd_cost < min_mmd_cost)
        {
            optimal_mmd_line_strip = line_strip;
            min_mmd_cost = trajectory_mmd_cost;
        }

        if (trajectory_sdf_cost > max_sdf_cost)
        {
            optimal_sdf_line_strip = line_strip;
            max_sdf_cost = trajectory_sdf_cost;
        }

        ma.markers.push_back(line_strip);
    }

    // Visualize optimal mmd trajectory in black color
    optimal_mmd_line_strip.color.r = 0.0;
    optimal_mmd_line_strip.color.g = 0.0;
    optimal_mmd_line_strip.color.b = 0.0;
    optimal_mmd_line_strip.scale.x = 0.06;
    optimal_mmd_line_strip.color.a = 1.0;
    ma.markers.push_back(optimal_mmd_line_strip);

    // Visualize optimal sdf trajectory in blue color
    optimal_sdf_line_strip.color.r = 0.0;
    optimal_sdf_line_strip.color.g = 0.0;
    optimal_sdf_line_strip.color.b = 1.0;
    optimal_sdf_line_strip.scale.x = 0.06;
    optimal_sdf_line_strip.color.a = 1.0;
    ma.markers.push_back(optimal_sdf_line_strip);

    pub_paths.publish(ma);
}

void current_state_callback2(
    const visualization_msgs::MarkerConstPtr &frames_msg,
    const nav_msgs::OdometryConstPtr &odometry_msg
)
{
    // Goal point
    Vector3d goal(22.0, -22.0, -2.5);

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

    // Transform to local frame
    // Vector3d local_goal = Tic.inverse() * (Ti.inverse() * goal);
    Vector3d local_goal(22.0, 0.0, 22.0);
    double local_goal_distance = local_goal.norm();
    Vector3d local_goal_dir = local_goal.normalized();

    // Each sampled trajectory is visualized as a line_strip
    // Create a marker array object that holds all line_strips
    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker direct_line_strip;
    direct_line_strip.header = odometry_msg->header;
    direct_line_strip.pose.orientation.w = 1.0;
    direct_line_strip.id = 1;
    direct_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    direct_line_strip.scale.x = 0.03; // LINE_STRIP markers use only the x component of scale for the line width
    direct_line_strip.color.a = 0.7;

    // Now draw a line or a straight line path
    for (int t = 0; t < 10; t++)
    {
        geometry_msgs::Point line_pt;
        Vector3d way_pt;
        way_pt << (t/9) * local_goal;

        line_pt.x = way_pt.x();
        line_pt.y = way_pt.y();
        line_pt.z = way_pt.z();

        direct_line_strip.points.push_back(line_pt);
    }
    ma.markers.push_back(direct_line_strip);

    // Now compute a STOMP trajectory from origin to local goal
    int num_goal = 50;
    int num = 20;

    double x_init = 2.0;
    double y_init = 0.0;
    double z_init = 2.0;

    double x_des_traj_init = x_init;
    double y_des_traj_init = y_init;
    double z_des_traj_init = z_init;

    // ################################# Hyperparameters
    double t_fin = 5;

    // ################################# noise sampling

    // ########### Random samples for batch initialization of heading angles
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_int_distribution<uint64_t> dis;

    MatrixXd identity = MatrixXd::Identity(num, num);
    MatrixXd A = diff(diff(identity));

    std::cout << "A matrix size is " << to_string(A.rows()) << ", " << to_string(A.cols()) << std::endl;
    
    MatrixXd temp_1 = MatrixXd::Zero(1, num);
    MatrixXd temp_2 = MatrixXd::Zero(1, num);
    MatrixXd temp_3 = MatrixXd::Zero(1, num);
    MatrixXd temp_4 = MatrixXd::Zero(1, num);

    temp_1(0, 0) = 1.0;
    temp_2(0, 0) = -2;
    temp_2(0, 1) = 1;
    temp_3(0, num-1) = -2;
    temp_3(0, num-2) = 1;
    temp_4(0, num-1) = 1.0;

    MatrixXd A_mat = MatrixXd::Zero(num+2, num);
    A_mat.row(0) = temp_1;
    A_mat.row(1) = temp_2;
    A_mat.block(2, 0, A.rows(), A.cols()) = A;
    A_mat.row(A.rows()+2) = temp_3;
    A_mat.row(A.rows()+3) = temp_4;

    A_mat = -A_mat;
    std::cout << "A_mat matrix size is " << to_string(A_mat.rows()) << ", " << to_string(A_mat.cols()) << std::endl;

    MatrixXd R = A_mat.transpose() * A_mat;
    std::cout << "R matrix size is " << to_string(R.rows()) << ", " << to_string(R.cols()) << std::endl;

    MatrixXd mu = MatrixXd::Zero(num, 1);
    MatrixXd cov = 0.03 * R.inverse();

    Eigen::EigenMultivariateNormal<double> *normX_solver = new Eigen::EigenMultivariateNormal<double>(mu, cov, true, dis(gen));
    Eigen::EigenMultivariateNormal<double> *normY_solver = new Eigen::EigenMultivariateNormal<double>(mu, cov, true, dis(gen));
    Eigen::EigenMultivariateNormal<double> *normZ_solver = new Eigen::EigenMultivariateNormal<double>(mu, 0.5*cov, true, dis(gen));

    // ################# Gaussian Trajectory Sampling
    MatrixXd eps_kx = normX_solver->samples(num_goal).transpose();
    MatrixXd eps_ky = normY_solver->samples(num_goal).transpose();
    MatrixXd eps_kz = normZ_solver->samples(num_goal).transpose();

    double x_fin = local_goal.x();
    double y_fin = local_goal.y();
    double z_fin = local_goal.z();

    VectorXd t_interp = VectorXd::LinSpaced(num, 0, t_fin);
    VectorXd x_interp = (x_des_traj_init + ((x_fin-x_des_traj_init)/t_fin) * t_interp.array()).matrix();
    VectorXd y_interp = (y_des_traj_init + ((y_fin-y_des_traj_init)/t_fin) * t_interp.array()).matrix();
    VectorXd z_interp = (z_des_traj_init + ((z_fin-z_des_traj_init)/t_fin) * t_interp.array()).matrix();

    MatrixXd x_samples(eps_kx.rows(), eps_kx.cols());
    MatrixXd y_samples(eps_kx.rows(), eps_kx.cols());
    MatrixXd z_samples(eps_kx.rows(), eps_kx.cols());
    
    x_samples = eps_kx;
    x_samples.rowwise() += x_interp.transpose();
    y_samples = eps_ky;
    y_samples.rowwise() += y_interp.transpose();
    z_samples = eps_kz;
    z_samples.rowwise() += z_interp.transpose();

    // z_samples.rowwise() = y_interp.transpose() + eps_kz.rowwise();
    std::cout << "samples matrix size is " << to_string(x_samples.rows()) << ", " << to_string(x_samples.cols()) << std::endl;

    // Publish the goals, straight line and the selected STOMP trajectory
    // Iterate over each sampled path
    // visualization_msgs::Marker optimal_mmd_line_strip;
    // visualization_msgs::Marker optimal_sdf_line_strip;
    // double min_mmd_cost = 100000;
    // double max_sdf_cost = -100000;
    
    for (int i = 0; i < x_samples.rows(); i++)
    {   
        visualization_msgs::Marker line_strip;
        
        line_strip.header = odometry_msg->header;
        // line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;

        line_strip.id = i;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        // LINE_STRIP markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.03;

        line_strip.color.a = 0.7;

        bool is_colliding = false;
        // Iterate over each point in the sampled path

        double trajectory_mmd_cost = 0.0;
        double trajectory_sdf_cost = 0.0;
        for (int j = 0; j < x_samples.cols(); j++)
        {   
            // Add point to the line strip
            geometry_msgs::Point line_pt;
            Vector3d line_pt_w;
            line_pt_w << x_samples(i, j), y_samples(i, j), z_samples(i, j);

            // line_pt_w = (rot * line_pt_w) + trans;

            // for (int oi = 0; oi < cuboids.size(); oi++) {
            //     double sdf_value = max(fabs(cuboids[oi].getDistanceToPoint(line_pt_w)) - 1.0, 0.0);
            //     double mmd_cost = getMMDcost(sdf_value) - 20.0;

            //     trajectory_mmd_cost += mmd_cost;
            //     trajectory_sdf_cost += sdf_value;
            // }

            line_pt.x = line_pt_w.x();
            line_pt.y = line_pt_w.y();
            line_pt.z = line_pt_w.z();
            line_strip.points.push_back(line_pt);
        }

        line_strip.color.r = 1.0;
        line_strip.color.g = 0.0;
        line_strip.color.b = 0.0;
            
        // if (trajectory_mmd_cost < min_mmd_cost)
        // {
        //     optimal_mmd_line_strip = line_strip;
        //     min_mmd_cost = trajectory_mmd_cost;
        // }

        // if (trajectory_sdf_cost > max_sdf_cost)
        // {
        //     optimal_sdf_line_strip = line_strip;
        //     max_sdf_cost = trajectory_sdf_cost;
        // }

        ma.markers.push_back(line_strip);
    }

    // // Visualize optimal mmd trajectory in black color
    // optimal_mmd_line_strip.color.r = 0.0;
    // optimal_mmd_line_strip.color.g = 0.0;
    // optimal_mmd_line_strip.color.b = 0.0;
    // optimal_mmd_line_strip.scale.x = 0.06;
    // optimal_mmd_line_strip.color.a = 1.0;
    // ma.markers.push_back(optimal_mmd_line_strip);

    // // Visualize optimal sdf trajectory in blue color
    // optimal_sdf_line_strip.color.r = 0.0;
    // optimal_sdf_line_strip.color.g = 0.0;
    // optimal_sdf_line_strip.color.b = 1.0;
    // optimal_sdf_line_strip.scale.x = 0.06;
    // optimal_sdf_line_strip.color.a = 1.0;
    // ma.markers.push_back(optimal_sdf_line_strip);

    pub_paths.publish(ma);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rpvio_planner");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);

    message_filters::Subscriber<sensor_msgs::PointCloud> sub_frame_cloud(n, "/rpvio_mapper/frame_cloud", 20);
    message_filters::Subscriber<visualization_msgs::Marker> sub_cuboids(n, "/rpvio_mapper/cuboids", 20);
    message_filters::Subscriber<nav_msgs::Odometry> sub_odometry(n, "/vins_estimator/odometry", 20);

    message_filters::TimeSynchronizer<sensor_msgs::PointCloud, nav_msgs::Odometry> sync(
        sub_frame_cloud,
        sub_odometry,
        100
    );
    sync.registerCallback(boost::bind(&current_state_callback, _1, _2));

    message_filters::TimeSynchronizer<visualization_msgs::Marker, nav_msgs::Odometry> sync2(
        sub_cuboids,
        sub_odometry,
        100
    );
    sync2.registerCallback(boost::bind(&current_state_callback2, _1, _2));
    // ros::Subscriber sub_odometry = n.subscribe("/rpvio_estimator/odometry", 1, current_state_callback);

    std::string SRC_PATH = "/home/tvvsstas/rpvio_ws/src/rp-vio/rpvio_estimator/src";
    MMDF.assign_weights(SRC_PATH+"/weight.csv");
    std::cout << "Assigned weights !" << std::endl;
    init_gmm_values(SRC_PATH);

    pub_paths = n.advertise<visualization_msgs::MarkerArray>("gaussian_paths", 1);
    pub_colliding_cloud = n.advertise<sensor_msgs::PointCloud>("colliding_cloud", 50);
    pub_free_cloud = n.advertise<sensor_msgs::PointCloud>("free_cloud", 50);
    ros::spin();

    return 0;
}
