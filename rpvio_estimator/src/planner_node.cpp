#include "planner.h"

map<int, Vector4d> plane_params;
vector<sensor_msgs::PointCloudConstPtr> mask_clouds;
vector<nav_msgs::OdometryConstPtr> odometry_msgs;
// map<int, vector<Vector3d>> reg_points;

map<int, Vector4d> gt_params;

void current_state_callback(
    // const sensor_msgs::PointCloudConstPtr &frames_msg,
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
    double y_fin = 0.0;
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

        // Line list is green
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        // Iterate over each point in the sampled path
        for (int j = 0; j < x_samples.cols(); j++)
        {   
            // Add point to the line strip
            geometry_msgs::Point line_pt;
            Vector3d line_pt_w;
            line_pt_w << x_samples(i, j), y_samples(i, j), z_samples(i, j);

            line_pt_w = (rot * line_pt_w) + trans;

            line_pt.x = line_pt_w.x();
            line_pt.y = line_pt_w.y();
            line_pt.z = line_pt_w.z();
            line_strip.points.push_back(line_pt);
            // file_samples << x_samples(i, j) << " " << y_samples(i, j) << " " << z_samples(i, j) << std::endl;
        }
        ma.markers.push_back(line_strip);
    }
    pub_paths.publish(ma);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rpvio_planner");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);

    // message_filters::Subscriber<sensor_msgs::PointCloud> sub_frame_cloud(n, "/rpvio_mapper/frame_cloud", 20);
    // message_filters::Subscriber<nav_msgs::Odometry> sub_odometry(n, "/rpvio_estimator/odometry", 20);

    // message_filters::TimeSynchronizer<nav_msgs::Odometry> sync(
    //     // sub_frame_cloud,
    //     sub_odometry,
    //     10
    // );
    // sync.registerCallback(boost::bind(&current_state_callback, _1));
    ros::Subscriber sub_odometry = n.subscribe("/rpvio_estimator/odometry", 1, current_state_callback);

    pub_paths = n.advertise<visualization_msgs::MarkerArray>("gaussian_paths", 1);
    ros::spin();

    return 0;
}
