#include "visualization.h"

using namespace ros;
using namespace Eigen;
ros::Publisher pub_odometry, pub_latest_odometry;
ros::Publisher pub_path, pub_relo_path;
ros::Publisher pub_point_cloud, pub_margin_cloud, pub_plane_cloud;
ros::Publisher pub_key_poses;
ros::Publisher pub_relo_relative_pose;
ros::Publisher pub_camera_pose;
ros::Publisher pub_camera_pose_visual;
nav_msgs::Path path, relo_path;

ros::Publisher pub_keyframe_pose;
ros::Publisher pub_keyframe_point;
ros::Publisher pub_extrinsic;

CameraPoseVisualization cameraposevisual(0, 1, 0, 1);
CameraPoseVisualization keyframebasevisual(0.0, 0.0, 1.0, 1.0);
static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);

map<int, vector<Vector4d>> reg_points;

// Not just register points
// store the mask_clouds as-is w.r.t timestamp
// also store the poses w.r.t timestamp
// Always recompute the poses

// sensor_msgs::PointCloud plane_cloud;

void registerPub(ros::NodeHandle &n)
{
    pub_latest_odometry = n.advertise<nav_msgs::Odometry>("imu_propagate", 1000);
    pub_path = n.advertise<nav_msgs::Path>("path", 1000);
    pub_relo_path = n.advertise<nav_msgs::Path>("relocalization_path", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud", 1000);
    pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>("history_cloud", 1000);
    pub_plane_cloud = n.advertise<sensor_msgs::PointCloud>("plane_cloud", 2);
    pub_key_poses = n.advertise<visualization_msgs::Marker>("key_poses", 1000);
    pub_camera_pose = n.advertise<nav_msgs::Odometry>("camera_pose", 1000);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
    pub_keyframe_pose = n.advertise<nav_msgs::Odometry>("keyframe_pose", 1000);
    pub_keyframe_point = n.advertise<sensor_msgs::PointCloud>("keyframe_point", 1000);
    pub_extrinsic = n.advertise<nav_msgs::Odometry>("extrinsic", 1000);
    pub_relo_relative_pose=  n.advertise<nav_msgs::Odometry>("relo_relative_pose", 1000);

    cameraposevisual.setScale(1);
    cameraposevisual.setLineWidth(0.05);
    keyframebasevisual.setScale(0.1);
    keyframebasevisual.setLineWidth(0.01);
}

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, const std_msgs::Header &header)
{
    Eigen::Quaterniond quadrotor_Q = Q ;

    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = quadrotor_Q.x();
    odometry.pose.pose.orientation.y = quadrotor_Q.y();
    odometry.pose.pose.orientation.z = quadrotor_Q.z();
    odometry.pose.pose.orientation.w = quadrotor_Q.w();
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    pub_latest_odometry.publish(odometry);
}

void printStatistics(const Estimator &estimator, double t)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    printf("position: %f, %f, %f\r", estimator.Ps[WINDOW_SIZE].x(), estimator.Ps[WINDOW_SIZE].y(), estimator.Ps[WINDOW_SIZE].z());
    ROS_DEBUG_STREAM("position: " << estimator.Ps[WINDOW_SIZE].transpose());
    ROS_DEBUG_STREAM("orientation: " << estimator.Vs[WINDOW_SIZE].transpose());
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        //ROS_DEBUG("calibration result for camera %d", i);
        ROS_DEBUG_STREAM("extirnsic tic: " << estimator.tic[i].transpose());
        ROS_DEBUG_STREAM("extrinsic ric: " << Utility::R2ypr(estimator.ric[i]).transpose());
        if (ESTIMATE_EXTRINSIC)
        {
            cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
            Eigen::Matrix3d eigen_R;
            Eigen::Vector3d eigen_T;
            eigen_R = estimator.ric[i];
            eigen_T = estimator.tic[i];
            cv::Mat cv_R, cv_T;
            cv::eigen2cv(eigen_R, cv_R);
            cv::eigen2cv(eigen_T, cv_T);
            fs << "extrinsicRotation" << cv_R << "extrinsicTranslation" << cv_T;
            fs.release();
        }
    }

    static double sum_of_time = 0;
    static int sum_of_calculation = 0;
    sum_of_time += t;
    sum_of_calculation++;
    ROS_DEBUG("vo solver costs: %f ms", t);
    ROS_DEBUG("average of time %f ms", sum_of_time / sum_of_calculation);

    sum_of_path += (estimator.Ps[WINDOW_SIZE] - last_path).norm();
    last_path = estimator.Ps[WINDOW_SIZE];
    ROS_DEBUG("sum of path %f", sum_of_path);
    if (ESTIMATE_TD)
        ROS_INFO("td %f", estimator.td);
}

void pubOdometry(const Estimator &estimator, const std_msgs::Header &header)
{
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.child_frame_id = "world";
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
        odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
        odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
        odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
        odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
        odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
        pub_odometry.publish(odometry);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose = odometry.pose.pose;
        path.header = header;
        path.header.frame_id = "world";
        path.poses.push_back(pose_stamped);
        pub_path.publish(path);

        Vector3d correct_t;
        Vector3d correct_v;
        Quaterniond correct_q;
        correct_t = estimator.drift_correct_r * estimator.Ps[WINDOW_SIZE] + estimator.drift_correct_t;
        correct_q = estimator.drift_correct_r * estimator.Rs[WINDOW_SIZE];
        odometry.pose.pose.position.x = correct_t.x();
        odometry.pose.pose.position.y = correct_t.y();
        odometry.pose.pose.position.z = correct_t.z();
        odometry.pose.pose.orientation.x = correct_q.x();
        odometry.pose.pose.orientation.y = correct_q.y();
        odometry.pose.pose.orientation.z = correct_q.z();
        odometry.pose.pose.orientation.w = correct_q.w();

        pose_stamped.pose = odometry.pose.pose;
        relo_path.header = header;
        relo_path.header.frame_id = "world";
        relo_path.poses.push_back(pose_stamped);
        pub_relo_path.publish(relo_path);

        // write result to file
        ofstream foutC(RPVIO_RESULT_PATH, ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(0);
        foutC << header.stamp.toSec() * 1e9 << ",";
        foutC.precision(5);
        foutC << estimator.Ps[WINDOW_SIZE].x() << ","
              << estimator.Ps[WINDOW_SIZE].y() << ","
              << estimator.Ps[WINDOW_SIZE].z() << ","
              << tmp_Q.w() << ","
              << tmp_Q.x() << ","
              << tmp_Q.y() << ","
              << tmp_Q.z() << ","
              << estimator.Vs[WINDOW_SIZE].x() << ","
              << estimator.Vs[WINDOW_SIZE].y() << ","
              << estimator.Vs[WINDOW_SIZE].z() << "," << endl;
        foutC.close();
    }
}

void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header)
{
    if (estimator.key_poses.size() == 0)
        return;
    visualization_msgs::Marker key_poses;
    key_poses.header = header;
    key_poses.header.frame_id = "world";
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::Marker::SPHERE_LIST;
    key_poses.action = visualization_msgs::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    key_poses.id = 0; //key_poses_id++;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        geometry_msgs::Point pose_marker;
        Vector3d correct_pose;
        correct_pose = estimator.key_poses[i];
        pose_marker.x = correct_pose.x();
        pose_marker.y = correct_pose.y();
        pose_marker.z = correct_pose.z();
        key_poses.points.push_back(pose_marker);
    }
    pub_key_poses.publish(key_poses);
}

void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header)
{
    int idx2 = WINDOW_SIZE - 1;

    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        int i = idx2;
        Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[0]);

        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        pub_camera_pose.publish(odometry);

        cameraposevisual.reset();
        cameraposevisual.add_pose(P, R);
        cameraposevisual.publish_by(pub_camera_pose_visual, odometry.header);
    }
}


void pubPointCloud(Estimator &estimator, const std_msgs::Header &header, const sensor_msgs::PointCloudConstPtr &mask_cloud)
{
    sensor_msgs::PointCloud point_cloud, loop_point_cloud;
    point_cloud.header = header;
    loop_point_cloud.header = header;

    bool is_plane_cloud_created = true;

    int RGB_HEX[5];
    RGB_HEX[0] = 0xff0000; // RED
    RGB_HEX[1] = 0x0000ff; // BLUE
    RGB_HEX[2] = 0xff00ff; // MAGENTA
    RGB_HEX[3] = 0xffff00; // YELLOW
    RGB_HEX[4] = 0x00ffff; // CYAN

    map<int, vector<Vector4d>> features_per_plane;
    map<int, Vector4d> plane_params;

    for (auto &it_per_id : estimator.f_manager.feature)
    {
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
            continue;

        int imu_i = it_per_id.start_frame;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

        Vector4d w_pt_;
        w_pt_ << w_pts_i(0), w_pts_i(1), w_pts_i(2), 1;
        reg_points[it_per_id.plane_id].push_back(w_pt_);

        geometry_msgs::Point32 p;
        p.x = w_pts_i(0);
        p.y = w_pts_i(1);
        p.z = w_pts_i(2);
        point_cloud.points.push_back(p);

        if (!is_plane_cloud_created) {           
            /* Plane visualizing START -----------------------------------------------------------------------------*/
            sensor_msgs::PointCloud plane_cloud;
            plane_cloud.header = header;
            
            sensor_msgs::ChannelFloat32 colors;
            colors.name = "rgb";

            int plane_id = it_per_id.plane_id;
            Vector3d normal;
            normal << estimator.para_N[plane_id][0], estimator.para_N[plane_id][1], estimator.para_N[plane_id][2];
            double d = estimator.para_d[plane_id][0];
            // ROS_DEBUG("Plane %d, d value is %f", plane_id, d);
            
            // for (int pid = 0; pid < estimator.para_d.size(); pid++)
                // ROS_INFO("Plane %d, d value is %f", pid, estimator.para_d[pid][0]);

            // for (int i = 0; i < mask_cloud->points.size(); i++) {
            //     double lambda = 0.0;

            //     geometry_msgs::Point32 p, m;
            //     m = mask_cloud->points[i];
            //     Vector3d c_point(m.x, m.y, m.z);

            //     // transform normal from world camera frame to current camera frame
            //     Vector3d normal_i = estimator.ric[0].inverse() * estimator.Rs[WINDOW_SIZE].inverse() * estimator.ric[0] * normal;

            //     // transform depth to current camera frame
            //     double d_c0 = d; // depth in world camera frame
            //     double d_i0 = d + estimator.tic[0].dot(estimator.ric[0] * normal); // depth in world imu frame
            //     double d_i = d_i0 - estimator.Ps[WINDOW_SIZE].dot(estimator.ric[0] * normal); // depth in current imu frame
            //     // double d_ci = d_i + estimator.tic[0].dot(estimator.Rs[WINDOW_SIZE].inverse() * estimator.ric[0] * normal);

            //     // find lambda
            //     lambda = (-d_i) / (normal_i.dot(c_point));
            //     // ROS_INFO("lambda value is %f", lambda);

            //     Vector3d pts_i =  lambda * c_point;
            //     Vector3d w_pts_i = estimator.Rs[WINDOW_SIZE] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[WINDOW_SIZE];

            //     p.x = w_pts_i(0);
            //     p.y = w_pts_i(1);
            //     p.z = w_pts_i(2);
            //     plane_cloud.points.push_back(p);
                
            //     int rgb = mask_cloud->channels[0].values[i];
            //     colors.values.push_back(rgb);
            // }
            ROS_INFO("Number of planes are %d", estimator.init_pids.size());
            for (int pi = 0; pi < estimator.init_pids.size(); pi++) {
                int pid = estimator.init_pids[pi];
                Vector3d normal;
                normal << estimator.para_N[pid][0], estimator.para_N[pid][1], estimator.para_N[pid][2];
                
                double a = estimator.para_N[pid][0];
                double b = estimator.para_N[pid][1];
                double c = estimator.para_N[pid][2];
                double d = estimator.para_d[pid][0];

                // Find the point on plane nearest to origin
                double r = d / (normal.dot(normal));
                Vector3d near_point = r * normal;

                // Find a direction along the plane surface
                Vector3d dir1;
                dir1 << normal(0) + 0.1, normal(1) + 0.1, normal(2) + 0.1;
                dir1.normalize();

                double r2 = d / (normal.dot(dir1));
                dir1 = dir1 * r2;

                dir1 = dir1 - near_point;
                dir1.normalize();

                // Find another direction perpendicular to above direction
                Vector3d dir2;
                dir2 = Utility::skewSymmetric(normal) * dir1;
                dir2.normalize();
                
                // Create meshgrid
                std::vector<Vector3d> mesh_grid;
                for (int i = 0; i < 20; i++) {
                    for (int j = 0; j < 20; j++) {
                        Vector3d pt;
                        pt = near_point + i * dir1 + j * dir2;
                        mesh_grid.push_back(pt);
                    }
                }

                for (int i = 0; i < mesh_grid.size(); i++) {
                    Vector3d xy = mesh_grid[i];
                    
                    geometry_msgs::Point32 p;
                    p.x = xy(0);
                    p.y = xy(1);
                    p.z = xy(2);
                    
                    plane_cloud.points.push_back(p);
                    
                    int rgb = mask_cloud->channels[0].values[0];
                    colors.values.push_back(rgb);
                    
                }
                ROS_INFO("a=%f, b=%f, c=%f, d=%f", a, b, c, d);
            }
            plane_cloud.channels.push_back(colors);
            pub_plane_cloud.publish(plane_cloud);
            /* Plane visualized END --------------------------------------------------------------------------------*/
            is_plane_cloud_created = true;
        }
    }
    pub_point_cloud.publish(point_cloud);

    // fit planes
    for (auto const& fpp: reg_points) {
        MatrixXd pts_mat(fpp.second.size(), 4);
    
        for (int pti = 0; pti < fpp.second.size(); pti++) {
            pts_mat.row(pti) = fpp.second[pti].transpose();
        }

        // find svd
        Vector4d params;
        Eigen::JacobiSVD<MatrixXd> pt_svd(pts_mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
        params = pt_svd.matrixV().col(pt_svd.matrixV().cols() - 1);

        // store plane params
        plane_params[fpp.first] = params;

        Vector3d pn;
        pn << params(0), params(1), params(2);
        double mag = pn.norm();
        pn.normalize();

        // ROS_INFO("ID=%d, a=%f, b=%f, c=%f, d=%f", fpp.first, pn(0), pn(1), pn(2), params(3)/mag);
    }

    /* Plane visualizing START -----------------------------------------------------------------------------*/
    sensor_msgs::PointCloud plane_cloud;
    plane_cloud.header = header;
    
    sensor_msgs::ChannelFloat32 colors;
    colors.name = "rgb";
    
    is_plane_cloud_created = false;

    int mask_i = 0;
    bool maski_found = false;

    for (int hi = 0; hi < (WINDOW_SIZE + 1); hi++) {
        if (mask_cloud->header.stamp.toSec() == estimator.Headers[hi].stamp.toSec()) {
            mask_i = hi;
            maski_found = true;
            break;
        }
    }

        // sensor_msgs::ChannelFloat32 colors;
        // colors.name = "rgb";
    // if (plane_cloud.channels.size() == 0) {
        // plane_cloud.channels.push_back(colors);
    // }
    
    vector<int> cur_pids;
    // for (map<int, array<double, 3>>::iterator it = estimator.para_N.begin(); it != estimator.para_N.end(); ++it){
    for (map<int, Vector4d>::iterator it = plane_params.begin(); it != plane_params.end(); ++it){
        cur_pids.push_back(it->first);
        ROS_INFO("---------------CURRENT PIDs----------------ID = %d --------------------", it->first);
    }

    // for (int pi = 0; pi < estimator.init_pids.size(); pi++) {
    for (int pi = 0; pi < cur_pids.size(); pi++) {
        // int pid = estimator.init_pids[pi];
        int pid = cur_pids[pi];
        // Vector3d normal;
        // normal << estimator.para_N[pid][0], estimator.para_N[pid][1], estimator.para_N[pid][2];
        
        // double a = estimator.para_N[pid][0];
        // double b = estimator.para_N[pid][1];
        // double c = estimator.para_N[pid][2];
        // double d = estimator.para_d[pid][0];

        Vector4d pp = plane_params[pid];

        Vector3d normal;
        normal << pp(0), pp(1), pp(2);
        double d = -pp(3)/normal.norm();
        normal.normalize();

        double a = normal[0];
        double b = normal[1];
        double c = normal[2];
        // double d = plane_params[pid][3];

        // Find the point on plane nearest to origin
        double r = d / (normal.dot(normal));
        Vector3d near_point = r * normal;

        // Find a direction along the plane surface
        Vector3d dir1;
        dir1 << normal(0) + 0.1, normal(1) + 0.1, normal(2) + 0.1;
        dir1.normalize();

        double r2 = d / (normal.dot(dir1));
        dir1 = dir1 * r2;

        dir1 = dir1 - near_point;
        dir1.normalize();

        // Find another direction perpendicular to above direction
        Vector3d dir2;
        dir2 = Utility::skewSymmetric(normal) * dir1;
        dir2.normalize();
        
        // Create meshgrid
        std::vector<Vector3d> mesh_grid;
        for (int i = 0; i < 20; i++) {
            for (int j = 0; j < 20; j++) {
                Vector3d pt;
                pt = near_point + i * dir1 + j * dir2;
                mesh_grid.push_back(pt);
            }
        }

        for (int i = 0; i < mesh_grid.size(); i++) {
            Vector3d xy = mesh_grid[i];

            // xy = (estimator.ric[0] * xy) + estimator.tic[0];
            
            geometry_msgs::Point32 p;
            p.x = xy(0);
            p.y = xy(1);
            p.z = xy(2);
            
            plane_cloud.points.push_back(p);
            
            // int rgb = mask_cloud->channels[0].values[0];
            int rgb = 0x000000;
            colors.values.push_back(rgb);
            
        }
        ROS_INFO("ID=%d, a=%f, b=%f, c=%f, d=%f", pid, a, b, c, d);
    }

    if (maski_found) {
        // ROS_INFO("---------------FOUND----------------ID = %d --------------------", mask_i);
        // for (auto const& pp: plane_params) {
            // if (!is_plane_cloud_created) {
            // Vector3d normal;
            // normal << pp.second[0], pp.second[1], pp.second[2];
            // double d = -pp.second[3]/normal.norm();
            // normal.normalize();

            // ROS_INFO("**************MASK_CLOUD SIZE IS %d **********************",  mask_cloud->points.size());
            // ROS_INFO("**************MASK_CLOUD NUM OF CHANNELS ARE %d **********************",  mask_cloud->channels.size());
            // double *pose_i = estimator.para_Pose[mask_i];
            double pose_i[7];
            for (int po = 0; po < 7; po++) {
                pose_i[po] = estimator.para_Pose[mask_i][po];
            }
            Eigen::Map<Vector3d> ti(pose_i);
            Eigen::Quaterniond qi;
            qi.coeffs() << pose_i[3], pose_i[4], pose_i[5], pose_i[6];

            Isometry3d TIC;
            TIC.linear() = estimator.ric[0];
            TIC.translation() = estimator.tic[0];
            
            Isometry3d Ti;
            Ti.linear() = estimator.Rs[mask_i];
            Ti.translation() = estimator.Ps[mask_i];

            // Isometry3d Tp = TIC.inverse() * Ti.inverse();

            for (unsigned int i = 0; i < mask_cloud->points.size(); i++) {
                int ppid = mask_cloud->channels[1].values[i];
                // ROS_INFO("-----------wanted----PLANE----------------ID = %d --------------------", ppid);
                if (find(cur_pids.begin(), cur_pids.end(), ppid) != cur_pids.end()) {
                // if (find(estimator.init_pids.begin(), estimator.init_pids.end(), ppid) != estimator.init_pids.end()) {
                // ROS_INFO("-----------found----PLANE----------------ID = %d --------------------", ppid);
                Vector4d pp = plane_params[ppid];

                Vector3d normal;
                normal << pp(0), pp(1), pp(2);
                double d = -pp(3)/normal.norm();
                normal.normalize();

                // Vector3d normal;
                // normal << 
                    // estimator.para_N[ppid][0],
                    // estimator.para_N[ppid][1],
                    // estimator.para_N[ppid][2];
                // normal.normalize();

                // double d = estimator.para_d[ppid][0] / normal.norm();

                // Vector4d pp;
                // pp << normal(0), normal(1), normal(2), -d;

                if (d < 100) {

                // ROS_INFO("_-_-_-__-----------___-----------a=%f, b=%f, c=%f, d=%f", normal(0), normal(1), normal(2), d);

                double lambda = 0.0;

                geometry_msgs::Point32 m;
                m = mask_cloud->points[i];
                Vector3d c_point(m.x, m.y, m.z);

                // // transform normal from world camera frame to current camera frame
                // Vector3d normal_i = estimator.ric[0].inverse() * estimator.Rs[mask_i].inverse() * estimator.ric[0] * normal;
                // normal_i.normalize();

                // // transform depth to current camera frame
                // double d_c0 = d; // depth in world camera frame
                // double d_i0 = d + estimator.tic[0].dot(estimator.ric[0] * normal); // depth in world imu frame
                // double d_i = d_i0 - estimator.Ps[mask_i].dot(estimator.ric[0] * normal); // depth in current imu frame
                // double d_ci = d_i + estimator.tic[0].dot(estimator.Rs[mask_i].inverse() * estimator.ric[0] * normal); // depth in current camera frame

                // // find lambda
                // lambda = (d_ci) / (normal_i.dot(c_point));
                // // ROS_INFO("lambda value is %f", lambda);

                // Vector3d pts_i =  lambda * c_point;
                // Vector3d w_pts_i = estimator.Rs[mask_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[mask_i];

                // Vector4d pp_ci = (TIC.inverse() * Ti * TIC).matrix().transpose() * pp;
                Vector4d pp_ci = (Ti * TIC).matrix().transpose() * pp;

                Vector3d normal_ci;
                normal_ci << pp_ci(0), pp_ci(1), pp_ci(2);
                double d_ci = -pp_ci(3)/normal_ci.norm();
                normal_ci.normalize();
                
                lambda = (d_ci) / (normal_ci.dot(c_point));

                c_point = lambda * c_point;

                // Transform c_point (current camera) to imu (current IMU)
                Vector3d i_point = estimator.ric[0] * c_point + estimator.tic[0];

                // Transform current imu point to world imu
                Vector3d i0_point = (estimator.Rs[mask_i] * i_point) + estimator.Ps[mask_i];

                // Transform world imu to world camera
                // Vector3d w_pts_i = estimator.ric[0].transpose() * i0_point - estimator.ric[0].transpose() * estimator.tic[0];
                Vector3d w_pts_i = i0_point;
                // Vector3d w_pts_i = c_point;

                // find lambda
                // lambda = d / (normal.dot(w_pts_i));
                
                // ROS_INFO("lambda value is %g", lambda);
                
                // w_pts_i = lambda * w_pts_i;
                // w_pts_i = estimator.ric[0] * lambda * w_pts_i + estimator.tic[0];

                geometry_msgs::Point32 p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                plane_cloud.points.push_back(p);
                
                int rgb = mask_cloud->channels[0].values[i];
                colors.values.push_back(rgb);
                }
                }
            }
                            
            // double a = pp.second[0];
            // double b = pp.second[1];
            // double c = pp.second[2];
            // double d = pp.second[3];

            // // Find the point on plane nearest to origin
            // double r = -d / (normal.dot(normal));
            // Vector3d near_point = r * normal;

            // // Find a direction along the plane surface
            // Vector3d dir1;
            // dir1 << normal(0) + 0.1, normal(1) + 0.1, normal(2) + 0.1;
            // dir1.normalize();

            // double r2 = -d / (normal.dot(dir1));
            // dir1 = dir1 * r2;

            // dir1 = dir1 - near_point;
            // dir1.normalize();

            // // Find another direction perpendicular to above direction
            // Vector3d dir2;
            // dir2 = Utility::skewSymmetric(normal) * dir1;
            // dir2.normalize();
            
            // // Create meshgrid
            // std::vector<Vector3d> mesh_grid;
            // for (int i = 0; i < 20; i++) {
            //     for (int j = 0; j < 20; j++) {
            //         Vector3d pt;
            //         pt = near_point + i * dir1 + j * dir2;
            //         mesh_grid.push_back(pt);
            //     }
            // }

            // for (int i = 0; i < mesh_grid.size(); i++) {
            //     Vector3d xy = mesh_grid[i];
                
            //     geometry_msgs::Point32 p;
            //     p.x = xy(0);
            //     p.y = xy(1);
            //     p.z = xy(2);
                
            //     plane_cloud.points.push_back(p);
                
            //     // int rgb = mask_cloud->channels[0].values[0];
            //     int rgb = 0x0000ff;
            //     colors.values.push_back(rgb);
            // }
            // is_plane_cloud_created = true;
            // }
        // }

        plane_cloud.channels.push_back(colors);
        pub_plane_cloud.publish(plane_cloud);
    }
    else {
        // ROS_INFO("------------------------POSE NOT FOUND----------------------------------");
    }

    /**
    for (auto const& pp: plane_params) {
        if (!is_plane_cloud_created) {
        Vector3d normal;
        normal << pp.second[0], pp.second[1], pp.second[2];
        // double d = pp.second[3];

        // for (int i = 0; i < mask_cloud->points.size(); i++) {
        //     double lambda = 0.0;

        //     geometry_msgs::Point32 p, m;
        //     m = mask_cloud->points[i];
        //     Vector3d c_point(m.x, m.y, m.z);

        //     // transform normal from world camera frame to current camera frame
        //     Vector3d normal_i = estimator.ric[0].inverse() * estimator.Rs[0].inverse() * estimator.ric[0] * normal;

        //     // transform depth to current camera frame
        //     double d_c0 = d; // depth in world camera frame
        //     double d_i0 = d + estimator.tic[0].dot(estimator.ric[0] * normal); // depth in world imu frame
        //     double d_i = d_i0 - estimator.Ps[0].dot(estimator.ric[0] * normal); // depth in current imu frame
        //     // double d_ci = d_i + estimator.tic[0].dot(estimator.Rs[WINDOW_SIZE].inverse() * estimator.ric[0] * normal);

        //     // find lambda
        //     lambda = (-d_i) / (normal_i.dot(c_point));
        //     // ROS_INFO("lambda value is %f", lambda);

        //     Vector3d pts_i =  lambda * c_point;
        //     Vector3d w_pts_i = estimator.Rs[0] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[0];

        //     p.x = w_pts_i(0);
        //     p.y = w_pts_i(1);
        //     p.z = w_pts_i(2);
        //     plane_cloud.points.push_back(p);
            
        //     int rgb = mask_cloud->channels[0].values[i];
        //     colors.values.push_back(rgb);
        // }
                        
        double a = pp.second[0];
        double b = pp.second[1];
        double c = pp.second[2];
        double d = pp.second[3];

        // Find the point on plane nearest to origin
        double r = -d / (normal.dot(normal));
        Vector3d near_point = r * normal;

        // Find a direction along the plane surface
        Vector3d dir1;
        dir1 << normal(0) + 0.1, normal(1) + 0.1, normal(2) + 0.1;
        dir1.normalize();

        double r2 = -d / (normal.dot(dir1));
        dir1 = dir1 * r2;

        dir1 = dir1 - near_point;
        dir1.normalize();

        // Find another direction perpendicular to above direction
        Vector3d dir2;
        dir2 = Utility::skewSymmetric(normal) * dir1;
        dir2.normalize();
        
        // Create meshgrid
        std::vector<Vector3d> mesh_grid;
        for (int i = 0; i < 20; i++) {
            for (int j = 0; j < 20; j++) {
                Vector3d pt;
                pt = near_point + i * dir1 + j * dir2;
                mesh_grid.push_back(pt);
            }
        }

        for (int i = 0; i < mesh_grid.size(); i++) {
            Vector3d xy = mesh_grid[i];
            
            geometry_msgs::Point32 p;
            p.x = xy(0);
            p.y = xy(1);
            p.z = xy(2);
            
            plane_cloud.points.push_back(p);
            
            // int rgb = mask_cloud->channels[0].values[0];
            int rgb = 0x0000ff;
            colors.values.push_back(rgb);
        }
        // is_plane_cloud_created = true;
        }
    }
    **/

    // int colour_counter = 0;
    // for (auto const& reg_pt: reg_points) {
    //     vector<Vector4d> pts = reg_pt.second;

    //     for (int pti = 0; pti < pts.size(); pti++) {
    //         geometry_msgs::Point32 p;
    //         p.x = pts[pti](0);
    //         p.y = pts[pti](1);
    //         p.z = pts[pti](2);

    //         plane_cloud.points.push_back(p);

    //         int rgb = RGB_HEX[colour_counter % 5];
    //         colors.values.push_back(rgb);
    //     }

    //     colour_counter++;
    // }

    /* Plane visualized END --------------------------------------------------------------------------------*/

    // pub margined potin
    sensor_msgs::PointCloud margin_cloud;
    margin_cloud.header = header;

    for (auto &it_per_id : estimator.f_manager.feature)
    { 
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        //if (it_per_id->start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id->solve_flag != 1)
        //        continue;

        // if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2 
            // && it_per_id.solve_flag == 1 )
        // {
            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

            geometry_msgs::Point32 p;
            p.x = w_pts_i(0);
            p.y = w_pts_i(1);
            p.z = w_pts_i(2);
            margin_cloud.points.push_back(p);
        // }
    }
    pub_margin_cloud.publish(margin_cloud);
}


void pubTF(const Estimator &estimator, const std_msgs::Header &header)
{
    if( estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    // body frame
    Vector3d correct_t;
    Quaterniond correct_q;
    correct_t = estimator.Ps[WINDOW_SIZE];
    correct_q = estimator.Rs[WINDOW_SIZE];

    transform.setOrigin(tf::Vector3(correct_t(0),
                                    correct_t(1),
                                    correct_t(2)));
    q.setW(correct_q.w());
    q.setX(correct_q.x());
    q.setY(correct_q.y());
    q.setZ(correct_q.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "world", "body"));

    // camera frame
    transform.setOrigin(tf::Vector3(estimator.tic[0].x(),
                                    estimator.tic[0].y(),
                                    estimator.tic[0].z()));
    q.setW(Quaterniond(estimator.ric[0]).w());
    q.setX(Quaterniond(estimator.ric[0]).x());
    q.setY(Quaterniond(estimator.ric[0]).y());
    q.setZ(Quaterniond(estimator.ric[0]).z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "body", "camera"));

    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = estimator.tic[0].x();
    odometry.pose.pose.position.y = estimator.tic[0].y();
    odometry.pose.pose.position.z = estimator.tic[0].z();
    Quaterniond tmp_q{estimator.ric[0]};
    odometry.pose.pose.orientation.x = tmp_q.x();
    odometry.pose.pose.orientation.y = tmp_q.y();
    odometry.pose.pose.orientation.z = tmp_q.z();
    odometry.pose.pose.orientation.w = tmp_q.w();
    pub_extrinsic.publish(odometry);

}

void pubKeyframe(const Estimator &estimator)
{
    // pub camera pose, 2D-3D points of keyframe
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR && estimator.marginalization_flag == 0)
    {
        int i = WINDOW_SIZE - 2;
        //Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Vector3d P = estimator.Ps[i];
        Quaterniond R = Quaterniond(estimator.Rs[i]);

        nav_msgs::Odometry odometry;
        odometry.header = estimator.Headers[WINDOW_SIZE - 2];
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();
        //printf("time: %f t: %f %f %f r: %f %f %f %f\n", odometry.header.stamp.toSec(), P.x(), P.y(), P.z(), R.w(), R.x(), R.y(), R.z());

        pub_keyframe_pose.publish(odometry);


        sensor_msgs::PointCloud point_cloud;
        point_cloud.header = estimator.Headers[WINDOW_SIZE - 2];
        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int frame_size = it_per_id.feature_per_frame.size();
            if(it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.start_frame + frame_size - 1 >= WINDOW_SIZE - 2 && it_per_id.solve_flag == 1)
            {

                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0])
                                      + estimator.Ps[imu_i];
                geometry_msgs::Point32 p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                point_cloud.points.push_back(p);

                int imu_j = WINDOW_SIZE - 2 - it_per_id.start_frame;
                sensor_msgs::ChannelFloat32 p_2d;
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.y());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.y());
                p_2d.values.push_back(it_per_id.feature_id);
                point_cloud.channels.push_back(p_2d);
            }

        }
        pub_keyframe_point.publish(point_cloud);
    }
}

void pubRelocalization(const Estimator &estimator)
{
    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(estimator.relo_frame_stamp);
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = estimator.relo_relative_t.x();
    odometry.pose.pose.position.y = estimator.relo_relative_t.y();
    odometry.pose.pose.position.z = estimator.relo_relative_t.z();
    odometry.pose.pose.orientation.x = estimator.relo_relative_q.x();
    odometry.pose.pose.orientation.y = estimator.relo_relative_q.y();
    odometry.pose.pose.orientation.z = estimator.relo_relative_q.z();
    odometry.pose.pose.orientation.w = estimator.relo_relative_q.w();
    odometry.twist.twist.linear.x = estimator.relo_relative_yaw;
    odometry.twist.twist.linear.y = estimator.relo_frame_index;

    pub_relo_relative_pose.publish(odometry);
}

void pubPlaneCloud(Estimator &estimator, const std_msgs::Header &header, const sensor_msgs::PointCloudConstPtr &mask_cloud)
{
    /**
     * Requirements:
     * image - for texture in the dense cloud of the plane segment
     * largest plane mask - for filtering plane pixels
     * plane parameters - of largest plane
     * pose - of the current image frame
     * K - for lifting points to normalized plane
     **/
    
    // publish dense point cloud of masked pixels with color/texture from feature tracker
    // estimate depth of the points using the plane params
    // republish dense point cloud with correct depth

    // - OR -

    // create a partial depth map from plane parameters and plane mask
    // simply lift plane pixels using this partial depth map

    sensor_msgs::PointCloud plane_cloud;
    plane_cloud.header = header;

    /**
     * Format
     * pointcloud
     *      channels
     *          name - rgb
     *          values - rgb values of the points
     *      points
     **/
    sensor_msgs::ChannelFloat32 colors;
    colors.name = "rgb";
    // for (int i = 0; i < 50; i++) {
    //     for (int j = 0; j < 50; j++) {
    //         geometry_msgs::Point32 p;
    //         p.x = i;
    //         p.y = j;
    //         p.z = 0.0;
    //         plane_cloud.points.push_back(p);
            
    //         int rgb = 0xaaff00; float float_rgb = *reinterpret_cast<float*>(&rgb);
    //         colors.values.push_back(rgb);
    //     }
    // }
    int plane_id = 39;
    Vector3d normal;
    normal << estimator.para_N[plane_id][0], estimator.para_N[plane_id][1], estimator.para_N[plane_id][2];
    double d = estimator.para_d[plane_id][0];
    ROS_INFO("Plane %d, d value is %f", plane_id, d);
    
    // for (int pid = 0; pid < estimator.para_d.size(); pid++)
        // ROS_INFO("Plane %d, d value is %f", pid, estimator.para_d[pid][0]);

    for (int i = 0; i < mask_cloud->points.size(); i++) {
        double lambda = 0.0;

        geometry_msgs::Point32 p, m;
        m = mask_cloud->points[i];
        Vector3d m_point(m.x, m.y, m.z);

        lambda = (-d - (normal.dot(estimator.Ps[WINDOW_SIZE]))) / (normal.dot(estimator.Rs[WINDOW_SIZE] * (estimator.ric[0] * m_point + estimator.tic[0])));
        // ROS_INFO("lambda value is %f", lambda);

        p.x = lambda * m.x;
        p.y = lambda * m.y;
        p.z = lambda * m.z;
        plane_cloud.points.push_back(p);
         
        int rgb = mask_cloud->channels[0].values[i];
        colors.values.push_back(rgb);
    }
    plane_cloud.channels.push_back(colors);
    pub_plane_cloud.publish(plane_cloud);
}