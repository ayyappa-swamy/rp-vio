#include "planer.h"

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rpvio_planner");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);

    message_filters::Subscriber<sensor_msgs::PointCloud> sub_frame_cloud(n, "/rpvio_estimator/frame_cloud", 10);
    message_filters::Subscriber<nav_msgs::Odometry> sub_odometry(n, "/rpvio_estimator/odometry", 10);

    message_filters::TimeSynchronizer<sensor_msgs::PointCloud, nav_msgs::Odometry> sync(
        sub_frame_cloud,
        sub_odometry,
        5
    );
    sync.registerCallback(boost::bind(&current_state_callback, _1, _2));

    pub_paths = n.advertise<nav_msgs::Path>("gaussian_path", 1000);
    ros::spin();

    return 0;
}
