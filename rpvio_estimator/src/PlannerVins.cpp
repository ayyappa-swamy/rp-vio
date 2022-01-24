#include <chrono>
#include "utilsPlannner.h"
#include "objects.h"
#include"parameters.h"
#include"kinodynamic_astar2.h"
#include "planner.h"
#include"STOMP.h"
#include"CEM.h"



fast_planner::KinodynamicAstar kAstar;
Initilizer::STOMP STOMPTrajectories ; 
 
Optimization::TrajectoryOptimization CEMOptim; 

Eigen::Vector3d startVel;
Eigen::Vector3d startAcc; 
Eigen::Vector3d goalVel; 

Eigen::MatrixXd x_samples; 
Eigen::MatrixXd y_samples;
Eigen::MatrixXd z_samples; 

ros::Publisher PubSTOMP; 
ros::Publisher CEMOptimTraj;

bool status ;
double deltaT = 0.1;  
nav_msgs::Path AstarTrajectory;
int numIters = 10;

ros::Publisher AstarTraj ; 

bool In= true ; 


Eigen::Vector3d PrevState;
Eigen::Vector3d CurState; 


int cnt =0 ;
int NumTraj_perturb = 75;
int NumPts_perTraj ; 

void current_state_callback2( const sensor_msgs::PointCloudConstPtr &frames_msg, const nav_msgs::OdometryConstPtr &odometry_msg )
{

    CurState << odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y,
                 odometry_msg->pose.pose.position.z ; 

    
    if( (cnt%10) == 0  ){

    
    In = true ; 

	Eigen::Vector3d goal( 25.0, -5.0, 5.0 );

    Eigen::Isometry3d Tic;

    Eigen::Matrix3d R; 
    Eigen::Vector3d T ; 

    R << 0,0,1,1,0,0,0,1,0 ;
    T << 0.50, 0, 0 ; 


    Tic.linear() = R  ;  //RIC[0];
    Tic.translation() = T  ; //TIC[0];



    Eigen::Vector3d trans;
    trans <<
        odometry_msg->pose.pose.position.x,
        odometry_msg->pose.pose.position.y,
        odometry_msg->pose.pose.position.z;

    double quat_x = odometry_msg->pose.pose.orientation.x;
    double quat_y = odometry_msg->pose.pose.orientation.y;
    double quat_z = odometry_msg->pose.pose.orientation.z;
    double quat_w = odometry_msg->pose.pose.orientation.w;
    Eigen::Quaterniond quat(quat_w, quat_x, quat_y, quat_z);

    Eigen::Isometry3d Ti;
    Ti.linear() = quat.normalized().toRotationMatrix();
    Ti.translation() = trans;

    // Transform to local frame
    Eigen::Vector3d local_goal = Tic.inverse() * (Ti.inverse() * goal);
    local_goal[1] = 0.0;
    double local_goal_distance = local_goal.norm();
    Eigen::Vector3d local_goal_dir = local_goal.normalized();

    Eigen::Vector3d MapStart  ; 
    MapStart << -10 , -10 , -10 ; 
    Eigen::Vector3d MapEnd ; 
    MapEnd << 100 , 100 , 100 ; 

    Eigen::Vector3d StartPose; 

    StartPose = trans; 


    std::vector<Eigen::Vector3d>  Centers ;

    for (unsigned int i = 0; i < frames_msg->points.size(); i += 8)
    {
        int p_id = (frames_msg->channels[0].values[i]);

        Point center(0, 0, 0);
        std::vector<Eigen::Vector3d> vertices;

        for (int vid = 0; vid < 8; vid++)
        {
            geometry_msgs::Point32 gpt =  frames_msg->points[i + vid];
            Point pt(gpt.x, gpt.y, gpt.z);
            vertices.push_back(pt);

            center += pt;
        }

        center = center/8;
        center = (Ti*Tic)*center ; 


        Centers.push_back(center);
    }


    kAstar.init( MapStart  , MapEnd , StartPose );

    if( cnt == 0 ){


    status  = kAstar.search( StartPose ,startVel ,  startAcc , goal  , goalVel , true, false , 0.0 ,  Centers);
    }
    else{


        status  = kAstar.search( StartPose ,startVel ,  startAcc , goal  , goalVel , false, false , 0.0 ,  Centers);

    }

    std::vector<Eigen::Vector3d> currTraj = kAstar.getKinoTraj(deltaT);

    int numPts = currTraj.size() ; 
    // std::cout << numPts << std::endl;
    x_samples = Eigen::MatrixXd::Zero( 1 ,numPts); 
    y_samples = Eigen::MatrixXd::Zero(1 ,numPts);
    z_samples= Eigen::MatrixXd::Zero(1 ,numPts ); 
    int pos_cnt =0 ;

    for(auto i = currTraj.begin(); i!=currTraj.end(); i++)
    {
                geometry_msgs::PoseStamped p;
                Eigen::Vector3d pos = *i;


                p.pose.position.x = float( pos(0) );
                p.pose.position.y = float(pos(1) );
                p.pose.position.z = float(pos(2) );


                x_samples(0 , pos_cnt) = pos(0);
                y_samples(0, pos_cnt ) =pos(1);
                z_samples(0 , pos_cnt ) = pos(2);
                pos_cnt +=1; 


                p.pose.orientation.w = float(1.0);

                AstarTrajectory.poses.push_back(p);
                AstarTrajectory.header.stamp = ros::Time::now();
                AstarTrajectory.header.frame_id = "/world";
    }

    AstarTraj.publish(AstarTrajectory);
    PrevState = CurState ;


    std::vector<Eigen::MatrixXd> initTrajectory;


    initTrajectory.push_back(x_samples );
    initTrajectory.push_back(y_samples);
    initTrajectory.push_back(z_samples);

    NumPts_perTraj = numPts ; 
    bool PubSTOMP_bin = false ;

    std::vector<Eigen::MatrixXd> STOMPTraj ;

    STOMPTraj = STOMPTrajectories.PerturbAstar( initTrajectory , NumTraj_perturb ,
    NumPts_perTraj ,PubSTOMP , PubSTOMP_bin  );

    Eigen::MatrixXd xPts;
    Eigen::MatrixXd yPts;
    Eigen::MatrixXd zPts; 
        std::cout << "Here " << std::endl;


    xPts = STOMPTraj.at(0);
    yPts = STOMPTraj.at(1);
    zPts = STOMPTraj.at(2);

    std::cout << "Here " << std::endl;

    CEMOptim.CrossEntropyOptimize(xPts ,yPts , zPts , numIters ,   Centers ,CEMOptimTraj , PubSTOMP );





    std::cout << " Start " << " " << x_samples(0, 0 ) << " " << y_samples(0 ,0 ) << " " << z_samples(0,0) << std::endl; 
    std::cout << " End " << "  " << x_samples(0 ,numPts-1) << " " << y_samples(0 ,numPts-1 ) << " " << z_samples(0,numPts-1) << std::endl; 
    std::cout << " ---------------------------" << std::endl;





    kAstar.reset(); 
}

std::cout << cnt << std::endl ;
cnt += 1; 

// AstarTraj.publish(AstarTrajectory);



}



int main(int argc, char **argv)
{

	ros::init(argc, argv, "VinsPlanner");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    message_filters::Subscriber<sensor_msgs::PointCloud> sub_frame_cloud(n, "/rpvio_mapper/frame_cloud", 20);
    message_filters::Subscriber<nav_msgs::Odometry> sub_odometry(n, "/vins_estimator/odometry", 20);
    PubSTOMP = n.advertise<visualization_msgs::Marker>( "/STOMP_vis", 0 );
    CEMOptimTraj = n.advertise<visualization_msgs::Marker>( "/OptimizedTraj", 0 );

    startVel = Eigen::Vector3d::Zero();
    startAcc = Eigen::Vector3d::Ones(); 
    goalVel  = Eigen::Vector3d::Zero();

    AstarTraj = n.advertise<nav_msgs::Path>( "/AstarTraj", 0 );
    kAstar.setParam(n);



    message_filters::TimeSynchronizer<sensor_msgs::PointCloud, nav_msgs::Odometry> sync2(
        sub_frame_cloud,
        sub_odometry,
        100);
    
  
    sync2.registerCallback(boost::bind(&current_state_callback2, _1, _2));

	

   

    ros::spin();







	return 0; 

}