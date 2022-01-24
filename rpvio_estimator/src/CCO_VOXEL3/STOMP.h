#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>    
#include <fstream>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "eigenmvn.h"

namespace Initilizer
{


	class STOMP
	{

	public:

		std::vector<Eigen::MatrixXd> InitSTOMP( Eigen::Vector3d Start ,  Eigen::Vector3d Goal  ,  int num_samples , ros::Publisher PubSTOMP , bool PubTraj  , 
		int NumTrajSamples  );



	};
} 





int VisulizeTrajectory( Eigen::MatrixXd  x_samples , Eigen::MatrixXd y_samples , Eigen::MatrixXd z_samples , ros::Publisher PubSTOMP  )
{



int color_counter = 0;
int num_samples = y_samples.rows() ; 
int num_pts_per_traj = y_samples.cols() ; 

for( int i =0 ; i < num_samples ; i++){
color_counter++;
visualization_msgs::Marker marker;
marker.type = visualization_msgs::Marker::LINE_LIST;
marker.action = visualization_msgs::Marker::ADD;

geometry_msgs::Point pt;

for( int j =0 ; j< num_pts_per_traj ; j++){

// visualization_msgs::Marker marker;
marker.header.frame_id = "map";
marker.header.stamp = ros::Time();
marker.ns = "my_namespace";
marker.id = color_counter;


pt.x = x_samples( i, j );
pt.y  = y_samples( i, j );
pt.z =z_samples( i, j );

marker.points.push_back(pt);


marker.scale.x = 0.1;
marker.scale.y = 0.1;
marker.scale.z = 0.1;
marker.color.a = 1; 
marker.color.r = 1.0 ; //(num_samples - float(color_counter))/num_samples;
marker.color.g = 0.5;
marker.color.b = 0.2*(num_samples - float(color_counter))/num_samples;
marker.pose.orientation.x = 0.0;
marker.pose.orientation.y = 0.0;
marker.pose.orientation.z = 0.0;
marker.pose.orientation.w = 1.0;
// std::cout << TopX( i, j ) << "  " << TopY( i, j ) << "  " << TopZ( i, j ) << std::endl;

}

marker.header.frame_id = "map";
marker.header.stamp = ros::Time();
marker.ns = "my_namespace";
marker.id = color_counter;


pt.x = x_samples( i, 74 );
pt.y  = y_samples( i, 74 );
pt.z =z_samples( i, 74 );

marker.points.push_back(pt);


marker.scale.x = 0.1;
marker.scale.y = 0.1;
marker.scale.z = 0.1;
marker.color.a = 1; 
marker.color.r = 1.0 ; //(num_samples - float(color_counter))/num_samples;
marker.color.g = 0.5;
marker.color.b = 0.2*(num_samples - float(color_counter))/num_samples;
marker.pose.orientation.x = 0.0;
marker.pose.orientation.y = 0.0;
marker.pose.orientation.z = 0.0;
marker.pose.orientation.w = 1.0;



PubSTOMP.publish( marker);
} 


return 0; 

}




std::vector<Eigen::MatrixXd> Initilizer::STOMP::InitSTOMP( Eigen::Vector3d Start ,  Eigen::Vector3d Goal  ,  int num_samples , ros::Publisher PubSTOMP , bool PubTraj  , 
	int NumTrajSamples    )
{

	std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_int_distribution<uint64_t> dis;



	// num_samples = number of samples in trajectory 

	double dex =  (Goal.x() - Start.x())/  num_samples;
	double dey =  (Goal.y() - Start.y())/  num_samples;
	double dez =  (Goal.z() - Start.z())/  num_samples;

	Eigen::MatrixXd StraightTraj(  num_samples+1 , 3 );

	StraightTraj.row(0) = Start ; 
	Eigen::Vector3d Temp; 
	double x,y,z; 



	for(int i =0 ; i < num_samples ; i++ ){

		x = Start.x() + dex*(i+1);
		y = Start.y() + dey*(i+1);
		z = Start.z() + dez*(i+1);

		Temp << x,y,z; 
		StraightTraj.row(i+1) = Temp; 

	}

	Eigen::MatrixXd A( num_samples , num_samples) ;
	Eigen::MatrixXd A_diff( num_samples-1 , num_samples) ;

	Eigen::MatrixXd A_diff2( num_samples-2 , num_samples) ;



	A = Eigen::MatrixXd::Identity( num_samples , num_samples );

	for(int i =0 ; i < num_samples-1 ; i++){
		for(int j =0 ; j < num_samples ; j++){

			A_diff(i, j ) =  A(i+1 , j ) - A(i, j ); 
		}
	}

	
	for(int i =0 ; i < num_samples-2 ; i++){
		for(int j =0 ; j < num_samples ; j++){

			A_diff2(i, j ) =  A_diff(i+1 , j ) - A_diff(i, j ); 
		}
	}

	Eigen::MatrixXd A_mat( num_samples+2 , num_samples);

	// std::cout<< A_diff2.rows() << "  " << A_diff2.cols() <<  std::endl;
	A_mat = Eigen::MatrixXd::Zero(  num_samples+2 , num_samples );
	A_mat(0,0) = 1;
	A_mat(1,0) = -2;
	A_mat(1,1) = 1 ;

	A_mat.block( 2, 0 , num_samples-2 , num_samples  ) = A_diff2 ; 

	A_mat( num_samples  ,num_samples-1   ) = -2;
	A_mat( num_samples  ,num_samples  -2 ) = 1;
	A_mat( num_samples +1  ,num_samples-1  ) = 1;

	A_mat = - A_mat;

	Eigen::MatrixXd R ;
	Eigen::MatrixXd cov ; 

	R = A_mat.transpose() *A_mat; 
	cov = 0.005*R.inverse(); 

	// std::cout<< cov.rows() << "  " << cov.cols() <<  std::endl;


	Eigen::MatrixXd mu = Eigen::MatrixXd::Zero(num_samples, 1);


    Eigen::EigenMultivariateNormal<double> *normX_solver = new Eigen::EigenMultivariateNormal<double>(mu, 0.5*cov, true, dis(gen));
    Eigen::EigenMultivariateNormal<double> *normY_solver = new Eigen::EigenMultivariateNormal<double>(mu, 0.5*cov, true, dis(gen));
    Eigen::EigenMultivariateNormal<double> *normZ_solver = new Eigen::EigenMultivariateNormal<double>(mu, 0.5*cov, true, dis(gen));


    Eigen::MatrixXd eps_kx = normX_solver->samples(NumTrajSamples).transpose();
    Eigen::MatrixXd eps_ky = normY_solver->samples(NumTrajSamples).transpose();
    Eigen::MatrixXd eps_kz = normZ_solver->samples(NumTrajSamples).transpose();

    double x_fin = Goal.x();
    double y_fin = Goal.y();
    double z_fin = Goal.z();
    int t_fin =5 ;

    Eigen::VectorXd t_interp = Eigen::VectorXd::LinSpaced(num_samples, 0, t_fin);
    Eigen::VectorXd x_interp = (Start.x() + ((x_fin-Start.x())/t_fin) * t_interp.array()).matrix();
    Eigen::VectorXd y_interp = (Start.y() + ((y_fin-Start.y())/t_fin) * t_interp.array()).matrix();
    Eigen::VectorXd z_interp = (Start.z() + ((z_fin-Start.z())/t_fin) * t_interp.array()).matrix();

    Eigen::MatrixXd x_samples(eps_kx.rows(), eps_kx.cols());
    Eigen::MatrixXd y_samples(eps_kx.rows(), eps_kx.cols());
    Eigen::MatrixXd z_samples(eps_kx.rows(), eps_kx.cols());	

    x_samples = eps_kx;
    x_samples.rowwise() += x_interp.transpose(); //  shape = ( num_trajectories , num_samples_per_trajectory)
    y_samples = 0.0*eps_ky;
    y_samples.rowwise() += y_interp.transpose();
    z_samples = eps_kz;
    z_samples.rowwise() += z_interp.transpose();

    std::vector<Eigen::MatrixXd> STOMPTraj;
    STOMPTraj.push_back( x_samples);
    STOMPTraj.push_back( y_samples);
    STOMPTraj.push_back( z_samples);



    if( PubTraj == true)
    {

    	VisulizeTrajectory(   x_samples ,  y_samples , z_samples ,  PubSTOMP  ) ;
    }


	return STOMPTraj; 

}