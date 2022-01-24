namespace Visualizer
{


	class ObstacleVis
	{

	public:

		int visulize_obstacles(Eigen::MatrixXd Center,  int num_obs ,  ros::Publisher PubObstacle  );



	};
} 


int Visualizer::ObstacleVis::visulize_obstacles(Eigen::MatrixXd Center,  int num_obs ,  ros::Publisher PubObstacle  )
{

double xPt ,  yPt , zPt ; 
visualization_msgs::Marker marker;
marker.type = visualization_msgs::Marker::CUBE_LIST;

for(int j =0 ; j < num_obs ; j++)
	{
		geometry_msgs::Point pt;

		pt.x = Center.row(j)(0);
		pt.y = Center.row(j)(1);
		pt.z = Center.row(j)(2);

		// std::cout <<xPt << "  " << yPt << " "<< zPt << std::endl;

		
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time();
		marker.ns = "my_namespace";
		marker.id = j;
		
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = xPt;
		marker.pose.position.y = yPt;
		marker.pose.position.z = zPt;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.points.push_back(pt);

		marker.scale.x = 1.5;
		marker.scale.y = 1.5;
		marker.scale.z = 1.5;
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		//only if using a MESH_RESOURCE marker type:
		


	}
	PubObstacle.publish( marker );


	return 0; 
}