from os import pipe
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Path
# from geometry_msgs import PoseStamped 
from nav_msgs.msg import Odometry

import airsim
import cvxpy
import numpy as np

import signal
import sys

path = []
client = airsim.MultirotorClient(ip='10.2.36.227')

## offset is measured in world frame 

xoffset = 0.5 
yoffset = 0 
zoffset = 2.7


cnt =0 
xPose = 0 
yPose = 0 
zPose = 0 



def path_cb(path_data):
	global cnt
	global xPose , yPose , zPose 

	# print( xPose , yPose , zPose)

	way_point_len = len(path_data.poses)
	

	yval = -path_data.poses[-1].pose.position.x
	xval = -path_data.poses[-1].pose.position.y
	zval = path_data.poses[-1].pose.position.z 

	print( xval, yval, zval  )


	EnvInfo = client.getMultirotorState()
	current_position = EnvInfo.kinematics_estimated.position


	print( " xval, yval, zval  " + str( ( xval, yval, zval)  ))


	# if( cnt == 0 or cnt == 1 ):

	xoffset = current_position.x_val  - yPose
	yoffset = current_position.y_val - xPose 
	zoffset = current_position.z_val  - zPose


	print( "xoffset , yoffset , zoffset " + str( (xoffset , yoffset , zoffset) ))
	print( " Init Pose " + str( (xPose , yPose , zPose ) ) )

	print(" Current Pose " + str( (current_position.x_val , current_position.y_val ,current_position.z_val) ))

	print( " result " + str(  (xval +xoffset , yval + yoffset , zval  ) ))

	# print( xval +xoffset , yval + yoffset , zval + zoffset ) 
	print(" ++++++++++++" + str( way_point_len) +" ++++++++ ")


	client.moveToPositionAsync(  xval  , yval   , - (zval - (-2) ) , 0.5, np.inf, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(True, 0.5)).join()





def VinsState_cb( VinsPose):

	global cnt 

	global xPose , yPose , zPose

	# print(" In VINS ")


	# if( cnt == 0 ):

	xPose = VinsPose.pose.pose.position.x
	yPose = VinsPose.pose.pose.position.y 
	zPose = VinsPose.pose.pose.position.z  

	cnt +=1 

	
	



def listener():

    rospy.init_node('path_commander', anonymous=True)

    print(" INSIDE listener ")
    rospy.Subscriber("CEMPath" , Path ,path_cb , queue_size= 10 )
    rospy.Subscriber("/vins_estimator/odometry" , Odometry ,VinsState_cb , queue_size= 10 )


    # rospy.Timer(rospy.Duration(2), control_update_callback)

    rospy.spin()








if __name__ == '__main__':
    listener()