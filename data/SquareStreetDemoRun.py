import airsim
import numpy as np
import time

# Read the path file
path = np.loadtxt('SquareStreetDemoPath.txt')

# Initialize the transform
R = np.array([
    [   0.707,  -0.707],
    [   0.707,  0.707 ]
])

# Transform first piece of trajectory

# invoke airsim path api
client = airsim.MultirotorClient(ip='10.2.36.227')
client.confirmConnection()
client.reset()
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()
client.hoverAsync().join()

new_path = []


for pose in path[15::3]:
    pt_2d = -np.array([[pose[1]],[pose[0]]])
    x_val, y_val = R.dot(pt_2d).flatten()
    z_val = pose[-1]
    new_path.append(airsim.Vector3r(x_val, y_val, -(z_val+2.45)))

    #client.moveToPositionAsync(x_val, y_val, -5, 1, 10, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(is_rate=False, yaw_or_rate=0.0)).join()
    #time.sleep(5)

client.moveOnPathAsync(new_path, 0.25, 1000, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(is_rate=False, yaw_or_rate=0.0)).join()

client.landAsync(timeout_sec=10).join()
client.armDisarm(False)
client.reset()
client.enableApiControl(False)
