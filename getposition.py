import multirotor.setup_path 
import airsim
import numpy as np
import os
import tempfile
import pprint
import cv2
import time
# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

#Get drone position
while True:
    print("\n   x value", np.round(client.simGetVehiclePose().position.x_val))
    print("   y_value", np.round(client.simGetVehiclePose().position.y_val))
    print("   z_value", np.round(client.simGetVehiclePose().position.z_val))
    time.sleep(3)
