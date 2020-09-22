import multirotor.setup_path 
import airsim
import numpy as np
import os
import tempfile
import pprint
import cv2

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
import pdb;pdb.set_trace();
state = client.getMultirotorState()
airsim.wait_key('Press any key to takeoff')
client.takeoffAsync().join()
name = 0
x_val,y_val,z_val = 0,0,-30
#while True:
# pitch = float(input("Pitch"))
# roll = float(input("Roll"))
# z = float(input("Z"))
# yaw = float(input("Yaw"))
# dur = float(input("Duration"))
# client.moveByRollPitchYawZAsync(roll,pitch,yaw,z,dur).join()    
client.moveToPositionAsync(x_val,y_val,z_val,5,yaw_mode=airsim.YawMode(is_rate=False)).join()  
client.hoverAsync().join()
airsim.wait_key('Press any key to move')
