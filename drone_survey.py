from multirotor import setup_path 
import airsim

import sys
import time
import argparse
#if we increase the size of the landscape we need to change the "quad=63"
def start(survey_path,x,y,z,z_min=-30,z_max=-60,velocity=50,quad=31):
    #path = [airsim.Vector3r(i,i,self.z) for i in range(0,self.size,15)] 
    client = airsim.MultirotorClient()
#    client.confirmConnection()
#    client.enableApiControl(True) 
    scale_x = quad * (10/x)
    scale_y = quad * (10/y)    
    path = [airsim.Vector3r(p[0]*scale_x+scale_x/2,p[1]*scale_y+scale_y/2,z_min+p[2]*(z_max-z_min)/z) for p in survey_path]      
    #print("Started drone_survey...")
#    client.armDisarm(True)
#    client.takeoffAsync().join()
    # landed = client.getMultirotorState().landed_state
    # if landed == airsim.LandedState.Landed:
    #     print("taking off...")
    #     client.takeoffAsync().join()

    # landed = client.getMultirotorState().landed_state
    # if landed == airsim.LandedState.Landed:
    #     print("takeoff failed - check Unreal message log for details")
    #     return        
    
    # client.hoverAsync().join()
    # time.sleep(2)
#
   # after hovering we need to re-enabled api control for next leg of the trip
    #client.enableApiControl(True)                        
#    print ('Path(Unreal Coordinates):',path)
    s = 1
    try:
#        client.simSetCameraOrientation("3", airsim.to_quaternion(0, 0, 0))
        for i,p in enumerate(path,start=1):
            #import pdb;pdb.set_trace();
#            [(0,0,0),(2,2,0)]
#            print(p.x_val,p.y_val,p.z_val)
            client.moveToPositionAsync(p.x_val,p.y_val,p.z_val,velocity,yaw_mode=airsim.YawMode(is_rate=False)).join()      
            #client.enableApiControl(True)                        
    except:
        errorType, value, traceback = sys.exc_info()
        print("moveOnPath threw exception: " + str(value))
        pass
    # print("Finished drone_survey...")
#    print("flying back home")
#    client.moveToPositionAsync(0, 0, 0, velocity).join()
            

#    print("landing...")
#    client.landAsync().join()
#
#    print("disarming.")
#    client.armDisarm(False)
#    breaker.value = 0

if __name__ == "__main__":
    # args = sys.argv
    # args.pop(0)
    # arg_parser = argparse.ArgumentParser("Usage: survey boxsize stripewidth altitude")
    # arg_parser.add_argument("--size", type=float, help="size of the box to survey", default=50)
    # arg_parser.add_argument("--stripewidth", type=float, help="stripe width of survey (in meters)", default=10)
    # arg_parser.add_argument("--altitude", type=float, help="altitude of survey (in positive meters)", default=20)
    # arg_parser.add_argument("--speed", type=float, help="speed of survey (in meters/second)", default=5)
    # args = arg_parser.parse_args(args)
    survey_path = [(0, 0),(1,1)]
    start(survey_path)
    
