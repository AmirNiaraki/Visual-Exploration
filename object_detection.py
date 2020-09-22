import multirotor.setup_path 
import airsim
import numpy as np
import os
import tempfile
import pprint
import cv2

def object_detection(img, client, lower=[100,60,150], upper=[110,70,160]):
    bb_list,label,prev_frame = [],{},0
    lower = np.array([100,60,150], dtype = "uint8")
    upper = np.array([110,70,160], dtype = "uint8")
    mask = cv2.inRange(img,lower,upper)
    number,labels,stats,centroids = cv2.connectedComponentsWithStats(mask,8,cv2.CV_32S)
    print ('Number:',number)
    #here we need the drone and camera location/orientation to use in transformation matrix to find the global position of the bounding box
    #list of (x_i,y_i) for the i_th object
    #x_pixel= topleft corner pixel number (0-255), x_val (0-630), x_global(63000):  x_global=x_val*100+(x_pixel-127.5)*(z_val*100)/134.48
    drone_position=client.simGetVehiclePose().position
    for num,i in enumerate(stats):
        if i[2] == mask.shape[1] or i[3] == mask.shape[0]:
            print ('Frame:',(i[0],i[1]),(i[0]+i[2],i[1]+i[3]))
            cen = centroids[num]
            continue
        bb = [(i[0],i[1]),(i[0]+i[2],i[1]+i[3])]        
        
        # finding the global position (0-63000) of the topleft corner of the bounding box with camera focal length of 134.48
        #object_global_pose=drone_position.x_val*100+(i[0]-127.5)*(drone_position.z_val*100)/134.48
        cen = centroids[num]
        #print (num,':',bb)
        bb_list.append(bb)

    return bb_list,drone_position.x_val,drone_position.y_val,cen
if __name__ == "__main__":                                                                                                                              
    #connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    name = 9
    client.hoverAsync().join()
    segment_id = client.simSetSegmentationObjectID('OrangeBall[\w]*',6,True)
    while True:
        responses = client.simGetImages([airsim.ImageRequest("3", airsim.ImageType.Segmentation,False,False)])
        for resp in responses:
            img = np.fromstring(resp.image_data_uint8, dtype=np.uint8)
            img = img.reshape(resp.height, resp.width, 3)
            boxes,x,y,centroid = object_detection(img, client)
            print ('Box:',boxes,'Centroid:',centroid)
            for i in boxes:
                cv2.rectangle(img,i[0],i[1],(0,255,0),2)
            cv2.imshow('Stream',img)
            key = cv2.waitKey(1) & 0xFF;
            if (key == 27 or key == ord('q') or key == ord('x')):
                print ('Pressed q')                
                bb_str = str(boxes).strip('[]')
                name += 1
                cv2.imwrite('./Images/'+str(name)+'.jpg',img)
                f = open("image_locations.txt", "a")
                f.write("Bounding Box:"+bb_str+"Centroid:"+str(centroid)+"\n")
                f.close()        