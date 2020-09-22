from object_detection import object_detection
import multirotor.setup_path 
import airsim
import cv2
import time
import sys
import numpy as np
import LinkedList
import traceback
import time
def initiliaze_nodes(survey_path,x,y,z):
    path=[]
    for i in survey_path[:]:
        path.append(state_to_ijk(i,x,y,z))
    ll = LinkedList
    nodes = ll.arrayToList(path,len(path))
    return nodes

def nearest_multiple(pos,sx,sy,sz,zm):
    x = pos.x_val//sx
    y = pos.y_val//sy
    if pos.z_val < zm - 2: 
        z = (pos.z_val-zm-2)//sz
    else:
        z = 0
    return int(x),int(y),int(z)

def check_locations(lbl, x_avg, y_avg, tol):
    #Check if the object has already been labeled
    # if len(loc) == 0:
    #     return False
    # max_loc = int(np.max(loc))
    # min_loc = int(np.min(loc))
    # print ('Inside check_locations:')
    count = 0    
    #print (lbl)
    for i in lbl.keys():
        # print (lbl[i][0],lbl[i][1])
        # print (max_loc,min_loc)
        # if (max_loc in range(lbl[i][0],lbl[i][1])) or (min_loc in range(lbl[i][0],lbl[i][1])): count += 1  
        if (x_avg in range(lbl[i][0]-tol,lbl[i][0]+tol)) and (y_avg in range(lbl[i][1]-tol),lbl[i][1]+tol): return False        
    else:
        return True
def state_to_ijk(state,x,y,z):
    Coverage=x*y
    [i,j,k]=[((state-1)%Coverage)//x, (state%Coverage-1)%x, (state-1)//Coverage]
    return i,j,k

def image_views(survey_path,reward_detection,x,y,z,breaker,z_min=-30,z_max=-60,quad=31,camera='3'):
    # print ('Starting segmentation:',reward_detection)
    # print ('Starting segmentation:',survey_path)    
    #print ('Array:',reward_detection[:])
    #import pdb;pdb.set_trace();
    #reward = np.zeros((10,10),dtype=int)
    client = airsim.MultirotorClient()
#    client.confirmConnection()
#    client.enableApiControl(True)
#    client.armDisarm(True)

    #Initialize windows
    cv2.namedWindow('Depth')
    cv2.namedWindow('Segmentation')
    cv2.namedWindow('Scene')

    #Set the segmentation id for the object to be detected    
    segment_id = client.simSetSegmentationObjectID('OrangeBall[\w]*',6,True)
    #object_global_info = client.simGetObjectPose('OrangeBall[\w]*')  
    #print (object_global_info)      
    scale_x = quad * (10/x)
    scale_y = quad * (10/y)
    scale_z = (z_max-z_min)/z
    # path = [(p[0]*quad,p[1]*quad) for p in survey_path]
    #path = [(p[0]*scale_x+scale_x/2,p[1]*scale_y+scale_y/2,z_min+p[2]*(z_max-z_min)/z) for p in survey_path]      
    
    lower = np.array([100,60,150], dtype = "uint8")
    upper = np.array([110,70,160], dtype = "uint8")
    curr_count, prev_count, tot_count,local_count,index = 0,0,0,0,0
    tolerance = 500
    locations = {'x':[],'y':[]}
    label,reward_dict = {},{}
    curr_drone_pos,prev_drone_pos = (0,0,0),(0,0,0)
    curr_survey_path, prev_survey_path = [],[]
    #ll.display(nodes)
    # nodes = initiliaze_nodes(survey_path,x,y,z)
    # curr_node = nodes.data
    # next_node = nodes.next
    # print ('Nodes:',curr_node,'->',next_node.data)
    while True:
        try:
            curr_survey_path = survey_path[:]
            nodes = initiliaze_nodes(survey_path,x,y,z)
            curr_node = nodes.data
            next_node = nodes.next
            #print ('Nodes:',curr_node,'->',next_node.data)
            boxes = []
            responses = client.simGetImages([
            airsim.ImageRequest(camera, airsim.ImageType.DepthPerspective,True),
            airsim.ImageRequest(camera, airsim.ImageType.Segmentation,False,False), 
            airsim.ImageRequest(camera, airsim.ImageType.Scene,False,False)
            ])  
            drone_position=client.simGetVehiclePose().position
            #rounded_coordinates = nearest_multiple(drone_position,scale_x,scale_y,scale_z,z_min)
            curr_drone_pos = nearest_multiple(drone_position,scale_x,scale_y,scale_z,z_min)            
            for resp in responses:
                if not resp.pixels_as_float:
                    img = np.fromstring(resp.image_data_uint8, dtype=np.uint8)
                    img = img.reshape(resp.height, resp.width, 3)
                    if resp.image_type == 5:
                        win_name = 'Segmentation'
                        mask = cv2.inRange(img,lower,upper)
                        number,labels,stats,centroids = cv2.connectedComponentsWithStats(mask,8,cv2.CV_32S)
                        curr_count = number - 1
                        for ind in range(1,number):
                            i = stats[ind]
                            cen = centroids[ind]
                            bb = [(i[0],i[1]),(i[0]+i[2],i[1]+i[3])]        
                            # finding the global position (0-63000) of the topleft corner of the bounding box with camera focal length of 134.48
                            #global y location of the object in UE coordinates
                            object_global_pose_y=drone_position.y_val*100+(cen[0]-127.5)*(-drone_position.z_val*100)/129.31
                            #global x location of the object in UE coordinates                            
                            object_global_pose_x=drone_position.x_val*100-(cen[1]-71.5)*(-drone_position.z_val*100)/129.31                        
                            #print ('Drone Pos:',drone_position.y_val*100)
                            #print (ind,':',bb,':', i[0], 'y:', object_global_pose_y, 'x:', object_global_pose_x)
                            locations['x'].append(object_global_pose_x)
                            locations['y'].append(object_global_pose_y)
                            #print (locations)
                            boxes.append(bb)
                        if prev_count > curr_count:          
                            #print ('Checking if the object is already detected') 
                            if len(locations['x'])>0 and len(locations['y'])>0:
                                X_avg = sum(locations['x'])//len(locations['x'])
                                Y_avg = sum(locations['y'])//len(locations['y'])        
                                check = check_locations(label,X_avg,Y_avg,tolerance)
                            #print ('X average:',X_avg,'Y average:',Y_avg)
                                if check:
                                    #print (locations)
                                    local_count += 1
                                    tot_count += 1
                                    # label[tot_count] = np.round([np.min(locations),np.max(locations)],).astype(int)
                                    label[tot_count] = (int(X_avg),int(Y_avg))
                                locations = {'x':[],'y':[]}
                                #print (label)
                        prev_count = curr_count
                        #boxes = object_detection(img, client)                    
                    elif resp.image_type == 0:
                        win_name = 'Scene'
                        for i in boxes:
                            cv2.rectangle(img,i[0],i[1],(0,255,0),2)
                else:
                    win_name = 'Depth'
                    img = airsim.list_to_2d_float_array(resp.image_data_float, resp.width, resp.height)
                    img = cv2.convertScaleAbs(img)
                cv2.imshow(win_name,img)

            if breaker.value == 0:
               # index = i_j_to_state(int(curr_node[0]/quad),int(curr_node[1]/quad),size)
               # reward_detection[index] = local_count
               #print ('Reward:',reward_detection)
               print ('Completed Episode. Exiting segmentation.py...')
               break
            
            # print ('Actual Drone location:',drone_position.z_val)
            # print ('Current Node:',curr_node,'Next node:',next_node.data)
            # print ('Drone Position:',rounded_coordinates)
            #if (next_node != None) and (rounded_coordinates == next_node.data):
            if curr_survey_path != prev_survey_path:
                #if curr_node not in reward_dict:
#                print ('Drone moved from ',curr_node,' to ',next_node.data) 
#                print ('Number of Detections: ',local_count)    
#                print ('Award assigned to: ',curr_node)      
                reward_detection.value = local_count
#                print ('Reward:',reward_detection.value)   
                reward_dict[curr_node] = local_count             
                local_count = 0                 
#                print ('Reward Dictionary:',reward_dict)
#                print ('Locations of detected objects:',label)                       
                #else:                    
                #    continue
                # curr_node = next_node.data
                # next_node = next_node.next  
                # index += 1                     
            prev_drone_pos = curr_drone_pos
            prev_survey_path = curr_survey_path
            key = cv2.waitKey(1) & 0xFF;
            if (key == 27 or key == ord('q') or key == ord('x')):
                break;

        except Exception as e:
            print ('Exception in segmentation.py',e)
            traceback.print_exc()
if __name__ == "__main__":
    #image_views("3")
    path = [(0,0),(5,5),(2,2)]
    ll = LinkedList
    ll_path = ll.arrayToList(path,len(path))
    print (ll_path)
    curr_node = ll_path.data
