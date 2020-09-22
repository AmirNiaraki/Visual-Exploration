# -*- coding: utf-8 -*-
"""
Solving the path planning for 5 constant winds

Created on Tue Feb 18 15:01:47 2020

@author: niaraki
"""


import numpy as np
np.set_printoptions(suppress=True)
from segmentation import image_views
from drone_survey import start
from multiprocessing import Process,Value,Array
from multirotor import setup_path 
import airsim
import sys
import time
import argparse

WorldHeight=10
WorldWidth=10
WorldZ=3

quad=31
Coverage=WorldHeight*WorldWidth
Start=1
step_state=Array('i',2)
reward_detection=Value('i',0)
tpe_list = []

Episodes=50
Epsilon=0.1
Alpha=0.5
Gamma=1
#C is the coefficient that determines the influence of detection over power cost
C=50
#Total number of balls in the Simulation Environment
All_balls=4


ActionUp=0
ActionRight=1
ActionDown=2
ActionLeft=3
ActionNE=4
ActionSE=5
ActionSW=6
ActionNW=7
ActionZUp=8
ActionZDown=9
#           0         1           2           3           4
Actions=[ActionUp, ActionRight, ActionDown, ActionLeft, ActionNE, 
#           5         6          7         8          9
         ActionSE, ActionSW, ActionNW, ActionZUp, ActionZDown]
ActionSize=np.size(Actions)
#Q_Sarsa=np.zeros((WorldHeight*WorldWidth*WorldZ, Wind_Layers, np.size(Actions)))

Wind_Max=0.85
Battery_Max=100
Winds=np.array([[Wind_Max,0],
                [0.5*Wind_Max,0],
                [0,0],
                [-0.5*Wind_Max,0],
                [-Wind_Max,0]])
friction=3
Wind_Layers=len(Winds)


#rewards=np.zeros((WorldHeight*WorldWidth, np.size(Actions)))
States=np.arange(1,Coverage+1)


def state_to_ijk(state):
    [i,j,k]=[((state-1)%Coverage)//WorldWidth, (state%Coverage-1)%WorldWidth, (state-1)//Coverage]
    return i,j,k

def ijk_to_state(i,j,k):
    state=(k*Coverage)+i*WorldWidth+j+1
    return state
#initializing the action-state value function/matrix
    

def q_init(q_value):
    for i in range(0,WorldHeight):
        for j in range (0,WorldWidth):
            for k in range (0,WorldZ):
                for action in Actions:
                    state=ijk_to_state(i,j,k)
                    if ((i==0 and (action==ActionUp or action==ActionNW or action==ActionNE)) or 
                        (i==(WorldHeight-1) and (action==ActionDown or action==ActionSW or action==ActionSE)) 
                    or  (j==0 and (action==ActionLeft or action==ActionSW or action==ActionNW)) or 
                        (j==(WorldWidth-1) and (action==ActionRight or action==ActionNE or action==ActionSE))
                    or  (k==0 and (action==ActionZDown))
                    or  (k==(WorldZ-1) and action==(ActionZUp))):    
                            q_value[state-1,:,action]=-100
#Any other condition can be enforced here in the initializaiton                                    
    return q_value   
#########################################################################################################
def power_cost(state_ijk,next_state_ijk,wind_layer):
    movement=np.array([next_state_ijk[0]- state_ijk[0], next_state_ijk[1]-state_ijk[1]])
    distance_scaler=quad*100/WorldWidth
    
    cost=friction*(pow((movement[0]-Winds[wind_layer,0]),2)+pow((movement[1]-Winds[wind_layer,1]),2))
#There is no cost for moving in Z direction
#    print('movement', movement[:], cost)
    return cost

#########################################################################################################
#The step function  terminates  when we run out of battery
    
def step(state,action,wind_layer,battery,terminal):
    [i,j,k]=state_to_ijk(state)
#    print(action)
    if action==ActionUp:
        state_= [max(i-1,0),j,k]
    if action==ActionDown:
        state_= [min(i+1,WorldHeight-1),j,k]
    if action==ActionRight:
        state_= [i,min(j+1,WorldWidth-1),k]
    if action==ActionLeft:
        state_= [i, max(0,j-1),k]
    if action==ActionNE:
        state_=[max(i-1,0),min(j+1,WorldWidth-1),k]
    if action==ActionSE:
        state_=[min(i+1,WorldHeight-1),min(j+1,WorldWidth-1),k]
    if action==ActionSW:
        state_=[min(i+1,WorldHeight-1), max(0,j-1),k]
    if action==ActionNW:
        state_=[max(i-1,0), max(0,j-1),k]
    if action==ActionZUp:
        state_=[i,j,min(k+1,WorldZ-1)]
    if action==ActionZDown:
        state_=[i,j,max(k-1,0)]        
        
    next_state=ijk_to_state(state_[0],state_[1],state_[2])   
    used_battery_step=power_cost(state_to_ijk(state),state_to_ijk(next_state),wind_layer)
    battery-=used_battery_step
    reward=-used_battery_step

    if ((i==0 and (action==ActionUp or action==ActionNW or action==ActionNE)) or 
                        (i==(WorldHeight-1) and (action==ActionDown or action==ActionSW or action==ActionSE)) 
                    or  (j==0 and (action==ActionLeft or action==ActionSW or action==ActionNW)) or 
                        (j==(WorldWidth-1) and (action==ActionRight or action==ActionNE or action==ActionSE))
                    or  (k==0 and (action==ActionZDown))
                    or  (k==(WorldZ-1) and action==(ActionZUp))):         
                        reward=-100
                        next_state=state
                        print("STEP tried to go out","in state",state,state_to_ijk(state),"and action",action)
    elif battery<1:
        next_state=Start
        terminal=1

    return next_state, reward, battery, terminal

#########################################################################################################
def choose_action(state, q_value,wind_layer, episode,path,action_list):
#Epsilon can be reduced based on episode number   
    if episode<20:
        epsilon=3*Epsilon
    elif episode>20 and episode<49:
        epsilon=Epsilon
    else:
        epsilon=0
#    x=episode/(Episodes+1)
#    epsilon=pow((Epsilon),(x/(1-x)))
    #Create a set of valid actions only!!!!! 
    valid_actions=Actions[:]
    [i,j,k]=state_to_ijk(state)
    visited_states=path[:]
    taken_actions=action_list[:]
    visited_states.pop()
    no_actions=False
    for action in Actions:
#        print([i,j,k],action)
        if      ((i==0 and (action==ActionUp or action==ActionNW or action==ActionNE)) 
            or  (i==(WorldHeight-1) and (action==ActionDown or action==ActionSW or action==ActionSE)) 
            or  (j==0 and (action==ActionLeft or action==ActionSW or action==ActionNW)) 
            or  (j==(WorldWidth-1) and (action==ActionRight or action==ActionNE or action==ActionSE))
            or  (k==0 and (action==ActionZDown))
            or  (k==(WorldZ-1) and action==(ActionZUp))):
#                print("before VA", valid_actions)    
                valid_actions.remove(action)

        else:
            for index,p in enumerate(visited_states):
                if p==state_to_ijk(state) and taken_actions[index]==action:
                    try:
                        valid_actions.remove(action)
                    except:
                        print('no valid actions, removed')
                        print(valid_actions)
                        no_actions=True
                        
#    print("state:",state,"valid_actions:", valid_actions[:])
    if len(valid_actions)==0 or no_actions==True:
        chosen_action='no valid actions'
        print('no valid actions, all removed')
    elif np.random.binomial(1,epsilon)==1:
        chosen_action=np.random.choice(valid_actions)
        
    else:   
        
        Values=q_value[state-1, wind_layer,:]
        chosen_action=np.random.choice([Action for Action,Value in enumerate(Values) if Value==np.max(Values)])
   
    return chosen_action
#########################################################################################################

def airsim_initialization():
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    client.takeoffAsync().join()                            
    

 
def airsim_communication(step_state):
    
    print(step_state[:])
    p1 = Process(target=start,args=([state_to_ijk(step_state[0]),state_to_ijk(step_state[1])],WorldWidth,WorldHeight,WorldZ))
    
    #q_detection=np.frombuffer(reward_detection.get_obj(),dtype=int)
    p1.start()
    p1.join()
    print ('Reward Detection (Drone Survey):',reward_detection.value)
    return reward_detection.value


#not expected Sarsa
def QL (q_value, wind_layer, episode, success,  StepSize=Alpha):

    state=Start
    battery=Battery_Max
    battery_index=[battery]
    path=[state_to_ijk(Start)]
    
    action_list=[]
    iteration=1
    rewards=0.0
    reward_airsim=0.0
    terminal=0
    ball_count=0
    #np.save("./Backup_variables/"+str(episode)+"_"+str(wind_layer)+".npy",q_value[:,wind_layer,:])

    while terminal==0:
        action=choose_action(state, q_value, wind_layer, episode, path,action_list)
        if action=='no valid actions':
                terminal=1
#            try:
                reward_movement=-200
                reward=reward_movement
                rewards+=reward

                prev_state=ijk_to_state(path[iteration-2][0],path[iteration-2][1],path[iteration-2][2])
                curr_state=ijk_to_state(path[iteration-1][0],path[iteration-1][1],path[iteration-1][2])
#                print("state",state,"prev",prev_state,"curr",curr_state,"\n",path, action_list,"iteration",iteration)
                q_value[prev_state-1,wind_layer, action_list[iteration-2]]+=StepSize*(reward+ Gamma*np.max(q_value[curr_state-1,wind_layer, :])-q_value[prev_state-1,wind_layer, action_list[iteration-2]])
                print('episode ended cuz the drone stuck with no actions!')
#            except:
#                print('episode ended cuz the drone stuck with no actions!')
        else:
            action_list.append(action)
            
            next_state,reward_movement,battery,terminal= step(state,action,wind_layer,battery,terminal)
    
    # This reward is from object detection for EACH STEP:
    
            step_state[0]=state
            step_state[1]=next_state
            reward_airsim=airsim_communication(step_state)
            
            ball_count+=reward_airsim
            reward=reward_movement+reward_airsim*C
            rewards+=reward
            q_value[state-1,wind_layer, action]+=StepSize*(reward+ Gamma*np.max(q_value[next_state-1,wind_layer,:])-q_value[state-1, wind_layer, action])
            state=next_state
        if ball_count>=All_balls:
            terminal=1
            print('found all the balls!')
            success+=1
            
        if terminal==0:
            path.append(state_to_ijk(state))
            
        battery_index.append(battery)
        iteration+=1
    
    path.append(state_to_ijk(Start))
    np.save("./Backup_variables/actions"+str(episode)+".npy",action_list)    
    np.save("./Backup_variables/"+str(episode)+"_"+str(wind_layer)+".npy",q_value[:,wind_layer,:])
    np.save("./Backup_variables/path"+str(episode)+".npy",path)
    np.save("./Backup_variables/Battery_per_step"+str(episode)+".npy", battery_index)
#    np.save("./)
    print('Path:',path,'actions:', action_list, 'number of steps',iteration)
    breaker.value = 0
    return rewards, q_value[:,wind_layer,:], path, battery_index, success



def Figure(wind_layer):
    Rewards_Sarsa=np.zeros(Episodes)
    Q_Sarsa=np.zeros((WorldHeight*WorldWidth*WorldZ, Wind_Layers, np.size(Actions)))
    Q_Sarsa=q_init(Q_Sarsa)
    R_sarsa_max=np.zeros((Wind_Layers)) 
    number_of_optimals=0
    e=0
    best_episodes=0
    
    for w in range(wind_layer-1,wind_layer):
        battery_w=[]
        Path_sarsa_opt=[None]
        while e <Episodes:
            timer_strt = time.time()
            print("episode:", e)
            reward_detection.value = 0
            p2 = Process(target=image_views,args=(step_state,reward_detection,WorldWidth,WorldHeight,WorldZ,breaker))
            p2.start()       
            path_sarsa_g=[None]                         
            r_sarsa, Q_Sarsa[:,w,:],path_sarsa_g, remained_battery, best_episodes=QL(Q_Sarsa,w,e,best_episodes)
            
            p2.join()
            breaker.value = 1            
            print("Time taken for the episode:",time.time() - timer_strt)
            print("episode",e,"resulted in", r_sarsa,"rewards")
            print("All balls found in", best_episodes)
            tpe_list.append(time.time() - timer_strt)
#            Rewards_Sarsa[e]=r_sarsa    
            Rewards_Sarsa[e]=r_sarsa
            if r_sarsa>R_sarsa_max[w]:
                best_episode=e
                R_sarsa_max[w]=r_sarsa
                Path_sarsa_opt=path_sarsa_g
                remained_battery_optimal=remained_battery
            
            if best_episodes==2*All_balls:
                e=Episodes
            
            e+=1
#convergence condition           

#recording all the rewards per episode                
           
    np.save("./Backup_variables/rewards_across_episodes.npy", Rewards_Sarsa[:])
    np.save("./Backup_variables/optimal_path.npy", Path_sarsa_opt)      
    np.save("./Backup_variables/time_per_episode.npy",tpe_list)                              

#    print("Rewards accross episodes:",Rewards_Sarsa[:], "optimal path", Path_sarsa_opt, "happened in episode ",best_episode, "and" ,remained_battery_optimal, "battery was left.")
    print("optimal path", Path_sarsa_opt, "happened in episode ",best_episode, "and" ,remained_battery_optimal, "battery was left.")
    
    np.set_printoptions(suppress=True)
    return Q_Sarsa,Rewards_Sarsa,Path_sarsa_opt, battery_w


if __name__=="__main__":
    airsim_initialization()
    breaker = Value('i',1)
    Q,Rewrds,Paths,Battery=Figure(1)
#    print(Q[:,0,:])
#    import pdb;pdb.set_trace()
    for w in range(1,2):
        print("end")

















