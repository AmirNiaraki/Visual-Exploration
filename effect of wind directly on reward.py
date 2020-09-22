# -*- coding: utf-8 -*-
"""
Solving the path planning for 6 constant winds

Created on Tue Feb 18 15:01:47 2020

@author: niaraki
"""


import numpy as np
np.set_printoptions(suppress=True)
from segmentation import image_views
from drone_survey import start
from multiprocessing import Process,Value,Array
#import matplotlib
#matplotlib.use('Agg')
#import matplotlib.pyplot as plt; plt.rcdefaults()
#from tqdm import tqdm


WorldHeight=4
WorldWidth=4
WorldZ=4

Coverage=WorldHeight*WorldWidth
#Goals=[23,41]
#Goals=[41]
Start=1
#Charging_Spot=370


Episodes=1000
Epsilon=0.1
Alpha=0.5
Gamma=1
#C is the coefficient that determines the influence of detection over power cpst
C=1

ActionUp=0
ActionRight=1
ActionDown=2
ActionLeft=3
ActionNE=4
ActionSE=5
ActionSW=6
ActionNW=7
Actions=[ActionUp, ActionRight, ActionDown, ActionLeft, ActionNE, ActionSE, ActionSW, ActionNW]
ActionSize=np.size(Actions)

Wind_Max=0.85
Battery_Max=100
Winds=np.array([[Wind_Max,0],[0.5*Wind_Max,0],[0,0],[-0.5*Wind_Max,0],[-Wind_Max,0]])
#high_wind=[Wind_Max,0]
#low_wind=[-0.5*Wind_Max,0]
friction=10
#Winds=np.array([high_wind])
Wind_Layers=len(Winds)


rewards=np.zeros((WorldHeight*WorldWidth, np.size(Actions)))
States=np.arange(1,Coverage+1)


def state_to_i_j(state):
    [i,j]=[(state-1)//WorldWidth, (state-1)%WorldWidth]
    return i,j

def i_j_to_state(i,j):
    state=i*WorldWidth+j+1
    return state
#initializing the action-state value function/matrix
def q_init(q_value):
    for i in range(0,WorldHeight):
        for j in range (0,WorldWidth):
            for action in Actions:
                state=i_j_to_state(i,j)
                if ((i==0 and (action==ActionUp or action==ActionNW or action==ActionNE)) or (i==(WorldHeight-1) and (action==ActionDown or action==ActionSW or action==ActionSE)) 
                or (j==0 and (action==ActionLeft or action==ActionSW or action==ActionNW)) or (j==(WorldWidth-1) and (action==ActionRight or action==ActionNE or action==ActionSE))):

                        q_value[state-1,:,action]=-100
#Any other condition can be enforced here in the initializaiton              
    return q_value   
#########################################################################################################
#returns a value betwee 0.5 and 1.5 depending on the drone movement with respect to wind
def power_cost(state_ij,next_state_ij,wind_layer):
    movement=np.array([next_state_ij[1]-state_ij[1],next_state_ij[0]-state_ij[0]])
    
    cost=friction*(pow((movement[0]-Winds[wind_layer,0]),2)+pow((movement[1]-Winds[wind_layer,1]),2))
#    print(cost)
    return cost

#########################################################################################################
#The step function  terminates  when we run out of battery
    
def step(state,action,wind_layer,battery,terminal):
    [i,j]=state_to_i_j(state)

    if action==ActionUp:
        state_= [max(i-1,0),j]
    if action==ActionDown:
        state_= [min(i+1,WorldHeight-1),j]
    if action==ActionRight:
        state_= [i,min(j+1,WorldWidth-1)]
    if action==ActionLeft:
        state_= [i, max(0,j-1)]
    if action==ActionNE:
        state_=[max(i-1,0),min(j+1,WorldWidth-1)]
    if action==ActionSE:
        state_=[min(i+1,WorldHeight-1),min(j+1,WorldWidth-1)]
    if action==ActionSW:
        state_=[min(i+1,WorldHeight-1), max(0,j-1)]
    if action==ActionNW:
        state_=[max(i-1,0), max(0,j-1)] 
        
    next_state=i_j_to_state(state_[0],state_[1])
    
    used_battery_step=power_cost(state_to_i_j(state),state_to_i_j(next_state),wind_layer)
    battery-=used_battery_step
    #print(used_battery_step)
    reward=-used_battery_step
    # if action>=4 and action<=7:
    #     reward=-1.4
    # else:
    #     reward=-1

    if ((i==0 and (action==ActionUp or action==ActionNW or action==ActionNE)) or (i==(WorldHeight-1) and (action==ActionDown or action==ActionSW or action==ActionSE)) 
        or (j==0 and (action==ActionLeft or action==ActionSW or action==ActionNW)) or (j==(WorldWidth-1) and (action==ActionRight or action==ActionNE or action==ActionSE))):
        reward=-100
        next_state=state
    elif battery<1:
        next_state=Start
        terminal=1

    return next_state, reward, battery, terminal

#########################################################################################################
def choose_action(state, q_value,wind_layer, episode):
#Epsilon can be reduced based on episode number   
    epsilon=Epsilon
#    x=episode/(Episodes+1)
#    epsilon=pow((Epsilon),(x/(1-x)))
    if np.random.binomial(1,epsilon)==1:
        return np.random.choice(Actions)
    else:
        Values=q_value[state-1, wind_layer,:]
        return np.random.choice([Action for Action,Value in enumerate(Values) if Value==np.max(Values)])
#########################################################################################################
 
#not expected Sarsa
def QL (q_value, wind_layer, episode, StepSize=Alpha):


    state=Start
    battery=Battery_Max
    battery_index=[battery]
    path=[state_to_i_j(Start)]
    
    action_list=[]
    iteration=1
    rewards=0.0
    terminal=0


    while terminal==0:
        action=choose_action(state, q_value, wind_layer, episode)
        action_list.append(action)   
        next_state,reward,battery,terminal= step(state,action,wind_layer,battery,terminal)
        rewards+=reward
        q_value[state-1,wind_layer, action]+=StepSize*(reward+ Gamma*np.max(q_value[next_state-1,wind_layer,:])-q_value[state-1, wind_layer, action])
        state=next_state
        if terminal==0:
            path.append(state_to_i_j(state))
        battery_index.append(battery)
        iteration+=1
    path.append(state_to_i_j(Start))
    #q_detection should come as a list with the length of len(path)-1
    path=[(0,0),(1,1),(1,2),(0,1),(0,0)]
    print(path)
    breaker = Value('i',1)    
    reward_detection = Array('i',len(path)-1)
    p1 = Process(target=start,args=(path,breaker,))
    p2 = Process(target=image_views,args=(path,reward_detection,breaker,))
    #q_detection=np.frombuffer(reward_detection.get_obj(),dtype=int)

    p1.start()
    p2.start()

    p1.join()
    p2.join()
    for s in range(len(action_list)):
        
        print("initial q values and rewards of detection", q_value[i_j_to_state(path[s][0],path[s][1]),wind_layer,action_list[s]])
        print(reward_detection[s], "\n")

    for s in range(len(action_list)):
         q_value[i_j_to_state(path[s][0],path[s][1]),wind_layer,action_list[s]]+=C*reward_detection[s]
    rewards+=sum(reward_detection)
    #print("path size:", len(path), "action size:", len(action_list), path)
    return rewards, q_value[:,wind_layer,:], path, battery_index

def Figure(wind_layer):
    Rewards_Sarsa=np.zeros(Episodes)
#    Battery_e=np.zeros(Episodes)
#    Rewards_QL=np.zeros(Episodes)
#    remained_battery=np.zeros(Episodes)
    Q_Sarsa=np.zeros((WorldHeight*WorldWidth, Wind_Layers, np.size(Actions)))
    Q_Sarsa=q_init(Q_Sarsa)
#    Q_QL=Q_Sarsa
#    r_sarsa=np.zeros(len(Goals))
#    path_sarsa_g=[None]*len(Goals) 
    path_sarsa_w=[]
    
    R_sarsa_max=np.zeros((Wind_Layers)) 
    
#    r_QL=np.zeros(len(Goals))
#    path_QL=np.zeros(len(Goals))
#    R_QL_max==np.zero((Wind_Layers,len(Goals)))
#    Path_QL_opt==np.zero((Wind_Layers,len(Goals)))
    
    for w in range(wind_layer-1,wind_layer):
        path_sarsa_w_opt=[]
        battery_w=[]
        Path_sarsa_opt=[None]
        for e in range(Episodes):
#            print(e)
            path_sarsa_e=[]
            battery_e=[]
            path_sarsa_g=[None]
#            r_sarsa=np.zeros(len(Goals))
            r_sarsa, Q_Sarsa[:,w,:],path_sarsa_g, remained_battery=QL(Q_Sarsa,w,e)
                
            if r_sarsa>R_sarsa_max[w]:
                R_sarsa_max[w]=r_sarsa
                Path_sarsa_opt=path_sarsa_g
                
            Rewards_Sarsa[e]+=r_sarsa
#            Battery_e[e] remained_battery
            path_sarsa_e.extend(path_sarsa_g)
            battery_e.extend(remained_battery)
            path_sarsa_w.append(path_sarsa_e)
            battery_w.append(battery_e)

            path_sarsa_w_opt.extend(Path_sarsa_opt)
#        print("\n for wind", Winds[w], "optimal path is:", path_sarsa_w_opt,"with size", len(path_sarsa_w_opt), "best reward is ", R_sarsa_max[w])
                
 
    np.set_printoptions(suppress=True)
     
#    print(Rewards_Sarsa)
#    print("episode:",e, "with reward" , r_sarsa, path_sarsa_e, len(path_sarsa_e))

#    plt.plot(Rewards_Sarsa, label='Sarsa')
#    plt.xlabel('Episodes')
#    plt.ylabel('Sum of rewards during episode')
#    plt.ylim([-600, 200])
#    plt.legend()
#    
#
#    plt.savefig('../images/figure_coverage.png')
#    plt.close()
##    print_actions_bar (Q_Sarsa)
    return Q_Sarsa,Rewards_Sarsa,path_sarsa_w, battery_w


if __name__=="__main__":
   
    #array =  
    Q,Rewrds,Paths,Battery=Figure(1)
    import pdb;pdb.set_trace()
    for w in range(1,2):
        #Q,Rewrds,Paths,Battery=Figure(w)                     
        # best_path=Paths[np.argmax(Rewrds)]
        # best_battery=Battery[np.argmax(Rewrds)]
        # print('\n w',w, '\n', best_battery, '\n the number of steps for this episode:' , best_path, '\n reward', max(Rewrds))
        #import pdb;pdb.set_trace();
        #best_path=[(0,0),(1,1),(2,2),(3,3),(4,4),(5,5)]
        #ret_matrix = Array('i',len(best_path))
        #import pdb;pdb.set_trace();
        print("end")


        
#
#    with open('../images/low_wind.txt', 'a') as file:
#        file.write('\n'+'\n')
#        for t in final_path:
#            file.write(str(t)+',')
#        file.write('\n')
#        for t in final_bat:
#            file.write(str(t)+',')
##        
#        


















