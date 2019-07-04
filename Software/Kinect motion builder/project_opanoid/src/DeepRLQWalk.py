#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
from __future__ import division
import numpy as np
import pickle
import rospy
import sys
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
import time
import pandas as pd
from matplotlib import pyplot as plt
from collections import deque
import random
from keras import Sequential
from keras.layers import Dense
from keras.optimizers import Adam

cmd_vel = "/servo_controller/command"
Imu_topic = "/imu"

class SelfBalance:
    def __init__(self):
        self.cmd_vel = rospy.Publisher(cmd_vel,Float64MultiArray,queue_size =1)
        self.subscriber = rospy.Subscriber(Imu_topic,Imu,callback=self.imu_callback)
        self.reset = rospy.ServiceProxy("/gazebo/reset_simulation",Empty)
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics",Empty)
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics",Empty)
        #self.pose_sub = rospy.Subscriber('/ground_truth/state',Odometry,callback=self.pose_callback)
        self.i =0
        self.count =0
        self.vel=Float64MultiArray()
        self.y_angle =0

    def imu_callback(self,data):
        self.y_angle = data.orientation.y
        
rospy.init_node('SelfBalance',anonymous=True)

rAll_list = []
gamma = 0.9
lr =0.7
leftThighX = np.arange(-60*1.5707,60*1.5707,1*1.5707)
leftThighY = np.arange(-60*1.5707,60*1.5707,1*1.5707)
leftKnee = np.arange(0,60*1.5707,1*1.5707)
rightThighX = np.arange(-60*1.5707,60*1.5707,1*1.5707)
rightThighY = np.arange(-60*1.5707,60*1.5707,1*1.5707)
rightKnee = np.arange(0,60*1.5707,1*1.5707)
leftAnkleX = np.arange(-60*1.5707,60*1.5707,1*1.5707)
leftAnkleZ = np.arange(-60*1.5707,60*1.5707,1*1.5707)
leftToe = np.arange(0,60*1.5707,1*1.5707)
rightAnkleX = np.arange(-60*1.5707,60*1.5707,1*1.5707)
rightAnkleZ = np.arange(-60*1.5707,60*1.5707,1*1.5707)
rightToe = np.arange(0,60*1.5707,1*1.5707)
actions = [leftThighX.tolist(), leftThighY.tolist(), leftKnee.tolist(), rightThighX.tolist(), rightThighY.tolist(), rightKnee.tolist(), leftAnkleX.tolist(), leftAnkleZ.tolist(), leftToe.tolist(), rightAnkleX.tolist(), rightAnkleZ.tolist(), rightToe.tolist()]

Robot = SelfBalance()
Robot.reset()

legal_actions = [len(actions[0]), len(actions[1]), len(actions[2]), len(actions[3]), len(actions[4]), len(actions[5]), len(actions[6]), len(actions[7]), len(actions[8]), len(actions[9]), len(actions[10]), len(actions[11])]

num_episodes =1500
epsilon =1
epsilon_decay =0.999
memory_size =1000
batch_size=100
show=True
angles = np.arange(-0.314,0.315,0.314)

scale=180/3.14

iters =2000
limit =0.085*scale

memory = deque(maxlen=memory_size)
Robot.reset()
time.sleep(1)
Robot.pause()
s=Robot.y_angle*scale
Robot.unpause()
actionArray = [actions[0][np.random.randint(0,legal_actions[0])], actions[1][np.random.randint(0,legal_actions[1])], actions[2][np.random.randint(0,legal_actions[2])], actions[3][np.random.randint(0,legal_actions[3])], actions[4][np.random.randint(0,legal_actions[4])], actions[5][np.random.randint(0,legal_actions[5])], actions[6][np.random.randint(0,legal_actions[6])], actions[7][np.random.randint(0,legal_actions[7])], actions[8][np.random.randint(0,legal_actions[8])], actions[9][np.random.randint(0,legal_actions[9])], actions[10][np.random.randint(0,legal_actions[10])], actions[11][np.random.randint(0,legal_actions[11])]]
a = Float64MultiArray(data=actionArray)
Robot.cmd_vel.publish(a)
Robot.pause()
s1=Robot.y_angle*scale
if abs(s1)<=limit:
    r=1
    experience =(s,r,a,s1)
    s=s1
else:
    r=-100
    Robot.reset()
    experience =(s,r,a,s1)
    s=Robot.y_angle

memory.append(experience)
Robot.unpause()

for _ in range(memory_size):
    actionArray = [actions[0][np.random.randint(0,legal_actions[0])], actions[1][np.random.randint(0,legal_actions[1])], actions[2][np.random.randint(0,legal_actions[2])], actions[3][np.random.randint(0,legal_actions[3])], actions[4][np.random.randint(0,legal_actions[4])], actions[5][np.random.randint(0,legal_actions[5])], actions[6][np.random.randint(0,legal_actions[6])], actions[7][np.random.randint(0,legal_actions[7])], actions[8][np.random.randint(0,legal_actions[8])], actions[9][np.random.randint(0,legal_actions[9])], actions[10][np.random.randint(0,legal_actions[10])], actions[11][np.random.randint(0,legal_actions[11])]]
    a = Float64MultiArray(data=actionArray)
    Robot.cmd_vel.publish(a)
    Robot.pause()
    s1=Robot.y_angle
    if abs(s1)<=limit:
        r=1
        experience =(s,r,a,s1)
        s=s1
    else:
        r=-100
        Robot.reset()
        experience =(s,r,a,s1)
        s=Robot.y_angle
    memory.append(experience)
    Robot.unpause()

if True:
    batches=random.sample(memory,batch_size)
    states= np.array([batch[0] for batch in batches])
    rewards= np.array([batch[1] for batch in batches])
    actions= np.array([batch[2] for batch in batches])
    new_states= np.array([batch[3] for batch in batches])
    
model = Sequential()
model.add(Dense(40,activation='relu',input_shape=(1,)))
model.add(Dense(40,activation='relu'))
model.add(Dense(legal_actions[0],activation='linear'))
model.compile(loss='mean_absolute_error',optimizer=Adam(lr=0.01),)
model.summary()

for i in range(num_episodes):
    
    Robot.reset()
    time.sleep(1)
    Robot.pause()
    s= Robot.y_angle*scale
    
    rAll =0
    r=0
    d = False
    j = 0
    
    for j in range(iters):
    #while True:
        #j=j+1
        #epsilon greedy. to choose random actions initially when Q is all zeros
        if np.random.random()< epsilon:
        		action0 = actions[0][np.random.randint(0,legal_actions[0])]
        		action1 = actions[1][np.random.randint(0,legal_actions[1])]
        		action2 = actions[2][np.random.randint(0,legal_actions[2])]
        		action3 = actions[3][np.random.randint(0,legal_actions[3])]
        		action4 = actions[4][np.random.randint(0,legal_actions[4])]
        		action5 = actions[5][np.random.randint(0,legal_actions[5])]
        		action6 = actions[6][np.random.randint(0,legal_actions[6])]
        		action7 = actions[7][np.random.randint(0,legal_actions[7])]
        		action8 = actions[8][np.random.randint(0,legal_actions[8])]
        		action9 = actions[9][np.random.randint(0,legal_actions[9])]
        		action10 = actions[10][np.random.randint(0,legal_actions[10])]
        		action11 = actions[11][np.random.randint(0,legal_actions[11])]
        		
        		actionArray = [action0, action1, action2, action3, action4, action5, action6, action7, action8, action9, action10, action11]
        		a = Float64MultiArray(data=actionArray)
        		epsilon = epsilon*epsilon_decay
        else:
            Q = model.predict(np.array([s]))
            actionArray = [actions[0][np.argmax(Q)], actions[1][np.argmax(Q)], actions[2][np.argmax(Q)], actions[3][np.argmax(Q)], actions[4][np.argmax(Q)], actions[5][np.argmax(Q)], actions[6][np.argmax(Q)], actions[7][np.argmax(Q)], actions[8][np.argmax(Q)], actions[9][np.argmax(Q)], actions[10][np.argmax(Q)], actions[11][np.argmax(Q)]]
            a = Float64MultiArray(data=actionArray)
        Robot.unpause()
        Robot.cmd_vel.publish(a)
        s1 =Robot.y_angle*scale
        Robot.pause()
        if abs(s1)>limit:
            d = True
            
        else:
            d = False
            r=1
        
        
        rAll=rAll+r
        
        
        if d:
            #time.sleep(1)
            if rAll<(iters-1):
                r =-100
                experience =(s,r,a,s1)
                memory.append(experience)
                if rAll !=0:
                    print("Episode %d Failed! Reward %d"%(i,rAll))
            rAll_list.append((i,rAll))
            
            break
        experience=(s,r,a,s1)
        memory.append(experience)
        if j>=(iters-1):
            print("Episode %d Passed! Reward %d after full episode"%(i,rAll))
            rAll_list.append((i,rAll))
            break
            
        s = s1
        #print("State %d"%s)
        Robot.unpause()
    batches=random.sample(memory,batch_size)
    states= np.array([batch[0] for batch in batches])
    rewards= np.array([batch[1] for batch in batches])
    actions= np.array([batch[2] for batch in batches])
    new_states= np.array([batch[3] for batch in batches])
    Qs =model.predict(states)
    new_Qs = model.predict(new_states)
    for i in range(len(rewards)):
        action_index=list(vels).index(actions[i])
        if rewards[i]==-100:
            Qs[i][action_index]=Qs[i][action_index]+ lr*(rewards[i])
        else:
            Qs[i][action_index]= Qs[i][action_index]+ lr*(rewards[i]+gamma*np.max(new_Qs[i]))
    model.fit(states,Qs,verbose=0)
    #epsilon = epsilon*epsilon_decay
    
def moving_average (values, window):
    weights = np.repeat(1.0, window)/window
    sma = np.convolve(values, weights, 'valid')
    return sma
    
eps =[ep for (ep,_) in rAll_list]
rewards=[reward for (_,reward) in rAll_list]

rewards = moving_average(rewards,100)

plot =plt.plot(rewards)
plt.ylabel('Rewards')
plt.xlabel('Episodes')
plt.title('Rewards vs Episodes for Learning Rate %f and Gamma %f'%(lr,gamma))
plt.savefig('Plot_with_lr_%f_gamma_%f_DQN.jpg'%(lr,gamma))

import pickle as pkl

pkl.dump(rAll_list,open('Rewards_with_lr_%f_gamma_%f_DQN.pkl'%(lr,gamma),'w'))

model_json = model.to_json()
with open("model2_lr_%f_episodes_%d.json"%(lr,num_episodes), "w") as json_file:
    json_file.write(model_json)
# serialize weights to HDF5
model.save_weights("model2_lr_%f_episodes_%d.h5"%(lr,num_episodes))
print("Saved model to disk")

