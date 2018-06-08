# -*- coding: utf-8 -*-
"""
Created on Fri Jun  8 15:50:58 2018

@author: ATI2
"""

import numpy as np
import gripper_movements_rpi as gmr
import Adafruit_PCA9685
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)
import skeleton_structure as sks
import time

#%% Define directions and thresholds
#directions = np.load('directions.npy')

# Data specific to this main file                                                                    

directions = np.array([[1,0],[1.,2],[1,3],[1,2],[2,2],[1,1],[1,3],[4,1],[4,20],[1,10],[2,20]])
#directions = np.array([[1,0],[2.,2],[3,3],[4,2],[5,2],[6,1],[1,3],[4,1],[4,20],[1,10],[2,20]])
zeroes = np.array([0.,0,10,0,0,0,0,0,10,10,0])
zeroes = zeroes.reshape((11,1))
#zeroes = np.zeros(shape=(11,1))
#zeroes = np.random.randn(11,1)
#zeroes_2 = (zeroes>0.5)*10
directions = np.append(directions,zeroes,axis=1)


servoDist_threshold = 3.1                                                       # Max distance travelled by the servo
angle_threshold = 7                                                             # Min angle that the catheter needs to be bent by
incremental_distance = 0                                                        # Keep track of previous distance

#%%
fully_closed_distance = gmr.angle_to_distance(0)
fully_opened_distance = gmr.angle_to_distance(180)
partially_opened_distance = gmr.angle_to_distance(60)
fully_bwd_distance = gmr.angle_to_distance(0)    

print('Bringing all cams to zeroeth position')

pwm.set_pwm(0,0,190)
pwm.set_pwm(1,0,590)
pwm.set_pwm(3,0,190)
time.sleep(5)


print('Going to home position')
gmr.home_position()
print('Done with home position')
print('Waiting for program to start in ...')
for i in range(3,0,-1):
    time.sleep(i)
    print(i)
    


#%%
distances = directions[:,0]
angles = directions[:,1]
rotational_angle = directions[:,2]

idx=0
flag=0
rotation_flag =0

    
while idx < np.size(distances,0):
    present_rot_angle = rotational_angle[idx]                                   #distance, bend angle and rotational angle at the point under 
    present_angle = angles[idx]                                                 #consideration. 
    present_distance = distances[idx]
    
    if present_rot_angle == 0:                                                  #Do calculations if no rotational angle
        if present_angle < angle_threshold:                                     #Compare the bend angle with the threshold that we setup
            incremental_distance = incremental_distance + present_distance      #Remember the incremental distances between points upto a certain bend is approached
            flag = 1                                                            #Flag is raised to keep note of the incrementation
        else:
            if flag:
                sks.push_catheter(servoDist_threshold,incremental_distance)         #When bend angle is approached, it pushes the catheter by the incremental distance
                incremental_distance = 0                                        #it had kept in memory so far. 
                flag = 0                                                        
            
            ### now do it for the current position
            sks.bend_catheter(present_angle)                                        #Bend the catheter by specific angle
            sks.push_catheter(servoDist_threshold,present_distance)                 #Push the catheter by appropriate distance after the bend

    else:                        
        if flag:
            sks.push_catheter(servoDist_threshold,incremental_distance)             #When bend angle is approached, it pushes the catheter by the incremental distance
            incremental_distance = 0                                            #it had kept in memory so far. 
            flag = 0
            
        sks.rotate_catheter(present_rot_angle)                                      #Rotate the plane of the catheter for the z-axis
        sks.bend_catheter(present_angle)                                            #Bend it by the bending angle 
        sks.rotate_catheter(-present_rot_angle)                                     #Rotate it back to its original plane
        sks.push_catheter(servoDist_threshold, present_distance)                    #Push the catheter by appropriate distance after the bend
    
    idx = idx + 1