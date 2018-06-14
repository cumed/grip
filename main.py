# -*- coding: utf-8 -*-
"""
Created on Thur Jun  14 9:36:00 

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

directions = np.array([[1,0],[1.,2],[1,3],[1,2],[2,50],[1,1],[1,3],[3,1],[4,70],[1,10],[2,20]])
zeroes = np.array([0.,0,30,0,0,0,0,0,60,40,0])
zeroes = zeroes.reshape((11,1))
directions = np.append(directions,zeroes,axis=1)


servoDist_threshold = 3.1                                                       # Max distance travelled by the servo
angle_threshold = 7                                                             # Min angle that the catheter needs to be bent by
incremental_distance = 0                                                        # Keep track of distances until a bend is supposed to happen
traversed_distance = 0                                                          # Keep track of total distance

servo_min = 190                                                                 # Min limit of 183 for Hitech-servos
servo_max = 595                                                                 # Max limit of 595 for Hitech-servos
#%%
distances = directions[:,0]
angles = directions[:,1]
rotational_angle = directions[:,2]
[lengths, OD] = get_properties()


idx             = 0                                                             #Index for points obtained from SVG
prop_idx        = 0                                                             #Index for the properites of the catheter
flag            = 0                                                             #Check for incremental distance until a bending is approached. 
rotation_flag   = 0                                                             #Check for incremental distance until a rotating angle is approached *currently unused*
lens            = lengths[prop_idx]
outer_diameter  = OD[prop_idx]
time_constant   = 1
#%% Needs to be changed to incorporate the function written by Sramana
fully_closed_distance       = gmr.angle_to_distance(0)                  
partially_opened_distance   = gmr.angle_to_distance(60)

fully_opened_distance       = gmr.angle_to_distance(180)                        # Position of front and back servos along x-direction
fully_bwd_distance          = gmr.angle_to_distance(0)                          # Position of cam for the back servo movement along y-direction



#%%
print('Bringing all cams to zeroeth position')

for channel in range(0,8):
    pwm.set_pwm(channel,0,servo_min)                                            # Setting all servo's to min position
time.sleep(time_constant)


print('Going to home position')
gmr.home_position()
print('Done with home position')
wait= input('Waiting for key to start the program')

while idx < np.size(distances,0):
    present_rot_angle = rotational_angle[idx]                                   #rotational angle at the point under consideration. 
    present_angle = angles[idx]                                                 #bending angle at the point under consideration. 
    present_distance = distances[idx]                                           #distance from the point under consideration to the next. 
    traversed_distance += present_distance
    if traversed_distance < lens:
        if present_rot_angle == 0:                                              #Do calculations if no rotational angle
            if present_angle < angle_threshold:                                 #Compare the bend angle with the threshold that we set.
                incremental_distance = incremental_distance + present_distance  #Remember the incremental distances between points upto a certain bend is approached
                flag = 1                                                        #Flag is raised to keep note of the incrementation
            else:
                if flag:
                    sks.push_catheter(servoDist_threshold,incremental_distance,
                                      outer_diameter)                           #When bend angle is approached, it pushes the catheter by the incremental distance,
                    incremental_distance = 0                                    #that it had kept in memory so far. 
                    flag = 0                                                        
                
                ### now do it for the current position
                sks.bend_catheter(present_angle,outer_diameter)                 #Bend the catheter by specific angle
                sks.push_catheter(servoDist_threshold,present_distance,
                                  outer_diameter)                               #Push the catheter by appropriate distance after the bend
    
        else:                        
            if flag:
                sks.push_catheter(servoDist_threshold,incremental_distance,
                                  outer_diameter)                               #When bend angle is approached, it pushes the catheter by the incremental distance,
                incremental_distance = 0                                        #that it had kept in memory so far. 
                flag = 0
                
            sks.rotate_catheter(present_rot_angle)                              #Rotate the plane of the catheter for the z-axis
            sks.bend_catheter(present_angle,outer_diameter)                     #Bend it by the bending angle 
            sks.rotate_catheter(-present_rot_angle)                             #Rotate it back to its original plane
            sks.push_catheter(servoDist_threshold, present_distance,
                              outer_diameter)                                   #Push the catheter by appropriate distance after the bend
        
        idx = idx + 1
    else:
        prop_idx = prop_idx + 1                                                 #Once the travelled length is greater than the length of material under consideration,
        lens = lengths[prop_idx]                                                #then move onto the next material which will have a different OD(*maybe*). 
        outer_diameter = OD[prop_idx]

