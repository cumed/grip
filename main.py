# -*- coding: utf-8 -*-
"""
Created on Tue Jun 19 1:45:00 2018

@author: ATI2-Pavan Gurudath
"""

import numpy as np
import gripper_movements_rpi as gmr
import Adafruit_PCA9685
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)
import skeleton_structure as sks
import time
import sys
import catheter_properties as cpro
#%% Define directions and thresholds
#directions = np.load('directions.npy')

# Data specific to this main file                                                                    

directions = np.array([[11,0],[12.,2],[10,3],[1,2],[21,50],[1,1],[1,3],[32,1],[41,70],[1,10],[2,20]])
zeroes = np.array([0.,0,30,0,0,0,0,0,60,40,0])
zeroes = zeroes.reshape((11,1))
directions = np.append(directions,zeroes,axis=1)


servoDist_threshold = 9.5                                                       # Max distance travelled by the back indexing servo(4.75*2)
angle_threshold = 5                                                             # Min angle required that the catheter needs to be bent by
rotationalAngle_threshold=5                                                     # Min angle required that the catheter needs to be rotated by
incremental_distance = 0                                                        # Keep track of distances until a bend is supposed to happen
traversed_distance = 0                                                          # Keep track of total distance

servo_min = 190                                                                 # Min limit of 183 for Hitech-servos
servo_max = 490                                                                 # Max limit of 595 for Hitech-servos
#%%
distances = directions[:,0]
angles = directions[:,1]
rotational_angle = directions[:,2]

#%%
catheter_ID = 1
properties_flag= 1                                                             #Set it to 1 if you require all the properties in one go. 
#[lengths, ODs, IDs, Materials, HysterisisFactors, HeatTimes, Xis, Yis, MandrelMaterials, MandrelODs] = cpro.get_properties(catheter_ID,properties_flag) 
cpro.get_properties(catheter_ID)
lengths = cpro.get_length()
OD = cpro.get_OD()
#lengths = [4.25,42,711.2]
#OD = [1.2, 1.67, 1.67]
#%%
idx             = 0                                                             #Index for points obtained from SVG
prop_idx        = 0                                                             #Index for the properites of the catheter
flag            = 0                                                             #Check for incremental distance until a bending is approached. 
rotation_flag   = 0                                                             #Check for incremental distance until a rotating angle is approached *currently unused*

lens            = lengths[prop_idx]
outer_diameter  = OD[prop_idx]
time_constant   = 1
#%% These distances are a part of gmr and declared there
#fully_closed_distance       = gmr.angle_to_distance(0)                  
#partially_opened_distance   = gmr.angle_to_distance(60)
#
#fully_opened_distance       = gmr.angle_to_distance(180)                        # Position of front and back servos along x-direction
#fully_bwd_distance          = gmr.angle_to_distance(0)                          # Position of cam for the back servo movement along y-direction
#


#%%
print('Bringing all cams to zeroeth position')

for channel in range(0,8):
    pwm.set_pwm(channel,0,servo_min)                                            # Setting all servo's to min position
time.sleep(time_constant)

#%%
print('Going to home position')
gmr.home_position()
print('Done with home position')
#%%

wait = input('Press 0 to exit the program')
if wait==0:
    sys.exit()
    
while idx < np.size(distances,0):
    present_rot_angle = rotational_angle[idx]                                   #rotational angle at the point under consideration. 
    present_angle = angles[idx]                                                 #bending angle at the point under consideration. 
    present_distance = distances[idx]                                           #distance from the point under consideration to the next. 
    traversed_distance += present_distance                                      #total distanced travelled from the tip of the catheter
    if traversed_distance < lens:
        if present_rot_angle < rotationalAngle_threshold:                                              #Do calculations if no rotational angle
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
    wait = input('Press 0 to exit the program')
    if wait ==0:
        sys.exit()