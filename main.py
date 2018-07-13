# -*- coding: utf-8 -*-
"""
Created on Thur Jun 21 13:08:00 2018

@author: ATI2-Pavan Gurudath
"""

import numpy as np
import gripper_movements_rpi as gmr
import Adafruit_PCA9685
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)
import skeleton_structure as sks
import sys
import catheter_properties as cpro
import os
import factors as fact
#%% Define directions and thresholds
#directions = np.load('halfCircle.npy')
currDir = os.path.dirname(os.path.realpath('__file__'))
filename = os.path.join(currDir,'npy\\38mmCircle.npy')
directions = np.load(filename)

# Data specific to this main file                                                                    

#directions = np.array([[4,0],[5.,2],[10,-30],[1,2],[21,50]])#,[1,1],[1,3],[32,1],[41,20],[1,10],[2,20]])
#zeroes = np.array([0.,0,0,0,0])#,0,0,0,60,60,0])
#zeroes = zeroes.reshape((5,1))
#directions = np.append(directions,zeroes,axis=1)
#directions = np.load('bends.npy')
#directions = np.transpose(directions)

#directions = directions[::-1]
#directions[:,1] = directions[:,1] *180/pi
#print(directions)

servoDist_threshold       = 6.0                                                 # Max distance travelled by the back indexing servo(4.75*2)
angle_threshold           = 1                                                # Min angle required that the catheter needs to be bent by
#neg_angle_threshold       = -1*angle_threshold
rotationalAngle_threshold = 0.01                                                # Min angle required that the catheter needs to be rotated by
#incremental_distance      = 0                                                  # Keep track of distances until a bend is supposed to happen
#traversed_distance        = 0                                                  # Keep track of total distance travelled
incremental_distance,traversed_distance = 0,0                                   # Keeping track of distances until a bend is supposed to happen and the total distance distance travelled
#servo_min = 190                                                                # Min limit of 183 for Hitech-servos
#servo_max = 490                                                                # Max limit of 600 for Hitech-servos
servo_min, servo_max = 190,500                                                  # Min,Max limit of 183,600 for Hitech-servos 
#%%
distances = directions[:,0]
angles = directions[:,1]
rotational_angle = directions[:,2]

#%%
#catheter_ID     = 3
#properties_flag = 1                                                             #Set it to 1 if you require all the properties in one go. 
#[lengths, ODs, IDs, Materials, HysterisisFactors, HeatTimes, Xis, Yis, MandrelMaterials, MandrelODs] = cpro.get_properties(catheter_ID,properties_flag) 
#cpro.get_properties(catheter_ID)                                             #Creates the current catheter sheet that'll have all the details for the <catheter_ID> catheter. 
lengths = cpro.get_length()                                                     #Get all the material cumulative lengths. 
OD = cpro.get_OD()
OD = fact.ODList
#lengths = [4.25,42,711.2]
#OD = [1.2, 1.67, 1.67]
#%%
idx             = 0                                                             #Index for points obtained from SVG
prop_idx        = 0                                                             #Index for the properites of the catheter
flag            = 0                                                             #Check for incremental distance until a bending is approached. 
rotation_flag   = 0                                                             #Check for incremental distance until a rotating angle is approached *currently unused*
idx, prop_idx, flag, rotation_flag = 0,0,0,0
lens            = lengths[prop_idx]
outer_diameter  = OD[prop_idx]
time_constant   = 1
#%% These distances are a part of gmr and declared there
#fully_closed_distance       = gmr.angle_to_distance(0)                  
#partially_opened_distance   = gmr.angle_to_distance(60)
#
#fully_opened_distance       = gmr.angle_to_distance(180)                        # Position of front and back servos along x-direction
#fully_bwd_distance          = gmr.angle_to_distance(0)                          # Position of cam for the back servo movement along y-direction

#%% Zeroing of cams
def zero_position():
    gmr.front_gripper(0)
    gmr.back_gripper(0)
    gmr.bendingPin_zero()
    gmr.back_gripper_indexing(0)

test = input('Zero the cams so that you can place the catheter from top')
if test ==1:
    zero_position()
    input('Press any key to continue after inserting the catheter')
else:
    slightlyMore_opened_distance = 2.2
    gmr.front_gripper(slightlyMore_opened_distance)
    gmr.back_gripper(slightlyMore_opened_distance)


#print('Bring all cams to zeroeth position')
##print('Bringing all cams to zeroeth position')
#
#for channel in range(0,16):
#    pwm.set_pwm(channel,0,servo_min)                                            # Setting all servo's to min position
#sleep(time_constant)
#print('All cams at zeroeth position')
#%%
print('Make all cams go to the "home" position')
gmr.home_position()
print('Done with home position')
#%%
#print('Starting main program')
input('Press any key to start the main program')
    
while idx < np.size(distances,0):
    present_rot_angle = rotational_angle[idx]                                  #rotational angle at the point under consideration. 
    present_angle = angles[idx]                                                #bending angle at the point under consideration. 
    present_distance = distances[idx]                                          #distance from the point under consideration to the next. 
    traversed_distance += present_distance                                     #total distanced travelled from the tip of the catheter
    print('----distance:'+str(present_distance)+'mm, angle:'+str(present_angle) +
          'degrees, rotation:'+str(present_rot_angle) + 'degrees.-----')
        
    if traversed_distance < lens:                                             #Check if the travelled distance is less than the length of the present material
        print("At index:" + str(idx) +" in directions")
        if present_rot_angle < rotationalAngle_threshold:                     #Do calculations if no rotational angle
#            if rotation_flag:
#                sks.rotate_catheter(0)
#                rotation_flag = 0               
            if abs(present_angle) < angle_threshold:                           #Compare the bend angle with the threshold that we set.
                incremental_distance = incremental_distance + present_distance #Remember the incremental distances between points upto a certain bend 
                flag = 1                                                       #Flag is raised to keep note of the incrementation
            else:
                if flag:
                    sks.push_catheter(servoDist_threshold,incremental_distance,
                                      outer_diameter)                          #When bend angle is approached, it pushes the catheter by the incremental distance,
                    incremental_distance = 0                                   #that it had kept in memory so far. 
                    flag = 0                                                        
                
                ### now do it for the current position
                sks.bend_catheter(present_angle,lens,outer_diameter)                #Bend the catheter by specific angle
                sks.push_catheter(servoDist_threshold,present_distance,
                                  outer_diameter)                              #Push the catheter by appropriate distance after the bend
    
        else:                        
            if flag:
                sks.push_catheter(servoDist_threshold,incremental_distance,
                                  outer_diameter)                              #When bend angle is approached, it pushes the catheter by the incremental distance,
                incremental_distance = 0                                       #that it had kept in memory so far. 
                flag = 0
            
            
#            sks.rotate_catheter(present_rot_angle)                             #Rotate the plane of the catheter for the z-axis        
            sks.new_rotate_catheter(present_rot_angle)                          #Rotate the plane of the catheter by taking care of our construction and restrictions
            
            sks.bend_catheter(present_angle,lens,outer_diameter)                    #Bend it by the bending angle 
            sks.push_catheter(servoDist_threshold, present_distance,
                              outer_diameter)                                  #Push the catheter by appropriate distance after the bend
                
        idx = idx + 1
        
    else:
        if idx==0 and traversed_distance > servoDist_threshold:
            sks.push_catheter(servoDist_threshold,traversed_distance,outer_diameter)
            idx = idx+1
        prop_idx = prop_idx + 1                                                 #Once the travelled length is greater than the length of material under 
        lens = lengths[prop_idx]                                                #consideration, then move onto the next material which might have a different OD. 
        outer_diameter = OD[prop_idx]
    
wait = input('Press 0 to exit the program')
if wait ==0:
    zero_position()
    print('Done')
else:
    print('Done')
    sys.exit()

