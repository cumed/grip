# -*- coding: utf-8 -*-
"""
Created on Fri Aug 3 17:00:00 2018 

@author: ATI-2 Pavan Gurudath
"""

import numpy as np
import gripper_movements_rpi as gmr
import Adafruit_PCA9685
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)
import skeleton_structure as sks
import catheter_properties as cpro
import os
import factors as fact


#%% Define directions so that the file takes the correct npy file into account. If master_main is running, then it'll save the .npy as runfile.npy
currDir = os.path.dirname(os.path.realpath('__file__'))

check = int(input('If running this file standalone, then press 1. Otherwise press 0 if running master_main.py and hit enter \n'))
if check==1:
    print('You are running this file by itself.\n')
    filename = input("Enter the npy file name without the \ file extension. For example: 26mmpigtail2.npy should be entered as 26mmpigtail2 \n")
    print(filename)
    fileStringName = 'npy/'+ str(filename) +'.npy'
#    filename = os.path.join(currDir,'npy')
#    filename = os.path.join(filename,fileStringName)
    directions = np.load(fileStringName)
else:
    filename = os.path.join(currDir,'runfile.npy')
    directions = np.load(filename)

#%%
incremental_distance,traversed_distance = 0,0                                   # Keeping track of distances until a bend is supposed to happen and the total distance distance travelled

distances = directions[:,0]                                                     # Reads the distance from the directions npy file i.e. first column
angles = directions[:,1]                                                        # Reads the bending angles from the directions npy file i.e. second column
rotational_angle = directions[:,2]                                              # Reads the rotational angle from the directions npy file i.e. third column
#%%
print('These are the bend angles for the first 13 points')
print(directions[0:12,1])
left_right= int(input('Do you want to flip these angles i.e. make them negative? \nIf so, Press 1. Otherwise, press 0 and hit enter \n'))
if left_right:
    angles = -1*angles

#%% Variables that can be changed

servoDist_threshold       = fact.servoDist_threshold                            # Max distance travelled by the back indexing servo(4.75*2)
angle_threshold           = fact.angle_threshold                                # Min angle required that the catheter needs to be bent by
rotationalAngle_threshold = fact.rotationalAngle_threshold                      # Min angle required that the catheter needs to be rotated by

servo_min, servo_max = fact.servo_min,fact.servo_max                            # Min,Max limit of 183,600 for Hitech-servos 



#%% Uncommenting block
#catheter_ID     = fact.catheter_ID                                             # The catheter code number from teh main database
#cpro.get_properties(catheter_ID)                                               # Creates the current catheter sheet that'll have all the details for the <catheter_ID> catheter. 

#properties_flag = 0                                                            # Set it to 1 if you require all the properties in one go. 
#[lengths, ODs, IDs, Materials, HysterisisFactors, HeatTimes, Xis, Yis, MandrelMaterials, MandrelODs] = cpro.get_properties(catheter_ID,properties_flag) 

#%% 
lengths = cpro.get_length()                                                     # Get all the material cumulative lengths. 
OD = cpro.get_OD()                                                              # Get all the ODs of the different material regions in the catheter

## Only for testing purpose, if necessary, uncomment these lines. 
#OD = fact.ODList                                                               # Obtains the ODs from factors.py if you want to change the OD quickly for testing purpose. 

#%%
#idx                                                                            # Index for points obtained from SVG
#prop_idx                                                                       # Index for the properites of the catheter
#flag                                                                           # Check for incremental distance until a bending is approached. 
#rotation_flag                                                                  # Check for incremental distance until a rotating angle is approached *currently unused*
idx, prop_idx, flag, rotation_flag = 0,0,0,0                                    # Reasons for each variable used is as in the above 4 lines

lens            = lengths[prop_idx]                                             # Obtains the length of the first material in the catheter
outer_diameter  = OD[prop_idx]                                                  # Obtains the OD of the first material in the catheter


#%% Zeroing of cams


test = input('Zero the cams so that you can place the catheter from top?\n 1 - Yes\n 0 - No')       #Press 1 if you want the grippers to be completely removed, so that its easy to insert the catheters during testing
if test ==1:
    gmr.zero_position()
    input('Press any key to continue after inserting the catheter to continue...')

#Move the grippers to a distance slightly more than the partially opened distance so that the catheter can slide through easily so that it can be positioned easily.
slightlyMore_opened_distance = 2.2
gmr.front_gripper(slightlyMore_opened_distance)
gmr.back_gripper(slightlyMore_opened_distance)

#%%
input('You can move the catheter and adjust it to send it to the zeroeth position. \n Once set, press 1 to make the cams go to "home" position')
gmr.home_position()

#%%
input('Press any key to start the main program')
while idx < np.size(distances,0):
    present_rot_angle = rotational_angle[idx]                                  #rotational angle at the point under consideration. 
    present_angle = angles[idx]                                                #bending angle at the point under consideration. 
    present_distance = distances[idx]                                          #distance from the point under consideration to the next. 
    traversed_distance += present_distance                                     #total distanced travelled from the tip of the catheter
    print('----distance:'+str(round(present_distance,2))+'mm, angle:'+str(round(present_angle,2)) +
          'degrees-----')
        
    if traversed_distance < lens:                                              #Check if the travelled distance is less than the length of the present material
        print("At index:" + str(idx) +" in directions")
        if present_rot_angle < rotationalAngle_threshold:                      #Do calculations if no rotational angle               
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
                sks.bend_catheter(present_angle,lens,outer_diameter)           #Bend the catheter by specific angle
                sks.push_catheter(servoDist_threshold,present_distance,
                                  outer_diameter)                              #Push the catheter by appropriate distance after the bend
                sks.straighten_bending(present_angle,lens,outer_diameter)
    
        else:                        
            if flag:
                sks.push_catheter(servoDist_threshold,incremental_distance,
                                  outer_diameter)                              #When bend angle is approached, it pushes the catheter by the incremental distance,
                incremental_distance = 0                                       #that it had kept in memory so far. 
                flag = 0
            
            sks.new_rotate_catheter(present_rot_angle)                         #Rotate the plane of the catheter by taking care of our construction and restrictions
            
            sks.bend_catheter(present_angle,lens,outer_diameter)               #Bend it by the bending angle 
            sks.push_catheter(servoDist_threshold, present_distance,
                              outer_diameter)                                  #Push the catheter by appropriate distance after the bend
            sks.straighten_bending(present_angle,lens,outer_diameter)     
        idx = idx + 1
        
    else:
        if idx==0 and traversed_distance > servoDist_threshold:
            sks.push_catheter(servoDist_threshold,traversed_distance,outer_diameter)
            idx = idx+1
        prop_idx = prop_idx + 1                                                #Once the travelled length is greater than the length of material under 
        lens = lengths[prop_idx]                                               #consideration, then move onto the next material which might have a different OD. 
        outer_diameter = OD[prop_idx]
    
gmr.bendingPin_zero()
input('You have completed shaping the catheter. Press any key to remove it.')
gmr.zero_position()



