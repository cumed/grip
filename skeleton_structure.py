# -*- coding: utf-8 -*-
"""
Created on Thur Jun 21 12:58:00

@author: ATI2-Pavan Gurudath
"""
#%% Import statements
import gripper_movements_rpi as gmr
import heating_control as htc
import catheter_properties as cpro

#%% Distances

# Split distances into smaller threshold and then send the list of distances
def remaining_distance(servoDist, remDist):                                    
    quotient = int(remDist // servoDist)
    remainder = remDist - servoDist*quotient
    pulseDist = []
    for number in range(quotient):
        pulseDist.append(servoDist)
    pulseDist.append(remainder)
    return pulseDist

# Pushing the catheter in front
def push_catheter(servoDist_threshold, Dist, outer_diameter):
    if Dist > servoDist_threshold:                                              #Check if the pushing distance is more than the servo's threshold distance
        pulse_distance = remaining_distance(servoDist_threshold, Dist)          #Split it up into threshold distances if it is greater
        for rDistances in pulse_distance:
            print('----------Push catheter by '+str(rDistances) + 'mm from a total_distance of ' + str(Dist) + 'mm-----------')
            gmr.push_action(rDistances)
    else:
        print('----------Push catheter by '+str(Dist) + 'mm----------')
        gmr.push_action(Dist)

#%%Bending and rotatin the catheter
def bend_catheter(angle, lens, outer_diameter):
    heating_time = cpro.get_heatTime(lens)
    print('-----------Heat the catheter for '+str(heating_time)+'seconds --------------')
    htc.startHeat(heating_time)
    print('----------Bend the catheter by '+str(angle)+'degrees----------')
    flag=1                                                                      #Flag plays no role right now, its there for any future requirement 
    gmr.bending_arm(angle, outer_diameter,flag)

def rotate_catheter(rot_angle):
    #This function has been written keeping in mind that the rotation is done only in one direction, i.e. 0-max angle possible by the servo. 
    #Whenever a negative angle is passed, it means that the call is to move it back to the zeroeth position. 
    #This function would have to be changed to take into account the new mechanism, which is currently put on hold i.e. an incremental movement of a max 
    #of 15 degrees in either direction. 
    print('----------Turn the plane by ' + str(rot_angle)+'degrees----------')                                
    if rot_angle>0:
        flag=1
                                                                         #Flag is raised once rotation is done in the positive direction
    else:
        flag=0                                                                  #Flag is put down if rotation is in the negative direction 
    print('Sednding rotation with FLAGGGGGGGL command'+str(flag) +' and rotational angle'+str(rot_angle))
    gmr.back_rotation(rot_angle,flag)
     

#%% main function

#distances = directions[:,0]
#angles = directions[:,1]
#rotational_angle = directions[:,2]
#
#idx=0
#flag=0
#rotation_flag =0
#
#    
#while idx < np.size(distances,0):
#    present_rot_angle = rotational_angle[idx]                                   #distance, bend angle and rotational angle at the point under 
#    present_angle = angles[idx]                                                 #consideration. 
#    present_distance = distances[idx]
#    
#    if present_rot_angle == 0:                                                  #Do calculations if no rotational angle
#        if present_angle < angle_threshold:                                     #Compare the bend angle with the threshold that we setup
#            incremental_distance = incremental_distance + present_distance      #Remember the incremental distances between points upto a certain bend is approached
#            flag = 1                                                            #Flag is raised to keep note of the incrementation
#        else:
#            if flag:
#                push_catheter(servoDist_threshold,incremental_distance)         #When bend angle is approached, it pushes the catheter by the incremental distance
#                incremental_distance = 0                                        #it had kept in memory so far. 
#                flag = 0                                                        
#            
#            ### now do it for the current position
#            bend_catheter(present_angle)                                        #Bend the catheter by specific angle
#            push_catheter(servoDist_threshold,present_distance)                 #Push the catheter by appropriate distance after the bend
#
#    else:                        
#        if flag:
#            push_catheter(servoDist_threshold,incremental_distance)             #When bend angle is approached, it pushes the catheter by the incremental distance
#            incremental_distance = 0                                            #it had kept in memory so far. 
#            flag = 0
#            
#        rotate_catheter(present_rot_angle)                                      #Rotate the plane of the catheter for the z-axis
#        bend_catheter(present_angle)                                            #Bend it by the bending angle 
#        rotate_catheter(-present_rot_angle)                                     #Rotate it back to its original plane
#        push_catheter(servoDist_threshold, present_distance)                    #Push the catheter by appropriate distance after the bend
#    
#    idx = idx + 1
