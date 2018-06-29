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
def remaining_distance(remDist,servoDist):                                    
    quotient = int(remDist // servoDist)
    remainder = remDist - servoDist*quotient
    pulseDist = []
    for number in range(quotient):
        pulseDist.append(servoDist)
    pulseDist.append(remainder)
    return pulseDist

# Pushing the catheter in front
def push_catheter(servoDist_threshold, Dist, outer_diameter):
    if Dist > servoDist_threshold:                                           #Check if the pushing distance is more than the servo's threshold distance
        pulse_distance = remaining_distance(Dist,servoDist_threshold)        #Split it up into threshold distances if it is greater
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
    flag=1                                                                   #Flag plays no role right now, its there for any future requirement 
    gmr.bending_arm(angle, outer_diameter,flag)

def rotate_catheter(rot_angle):
    #This function has been written keeping in mind that the rotation is done only in one direction, i.e. 0-max angle possible by the servo. 
    #Whenever a negative angle is passed, it means that the call is to move it back to the zeroeth position. 
    #This function would have to be changed to take into account the new mechanism, which is currently put on hold i.e. an incremental movement of a max 
    #of 15 degrees in either direction. 
    print('----------Turn the plane to ' + str(rot_angle)+'degrees----------')                                
#    if rot_angle>0:
#        flag=1                                                               #Flag is raised once rotation is done in the positive direction
#    else:
#        flag=0                                                               #Flag is put down if rotation is in the negative direction 
    
#    print('Sending rotation with FLAGGGGGGGL command'+str(flag) +' and rotational angle'+str(rot_angle))
#    gmr.back_rotation(rot_angle,flag)
    gmr.back_rotation(rot_angle)
