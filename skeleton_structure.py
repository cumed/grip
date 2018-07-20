# -*- coding: utf-8 -*-
"""
Created on Fri Jul 20 15:24:37 2018

@author: ATI-2 Pavan Gurudath
"""

#%% Import statements
import gripper_movements_rpi as gmr
import sys
import factors as fact
#import heating_control as htc
#import catheter_properties as cpro

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
distanceFactor=fact.distanceFactor
def push_catheter(servoDist_threshold, Dist, outer_diameter):
    pulse_distance = remaining_distance(Dist,servoDist_threshold)        #Split it up into threshold distances if it is greater
    for rDistances in pulse_distance:
        gmr.push_action(rDistances*distanceFactor)


#%%Bending and rotating the catheter
def bend_catheter(angle, lens, outer_diameter):
    print('****--Bend the catheter by '+str(angle)+'degrees--****')
    gmr.bending_arm(angle, lens, outer_diameter)


def new_rotate_catheter(rot_angle):
    wait = input("You're at the new rotation method. Do you want to continue? Press 0 to exit")
    if wait == 0:
        sys.exit()
    print('----****---Turn the plane to ' + str(rot_angle)+'degrees---****----') 
    gmr.new_back_rotation(rot_angle)
    

#def rotate_catheter(rot_angle):
#    #This function has been written keeping in mind that the rotation is done only in one direction, i.e. 0-max angle possible by the servo. 
#    #Whenever a negative angle is passed, it means that the call is to move it back to the zeroeth position. 
#    #This function would have to be changed to take into account the new mechanism, which is currently put on hold i.e. an incremental movement of a max 
#    #of 15 degrees in either direction. 
#    wait = input("You're at the old rotation method. Do you want to continue? Press 0 to exit")
#    if wait == 0:
#        sys.exit()
#    print('----****----Turn the plane to ' + str(rot_angle)+'degrees----****----')                                
#    gmr.back_rotation(rot_angle)