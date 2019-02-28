# -*- coding: utf-8 -*-
"""
Created on Fri Aug 3 17:00:00 2018

@author: ATI-2 Pavan Gurudath
"""

#%% Import statements
import gripper_movements_rpi as gmr
import factors as fact
#import heating_control as htc
#import catheter_properties as cpro
import time

#%% Indexing
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
    pulse_distance = remaining_distance(Dist,servoDist_threshold)             # Split it up into threshold distances if it is greater
    for rDistances in pulse_distance:
        gmr.push_action(rDistances*distanceFactor)


#%%Bending and rotating the catheter

# Function to call the bending action
def bend_catheter(angle, lens, outer_diameter):
    time.sleep(3)
    gmr.bending_arm(angle, lens, outer_diameter)

def straighten_bending(angle,lens,outer_diameter):
    print('----** Straightening right pin** ---')
    time.sleep(3)
    gmr.bending_arm_back(angle, lens, outer_diameter)
# Function to call the rotation action
def new_rotate_catheter(rot_angle):
    print('----****---Turn the plane to ' + str(rot_angle)+'degrees---****----') 
    gmr.new_back_rotation(rot_angle)
    

