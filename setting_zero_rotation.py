# -*- coding: utf-8 -*-
"""
Created on Wed Jul 25 12:45:02 2018

@author: ATI-2 Pavan Gurudath
"""

from time import sleep
import Adafruit_PCA9685
import numpy as np
import math
from math import pi
import heating_control as htc
import catheter_properties as cpro
import factors as fact

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)

#%% Declarations
servo_min = 190                                                              # Min limit of 183 for Hitech-servos
servo_max = 500                                                              # Max limit of 600 for Hitech-servos
time_constant = 0.25                                                            # Time for the Rpi to wait for the servo to complete its task
# from_low = 0                                                               # Smallest angle that you'd want the cam to be at
#from_high = 180                                                             # Largest angle that you'd want the cam to be at
e_gripper = 1.59                                                             # eccentricity of gripper cams - 1.59mm
e_bending = 9.25                                                             # eccentricity of bending cam - 9.25mm
e_backidx = 4.75                                                             # eccentricity of back indexing gripper cam - 4.75mm
d_pins = fact.d_pins                                                                # Distance between the bending pins (edge-to-edge) **0.207inch**
y_i =   fact.y_i                                                                    # Distance between the front gripper and the bending pins

#%% Declare all channels
ch_backGripper = 0
ch_frontGripper = 3
ch_backidxGripper = 1
ch_bendingPins = 5
ch_rotatingArm = 8


#%%
from_angles = {
        'positive bend': [-90,90],                                           # If the bending is taking place for a positive angle, then the bending pins need to move to the right                 
        'negative bend': [90,-90],                                           # If the bending is taking place for a negative angle, then the bending pins need to move to the left
        }
bendPinsFactor = 0
zeroethPosition = 0                                                          # The zeroeth position of the rotational servo
rotationalAngle_threshold = 15

#%%
def angle_to_pulse(angle,from_low=0,from_high=180):
    pulse = (angle-from_low)*(servo_max-servo_min)/(from_high-from_low) + servo_min
    return int(pulse)

# Converts linear distance using the cam to the servo angle b/w 0-180 (cam_formulae)
def distance_to_angle(distance,e):                                           # 1.59mm
    angle = np.arccos((e-distance)/e)*180/pi                                 # theta in degrees
    return angle

# Converts the servo angle to the linear distance (cam_formulae)
def angle_to_distance(angle,e):
    distance = e-e*np.cos(angle*pi/180)
    return distance

# Returns the pulse that is required to achieve the linear distance
def distance_to_pulse(distance,e,from_low = 0, from_high = 180):             # eccentricity of cam - 1.59mm
    angle = distance_to_angle(distance,e)
    return angle_to_pulse(angle,from_low,from_high)                          # pulse = angle_to_pulse(theta,from_low,from_high)

#%%
def back_gripper(f_distance,e=e_gripper,channel=ch_backGripper,timeConstant = time_constant):
    #Let flag just be there for now. 
    #No use of it right now since its just max or min position ***UPDATE: Flag has been removed***
#    print('Back gripper moving by '+str(f_distance))
    pulse = distance_to_pulse(f_distance,e)                                  # Calculate pulse to be sent by Rpi for back gripper's movement
    pwm.set_pwm(channel,0,pulse)
    sleep(timeConstant)
#    print('Back gripper movement done. Channel:'+str(channel)+', Eccentricity:'+str(e)+', Pulse: '+str(pulse))


def front_gripper(f_distance,e=e_gripper,channel=ch_frontGripper,timeConstant = time_constant):
    #Let flag be there for now, even though its just max or min position.    
    #No use of it right now since its just max or min position ***UPDATE: Flag has been removed***
#    print('Front gripper moving by '+str(f_distance))
    pulse = distance_to_pulse(f_distance,e)                                  # Calculate pulse to be sent by Rpi for front gripper's movement
    pwm.set_pwm(channel,0,pulse)
    sleep(timeConstant)
#    print('Front gripper movement done. Channel:'+str(channel)+' , Eccentricity:'+str(e)+', Pulse: '+str(pulse))


def back_gripper_indexing(distance,e=e_backidx,channel=ch_backidxGripper,timeConstant = time_constant):
    #Let flag be there for now, even though its just max or min position.    
    #No use of it right now since its just max or min position ***UPDATE: Flag has been removed***
    
    #command it to move either by servoDist_threshold or a particular distance. 
#    print('Back gripper moving forward by '+str(distance)+'mm ')
    pulse = distance_to_pulse(distance,e)                                    # Calculate pulse to be sent by Rpi for back indexing gripper's movement
    pwm.set_pwm(channel,0,pulse)
    sleep(timeConstant)

#%%
def split_angles(angle,rotAngle_threshold=rotationalAngle_threshold):
    quotient = int(angle//rotAngle_threshold)
    remainder = angle - rotAngle_threshold*quotient
    rotAngles = []
    for ang in range(quotient):
        rotAngles.append(rotAngle_threshold)
    rotAngles.append(remainder)
    return rotAngles

def rotationalAngle_to_servoAngle(angle):
    #this function defines the mapping of rotational angle to servo angle 
    servoAngle =-8e-5*(angle**4) + 0.0132*(angle**3) - 0.4302*(angle**2) + 9.641*angle + 78.688
    return servoAngle


def rotateTheCatheterByPositiveAngle(angle_list):
    for angles in angle_list:
        back_gripper(fully_closed_distance)
        front_gripper(partially_opened_distance)
        rotateThisCatheter(angles)
        
        front_gripper(fully_closed_distance)
        back_gripper(partially_opened_distance)
        rotateThisCatheter(zeroethPosition)
        
        back_gripper(fully_closed_distance)
    
def rotateTheCatheterByNegativeAngle(angle_list):
    for angles in angle_list:
        front_gripper(fully_closed_distance)
        back_gripper(partially_opened_distance)
        rotateThisCatheter(angles)
        
        back_gripper(fully_closed_distance)
        front_gripper(partially_opened_distance)
        rotateThisCatheter(zeroethPosition)
        
        front_gripper(fully_closed_distance)
    

def rotateThisCatheter(angle,channel = ch_rotatingArm,timeConstant = time_constant):
    servoAngle = rotationalAngle_to_servoAngle(angle)
    pulse = angle_to_pulse(servoAngle)
    pwm.set_pwm(channel,0,pulse)
    sleep(timeConstant)
    
def new_back_rotation(angle,flag=0,channel=ch_rotatingArm,timeConstant = time_constant):
    #command it to rotate by a particular angle
    #Possible use the flag to control rotational_angle= 0. 
    #Rotating the planes only by 15 degrees at first

    if angle>0:
        angle_list = split_angles(angle)
        rotateTheCatheterByPositiveAngle(angle_list)
    elif angle<0:
        angle_list = split_angles(abs(angle))
        rotateTheCatheterByNegativeAngle(angle_list)
    else:
        rotateThisCatheter(angle)
        
        
fully_closed_distance       = 3.18                                           # Distance to close the gripper - 1.58 mm
partially_opened_distance   = 2.8                                           # Distance to just reach the gripper - 1.06mm
slightlyMore_opened_distance = 2.2

#*DEFAULT ALL THE TIME*           
fully_opened_distance       = 0 #angle_to_distance(0,e_gripper)                 #Position of front and back servos along x-direction (Default for all sizes )
fully_bwd_distance          = 0 #angle_to_distance(0,e_backidx)

while True:
    angle = input('Enter angle')
    if angle == 5000:
        angle = input('Enter angle')
        new_back_rotation(angle)
        
    elif angle ==1000:
        pulse = input('Enter angle')
        pwm.set_pwm(ch_rotatingArm,0,pulse)
        sleep(time_constant*2)
    
    else: 
        pulse = angle_to_pulse(angle)
        pwm.set_pwm(ch_rotatingArm,0,pulse)
        sleep(time_constant*2)
            
    
        