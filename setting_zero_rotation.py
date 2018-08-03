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
time_constant = 1                                                            # Time for the Rpi to wait for the servo to complete its task
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
zeroethPosition = 15                                                          # The zeroeth position of the rotational servo
rotationalAngle_threshold = 14

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
    if remainder !=0:
        rotAngles.append(remainder)
    return rotAngles

def rotationalAngle_to_servoAngle(angle):
    #this function defines the mapping of rotational angle to servo angle 
    angle = rotationalAngle_threshold-angle
    servoAngle =-8e-5*(angle**4) + 0.0132*(angle**3) - 0.4302*(angle**2) + 9.641*angle + 78.688
    return servoAngle


def rotateTheCatheterByPositiveAngle(angle_list):
    for angles in angle_list:
##        print('Performing rotation for '+str(angles) + 'for a total angle of '+str(sum(angle_list)))
##        print('Back gripper fully closed')
##        back_gripper(fully_closed_distance)
##        print('Front gripper partially opened')
##        front_gripper(partially_opened_distance)
##        print('Rotating catheter')
#        rotateThisCatheter(angles)
#        
##        print('Front gripper fully closed')
##        front_gripper(fully_closed_distance)
##        print('Back gripper partially opened')
##        back_gripper(partially_opened_distance)
##        print('Moving the catheter back to zeroeth position')
##        rotateThisCatheter(zeroethPosition)
##        
##        print('Back gripper fully closed')
##        back_gripper(fully_closed_distance)
##        
##        print('Done with '+ str(angle_list.index(angles)) +' round of rotation')
    print('Done with the rotation completely')

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
    print('Servo angle: '+str(servoAngle))
    pulse = angle_to_pulse(servoAngle)
    print('Pulse: '+str(pulse))
    pwm.set_pwm(channel,0,pulse)
    sleep(timeConstant)
    
def new_back_rotation(angle,flag=0,channel=ch_rotatingArm,timeConstant = time_constant):
    #command it to rotate by a particular angle
    #Possible use the flag to control rotational_angle= 0. 


    if angle>0:
        angle_list = split_angles(angle)
        print(angle_list)
        rotateTheCatheterByPositiveAngle(angle_list)
    elif angle<0:
        angle_list = split_angles(abs(angle))
        print(angle_list)
#        rotateTheCatheterByNegativeAngle(angle_list)
    else:
        rotateThisCatheter(angle)
        print('0')
        
#%% 
        
def bendingPin_zero(e=e_bending,channel=ch_bendingPins, timeConstant = time_constant):
##    print('Do we move bending pins back to zeroeth position')
#    pulse_zero = angle_to_pulse(0,from_low_b=-90,from_high_b=90)
    pulse_zero = angle_to_pulse(0,-90,90)                                    # Calculate pulse to be sent by Rpi to move the bending pins to the zeroeth position
    print(pulse_zero)
    pwm.set_pwm(channel,0,pulse_zero)
    sleep(timeConstant)
    
def zero_position():
    front_gripper(0)
    back_gripper(0)
    back_gripper_indexing(0)
    bendingPin_zero()
    new_back_rotation(zeroethPosition)

def push_action(distance):
    front_gripper(partially_opened_distance)                                
    back_gripper(fully_closed_distance)
    back_gripper_indexing(distance*fact.distanceFactor)
    front_gripper(fully_closed_distance)
    back_gripper(partially_opened_distance)
    back_gripper_indexing(fully_bwd_distance)
    back_gripper(fully_closed_distance)
    print('Catheter pushed by '+str(distance)+'mm')
    
def reversePush_action(distance):
#    print('Front gripper partially opened')
    front_gripper(fully_closed_distance)
    back_gripper(fully_opened_distance)
    back_gripper_indexing(distance*fact.distanceFactor)
    back_gripper(fully_closed_distance)
    front_gripper(partially_opened_distance)
    back_gripper_indexing(fully_bwd_distance)
    front_gripper(fully_closed_distance)

def reversePush_action(distance):
#    print('Front gripper partially opened')
    front_gripper(fully_closed_distance)
    back_gripper(fully_opened_distance)
    back_gripper_indexing(distance*fact.distanceFactor)
    back_gripper(fully_closed_distance)
    front_gripper(partially_opened_distance)
    back_gripper_indexing(fully_bwd_distance)
    front_gripper(fully_closed_distance)
    
#%%
        
fully_closed_distance       = 3.18                                           # Distance to close the gripper - 1.58 mm
partially_opened_distance   = 2.8                                           # Distance to just reach the gripper - 1.06mm
slightlyMore_opened_distance = 2.2

#*DEFAULT ALL THE TIME*           
fully_opened_distance       = 0 #angle_to_distance(0,e_gripper)                 #Position of front and back servos along x-direction (Default for all sizes )
fully_bwd_distance          = 0 #angle_to_distance(0,e_backidx)


zero_position()

OD = fact.OD 
lens =3
while True:
    angle = input('Input angle')
    
    
    if angle == 200:
        print('Grippers- Fully Opened distance')
        front_gripper(fully_opened_distance)
        back_gripper(fully_opened_distance)
        
    elif angle == 500:
        noftimes = input('Number of times - 1mm')
        distance = input('Each increment of distance?')
        for ele in range(0,noftimes):
            push_action(distance)
<<<<<<< HEAD

=======
            
>>>>>>> 06b2befc794c483e5a2b2467f021f5deab094815
    elif angle==800:
        print('Grippers - slightly more opened position')
        front_gripper(slightlyMore_opened_distance)
        back_gripper(slightlyMore_opened_distance)
<<<<<<< HEAD

=======
        
>>>>>>> 06b2befc794c483e5a2b2467f021f5deab094815
    elif angle==900:
        noftimes = input('Number of times')
        distance = input('Each increment of distance?')
        for ele in range(0,noftimes):
            reversePush_action(distance)
<<<<<<< HEAD
            
=======
    
>>>>>>> 06b2befc794c483e5a2b2467f021f5deab094815
    else:
        new_back_rotation(angle)
        
