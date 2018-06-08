# -*- coding: utf-8 -*-
"""
Created on Wed Jun  6 11:50:32 2018

@author: ATI2-Pavan Gurudath
"""
#%% Import statements
import time
from time import sleep
import Adafruit_PCA9685
import numpy as np


#%% PWM initializing
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)

#%% 
servo_min = 190                                                                 #Min limit of 183 for Hitech-servos
servo_max = 595                                                                 #Max limit of 595 for Hitech-servos
time_constant = 3                                                               #Time for the Rpi to wait for the servo to complete its task
from_low = 0                                                                    #Smallest angle that you'd want the cam to be at
from_high = 180 
e = 1.59                                                                        #eccentricity of cam - 1.59mm
#%%
def angle_to_pulse(angle):
    pulse = 0
    pulse = (angle-from_low)*(servo_max-servo_min)/(from_high-from_low) + servo_min
##    print(pulse)
    return int(pulse)

def angle_to_distance(angle):
    e=1.59
    distance = e-e*np.cos(angle*np.pi/180)
    return distance

def distance_to_angle(distance):
    # Convert distance to servo angle b/w 0-180 (cam_formulae)
    e = 1.59 #1.59mm
    theta = np.arccos((e-distance)/e)*180/np.pi                                 #theta in degrees
    return theta

def distance_to_pulse(distance):
    global e                #eccentricity of cam - 1.59mm
    theta = np.arccos((e-distance)/e)*180/np.pi
    pulse = 0
    pulse = (theta-from_low)*(servo_max-servo_min)/(from_high-from_low) + servo_min
##    print(pulse)
    return int(pulse)
    
def back_gripper(flag,distance,channel=0,timeConstant = time_constant):
    #command it to open/close based on flag. Let flag be distance for now, even
    #though its just max or min position 
    #flag=0 --> open
    #flag=1 --> close
                                                   
    
#    from_low = 0                                                                #Smallest angle that you'd want the cam to be at
#    from_high = 180                                                             #Largest angle that you'd want the cam to be at
#    to_low = servo_min                                                          #least pulse that the servo can take
#    to_high = servo_max                                                         #max pulse that the servo can take
    
#    angle = distance_to_angle(distance)                                         #Calculate angle for servo to move for that distance to
#                                                                                be achieved
#    pulse = angle_to_pulse(angle, from_low,from_high, to_low, to_high)          #Calculate pulse to be sent to Rpi 
##    print('back gripper')
    pulse = distance_to_pulse(distance)                                          #Calculate pulse to be sent to Rpi
    pwm.set_pwm(channel,0,pulse)
    sleep(timeConstant)
##    print('back gripper done')
       
  
def front_gripper(flag,distance,channel=3,timeConstant = time_constant):
    #command it to open/close based on flag. Let flag be distance for now, even
    #though its just max or min position.    
##    print('front gripper')
    pulse = distance_to_pulse(distance)
    pwm.set_pwm(channel,0,pulse)
    sleep(timeConstant)
##    print('front gripper done')
  

def back_gripper_x(flag,distance,channel=1,timeConstant = time_constant):
    #command it to move either by servoDist_threshold or a particular distance. 
##    print('back gripper x-direction')
    pulse = distance_to_pulse(distance)
    pwm.set_pwm(channel,0,pulse)
    sleep(timeConstant)
##    print('back gripper x-direction done')
            
    
def back_rotation(flag,angle,channel=2,timeConstant = time_constant):
    #command it to rotate by a particular angle
    print('Rotating by '+str(angle)+'degrees')
    pulse = angle_to_pulse(angle)
    pwm.set_pwm(channel,0,pulse)
    sleep(timeConstant)
    
def bAngle_to_bDist(angle,material_type,fr_size):
    #Formulae to be used in order to get the distance by which the catheter 
    #needs to be bent to obtain the right shape and thereby convert that distance
    #to the pulse
    bDist = 1.5
    return bDist


def bending_arm(flag,angle,material_type=1,fr_size=5,channel=5):
    #command it to move by a particular distance to achieve the bending angle
    #### Need to know mechanism ####
    print('Bending by ' +str(angle)+'degrees ')
    if angle<90:
        bDist = bAngle_to_bDist(angle,material_type,fr_size)
        bPulse = distance_to_pulse(bDist)
        pwm.set_pwm(channel,0,bPulse)
    else:
        angle = 90-angle   #Comment
        #Give it a pulse such that it moves in the opposite direction by the same 
        #distance as that calculated from the "if" block. 

def push_action(distance):
    flag=1
    print('Front gripper partially opened')
    front_gripper(flag,partially_opened_distance)
    
    print('Back gripper fully closed')
    back_gripper(flag,fully_closed_distance)
    
    print('Back grippper moving forward by '+str(distance))
    back_gripper_x(flag,distance)
    
    print('Front gripper fully closed')
    front_gripper(flag,fully_closed_distance)
    
    print('Back gripper partially opened')
    back_gripper(flag,partially_opened_distance)
    
    print('Back gripper moved backwards to original position')
    back_gripper_x(flag,fully_bwd_distance)
    
    print('Back gripper fully closed')
    back_gripper(flag,fully_closed_distance)

      
def home_position():
    flag=1
    
    print('Front gripper partially opened')
    front_gripper(flag,partially_opened_distance)
    
    print('Back gripper partially opened')
    back_gripper(flag,partially_opened_distance)
    
    print('Back gripper moved backwards to zeroeth position')
    back_gripper_x(flag,fully_bwd_distance)
    
    print('Bending pins moved to zeroeth position')
    bending_arm(flag,partially_opened_distance)
    
    print('Back gripper on the plane at zeroeth angle')
    back_rotation(flag,0)

#%% main function


fully_closed_distance = angle_to_distance(0)
fully_opened_distance = angle_to_distance(180)
partially_opened_distance = angle_to_distance(60)
fully_bwd_distance = angle_to_distance(0)                                     #Needs to be changed after the cams are mounted properly


if __name__ == "__main__":
    print('Bringing all cams to zeroeth position')
    
    pwm.set_pwm(0,0,190)
    pwm.set_pwm(1,0,590)
    pwm.set_pwm(3,0,190)
    time.sleep(5)
    
    
    print('Going to home position')
    home_position()
    print('Done with home position')
    print('Waiting for push to start in ...')
    for i in range(3,0,-1):
        time.sleep(i)
        print(i)
        
    push_action(0.1)
    print('Done with one gripper action and moving back to home position in ...')
    for i in range(3,0,-1):
        time.sleep(i)
        print(i)
    
    home_position()
    print('Disconnect all supplies.')

    