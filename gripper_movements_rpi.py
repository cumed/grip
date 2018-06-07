# -*- coding: utf-8 -*-
"""
Created on Wed Jun  6 11:50:32 2018

@author: ATI2-Pavan Gurudath
"""
from time import sleep
import Adafruit_PCA9685
import numpy as np

pwm = Adafruit_PCA9685.PCA9685()

servo_min = 190             #Min limit of 183 for Hitech-servos
servo_max = 595             #Max limit of 595 for Hitech-servos
time_constant = 2
from_low = 0                                                                #Smallest angle that you'd want the cam to be at
from_high = 180 
e = 1.59                    #eccentricity of cam - 1.59mm

def angle_to_pulse(angle):
    pulse = 0
    pulse = (angle-from_low)*(servo_max-servo_min)/(from_high-from_low) + servo_min
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
    return int(pulse)
    
def back_gripper(flag,distance,channel=0,timeConstant = time_constant):
    #command it to open/close based on flag. Let flag be distance for now, even
    #though its just max or min position 
    #flag=0 --> open
    #flag=1 --> close
                                                   #Time for the Rpi to wait for the servo to complete its task
    
#    from_low = 0                                                                #Smallest angle that you'd want the cam to be at
#    from_high = 180                                                             #Largest angle that you'd want the cam to be at
#    to_low = servo_min                                                          #least pulse that the servo can take
#    to_high = servo_max                                                         #max pulse that the servo can take
    
#    angle = distance_to_angle(distance)                                         #Calculate angle for servo to move for that distance to
#                                                                                be achieved
#    pulse = angle_to_pulse(angle, from_low,from_high, to_low, to_high)          #Calculate pulse to be sent to Rpi 
    pulse = distance_to_pulse(distance)                                         #Calculate pulse to be sent to Rpi
    pwm.set_pwm(channel,0,pulse)
    sleep(timeConstant)

       
  
def front_gripper(flag,distance,channel=3,timeConstant = time_constant):
    #command it to open/close based on flag. Let flag be distance for now, even
    #though its just max or min position.    
    pulse = distance_to_pulse(distance)
    pwm.set_pwm(channel,0,pulse)
    sleep(timeConstant)
  
  

def back_gripper_x(flag,distance,channel=1,timeConstant = time_constant):
    #command it to move either by servoDist_threshold or a particular distance. 
    pulse = distance_to_pulse(distance)
    pwm.set_pwm(channel,0,pulse)
    sleep(timeConstant)
    
            
    
def back_rotation(flag,angle,channel=2,timeConstant = time_constant):
    #command it to rotate by a particular angle
    pulse = angle_to_pulse(angle)
    pwm.set_pwm(channel,0,pulse)
    sleep(timeConstant)
    
    
#def bending_arm(angle,channel=5):
#    #command it to move by a particular distance to achieve the bending angle
#    #### Need to know mechanism ####
#    return distance
    

def push_action(distance):
    flag=1
    fully_closed_distance = angle_to_distance(180)            #3.18    
    fully_opened_distance = angle_to_distance(0)              #0
    partially_opened_distance = angle_to_distance(60)         #
    fully_bwd_distance = fully_opened_distance
    
    front_gripper(flag,partially_opened_distance)
    back_gripper(flag,fully_closed_distance)
    back_gripper_x(flag,distance)
    front_gripper(flag,fully_closed_distance)
    back_gripper(flag,partially_opened_distance)
    back_gripper_x(flag,fully_bwd_distance)
    back_gripper(flag,fully_closed_distance)
    
    
def home_position():
    flag=1
    partially_opened_distance = distance_to_angle(60)
    fully_bwd_distance = distance_to_angle(0)
    
    front_gripper(flag,partially_opened_distance)
    back_gripper(flag,partially_opened_distance)
    back_gripper_x(flag,fully_bwd_distance)
#    back_rotation(flag,0)
    
push_action(3)