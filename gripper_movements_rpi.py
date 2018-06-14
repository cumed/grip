# -*- coding: utf-8 -*-
"""
Created on Thur Jun  14 9:36:00 

@author: ATI2-Pavan Gurudath
"""
#%% Import statements
import time
from time import sleep
import Adafruit_PCA9685
import numpy as np
import math
from math import pi

#%% Declarations
servo_min = 190                                                                 #Min limit of 183 for Hitech-servos
servo_max = 595                                                                 #Max limit of 600 for Hitech-servos
time_constant = 1                                                               #Time for the Rpi to wait for the servo to complete its task
from_low = 0                                                                    #Smallest angle that you'd want the cam to be at
from_high = 180                                                                 #Largest angle that you'd want the cam to be at
e = 1.59                                                                        #eccentricity of cam - 1.59mm
d_pins = 9.5                                                                    #Distance between the bending pins (edge-to-edge) **3/8th inch**
y_i =   2                                                                       #Distance between the front gripper and the bending pins **80 thou inch**

#%% Mapping the angle on the servo to the pulse range
def angle_to_pulse(angle):
    pulse = 0
    pulse = (angle-from_low)*(servo_max-servo_min)/(from_high-from_low) + servo_min
    return int(pulse)

#%% Mapping the angle on the servo to the linear movement obtained using the cam
def angle_to_distance(angle):
    e=1.59
    distance = e-e*np.cos(angle*pi/180)
    return distance

#%% Converts linear distance using the cam to the servo angle b/w 0-180 (cam_formulae)
def distance_to_angle(distance,e=1.59):                                         #1.59mm
    theta = np.arccos((e-distance)/e)*180/pi                                    #theta in degrees
    return theta

#%% Returns the pulse that is required to achieve the linear distance
def distance_to_pulse(distance,e=1.59):                                          #eccentricity of cam - 1.59mm
    theta = np.arccos((e-distance)/e)*180/pi
    pulse = 0
    pulse = (theta-from_low)*(servo_max-servo_min)/(from_high-from_low) + servo_min
    return int(pulse)

#%% Returns the distance that the bending pins need to move for the bend to happen
def bendAngle_to_bendDist(angle,outer_diameter):
    #Formulae to be used in order to get the distance by which the catheter 
    #needs to be bent to obtain the right shape and thereby convert that distance
    #to the pulse
    x_i = (d_pins - outer_diameter)/2
    bendDist = x_i + y_i *math.tan(math.radians(angle))
    return bendDist

def bendDist_to_bendPulse(angle,bendDist,e=1.59):
    theta = np.arccos((e-bendDist)/e)*180/pi
    
    if angle>0:
        to_low_b    = 190
        to_high_b  = 500
        from_low_b  = -90
        from_high_b = 90

    else:
        to_low_b      = 190
        to_high_b     = 500
        from_low_b    = 90
        from_high_b   = -90
        
        
    pulse = (theta-from_low_b)*(to_high_b-to_low_b)/(from_high_b-from_low_b)+to_low_b
    return pulse

#%% Gripper movements    
def back_gripper(flag,distance,channel=0,timeConstant = time_constant):
    #command it to open/close based on flag. Let flag just be there for now. 
    #No use of it right now since its just max or min position 
    #flag=0 --> open
    #flag=1 --> close
                                                   
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
            
    
def back_rotation(flag,angle,channel=7,timeConstant = time_constant):
    #command it to rotate by a particular angle
    print('Rotating by '+str(angle)+'degrees')
    if flag==1:
        pulse = angle_to_pulse(angle)
        pwm.set_pwm(channel,0,pulse)
        sleep(timeConstant)
    else:
        pulse = angle_to_pulse(0)
        pwm.set_pwm(channel,0,pulse)
        sleep(timeConstant)
    

def bending_arm(flag,angle,outer_diameter,channel=5,timeConstant = time_constant):
    #command it to move by a particular distance to achieve the bending angle
    #Home position is at the center. Therefore, assume it is at an angle 90 on its servo, since middle position. 
    #Depending upon positive or negative angle, the bending pins moves either to the left(-ve) or to right(+ve)
    #Need to map that distance to the angle.
    bendDist = bendAngle_to_bendDist(abs(angle),outer_diameter)
    pulse = bendDist_to_bendPulse(angle,bendDist)
    pwm.set_pwm(channel,0,pulse)
    sleep(timeConstant)
    print('wait for a while and bring back to zeroeth position')
    pwm.set(channel,0,0) #Zeroeth position ********************NEED TO DETERMINE ZEROETH*************************************

#%%        
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
    
    print('Back gripper moved backwards to home position')
    back_gripper_x(flag,fully_bwd_distance)
    
    print('Bending pins moved to home position')
    bending_arm(flag,0)
    
    print('Back gripper on the plane at home angle')
    back_rotation(flag,0)



#%% Needs to be changed to incorporate the function written by Sramana
fully_closed_distance       = angle_to_distance(0)                  
partially_opened_distance   = angle_to_distance(60)

fully_opened_distance       = angle_to_distance(180)                        # Position of front and back servos along x-direction
fully_bwd_distance          = angle_to_distance(0)                          # Position of cam for the back servo movement along y-direction


#%% main function
if __name__ == "__main__":
    pwm = Adafruit_PCA9685.PCA9685()
    pwm.set_pwm_freq(60)
    

    
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

    
