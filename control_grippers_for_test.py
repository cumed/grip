# -*- coding: utf-8 -*-
"""
Created on Wed Jul 11 09:47:10 2018

@author: ATI-2 Pavan Gurudath
"""

from time import sleep
import Adafruit_PCA9685
import numpy as np
import math
from math import pi
#import heating_control as htc
#import catheter_properties as cpro

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)

servo_min = 190                                                              # Min limit of 183 for Hitech-servos
servo_max = 500                                                              # Max limit of 600 for Hitech-servos
time_constant = 0.3                                                            # Time for the Rpi to wait for the servo to complete its task
# from_low = 0                                                               # Smallest angle that you'd want the cam to be at
#from_high = 180                                                             # Largest angle that you'd want the cam to be at
e_gripper = 1.59                                                             # eccentricity of gripper cams - 1.59mm
e_bending = 9.25                                                             # eccentricity of bending cam - 9.25mm
e_backidx = 4.75                                                             # eccentricity of back indexing gripper cam - 4.75mm
d_pins = 5.30                                                                # Distance between the bending pins (edge-to-edge) **0.207inch**
y_i =   1.92                                                                    # Distance between the front gripper and the bending pins

#%% Declare all channels
ch_backGripper = 0
ch_frontGripper = 3
ch_backidxGripper = 1
ch_bendingPins = 5
ch_rotatingArm = 8

#%%
fully_closed_distance       = 3.18                                          # Distance to close the gripper - 1.58 mm
##partially_opened_distance   = input('Enter partially_opened_distance')                                          # Distance to just reach the gripper - 1.06mm
partially_opened_distance = 2.8
#*DEFAULT ALL THE TIME*           
fully_opened_distance,fully_bwd_distance = 0,0                                              #Position of front and back servos along x-direction (Default for all sizes )
slightlyMore_opened_distance = 2.2

#%%
from_angles = {
        'positive bend': [-90,90],                                           # If the bending is taking place for a positive angle, then the bending pins need to move to the right                 
        'negative bend': [90,-90],                                           # If the bending is taking place for a negative angle, then the bending pins need to move to the left
        }
bendPinsFactor = 0.4
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


#%% Gripper movements    
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
#    print('Back gripper y-direction movement done. Channel:'+str(channel)+' , Eccentricity:'+str(e)+', Pulse: '+str(pulse))
    
    
#%% Bending angles and movements 
# Returns the distance that the bending pins need to move for the bend to happen
def bendAngle_to_bendDist(angle,outer_diameter):
    #This function defines the distance by which the bending pins need to move
    #to hit the catheter and bend it by the bending angle to obtain the right
    #shape and thereby convert that distance to the pulse
    bendPinsFactor = input('Enter factor +//- 0.4 ')
    x_i = (d_pins - outer_diameter)/2 - bendPinsFactor                                      # Distance the pin has to move to touch the catheter
    fudge_factor = fudge_func()
    bendDist = x_i + y_i *math.tan(math.radians(angle))*fudge_factor -1.3        # x_i + the distance for the supposed bend
    if math.isnan(bendDist):
        print('Gonna crash here. Angle:'+str(angle))
    return bendDist

def bendDist_to_bendPulse(angle,bendDist,e=e_bending):
    servos_angle = distance_to_angle(bendDist,e)
    if angle>0:
        from_low_b, from_high_b = from_angles.get('positive bend')
    elif angle<0:
        from_low_b, from_high_b = from_angles.get('negative bend')
    else:
        pos = input('Enter 1 for right, and 0 for left')
        if pos==1:
            from_low_b,from_high_b = from_angles.get('positive bend')
        else:
            from_low_b,from_high_b = from_angles.get('negative bend')
    
    pulse = angle_to_pulse(servos_angle,from_low_b,from_high_b)
    if math.isnan(pulse):
        print('Gonna crash here. Angle:'+str(angle) +'bendDist:' +bendDist +
              ' and servos_angle:' +str(servos_angle))
    return pulse

def fudge_func():
    #Call a function that contains the details, such as bend angle, OD, material
    #Somehow obtaine a formulae that would return the factor
    fudge_factor=1
    return fudge_factor

#%% Bending movements
def bendingPin_zero(e=e_bending,channel=ch_bendingPins, timeConstant = time_constant):
##    print('Do we move bending pins back to zeroeth position')
#    pulse_zero = angle_to_pulse(0,from_low_b=-90,from_high_b=90)
    pulse_zero = angle_to_pulse(0,-90,90)                                    # Calculate pulse to be sent by Rpi to move the bending pins to the zeroeth position
    print(pulse_zero)
    pwm.set_pwm(channel,0,pulse_zero)
    sleep(timeConstant)
#    print('Bending pins are back to zeroeth position. Channel:'+str(channel) + ' , Eccentricity:'+str(e))
##    print('Bending pins are back to zeroeth position')

def bending_arm(angle,lens,outer_diameter,e=e_bending,channel=ch_bendingPins,timeConstant = time_constant):
    #command it to move by a particular distance to achieve the bending angle
    #Home position is at the center. Therefore, assume it is at an angle 90 on its servo, since middle position. 
    #Depending upon positive or negative angle, the bending pins moves either to the left(-ve) or to right(+ve)
    #Need to map that distance to the angle.
    print('Start bending?')
    bendDist = bendAngle_to_bendDist(abs(angle),outer_diameter)
    pulse = bendDist_to_bendPulse(angle,bendDist,e)                          # Calculate pulse to be sent from Rpi to the bending arm to achieve the necessary bend
    pwm.set_pwm(channel,0,pulse)
    sleep(timeConstant)
    print('Bending pins are making a bend of ' + str(angle)+'degrees by bending a distance of '+str(bendDist) + 'mm. Pulse: '+str(pulse))
#    input('Press 1 to finish bending and bring it back to zeroeth position.')
    
#    heating_time = cpro.get_heatTime(lens)
#    print('-----------Heat the catheter for '+str(heating_time)+'seconds --------------')
#    htc.startHeat(heating_time)
    
#    for i in range(0,1):                                                  #Uncomment these two lines when the waiting is removed
#        print('Waiting for 3 seconds')
#        sleep(3)
#        print('Waiting for '+str(i)+' seconds...')
    input('Press 1 to make it go to its zeroeth position')    
    bendingPin_zero()
    print('Bending finished')

def push_action(distance):
#    print('Front gripper partially opened')
    front_gripper(partially_opened_distance)                                
    
#    print('Back gripper fully closed')
    back_gripper(fully_closed_distance)
    
    print('Back grippper moving forward by '+str(distance)+'mm')
    back_gripper_indexing(distance)
    
#    print('Front gripper fully closed')
    front_gripper(fully_closed_distance)
    
#    print('Back gripper partially opened')
    back_gripper(partially_opened_distance)
    
#    print('Back gripper moved backwards to original position')
    back_gripper_indexing(fully_bwd_distance)
    
#    print('Back gripper fully closed')
    back_gripper(fully_closed_distance)
    print('Catheter pushed by '+str(distance)+'mm')
      
def home_position():
    
#    print('Front gripper partially opened')
    front_gripper(partially_opened_distance)
    
#    print('Back gripper partially opened')
    back_gripper(partially_opened_distance)
    
#    print('Back gripper moved backwards to home position')
    back_gripper_indexing(fully_bwd_distance)
    
#    print('Bending pins moved to home position')
    bendingPin_zero()
    
#    print('Back gripper on the plane at home angle')
#    back_rotation(0,flag)
#    new_back_rotation(zeroethPosition)


def zero_position():
    front_gripper(0)
    back_gripper(0)
    bendingPin_zero()
    back_gripper_indexing(0)

zero_position()
input('Continue?')   
home_position()
OD = 1.58
lens =3
while True:
##    wait = input('Do you want to continue')
    angle = input('Enter angle')
    
    if angle ==100:
        print('zeroing')
        bendingPin_zero()
    elif angle==200:
        front_gripper(fully_opened_distance)
        back_gripper(fully_opened_distance)
    elif angle==300:
        front_gripper(fully_closed_distance)
        back_gripper(fully_closed_distance)
    elif angle==400:
        front_gripper(partially_opened_distance)
        back_gripper(partially_opened_distance)
    elif angle==500:
        noftimes = input('Number of times')
        for ele in range(0,noftimes+1):
            push_action(5)
    elif angle==600:
        home_position()
    elif angle ==700:
        zero_position()
    elif angle ==800:
        front_gripper(slightlyMore_opened_distance)
        back_gripper(slightlyMore_opened_distance)
    else:
        angle = int(angle)
        bending_arm(angle,lens,OD)
print('Done')
