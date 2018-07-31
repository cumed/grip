# -*- coding: utf-8 -*-
"""
Created on Mon Jul 23 10:40:30 2018
new bend
@author: ATI-2 Pavan Gurudath
"""

from time import sleep
import Adafruit_PCA9685
import numpy as np
import math
from math import pi
import factors as fact
#import heating_control as htc
#import catheter_properties as cpro

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)

servo_min = 190                                                              # Min limit of 183 for Hitech-servos
servo_max = 500                                                              # Max limit of 600 for Hitech-servos
time_constant = fact.time_constant                                           # Time for the Rpi to wait for the servo to complete its task
# from_low = 0                                                               # Smallest angle that you'd want the cam to be at
#from_high = 180                                                             # Largest angle that you'd want the cam to be at
e_gripper = 1.59                                                             # eccentricity of gripper cams - 1.59mm
e_bending = 9.25                                                             # eccentricity of bending cam - 9.25mm
e_backidx = 4.75                                                             # eccentricity of back indexing gripper cam - 4.75mm
d_pins = fact.d_pins                                                         # Distance between the bending pins (edge-to-edge) **0.207inch**
y_i =   fact.y_i                                                             # Distance between the front gripper and the bending pins

#%% Declare all channels
ch_backGripper = 0
ch_frontGripper = 3
ch_backidxGripper = 1
ch_bendingPins = 5
ch_rotatingArm = 8

#%%
fully_closed_distance       = 3.18                                           # Distance to close the gripper - 1.58 mm
##partially_opened_distance   = input('Enter partially_opened_distance')     # Distance to just reach the gripper - 1.06mm
partially_opened_distance = 2.7
#*DEFAULT ALL THE TIME*           
fully_opened_distance,fully_bwd_distance = 0,0                               # Position of front and back servos along x-direction (Default for all sizes )
slightlyMore_opened_distance = 2.2

#%%
from_angles = {
        'positive bend': [-90,90],                                           # If the bending is taking place for a positive angle, then the bending pins need to move to the right                 
        'negative bend': [90,-90],                                           # If the bending is taking place for a negative angle, then the bending pins need to move to the left
        }

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
#bendPinsFactor = fact.bendPinsFactor
#print('bendpins factor: '+str(bendPinsFactor))
def bendAngle_to_bendDist(angle,outer_diameter):
    #This function defines the distance by which the bending pins need to move
    #to hit the catheter and bend it by the bending angle to obtain the right
    #shape and thereby convert that distance to the pulse   
    if angle>0:
        x_i = (d_pins - outer_diameter)/2 - fact.bendPinsFactorPos                                        # Distance the pin has to move to touch the catheter
    elif angle<0:
        x_i = (d_pins - outer_diameter)/2 - fact.bendPinsFactorNeg
    else:
        pos = input('Enter 1 for positive bend, and 0 for negative bend')
        if pos==1:
            x_i = (d_pins - outer_diameter)/2 - fact.bendPinsFactorPos
        else:
            x_i = (d_pins - outer_diameter)/2 - fact.bendPinsFactorNeg

    bendDist = x_i + y_i *math.tan(math.radians(abs(angle)))*fudge_func(angle)         # x_i + the distance for the supposed bend
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


small_angle_fudge = fact.smallAngleFudge
def fudge_func(angle):
    #Call a function that contains the details, such as bend angle, OD, material
    #Somehow obtaine a formulae that would return the factor
    if angle>=0:
        if angle<=small_angle_fudge:
            fudge_factor = fact.fudgeposFour 
            print(fudge_factor)
            print('fudgeposFour')
        else:
            fudge_factor = fact.fudgepos                                                  
            print(fudge_factor)
            print('fudge pos')
    else:
        if angle>=-small_angle_fudge:
            fudge_factor = fact.fudgenegFour
            print(fudge_factor)
            print('fudgenegFour')
        else:
            fudge_factor = fact.fudgeneg                                             
            print(fudge_factor)
            print('fudge neg')
    return fudge_factor

def factor_of_half_bendDist(distance):
    factor = fact.xDistPins 
    return distance/factor

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

angleRedFactor = fact.angleRedFactor
def bending_arm(angle,lens,outer_diameter,e=e_bending,channel=ch_bendingPins,timeConstant = time_constant):
    #command it to move by a particular distance to achieve the bending angle
    #Home position is at the center. Therefore, assume it is at an angle 90 on its servo, since middle position. 
    #Depending upon positive or negative angle, the bending pins moves either to the left(-ve) or to right(+ve)
    #Need to map that distance to the angle.
#    print('Start bending?')

    angle = angle*angleRedFactor
    bendDist = bendAngle_to_bendDist(angle,outer_diameter)
    pulse = bendDist_to_bendPulse(angle,bendDist,e)                          # Calculate pulse to be sent from Rpi to the bending arm to achieve the necessary bend
    print(pulse)
#    pulse = input('Enter pulse')
    pwm.set_pwm(channel,0,pulse)
    sleep(timeConstant)
    print('Bend of ' + str(round(angle,2))+'degrees -- Bending distance '+str(round(bendDist,2)) + 'mm. -- Pulse: '+str(pulse))
    input('Press 1 to finish bending and bring it back to zeroeth position.')
    
#    heating_time = cpro.get_heatTime(lens)
#    print('-----------Heat the catheter for '+str(heating_time)+'seconds --------------')
#    htc.startHeat(heating_time)
    
#    for i in range(0,1):                                                  #Uncomment these two lines when the waiting is removed
#        print('Waiting for 3 seconds')
#        sleep(3)
#        print('Waiting for '+str(i)+' seconds...')
        
    bendingPin_zero()
#    print('Bending finished')
def back_rotation(angle,channel=ch_rotatingArm,timeConstant = time_constant):
    #command it to rotate by a particular angle
    #Let flag be there for now, even though its just max or min position.    
    #No use of it right now since its just max or min position ***UPDATE: Flag has been removed***
#    if flag==1:
    print('Rotating the plane...')
    pulse = angle_to_pulse(angle)
    pwm.set_pwm(channel,0,pulse)
    sleep(timeConstant)
    print('Rotated the plane to '+str(angle)+'degrees. Pulse given' + str(pulse))
#    else:
#        print('We are now moving the rotating plane back to zero')
#        pulse = angle_to_pulse(0)
#        pwm.set_pwm(channel,0,pulse)
#        sleep(timeConstant)
#        print('Rotated the plane back to 0 degrees. Pulse given' + str(pulse))


#%%    
def push_action(distance):
#    print('Front gripper partially opened')
    front_gripper(partially_opened_distance)                                
    
#    print('Back gripper fully closed')
    back_gripper(fully_closed_distance)
    
    print('Back grippper moving forward by '+str(distance*fact.distanceFactor)+'mm')
    back_gripper_indexing(distance*fact.distanceFactor)
    
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


def reversePush_action(distance):
#    print('Front gripper partially opened')
    front_gripper(fully_closed_distance)
    back_gripper(fully_opened_distance)
    back_gripper_indexing(distance*fact.distanceFactor)
    back_gripper(fully_closed_distance)
    front_gripper(partially_opened_distance)
    back_gripper_indexing(fully_bwd_distance)
    front_gripper(fully_closed_distance)


pwm.set_pwm(ch_rotatingArm,0,angle_to_pulse(170))
sleep(time_constant*2)    
bendingPin_zero()
OD = fact.OD 
lens =3
front_gripper(partially_opened_distance)
back_gripper(partially_opened_distance)
back_gripper_indexing(0)
while True:
##    wait = input('Do you want to continue')
    angle = input('Enter angle')
#    bendPinsFactor = input('Enter bend pins factor')
    if angle ==100:
        print('zeroing')
        bendingPin_zero()
    elif angle==200:
        print('Grippers- Fully Opened distance')
        front_gripper(fully_opened_distance)
        back_gripper(fully_opened_distance)
    elif angle==300:
        print('Grippers- Fully Closed distance')
        front_gripper(fully_closed_distance)
        back_gripper(fully_closed_distance)
    elif angle==400:
        print('Grippers- Partially Opened distance')
        front_gripper(partially_opened_distance)
        back_gripper(partially_opened_distance)
    elif angle==500:
        noftimes = input('Number of times - 1mm')
        distance = input('Each increment of distance?')
        for ele in range(0,noftimes):
            push_action(distance)
    elif angle==600:
        pulse = input('Enter pulse')
        pwm.set_pwm(ch_bendingPins,0,pulse)
        sleep(0.3)
        bendingPin_zero()
    elif angle==700:
        print('Zero position')
        zero_position()
    elif angle==800:
        print('Grippers - slightly more opened position')
        front_gripper(slightlyMore_opened_distance)
        back_gripper(slightlyMore_opened_distance)
    elif angle==900:
        noftimes = input('Number of times')
        distance = input('Each increment of distance?')
        for ele in range(0,noftimes):
            reversePush_action(distance)
    else:
        angle = int(angle)
        if angle>0:
            angle = angle + fact.positiveAngleOffset
        elif angle<0:
            angle = angle + fact.negativeAngleOffset
        bending_arm(angle,lens,OD)
##    rotangle = input('Enter rot angle')
##    back_rotation(rotangle)
print('Done')
