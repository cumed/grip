# -*- coding: utf-8 -*-
"""
Created on Fri Aug 3 17:00:00 2018

@author: ATI-2 Pavan Gurudath
"""

#%% Import statements
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
servo_min = fact.servo_min                                                              # Min limit of 183 for Hitech-servos
servo_max = fact.servo_max                                                              # Max limit of 600 for Hitech-servos

e_gripper = fact.e_gripper                                                             # eccentricity of gripper cams - 1.59mm
e_bending = fact.e_bending                                                             # eccentricity of bending cam - 9.25mm
e_backidx = fact.e_backidx                                                             # eccentricity of back indexing gripper cam - 4.75mm

time_constant = fact.time_constant                                           # Time for the Rpi to wait for the servo to complete its task
d_pins = fact.d_pins                                                                # Distance between the bending pins (edge-to-edge) **0.207inch**
y_i =   fact.y_i                                                                    # Distance between the front gripper and the bending pins

#%% Declare all channels
ch_backGripper = fact.ch_backGripper
ch_frontGripper = fact.ch_frontGripper
ch_backidxGripper = fact.ch_backidxGripper
ch_bendingPins_left = fact.ch_bendingPins_left
ch_bendingPins_right = fact.ch_bendingPins_right
ch_rotatingArm = fact.ch_rotatingArm


#%%
from_angles = {
        'positive bend': [-180,180],                                           # If the bending is taking place for a positive angle, then the bending pins need to move to the right                 
        'negative bend': [180,-180],                                           # If the bending is taking place for a negative angle, then the bending pins need to move to the left
        }

zeroethPosition = fact.zeroethPositionOfRotation                                                          # The zeroeth position of the rotational servo
rotationalAngle_maxPerRound = fact.rotationalAngle_maxPerRound               # Maximum angle our system can rotate.
#%% Gripper servo angles and movements 
# Mapping the angle on the servo to the pulse range
def angle_to_pulse(angle,from_low=0,from_high=180):
    pulse = (angle-from_low)*(servo_max-servo_min)/(from_high-from_low) + servo_min
    return int(pulse)

# Converts linear distance using the cam to the servo angle b/w 0-180 (cam_formulae)
def distance_to_angle(distance,e):                                           
    angle = np.arccos((e-distance)/e)*180/pi                                 # angle in degrees
    return angle

# Converts the servo angle to the linear distance (cam_formulae)
def angle_to_distance(angle,e):
    distance = e-e*np.cos(angle*pi/180)
    return distance

# Returns the pulse that is required to achieve the linear distance
def distance_to_pulse(distance,e,from_low = 0, from_high = 180):             # eccentricity of cam - 1.59mm
    angle = distance_to_angle(distance,e)
    return angle_to_pulse(angle,from_low,from_high)                          # pulse = angle_to_pulse(theta,from_low,from_high)


#%% Bending angles and movements 
# Returns the distance that the bending pins need to move for the bend to happen
def bendAngle_to_bendDist(angle,outer_diameter):
    #This function defines the distance by which the bending pins need to move
    #to hit the catheter and bend it by the bending angle to obtain the right
    #shape and thereby convert that distance to the pulse   
    if angle>=0:
        x_i = (d_pins - outer_diameter)/2 - fact.bendPinsFactorPos                                        # Distance the pin has to move to just touch the catheter
    elif angle<0:
        x_i = (d_pins - outer_diameter)/2 - fact.bendPinsFactorNeg

    fudge_factor = fudge_func(angle)
    bendDist = x_i + y_i *math.tan(math.radians(abs(angle)))*fudge_factor         # x_i + the distance for the supposed bend
    if math.isnan(bendDist):
        print('Gonna crash here. Angle:'+str(angle))
    return bendDist

# Function that determines as to which side the bending pins will have to move for its pseudo zeroeth position without going back to zero. 
def comparison(angle,outer_diameter):
    if angle>0:
        return (d_pins - outer_diameter)/2 - fact.bendPinsFactorPos
    else:
        return (d_pins - outer_diameter)/2 - fact.bendPinsFactorNeg

# Function that converts bending distance to the necessary pulse. 
def bendDist_to_bendPulse(angle,bendDist,e=e_bending):
    servos_angle = distance_to_angle(bendDist,e)
    if angle>=0:
        from_low_b, from_high_b = from_angles.get('positive bend')
    else:
        from_low_b, from_high_b = from_angles.get('negative bend')
        
    pulse = angle_to_pulse(servos_angle,from_low_b,from_high_b)
    if math.isnan(pulse):
        print('Gonna crash here. Angle:'+str(angle) +'bendDist:' +bendDist +
              ' and servos_angle:' +str(servos_angle))
    return pulse

small_angle_fudge = fact.smallAngleFudge
def fudge_func(angle):
    #Call a function that contains the details, such as bend angle, OD, material
    #Somehow obtaine a formulae that would return the factor
    #Currently, the fudge factor is decided based on trial and error, wherein the 
    #angles are split as follows (-inf,-small_angle_fudge), [-small_angle_fudge,0],
    #[0,small_angle_fudge], (small_angle_fudge,inf)
    
    if angle>=0:
        if angle<=small_angle_fudge:
            fudge_factor = fact.fudgeposFour 
        else:
            fudge_factor = fact.fudgepos                                                  

    else:
        if angle>=-small_angle_fudge:
            fudge_factor = fact.fudgenegFour
        else:
            fudge_factor = fact.fudgeneg                                             

    return fudge_factor

def factor_of_half_bendDist(distance):
    factor = fact.xDistPins 
    return distance/factor
#%% Gripper movements    
def back_gripper(f_distance,e=e_gripper,channel=ch_backGripper,timeConstant = time_constant):
    pulse = distance_to_pulse(f_distance,e)                                  # Calculate pulse to be sent by Rpi for back gripper's movement
    pwm.set_pwm(channel,0,pulse)
    sleep(timeConstant)

def front_gripper(f_distance,e=e_gripper,channel=ch_frontGripper,timeConstant = time_constant):
    pulse = distance_to_pulse(f_distance,e)                                  # Calculate pulse to be sent by Rpi for front gripper's movement
    pwm.set_pwm(channel,0,pulse)
    sleep(timeConstant)

def back_gripper_indexing(distance,e=e_backidx,channel=ch_backidxGripper,timeConstant = time_constant):
    pulse = distance_to_pulse(distance,e)                                    # Calculate pulse to be sent by Rpi for back indexing gripper's movement
    pwm.set_pwm(channel,0,pulse)
    sleep(timeConstant)

    
#%% Bending movements
def bendingPin_zero(e=e_bending,channel_left=ch_bendingPins_left,channel_right=ch_bendingPins_right, timeConstant = time_constant):
    pulse_zero = angle_to_pulse(0,-180,180)                                    # Calculate pulse to be sent by Rpi to move the bending pins to the zeroeth position
    pwm.set_pwm(channel_left,0,pulse_zero)
    pwm.set_pwm(channel_right,0,pulse_zero)
    sleep(timeConstant)

angleRedFactor = fact.angleRedFactor
def bending_arm(angle,lens,outer_diameter,e=e_bending,channel_left=ch_bendingPins_left,channel_right=ch_bendingPins_right,timeConstant = time_constant):
    #command it to move by a particular distance to achieve the bending angle
    #Home position is at the center. Therefore, assume it is at an angle 90 on its servo, since middle position. 
    #Depending upon positive or negative angle, the bending pins moves either to the left(-ve) or to right(+ve)
    #Need to map that distance to the angle.
    
    #Send it to zero
#    bendingPin_zero()

    
    #Let the bend happen
    angle = angle*angleRedFactor
    if angle > 0:
        
        bendDist_left = bendAngle_to_bendDist(angle,outer_diameter)
        pulse_left = bendDist_to_bendPulse(angle,bendDist_left,e)                  # Calculate pulse to be sent from Rpi to the bending arm to achieve the necessary bend
        pwm.set_pwm(channel_left,0,pulse_left)
        pwm.set_pwm(channel_right,0,pulse_left)
        sleep(2)
        print('Bend of ' + str(round(angle,2))+'degrees -- Bending distance '+str(round(bendDist_left,2)) + 'mm. -- Pulse: '+str(pulse_left))
        half_bendDist = factor_of_half_bendDist(bendDist_left)
        half_bendDist_left = min(half_bendDist,comparison(angle,outer_diameter))
        halfPulse = bendDist_to_bendPulse(angle,half_bendDist_left,e)
        pwm.set_pwm(channel_left,0,halfPulse)
#        pwm.set_pwm(channel_right,0,halfPulse)
        sleep(timeConstant)
#    input('Press 1 to finish bending and bring it back to zeroeth position.')
    
    #Heat the catheter
    elif angle <0:
        bendDist_right = bendAngle_to_bendDist(angle,outer_diameter)
        pulse_right = bendDist_to_bendPulse(angle,bendDist_right,e)                  # Calculate pulse to be sent from Rpi to the bending arm to achieve the necessary bend
        pwm.set_pwm(channel_right,0,pulse_right)
        sleep(timeConstant)
        print('Bend of ' + str(round(angle,2))+'degrees -- Bending distance '+str(round(bendDist_right,2)) + 'mm. -- Pulse: '+str(pulse_right))
#        half_bendDist = factor_of_half_bendDist(bendDist_right)
#        half_bendDist_right = min(half_bendDist,comparison(angle,outer_diameter))
#        halfPulse = bendDist_to_bendPulse(angle,half_bendDist_right,e)
#        pwm.set_pwm(channel_right,0,halfPulse)
#        sleep(timeConstant)
    heating_time = cpro.get_heatTime(lens)
    htc.startHeat(heating_time)
    
    #Send it back to half the distance
    
    

    #Send it to zero
#    bendingPin_zero()
def bending_arm_back(angle, lens, outer_diameter,e=e_bending,channel_left=ch_bendingPins_left,channel_right=ch_bendingPins_right,timeConstant = time_constant):   
    angle = angle*angleRedFactor
    if angle > 0:
        bendDist_left = bendAngle_to_bendDist(angle,outer_diameter)
        half_bendDist = factor_of_half_bendDist(bendDist_left)
        half_bendDist_left = min(half_bendDist,comparison(angle,outer_diameter))
        halfPulse = bendDist_to_bendPulse(angle,half_bendDist_left,e)
        #pwm.set_pwm(channel_left,0,halfPulse)
        pwm.set_pwm(channel_right,0,halfPulse)
        sleep(timeConstant)
#    input('Press 1 to finish bending and bring it back to zeroeth position.')
    
    #Heat the catheter
    elif angle <0:
        bendDist_right = bendAngle_to_bendDist(angle,outer_diameter)
        half_bendDist = factor_of_half_bendDist(bendDist_right)
        half_bendDist_right = min(half_bendDist,comparison(angle,outer_diameter))
        halfPulse = bendDist_to_bendPulse(angle,half_bendDist_right,e)
        pwm.set_pwm(channel_right,0,halfPulse)
        sleep(timeConstant)
   
#%% Rotating movements    

# Split the rotational angles based on the maximum angle that the system can move about
def split_angles(angle,rotAngle_threshold=rotationalAngle_maxPerRound):
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

# main rotation function
def new_back_rotation(angle,flag=0,channel=ch_rotatingArm,timeConstant = time_constant):
    if angle>0:
        angle_list = split_angles(angle)
        rotateTheCatheterByPositiveAngle(angle_list)
    elif angle<0:
        angle_list = split_angles(abs(angle))
        rotateTheCatheterByNegativeAngle(angle_list)
    else:
        rotateThisCatheter(angle)
 

# Gripper movements for rotation about positive angle
def rotateTheCatheterByPositiveAngle(angle_list):
    for angles in angle_list:
        back_gripper(fully_closed_distance)
        front_gripper(partially_opened_distance)
        rotateThisCatheter(angles)
        
        front_gripper(fully_closed_distance)
        back_gripper(partially_opened_distance)
        rotateThisCatheter(zeroethPosition)
        
        back_gripper(fully_closed_distance)

# Gripper movements for rotation about negative angle    
def rotateTheCatheterByNegativeAngle(angle_list):
    for angles in angle_list:
        front_gripper(fully_closed_distance)
        back_gripper(partially_opened_distance)
        rotateThisCatheter(angles)
        
        back_gripper(fully_closed_distance)
        front_gripper(partially_opened_distance)
        rotateThisCatheter(zeroethPosition)
        
        front_gripper(fully_closed_distance)
    
# Function that sends the pulse for the rotation system
def rotateThisCatheter(angle,channel = ch_rotatingArm,timeConstant = time_constant):
    servoAngle = rotationalAngle_to_servoAngle(angle)
    pulse = angle_to_pulse(servoAngle)
    pwm.set_pwm(channel,0,pulse)
    sleep(timeConstant)
                     
#%%   
def push_action(distance):
    front_gripper(partially_opened_distance)                                
    back_gripper(fully_closed_distance)
    back_gripper_indexing(distance)
    front_gripper(fully_closed_distance)
    back_gripper(partially_opened_distance)
    back_gripper_indexing(fully_bwd_distance)
    back_gripper(fully_closed_distance)

    print('Catheter pushed by '+str(round(distance,2))+'mm')
      
def home_position():
    front_gripper(partially_opened_distance)
    back_gripper(partially_opened_distance)
    back_gripper_indexing(fully_bwd_distance)
    bendingPin_zero()
    new_back_rotation(zeroethPosition)

def zero_position():
    front_gripper(0)
    back_gripper(0)
    bendingPin_zero()
    back_gripper_indexing(0)
#%% Functions for fully closed and partially opened distance if the grippers
# have to be closed by different distances. 
# Uncomment the call for fully_closed_distance and partially_opened_distance 
# in the next section and comment the subsequent lines of that section

def get_fullyClosedDistance(fr_size):
   fcd_angle = {
           3:4,
           4:4,
           5:3.18,
           6:4,
           7:4,
           8:4
           }
   return fcd_angle.get(fr_size)
 
def get_partiallyOpenedDistance(fr_size):
    pod_angle = {
           3:3.5,                                        #******************Needs to be experimentally determined**************
           4:3.5,
           5:2.8,
           6:3.5,
           7:3.5,
           8:3.5
           }
    return pod_angle.get(fr_size)

#%% Needs to be changed if you want flexibility in fully closed and partially opened distance for different OD sizes. Currently developed for 5Frenchsize

#fr_size = cpro.getFr_size()                                                  # Write a function to get the french size of the catheter
#fully_closed_distance      = get_fullyClosedDistance(fr_size)                # Position of front and back servos along x-direction for different OD       
#partially_opened_distance  = get_partiallyOpenedDistance(fr_size)            # Position of front and back servos along x-direction for different OD

fully_closed_distance       = 3.18                                           # Distance to close the gripper - 1.58 mm
partially_opened_distance   = 2.8                                            # Distance to just reach the gripper - 1.06mm
slightlyMore_opened_distance = 2.2

#%% fully_opened_distance
fully_opened_distance       = 0                                              # Distance to open the grippers completely 
fully_bwd_distance          = 0                                              # Distance to keep the indexing camera behind 


