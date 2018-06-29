# -*- coding: utf-8 -*-
"""
Created on Thur Jun 21 13:08:00 2018

@author: ATI2-Pavan Gurudath
"""
#%% Import statements
from time import sleep
import Adafruit_PCA9685
import numpy as np
import math
from math import pi
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)

#%% Declarations
servo_min = 190                                                              #Min limit of 183 for Hitech-servos
servo_max = 500                                                              #Max limit of 600 for Hitech-servos
time_constant = 1                                                            #Time for the Rpi to wait for the servo to complete its task
#from_low = 0                                                                 #Smallest angle that you'd want the cam to be at
#from_high = 180                                                              #Largest angle that you'd want the cam to be at
e_gripper = 1.59                                                             #eccentricity of gripper cams - 1.59mm
e_bending = 9.25                                                             #eccentricity of bending cam - 9.25mm
e_backidx = 4.75                                                             #eccentricity of back indexing gripper cam - 4.75mm
d_pins = 5.25                                                                #Distance between the bending pins (edge-to-edge) **0.207inch**
y_i =   3                                                                    #Distance between the front gripper and the bending pins

#%% Declare all channels
ch_backGripper = 0
ch_frontGripper = 3
ch_backidxGripper = 1
ch_bendingPins = 5
ch_rotatingArm = 8


#%%
from_angles = {
        'positive bend': [-90,90],                                                            
        'negative bend': [90,-90],                                                            
        }

#%% Gripper servo angles and movements 
    # Mapping the angle on the servo to the pulse range
def angle_to_pulse(angle,from_low=0,from_high=180):
    pulse = (angle-from_low)*(servo_max-servo_min)/(from_high-from_low) + servo_min
    return int(pulse)

# Converts linear distance using the cam to the servo angle b/w 0-180 (cam_formulae)
def distance_to_angle(distance,e):                                           #1.59mm
    angle = np.arccos((e-distance)/e)*180/pi                                 #theta in degrees
    return angle

# Converts the servo angle to the linear distance (cam_formulae)
def angle_to_distance(angle,e):
    distance = e-e*np.cos(angle*pi/180)
    return distance

# Returns the pulse that is required to achieve the linear distance
def distance_to_pulse(distance,e,from_low = 0, from_high = 180):              #eccentricity of cam - 1.59mm
    angle = distance_to_angle(distance,e)
    return angle_to_pulse(angle,from_low,from_high)                          #pulse = angle_to_pulse(theta,from_low,from_high)


#%% Bending angles and movements 
# Returns the distance that the bending pins need to move for the bend to happen
def bendAngle_to_bendDist(angle,outer_diameter):
    #This function defines the distance by which the bending pins need to move
    #to hit the catheter and bend it by the bending angle to obtain the right
    #shape and thereby convert that distance to the pulse
    x_i = (d_pins - outer_diameter)/2                                        #Distance the pin has to move to touch the catheter
    fudge_factor = fudge_func()
    bendDist = x_i + y_i *math.tan(math.radians(angle))*fudge_factor         #x_i + the distance for the supposed bend
    if math.isnan(bendDist):
        print('Gonna crash here. Angle:'+str(angle))
    return bendDist

def bendDist_to_bendPulse(angle,bendDist,e=e_bending):
    servos_angle = distance_to_angle(bendDist,e)
    if angle>0:
#        from_low_b  = -90
#        from_high_b = 90
        from_low_b, from_high_b = from_angles.get('positive bend')
    else:
#        from_low_b    = 90
#        from_high_b   = -90
        from_low_b, from_high_b = from_angles.get('negative bend')
        
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

#%% Gripper movements    
def back_gripper(f_distance,e=e_gripper,flag=1,channel=ch_backGripper,timeConstant = time_constant):
    #Let flag just be there for now. 
    #No use of it right now since its just max or min position                                                   
#    print('Back gripper moving by '+str(f_distance))
    pulse = distance_to_pulse(f_distance,e_gripper)                          #Calculate pulse to be sent to Rpi
    pwm.set_pwm(channel,0,pulse)
    sleep(timeConstant)
#    print('Back gripper movement done. Channel:'+str(channel)+', Eccentricity:'+str(e)+', Pulse: '+str(pulse))


def front_gripper(f_distance,e=e_gripper,flag=1,channel=ch_frontGripper,timeConstant = time_constant):
    #Let flag be there for now, even though its just max or min position.    
#    print('front gripper')
#    print('Front gripper moving by '+str(f_distance))
    pulse = distance_to_pulse(f_distance,e)
    pwm.set_pwm(channel,0,pulse)
    sleep(timeConstant)
#    print('Front gripper movement done. Channel:'+str(channel)+' , Eccentricity:'+str(e)+', Pulse: '+str(pulse))


def back_gripper_indexing(distance,e=e_backidx,flag=1,channel=ch_backidxGripper,timeConstant = time_constant):
    #command it to move either by servoDist_threshold or a particular distance. 
#    print('Back gripper moving forward by '+str(distance)+'mm ')
    pulse = distance_to_pulse(distance,e)
    pwm.set_pwm(channel,0,pulse)
    sleep(timeConstant)
#    print('Back gripper y-direction movement done. Channel:'+str(channel)+' , Eccentricity:'+str(e)+', Pulse: '+str(pulse))
    
#%% Bending movements
def bendingPin_zero(e=e_bending,channel=ch_bendingPins, timeConstant = time_constant):
    print('Do we move bending pins back to zeroeth position')
#    pulse_zero = angle_to_pulse(0,from_low_b=-90,from_high_b=90)
    pulse_zero = angle_to_pulse(0,-90,90)
    pwm.set_pwm(channel,0,pulse_zero)
    sleep(timeConstant)
#    print('Bending pins are back to zeroeth position. Channel:'+str(channel) + ' , Eccentricity:'+str(e))
    print('Bending pins are back to zeroeth position')

def bending_arm(angle,outer_diameter,flag=1,e=e_bending,channel=ch_bendingPins,timeConstant = time_constant):
    #command it to move by a particular distance to achieve the bending angle
    #Home position is at the center. Therefore, assume it is at an angle 90 on its servo, since middle position. 
    #Depending upon positive or negative angle, the bending pins moves either to the left(-ve) or to right(+ve)
    #Need to map that distance to the angle.
    print('Start bending?')
    bendDist = bendAngle_to_bendDist(abs(angle),outer_diameter)
    pulse = bendDist_to_bendPulse(angle,bendDist,e)
    pwm.set_pwm(channel,0,pulse)
    sleep(timeConstant)
    print('Bending pins are making a bend of ' + str(angle)+'degrees by bending a distance of '+str(bendDist) + 'mm. Pulse: '+str(pulse))
#    input('Press 1 to finish bending and bring it back to zeroeth position.')
    
    for i in range(0,1):                                                  #Uncomment these two lines when the waiting is removed
        print('Waiting for 3 seconds')
        sleep(3)
#        print('Waiting for '+str(i)+' seconds...')
        
    bendingPin_zero()
    print('Bending finished')

def back_rotation(angle,flag=0,channel=ch_rotatingArm,timeConstant = time_constant):
    #command it to rotate by a particular angle
    
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
        
#%% Rotating movements    
    
''' **************COMMENT THIS ENTIRE CELL BLOCK IF YOU WANT TO RUN WITHOUT 
        THE NEW ROTATION (JULY 4 WEEK) *****************************'''
        
total_rotation = 0
temp_rotation = 0
rotAngle_threshold = 15
def rotationalAngle_to_servoAngle(angle):
    #this function defines the mapping of rotational angle to servo angle 
    #since there is a restriction of only 15degrees
    return angle

def back_rotation(angle,flag=0,channel=ch_rotatingArm,timeConstant = time_constant):
    #command it to rotate by a particular angle
    #Possible use the flag to control rotational_angle= 0. 
    if angle>0:
        zeroAngle_flag=1
        if angle>rotAngle_threshold:
            ### Rotate by breaking that angle in terms of 15
            print('Breaking up '+str(angle)+'degrees in intervals of 15')
            fifteenAngle_flag = 1
            angles_breakup = split_angles(angle)
            
        else:
            fifteenAngle_flag = 0
            angles_breakup = [angle]
#            print('Rotating the plane to '+str(angle)+'degrees')
#            servo_angle = rotationalAngle_to_servoAngle(angle)
#            pulse = angle_to_pulse(servo_angle)
#            pwm.set_pwm(channel,0,pulse)
#            sleep(timeConstant)
    else:
        zeroAngle_flag=0
        if abs(angle) > rotAngle_threshold:
            print('Breaking up '+str(angle)+'degrees in intervals of -15')
            fifteenAngle_flag = 1
            angles_breakup = split_angles(abs(angle))
        else:
            fifteenAngle_flag = 0
            angles_breakup = [abs(angle)]
    flaggs = (zeroAngle_flag,fifteenAngle_flag)
    rotate_this_catheter(flaggs,angles_breakup)
    
def rotate_this_catheter(flaggs,angles_breakup,channel = ch_rotatingArm,timeConstant = time_constant):
    zeroAngle_flag,fifteenAngle_flag = flaggs
    home_pos_for_rotation(zeroAngle_flag)
    if zeroAngle_flag and not fifteenAngle_flag:
        servoAngle = rotationalAngle_to_servoAngle(angles_breakup)
        pulse = angle_to_pulse(servoAngle)
        pwm.set_pwm(channel,0,pulse)
        sleep(timeConstant)
    elif not zeroAngle_flag and not fifteenAngle_flag:
        
        
        
def home_pos_for_rotation(zeroAngle_flag):
    if not zeroAngle_flag:
        front_gripper(partially_opened_distance)
        back_gripper(fully_closed_distance)
    else:
        front_gripper(fully_closed_distance)
        back_gripper(partially_opened_distance)
        rotationOfCatheter(15)
        back_gripper(fully_closed_distance)
        front_gripper(partially_opened_distance)

def rotationOfCatheter(angle,channel = ch_rotatingArm):
    servoAngle = rotationalAngle_to_servoAngle(angle)
    pulse = angle_to_pulse(servoAngle)
    pwm.set_pwm(channel,0,pulse)
    
            

def split_angles(angle,rotAngle_threshold=15):
    quotient = int(angle//rotAngle_threshold)
    remainder = angle - rotAngle_threshold*quotient
    rotAngles = []
    for ang in range(quotient):
        rotAngles.append(rotAngle_threshold)
    rotAngles.append(remainder)
    print(rotAngles)
    return rotAngles
    
##    if flag==1:
#    print('Rotating the plane...')
#    pulse = angle_to_pulse(angle)
#    pwm.set_pwm(channel,0,pulse)
#    sleep(timeConstant)
#    print('Rotated the plane to '+str(angle)+'degrees. Pulse given' + str(pulse))
##    else:
##        print('We are now moving the rotating plane back to zero')
##        pulse = angle_to_pulse(0)
##        pwm.set_pwm(channel,0,pulse)
##        sleep(timeConstant)
##        print('Rotated the plane back to 0 degrees. Pulse given' + str(pulse))
        
#%%        
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
    flag=0
    
#    print('Front gripper partially opened')
    front_gripper(partially_opened_distance)
    
#    print('Back gripper partially opened')
    back_gripper(partially_opened_distance)
    
#    print('Back gripper moved backwards to home position')
    back_gripper_indexing(fully_bwd_distance)
    
#    print('Bending pins moved to home position')
    bendingPin_zero()
    
#    print('Back gripper on the plane at home angle')
    back_rotation(0,flag)
    

#%%
def get_fullyClosedDistance(OD,e=e_gripper):
   fcd_angle = {
           3:angle_to_distance(0,e),                                        #******************Needs to be experimentally determined**************
           4:angle_to_distance(30,e),
           5:angle_to_distance(60,e),
           6:angle_to_distance(90,e),
           7:angle_to_distance(120,e),
           8:angle_to_distance(180,e)
           }
   return fcd_angle.get(OD)

def get_partiallyOpenedDistance(OD,e=e_gripper):
    pod_angle = {
           3:angle_to_distance(0,e),                                        #******************Needs to be experimentally determined**************
           4:angle_to_distance(30,e),
           5:angle_to_distance(60,e),
           6:angle_to_distance(90,e),
           7:angle_to_distance(120,e),
           8:angle_to_distance(180,e)
           }
    return pod_angle.get(OD)

#%% Needs to be changed if you want flexibility in fully closed and partially opened distance for different OD sizes. Currently developed for 5Frenchsize
    
#fully_closed_distance      = get_fullyClosedDistance(OD,e)                  #Position of front and back servos along x-direction for different OD       
#partially_opened_distance  = get_partiallyOpenedDistance(OD,e)              #Position of front and back servos along x-direction for different OD

#fully_closed_distance       = angle_to_distance(90,e_gripper)               #Position of front and back servos along x-direction (Default for all sizes)
#partially_opened_distance   = angle_to_distance(70.52,e_gripper)            #Position of front and back servos along x-direction (Default for all sizes)
fully_closed_distance       = 1.58                                           # Distance to close the gripper - 1.58 mm
partially_opened_distance   = 1.06                                           # Distance to just reach the gripper - 1.06mm

#*DEFAULT ALL THE TIME*           
fully_opened_distance       = angle_to_distance(0,e_gripper)                 #Position of front and back servos along x-direction (Default for all sizes )
fully_bwd_distance          = angle_to_distance(0,e_backidx)


#%% main function
if __name__ == "__main__":
    pwm = Adafruit_PCA9685.PCA9685()
    pwm.set_pwm_freq(60)
    

    
    print('Bringing all cams to zeroeth position')
    
    pwm.set_pwm(0,0,190)
    pwm.set_pwm(1,0,590)
    pwm.set_pwm(3,0,190)
    sleep(5)
    
    
    print('Going to home position')
    home_position()
    print('Done with home position')
    print('Waiting for push to start in ...')
    for i in range(3,0,-1):
        sleep(1)
        print(i)
        
    push_action(0.1)
    print('Done with one gripper action and moving back to home position in ...')
    for i in range(3,0,-1):
        sleep(1)
        print(i)
    
    home_position()
    print('Disconnect all supplies.')
