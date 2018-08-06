# -*- coding: utf-8 -*-
"""
Created on Fri Jul 20 15:24:37 2018

@author: ATI-2 Pavan Gurudath
"""
#%% 
 
time_constant = 0.26

servoDist_threshold       = 6                                                 # Max distance travelled by the back indexing servo(4.75*2)
angle_threshold           = 1                                                   # Min angle required that the catheter needs to be bent by
rotationalAngle_threshold = 1  

#catheter_ID = input('Enter the catheter ID that you'd like to run i.e. Catheter code from Master database')

#%% Do not change these factors unless system is changed. 
e_gripper = 1.59                                                             # eccentricity of gripper cams - 1.59mm
e_bending = 9.25                                                             # eccentricity of bending cam - 9.25mm
e_backidx = 4.75                                                             # eccentricity of back indexing gripper cam - 4.75mm

ch_backGripper = 0
ch_frontGripper = 3
ch_backidxGripper = 1
ch_bendingPins = 5
ch_rotatingArm = 8


servo_min = 190
servo_max = 500


#%% Factors responsible for rotation
rotationalAngle_maxPerRound = 15
zeroethPositionOfRotation = 0                                               # Maximum angle our system can rotate.
#%% 
#bendPinsFactorPos = float(input('Enter bend pins factor positive?')) 
bendPinsFactorPos = 2.05
#bendPinsFactorPos = 0
#bendPinsFactorNeg = 1.95
bendPinsFactorNeg = 2.0
#bendPinsFactorNeg = float(input('Enter bend pins factor negative'))

angleRedFactor = 1                                                      # Angle reduction factor
distanceFactor = 1                                                      # Multiplicative indexing factor for distances

y_i = 1.78
d_pins = 6.35


#positiveAngleOffset = 0
#negativeAngleOffset = 0
OD = 1.62
ODList = [OD,OD,OD,OD]
xDistPins = 2

#smallAngleFudge = input('Enter small fudge factor')
smallAngleFudge = 1.8

#fudgepos = input('Enter fudge positive')
fudgepos = 2.3
fudgeposFour = fudgepos+smallAngleFudge

#fudgeneg = input('Enter fudge negative')
fudgeneg = 2.6
fudgenegFour = fudgeneg+smallAngleFudge


