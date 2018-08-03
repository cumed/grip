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


servo_min = 190
servo_max = 500
#%% 
#bendPinsFactorPos = float(input('Enter bend pins factor positive?')) 
bendPinsFactorPos = 2.05
#bendPinsFactorPos = 0
#bendPinsFactorNeg = 1.95
bendPinsFactorNeg = 2.0
#bendPinsFactorNeg = float(input('Enter bend pins factor negative'))

angleRedFactor = 1
distanceFactor = 1
y_i = 1.78
d_pins = 6.35
OD = 1.62
positiveAngleOffset = 0
negativeAngleOffset = 0
ODList = [OD,OD,OD,OD]
xDistPins = 2

#a = input('Enter small fudge factor')
a=1.8
#fudgepos = input('Enter fudge positive')
fudgepos = 2.3
fudgeposFour = fudgepos+a

#fudgeneg = input('Enter fudge negative')
fudgeneg = 2.6
fudgenegFour = fudgeneg+a

smallAngleFudge = 5
