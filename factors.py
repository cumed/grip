# -*- coding: utf-8 -*-
"""
Created on Fri Jul 20 15:24:37 2018

@author: ATI-2 Pavan Gurudath
"""
#%% 

time_constant = 0.25

servoDist_threshold       = 6                                                 # Max distance travelled by the back indexing servo(4.75*2)
angle_threshold           = 1                                                   # Min angle required that the catheter needs to be bent by
rotationalAngle_threshold = 1  

#catheter_ID = 3

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
xDistPins = 3

a = 5
fudgepos = input('Enter fudge positive')
#fudgepos = 1.4
fudgeposFour = fudgepos+a

fudgeneg = input('Enter fudge negative')
#fudgeneg = 1.45
fudgenegFour = fudgeneg+a

smallAngleFudge = input('Enter the small angle')
