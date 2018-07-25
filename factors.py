# -*- coding: utf-8 -*-
"""
Created on Fri Jul 20 15:24:37 2018

@author: ATI-2 Pavan Gurudath
"""

bendPinsFactorPos = 1.7 #input('Enter bend pins factor positive?')
#bendPinsFactorPos = 1.5
#bendPinsFactorPos = 0
#bendPinsFactorNeg = 0
bendPinsFactorNeg = 1.55 #input('Enter bend pins factor negative')
##bendPinsFactor = 1.65
angleRedFactor = 1
distanceFactor = 1
y_i = 1.78
d_pins = 5.25
OD = 1.62
positiveAngleOffset = 0
negativeAngleOffset = 0
ODList = [OD,OD,OD,OD]
fudgepos = input('Enter fudge positive')
fudgeneg = input('Enter fudge negative')
#fudgepos = 1.95
#fudgeposFour = 1
fudgeposFour = fudgepos+0.3
#fudgeneg = input('Enter fudge negative')
#fudgeneg = 1.95
fudgenegFour = fudgeneg+0.5
xDistPins = 3
