# -*- coding: utf-8 -*-
"""
Created on Wed Jun 13 15:17:41 2018

@author: Sramana Dan
"""
#import RPi.GPIO as GPIO
#import time
import pandas as pd

def heating_control(OD): # triggers the SSR for required amount of time to heat each portion of the catheter during a bend
    column_OD =0
    column_heat =0
    data = pd.read_excel('Catheter properties.xlsx')
    Count_Row=data.shape[0] #gives number of row count
    for column in range(2,13):
        if data.iloc[3,column]=="OD (mm)":
              column_OD = column
              print("OD column number is:")
              print(column)
        elif data.iloc[3,column]=="Heat Time":
              column_heat = column
              print("heat time column number is:")
              print(column)
        
    for row in range(Count_Row):
         if data.iloc[row,column_OD]== OD:
             heating_time = data.iloc[row,column_heat]
             print("The heating time is ")
             print(heating_time)
#    GPIO.setmode(GPIO.BCM)
#    GPIO.setup(18, GPIO.OUT)
#    GPIO.output(18, GPIO.HIGH)
#    time.sleep(heating_time)
#    GPIO.output(18, GPIO.LOW)
#    GPIO.cleanup()


    

def get_lengths(): # returns a list of the cumilative lenghts of the different materials in the catheter
    data = pd.read_excel('Catheter properties.xlsx')
    cumilative_length = 0
    lengths = []
    for column in range(2,13):
        if data.iloc[3,column]=="Length (mm)":
              column_len = column
    
    for row in range(4,7):
        cumilative_length = cumilative_length + data.iloc[row,column_len]
        lengths.append(cumilative_length)    
    return lengths

def get_OD(length):
    data = pd.read_excel('Catheter properties.xlsx')
    for column in range(2,13):
        if data.iloc[3,column]=="Length (mm)":
              column_len = column
              print("length column number is:")
              print(column)
        elif data.iloc[3,column]=="OD (mm)":
              column_od = column
              print("od column number is:")
              print(column)
    for row in range(4,7):   
       if length > 0 and length <=(data.iloc[row,column_len]):
           return data.iloc[row,column_od]
       elif length> data.iloc[row,column_len] and length < data.iloc[row+1,column_len]:
           return data.iloc[row+1,column_od]
       else:
           return data.iloc[row+2,column_od]

def get_properties():
    
    
    OD = []
    k = 0
    print(lengths)
    for row in range(4,7):
        OD.append(get_OD(lengths[k]))
        k = k+1 
        
    lengths = get_lengths()
    properties = [lengths,OD]
    return properties
        
def get_fullyClosedAngle(OD):
   angle = OD * 1.5
   return angle 
    
def get_partiallyOpenAngle(OD):
    angle = OD * 1.2
    return angle


heating_control(1.2)
l = get_lengths()
od = get_OD(3)
properties = get_properties()