# -*- coding: utf-8 -*-
"""
Created on Tue Jun 19 1:45:00 2018

@author: Pavan Gurudath
"""

#import RPi.GPIO as GPIO
#import time
import pandas as pd

def heating_control(OD): # triggers the SSR for required amount of time to heat each portion of the catheter during a bend
    
    data = pd.read_excel('CurrentCatheter.xlsx')     
    Count_Row=data.shape[0]
    for row in range(0,Count_Row-1):
         if data.iloc[row,data.columns.get_loc("OD (mm)")]== OD:
             heating_time = data.iloc[row,data.columns.get_loc("Heat Time")]
             print("The heating time is ")
             print(heating_time)
#    GPIO.setmode(GPIO.BCM)     #uncomment these to operate the relay based on the heating time of the catheter element
#    GPIO.setup(18, GPIO.OUT)
#    GPIO.output(18, GPIO.HIGH)
#    time.sleep(heating_time)
#    GPIO.output(18, GPIO.LOW)
#    GPIO.cleanup()


    

def get_length(): # returns a list of the cumulative lenghts of the different materials in the catheter
    data = pd.read_excel('CurrentCatheter.xlsx')
    Count_Row=data.shape[0]
    cumulative_length = 0
    lengths = []
    for row in range(0,Count_Row-1):
        cumulative_length = cumulative_length + data.iloc[row,data.columns.get_loc("Length (mm)")]
        lengths.append(cumulative_length)    

    return lengths

def get_OD(): # returns a list of the ODs of the different materials in the catheter
    data = pd.read_excel('CurrentCatheter.xlsx')
    Count_Row=data.shape[0]
    ODs = []
    for row in range(0,Count_Row-1):
        ODs.append(data.iloc[row,data.columns.get_loc("OD (mm)")])    

    return ODs

def get_ID(): # returns a list of the IDs of the different materials in the catheter
    data = pd.read_excel('CurrentCatheter.xlsx')
    Count_Row=data.shape[0]
    IDs = []
    for row in range(0,Count_Row-1):
        IDs.append(data.iloc[row,data.columns.get_loc("ID (mm)")])    

    return IDs

def get_Material(): # returns a list of the Material names of the different materials in the catheter
    data = pd.read_excel('CurrentCatheter.xlsx')
    Count_Row=data.shape[0]
    Material = []
    for row in range(0,Count_Row-1):
        Material.append(data.iloc[row,data.columns.get_loc("Material")])    

    return Material

def get_Hysterisis(): # returns a list of the Hysteresis factors of the different materials in the catheter
    data = pd.read_excel('CurrentCatheter.xlsx')
    Count_Row=data.shape[0]
    Hysterisis = []
    for row in range(0,Count_Row-1):
        Hysterisis.append(data.iloc[row,data.columns.get_loc("Hysteresis factor")])    

    return Hysterisis

def get_HeatTime(): # returns a list of the heat times of the different materials in the catheter
    data = pd.read_excel('CurrentCatheter.xlsx')
    Count_Row=data.shape[0]
    HeatTime = []
    for row in range(0,Count_Row-1):
        HeatTime.append(data.iloc[row,data.columns.get_loc("Heat Time")])    

    return HeatTime

def get_Xi(): # returns a list of the Xis of the different materials in the catheter
    data = pd.read_excel('CurrentCatheter.xlsx')
    Count_Row=data.shape[0]
    Xi = []
    for row in range(0,Count_Row-1):
        Xi.append(data.iloc[row,data.columns.get_loc("Xi (distance between pin wall to catheter wall)")])    

    return Xi

def get_Yi(): # returns a list of the Yis of the different materials in the catheter
    data = pd.read_excel('CurrentCatheter.xlsx')
    Count_Row=data.shape[0]
    Yi = []
    for row in range(0,Count_Row-1):
        Yi.append(data.iloc[row,data.columns.get_loc("Yi (Dist end of grippers to bending pin)")])    

    return Yi

def get_MandrelMaterial(): # returns a list of the mandrel materials for the different materials in the catheter
    data = pd.read_excel('CurrentCatheter.xlsx')
    Count_Row=data.shape[0]
    MandrelMaterial = []
    for row in range(0,Count_Row-1):
        MandrelMaterial.append(data.iloc[row,data.columns.get_loc("Mandrel Material")])    

    return MandrelMaterial

def get_MandrelOD(): # returns a list of the mandrel ODs of the different materials in the catheter
    data = pd.read_excel('CurrentCatheter.xlsx')
    Count_Row=data.shape[0]
    MandrelOD = []
    for row in range(0,Count_Row-1):
        MandrelOD.append(data.iloc[row,data.columns.get_loc("Mandrel OD (mm)")])    

    return MandrelOD

def get_properties(CatheterID,flag=0):
    
    data = pd.read_excel('Catheter properties.xlsx')   
    #headers = list(data.columns.values)
    Count_Row=data.shape[0]
    print(Count_Row)
    for row in range(0,Count_Row):
        if data.iloc[row,0] == CatheterID:
            print("creating CurrentCatheter.xlsx") # writing the properties of the required catheter onto an excel sheet CurrentCatheter.xlsx
            noMaterials = int(data.iloc[row,1])
            table = []
            for table_index in range(0,noMaterials):
                table.append(data.iloc[row+table_index,2:])
            writer = pd.ExcelWriter('CurrentCatheter.xlsx')
            df = pd.DataFrame(table)
            df.to_excel(writer,'Sheet1')
            writer.save()
            break
        
    lengths = get_length()
    ODs = get_OD()
    IDs = get_ID()
    Materials = get_Material()
    HysterisisFactors = get_Hysterisis()
    HeatTimes = get_HeatTime()
    Xis = get_Xi()
    Yis = get_Yi()
    MandrelMaterials = get_MandrelMaterial()
    MandrelODs = get_MandrelOD()
    if flag:
        properties = [lengths, ODs, IDs, Materials, HysterisisFactors, HeatTimes, Xis, Yis, MandrelMaterials, MandrelODs]
        return properties
        
#properties = get_properties(0,1)
#print(properties)
#heating_control(get_OD()[2])