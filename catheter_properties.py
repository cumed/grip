# -*- coding: utf-8 -*-
"""
Created on Thu Jul 19 15:25:45 2018

@author: ATI-2 Pavan Gurudath
"""


#import RPi.GPIO as GPIO
#import time
import pandas as pd

#def heating_control(OD): # triggers the SSR for required amount of time to heat each portion of the catheter during a bend
#    
#    data = pd.read_excel('CurrentCatheter.xlsx')     
#    count_row=data.shape[0]
#    for row in range(0,count_row):
#         if data.iloc[row,data.columns.get_loc("OD (mm)")]== OD:
#             heating_time = data.iloc[row,data.columns.get_loc("Heat Time")]
#             print("The heating time is ")
#             print(heating_time)
##    GPIO.setmode(GPIO.BCM)     #uncomment these to operate the relay based on the heating time of the catheter element
##    GPIO.setup(18, GPIO.OUT)
##    GPIO.output(18, GPIO.HIGH)
##    time.sleep(heating_time)
##    GPIO.output(18, GPIO.LOW)
##    GPIO.cleanup()
   

def get_length(): 
    # returns a list of the cumulative lenghts of the different materials in the catheter
    data = pd.read_excel('CurrentCatheter.xlsx')
    count_row=data.shape[0]
    cumulative_length = 0
    lengths = []
    for row in range(0,count_row):
        cumulative_length = cumulative_length + data.iloc[row,data.columns.get_loc("Length (mm)")]
        lengths.append(cumulative_length)    

    return lengths

def get_OD(): 
    # returns a list of the ODs of the different materials in the catheter
    data = pd.read_excel('CurrentCatheter.xlsx')
    count_row=data.shape[0]
    ODs = []
    for row in range(0,count_row):
        ODs.append(data.iloc[row,data.columns.get_loc("OD (mm)")])    
    return ODs

def get_ID(): 
    # returns a list of the IDs of the different materials in the catheter
    data = pd.read_excel('CurrentCatheter.xlsx')
    count_row=data.shape[0]
    IDs = []
    for row in range(0,count_row):
        IDs.append(data.iloc[row,data.columns.get_loc("ID (mm)")])    

    return IDs

def get_material(): 
    # returns a list of the material names of the different materials in the catheter
    data = pd.read_excel('CurrentCatheter.xlsx')
    count_row=data.shape[0]
    material = []
    for row in range(0,count_row):
        material.append(data.iloc[row,data.columns.get_loc("Material")])    
    return material

def get_materialName():
    data = pd.read_excel('CurrentCatheter.xlsx')
    count_row = data.shape[0]
    materialName = []
    for row in range(0,count_row):
        materialName.append(data.iloc[row,data.columns.get_loc("Materials")])
    return materialName

def get_hysterisis(): # returns a list of the Hysteresis factors of the different materials in the catheter
    data = pd.read_excel('CurrentCatheter.xlsx')
    count_row=data.shape[0]
    hysterisis = []
    for row in range(0,count_row):
        hysterisis.append(data.iloc[row,data.columns.get_loc("Hysteresis factor")]) 
    return hysterisis

def get_heatTime(lens=None): # returns a list of the heat times of the different materials in the catheter
    data = pd.read_excel('CurrentCatheter.xlsx')
    count_row=data.shape[0]
    heatTime = []
    for row in range(0,count_row):
        heatTime.append(data.iloc[row,data.columns.get_loc("Heat Time")])    
    if lens != None:
        loc = get_length().index(lens)
        heatTime = heatTime[loc]
        return heatTime
    
    return heatTime

#def get_Xi(): # returns a list of the Xis of the different materials in the catheter
#    data = pd.read_excel('CurrentCatheter.xlsx')
#    count_row=data.shape[0]
#    Xi = []
#    for row in range(0,count_row):
#        Xi.append(data.iloc[row,data.columns.get_loc("Xi (distance between pin wall to catheter wall)")])    
#
#    return Xi

def get_Yi(): # returns a list of the Yis of the different materials in the catheter
    data = pd.read_excel('CurrentCatheter.xlsx')
    count_row=data.shape[0]
    Yi = []
    for row in range(0,count_row):
        Yi.append(data.iloc[row,data.columns.get_loc("Yi (Dist end of grippers to bending pin)")])    

    return Yi

def get_mandrelmaterial(): # returns a list of the mandrel materials for the different materials in the catheter
    data = pd.read_excel('CurrentCatheter.xlsx')
    count_row=data.shape[0]
    mandrelmaterial = []
    for row in range(0,count_row):
        mandrelmaterial.append(data.iloc[row,data.columns.get_loc("Mandrel Material")])    

    return mandrelmaterial

def get_mandrelOD(): # returns a list of the mandrel ODs of the different materials in the catheter
    data = pd.read_excel('CurrentCatheter.xlsx')
    count_row=data.shape[0]
    mandrelOD = []
    for row in range(0,count_row):
        mandrelOD.append(data.iloc[row,data.columns.get_loc("Mandrel OD (mm)")])    

    return mandrelOD

def get_properties(CatheterID,flag=0):                                                  # Only if flag is raised, then return all the properties
    
    data = pd.read_excel('Catheter properties.xlsx')   
    #headers = list(data.columns.values)
    count_row=data.shape[0]
    for row in range(0,count_row):
        if data.iloc[row,0] == CatheterID:
            print("creating CurrentCatheter.xlsx") # writing the properties of the required catheter onto an excel sheet CurrentCatheter.xlsx
            nomaterials = int(data.iloc[row,1])
            table = []
            for table_index in range(0,nomaterials):
                table.append(data.iloc[row+table_index,2:])
            writer = pd.ExcelWriter('CurrentCatheter.xlsx')
            df = pd.DataFrame(table)
            df.to_excel(writer,'Sheet1')
            writer.save()
            break
    
    if flag:    
        lengths = get_length()
        ODs = get_OD()
        IDs = get_ID()
        materials = get_material()
        hysterisisFactors = get_hysterisis()
        heatTimes = get_heatTime()
#    Xis = get_Xi()
        Yis = get_Yi()
        mandrelmaterials = get_mandrelmaterial()
        mandrelODs = get_mandrelOD()
    
        properties = [lengths, ODs, IDs, materials, hysterisisFactors, heatTimes, Yis, mandrelmaterials, mandrelODs]
        return properties

if __name__ == "__main__":
    get_properties(1)        
#properties = get_properties(0,1)
#print(properties)
#heating_control(get_OD()[2])