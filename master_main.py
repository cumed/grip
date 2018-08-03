# -*- coding: utf-8 -*-
"""
Created on Fri Aug  3 10:06:26 2018

@author: ATI-2 Pavan Gurudath
"""


import numpy as np
import os
import getcathetershape as shape
import get_angle as angle
import pandas as pd

currDir = os.path.dirname(os.path.realpath('__file__'))

check = int(input('Are the distances and angles in an SVG file or Excel file? \n Press 1 if SVG \n Press 2 if Excel \n'))
if check ==1:
    svg_fileName = str(input('Enter the name of the svg file without the file extension .svg \n')+'.svg')
    svg_fileName = os.path.join(currDir,'svgs/',svg_fileName)
    [x,y] = shape.svg_to_points(svg_fileName)                                    
    append_z = np.zeros(len(x))                                                   # 
    data = np.vstack((x,y,append_z)).T                                            # For now, 0s are appended for the z dimension
    directions = angle.return_bends(data)                                         # storing the final array of distances, bend and rotational angles
else:
    #%% In case the inputs are from .xlsx file (Phase 2 scopic)
    excel_fileName = str(input('Enter the name of the excel file without the file extension .xlsx \n')+'.xlsx')
    data = pd.read_excel(excel_fileName)
    x = data.iloc[:,0]
    y = data.iloc[:,1]
    z = data.iloc[:,2]
    directions = np.column_stack(x,y,z)

np.save('runfile.npy',directions)
os.system('sudo python main.py')