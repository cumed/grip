# -*- coding: utf-8 -*-
"""
Created on Fri Aug 3 17:00:00 2018

@author: ATI-2 Pavan Gurudath
"""

import numpy as np
import os
import getcathetershape as shape
import get_angle as angle
import pandas as pd

#Takes the location of the current directory
currDir = os.path.dirname(os.path.realpath('__file__'))

check = int(input('Are the distances and angles in an SVG file or Excel file? \n Press 1 if SVG \n Press 2 if Excel \n'))
if check ==1:
    svg_fileName = input('Enter the name of the svg file without the file extension .svg \n')
#    svg_Dir = os.path.join(currDir,'svgs')                                        # Combines the current directory path with svgs
#    svg_fileName = os.path.join(svg_Dir,svg_fileName)
    svg_fileName = 'svgs/'+ str(svg_fileName) + '.svg'
    [x,y] = shape.svg_to_points(svg_fileName)                                     # Obtains the x and y coordinates
    data = np.vstack((x,y,np.zeros(len(x)))).T                                    # For now, 0s are appended for the z dimension
    interpolation = int(input('Enter the distance at which you want to make the bends \n'))
    directions = angle.return_bends(data,interpolation)                                         # storing the final array of distances, bend and rotational angles
else:
    #%% In case the inputs are from .xlsx file (Phase 2 scopic)
    excel_fileName = str(input('Enter the name of the excel file without the file extension .xlsx \n')+'.xlsx')
    data = pd.read_excel('excelfiles/'+ excel_fileName)
    x = data.iloc[:,0]
    y = data.iloc[:,1]
    z = data.iloc[:,2]
    data = np.column_stack(x,y,z)
    interpolation = int(input('Enter the distance at which you want to make the bends \n'))
    directions = angle.return_bends(data,interpolation)     
np.save('runfile.npy',directions)
import main                                             #Runs the main.py file 