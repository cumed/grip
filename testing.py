# -*- coding: utf-8 -*-
"""
Created on Tue Jun  5 10:50:44 2018

@author: thinkpad
"""

import numpy as np
import matplotlib as ml
import matplotlib.pyplot as plt

#data1 = np.array(([2,4,0],[1,3,-1],[2,2,3],[2,1,0]))
[x,y] = svg.svg_to_points("testOut.txt")  
len_z =len(x)
append_z = np.zeros(len_z)
dat1 = np.vstack((x,y,append_z)).T

def display(data):
#    [directions, z_bends] = get_angles(data)    # get angles and distances from x, y points
#    bloo = plot_points(directions)  #retrieve x, y points from angles and distances
    plt.plot(data[:,0],data[:,1],data[:,2]) # plot w original to check it's all gucci
    plt.axis('equal')
    plt.show()
    
def translate_data(data,row):
    l = row
    #l = len(data)-1
    row1 = data[l,:]        #Last row from data
    t_dat = np.empty((0, 3))
    #print('row 1', row1)
    for row in data:
        new_row = row - row1
        t_dat = np.vstack((t_dat,new_row))
    return t_dat

print("The points are: ")
print(translate_data(data1,0))

display(translate_data(data1,3))