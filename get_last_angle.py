# -*- coding: utf-8 -*-
"""
Created on Fri Jun  1 13:39:12 2018

@author: Sramana Dan
"""

#testing get_angle

import numpy as np
import getcathetershape as svg
import pointFile as pf



pf.makePointFile("curvetest.svg")
#[x,y]=svg.svg_to_points("testout.txt")

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

    
def calc_distance(init_x, init_y, final_x,final_y,init_z, final_z):
    d = np.sqrt((final_x - init_x)**2 + (final_y - init_y)**2 + (final_z - init_z)**2)
    return d

def find_quadrant(x,y):
    if (x >= 0) and (y >= 0):
        quadrant = 1
    elif (x < 0) and (y >= 0):
        quadrant = 2
    elif (x < 0) and (y < 0):
        quadrant = 3
    elif (x >= 0) and (y < 0):
        quadrant = 4
    else:
        quadrant = None
    return quadrant
        
def rotate(init_z, final_z):
    if (abs(final_z - init_z))> 0:
       return 1
    else:
       return 0

def rotate_angle(flag,init_x,final_x,init_y,final_y,init_z, final_z): # in case of a 3 dimensional bend, this calculates the rotation angle which brings the rotation plane to the x-y plane
    if (flag == 0):
        return 0
    else:
        dy = final_y - init_y
        dz = final_z - init_z
        angle = np.arctan(abs(dz/dy)) # tentative angle to rotate the catheter by  90 - taninv(z/y)
        return angle

def bend_angle(flag,init_x, final_x, init_y, final_y, init_z, final_z):    
    dy = final_y - init_y
    dx = final_x - init_x
    dz = final_z - init_z
   
    if (flag ==0):
        quad = find_quadrant(final_x, final_y)
        if quad ==1 or quad == 3:
             angle = np.arctan(abs(dy/dx))
        elif quad == 2 or quad == 4:
            angle = np.pi/2 - np.arctan(abs(dy/dx))
        return angle
    else:
        quad = find_quadrant(final_y, final_z)
        if quad ==1 or quad == 3:
             angle = np.arctan(abs(dy/dz))
        elif quad == 2 or quad == 4:
            angle = np.pi/2 - np.arctan(abs(dy/dz))
        return angle

#def calc_angle(curr_x, curr_y, next_x, next_y):
#    prev = 
[x,y] = svg.svg_to_points("testOut.txt")  
len_z =len(x)
append_z = np.zeros(len_z)
dat1 = np.vstack((x,y,append_z)).T
#dat1 = np.array(([2,4,0],[1,3,-1],[2,2,3],[2,1,0]))
print("dat1:")
print(dat1)
#print("after translation :")
#print(translate_data(dat1,0))
x = dat1[0:]

rotate_flag = np.zeros(len(x))
r = np.zeros(len(x))
theta = np.zeros(len(x))
beta = np.zeros(len(x))
for i in range(len(dat1)-1):
    dat1= translate_data(dat1,i)
    if (i == 0):
        rotate_flag[i]=0
        r[i] = 0
        theta[i] = 0
        beta[i] = 0
    else:
        print("The coordinates for this iteration are:")
        print(i)
        #print(dat1)
        print(dat1[i-1][0],dat1[i][0],dat1[i-1][1],dat1[i][1],dat1[i-1][2],dat1[i][2])
        rotate_flag[i] = rotate(dat1[i-1][2],dat1[i][2])
        r[i] = calc_distance(dat1[i-1][0],dat1[i][0],dat1[i-1][1],dat1[i][1],dat1[i-1][2],dat1[i][2])
        theta[i] = bend_angle(rotate_flag[i],dat1[i-1][0],dat1[i][0],dat1[i-1][1],dat1[i][1],dat1[i-1][2],dat1[i][2])
        beta[i] = rotate_angle(rotate_flag[i],dat1[i-1][0],dat1[i][0],dat1[i-1][1],dat1[i][1],dat1[i-1][2],dat1[i][2])












