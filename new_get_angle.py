# -*- coding: utf-8 -*-
"""
Created on Fri Jun  1 13:39:12 2018

@author: Sramana Dan
"""
"""

Updated on Mon Jun 18 

Updated to include linear interpolation 
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


#def convertToMm(data):
    
def findNextPoint(data, aindx, base, desiredLength, angles): #returns the next point for linear interpolation
    aindx = aindx
    bindx = aindx+1
    a = base
    b = data[bindx]

    while True:
        if np.linalg.norm(base-a) < desiredLength and np.linalg.norm(base-b) < desiredLength:
#            print('Walking further')
            aindx += 1
            bindx += 1
            if bindx == len(data):
                return None, None
            a = data[aindx]
            b = data[bindx]
        elif (np.linalg.norm(base-a) < desiredLength and np.linalg.norm(base-b) > desiredLength) or (np.linalg.norm(base-a) > desiredLength and np.linalg.norm(base-b) < desiredLength):
#            print('Bisection starting')
            while True:
                c = (a+b)*.5

                if abs(np.linalg.norm(base-c) - desiredLength) < .0001:
#                    print('returning')
                    return c, aindx
                if np.linalg.norm(base-a) < desiredLength and np.linalg.norm(base-c) > desiredLength:
                    b = c
                elif np.linalg.norm(base-c) < desiredLength and np.linalg.norm(base-b) > desiredLength:
                    a = c

def SpacedInterp(data, angles): # returns a set of points after linear interpolation of input data points
    angles = angles [::-1]
    cap = .03*(max(data[:,1]))  # cap is the max distance between any two interpolated points
    constant = .1
    pointHistory = np.empty((0,3))
    desiredLength = 4
    nextPoint, aindx = findNextPoint(data, 0, data[0], desiredLength, angles)
    #print("nextpoint is ",nextPoint)
    pointHistory = np.vstack((pointHistory,nextPoint))
    index = 0
    while nextPoint is not None:
        if angles.size==1:      # If angles only contains one value
            desiredLength = 4   # let desiredLength be small and uniform
        else:
            if index < (len(angles)-1): # avoid indexing error
                if(angles[index] == 0):
                    desiredLength = abs(constant / angles[index+1])
                else:
                    desiredLength = abs(constant/angles[index]) # point distance is inversely proportional to angle
            if desiredLength > cap:     # don't let the spacing rise above cap value
                desiredLength = cap
            index += 1                  # run through all the values in angles

        #if index < (len(angles)-1):    # a useful debug statement
            #print(index, 'desired length', desiredLength, 'angle', angles[index-1], 'dL', abs(constant/angles[index-1]))
        pointHistory = np.vstack((pointHistory,nextPoint))
        nextPoint, aindx = findNextPoint(data, aindx, nextPoint, desiredLength, angles)
    return pointHistory
   
def calc_distance(init_x, init_y, final_x,final_y,init_z, final_z): # returns distance between previous and current point
    d = np.sqrt((final_x - init_x)**2 + (final_y - init_y)**2 + (final_z - init_z)**2)
    return d

def find_quadrant(x,y): # returns the quadrant in which the point is located to assign sign to angle
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
        
def rotate(init_z, final_z): # checks if there is a z-dimension change to indicate if catheter needs to be rotated 
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
#dat = np.array(([4,2,0],[1,3,-1],[2,2,3],[2,1,0]))
#dat1 = np.vstack(dat)
#print("dat1:")
#print("after translation :")
#print(translate_data(dat1,0))
def return_angles(dat1): # returns the final list of distances, theta and beta angles the shape the catheter
    
    #x = dat1[0:]
    initial_points = SpacedInterp(dat1, angles = np.array([0]))
    #print(initial_points)
    x = initial_points[0:]
    rotate_flag = np.zeros(len(x))
    r = np.zeros(len(x))
    theta = np.zeros(len(x))
    beta = np.zeros(len(x))
    dat1 = initial_points
    print("data[300] :", dat1[300])
    print(dat1[301])
    for i in range(len(initial_points)-1):
        dat1= translate_data(initial_points,i)
        
        if (i == 0):
            rotate_flag[i]=0
            r[i] = 0
            theta[i] = 0
            beta[i] = 0
        else:
            if i in range(300,305):
               # print(dat1)
                print("The coordinates for this iteration are:")
                print(i)
                print(dat1[i-1][0],dat1[i][0],dat1[i-1][1],dat1[i][1],dat1[i-1][2],dat1[i][2])
            rotate_flag[i] = rotate(dat1[i-1][2],dat1[i][2])
            r[i] = calc_distance(dat1[i-1][0],dat1[i][0],dat1[i-1][1],dat1[i][1],dat1[i-1][2],dat1[i][2])
            theta[i] = bend_angle(rotate_flag[i],dat1[i-1][0],dat1[i][0],dat1[i-1][1],dat1[i][1],dat1[i-1][2],dat1[i][2])
            beta[i] = rotate_angle(rotate_flag[i],dat1[i-1][0],dat1[i][0],dat1[i-1][1],dat1[i][1],dat1[i-1][2],dat1[i][2])
            #print(r[i])
    return [r,theta,beta]

bends = return_angles(dat1)













