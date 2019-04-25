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
import matplotlib.pyplot as plt
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits import mplot3d
import math
import pandas as pd

def translate_data(data,row): # to shift the points such that the current point is at the origin 0,0,0
    l = row
    #l = len(data)-1
    row1 = data[l,:]        #Last row from data
    t_data = np.empty((0, 3))
    #print('row 1', row1)
    for row in data:
        new_row = row - row1
        t_data = np.vstack((t_data,new_row))
    return t_data

def plot_points(directions, angles): # plot back the points from the obtained bend directions and angles - for verification
    distances = directions
    angles = angles
    distances = distances[::-1]
    angles = angles[::-1]
    xn = 0
    yn = 0
    ang = 0
    xarr = [0]
    yarr = [0]
    n = 0
    for d in distances:
        a = angles[n]
        ang = a + ang
        xi = d*np.sin(ang)
        yi = d*np.cos(ang)
        xn = xn - xi 
        yn = yn + yi
        xarr.append(xn)
        yarr.append(yn)
        n = n + 1
    test = np.vstack((xarr, yarr)).T
    return test

def plot_points3D(r, theta,beta): # plot back the points from the obtained bend directions and angles - for verification
    distances = r
    #distances = distances[::-1]
    #angles = angles[::-1]
    xn = 0
    yn = 0
    zn = 0
    a = 0
    b = 0
    ang1 = 0
    ang2 = 0
    xarr = [0]
    yarr = [0]
    zarr = [0]
    n = 0
    for d in distances:
        ang1 = theta[n]
        ang2 = beta[n]
        ang1 = a + ang1
        ang2 = b + ang2
        xi = d*np.sin(ang1)
        yi = d*np.cos(ang1)
        zi = d*np.sin(ang2)
        xn = xn - xi 
        yn = yn + yi
        zn = zn + zi
        xarr.append(xi)
        yarr.append(yi)
        zarr.append(zi)
        n = n + 1
    test = np.vstack((xarr, yarr, zarr)).T
    #print(test)
    return test
    
def findNextPoint(data, aindx, base, desiredLength, angles): #returns the next point for linear interpolation
    aindx = aindx
    bindx = aindx+1
    a = base
    b = data[bindx]

    while True:
        if np.linalg.norm(base-a) < desiredLength and np.linalg.norm(base-b) < desiredLength:
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

def SpacedInterp(data,interpolation,angles): # returns a set of points after linear interpolation of input data points
    angles = angles [::-1]
    cap = .03*(max(data[:,1]))  # cap is the max distance between any two interpolated points
    constant = .1
    pointHistory = np.empty((0,3))
    desiredLength = interpolation # interpolation occurs at this length. 
    nextPoint, aindx = findNextPoint(data, 0, data[0], desiredLength, angles)
    #print("nextpoint is ",nextPoint)
    pointHistory = np.vstack((pointHistory,nextPoint))
    index = 0
    while nextPoint is not None:
        if angles.size==1:      # If angles only contains one value
            desiredLength = interpolation # interpolation occurs at this length
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
   
def calc_distance(init_x,final_x,init_y,final_y,init_z, final_z): # returns distance between previous and current point
    distance = np.sqrt((final_x - init_x)**2 + (final_y - init_y)**2 + (final_z - init_z)**2)
    return distance
        
def check_rotate(curr_z): # returns 1 if there is a rotation of plane required at that point
    # checks if there is a z-dimension change to indicate if catheter needs to be rotated 
    if curr_z != 0:
       return 1
    else:
       return 0
   
def rotate_angle(flag,curr_x,curr_y,curr_z, ): # in case of a 3 dimensional bend, this calculates the rotation angle which brings the rotation plane to the x-y plane
    if (flag == 0):
        return 0
    else:
        abs_angle = np.arcsin(curr_z/(np.sqrt(curr_x**2 + curr_y**2 + curr_z**2))) # angle between a line and x-y plane
        
        
    return angle

global rotation
def rotation(flag,prev_x,prev_y,prev_z,curr_x,curr_y,curr_z,next_x,next_y,next_z): 
    a1= (next_y-curr_y)*(curr_z-prev_z) - (curr_y-prev_y)*(next_z-prev_z)
    a2= (next_y-curr_y)*(next_z-prev_z) - (curr_y-prev_y)*(next_z-curr_z)
    b1= (curr_x-prev_x)*(curr_z-prev_z) - (curr_x-prev_x)*(next_z-prev_z)
    b2= (next_x-curr_x)*(next_z-prev_z) - (curr_x-prev_x)*(next_z-curr_z)
    c1= (next_y-curr_y)*(curr_y-prev_y) - (curr_x-prev_x)*(next_y-prev_y)
    c2= (next_y-curr_y)*(curr_z-prev_z) - (curr_y-prev_y)*(next_z-prev_z)
    
    if (flag == 0):
        return 0
    else:
        angle = np.arccos(((a1*a2)+(b1*b2)+(c1*c2))/(np.sqrt(a1*a1 + b1*b1 + c1*c1) * np.sqrt(a2*a2 + b2*b2 + c2*c2))) # angle between a line and x-y plane
        
    return angle   

def bend_angle(flag,prev_x,prev_y,prev_z,curr_x,curr_y,curr_z,next_x,next_y,next_z): # returns the bend angle taking in 3 points (previous, current, next) each time
    a = calc_distance(curr_x,next_x,curr_y,next_y,curr_z,next_z) # calculating distances to apply the cosine formula 
    b = calc_distance(prev_x,next_x,prev_y,next_y,prev_z,next_z) 
    c = calc_distance(curr_x,prev_x,curr_y,prev_y,curr_z,prev_z)
    direction = (prev_x - curr_x)*(next_y - curr_y)-(prev_y - curr_y)*(next_x -curr_x) # to check if bend is clockwise or counter-clockwise
    if direction > 0: # clockwise bend angle
        if (a**2 + c**2 - b**2)/(2*a*c) > 1:
            angle = np.pi - np.pi/2
        else:
            angle = np.pi - np.arccos(round((a**2 + c**2 - b**2) /(2*a*c),5))
    else :  # counterclockwise bend angle
        if (a**2 + c**2 - b**2)/(2*a*c) > 1:
            angle = -(np.pi - np.pi/2)
        else:
            angle =-(np.pi - np.arccos(round((a**2 + c**2 - b**2) /(2*a*c),5)))
    if(math.isnan(angle)): # to take care of possible Not A Number cases
        angle = 0
    if angle == -0:
        angle = 0
    
    return angle


def return_bends(data,interpolation): # returns the final array of distances, theta and beta angles the shape the catheter
    
    x = data[0:]
    initial_points = SpacedInterp(data,interpolation, angles = np.array([0])) # interpolating the data points 
    #initial_points = data
    data = initial_points[1:]
    print(initial_points)
    x = initial_points[0:] 
    rotate_flag = np.zeros(len(x))
    r = np.zeros(len(x))
    theta = np.zeros(len(x))
    beta = np.zeros(len(x)) 
#    print("data[1] :", data[1])
    for i in range(0,len(data)-1):        
        
       prev_x = data[i-1][0]   # previous, current and next point coordinates
       prev_y = data[i-1][1]
       prev_z = data[i-1][2]
       curr_x = data[i][0]
       curr_y = data[i][1]
       curr_z = data[i][2]
       next_x = data[i+1][0]
       next_y = data[i+1][1]
       next_z = data[i+1][2]
        
       rotate_flag[i] = check_rotate(data[i][2]) # checks if plane rotation is required
       r[i] = calc_distance(data[i][0],data[i+1][0],data[i][1],data[i+1][1],data[i][2],data[i+1][2]) # distance between the consecutive points 
         
       if i == 0:
           theta[i] = 0 # as there cannot be a bend angle for the first point
           beta[i] = 0
       else:
           theta[i] = bend_angle(rotate_flag[i],prev_x,prev_y,prev_z,curr_x, curr_y,curr_z,next_x,next_y,next_z)
           if np.degrees(theta[i]) >60 or np.degrees(theta[i])<-60:
               theta[i] = 0
           #beta[i] = rotation(rotate_flag[i],prev_x,prev_y,prev_z,curr_x, curr_y,curr_z,next_x,next_y,next_z)
           beta[i] = rotate_angle(rotate_flag[i],curr_x,curr_y,curr_z)
    #if theta[i] == 0 for i in range(0,len(theta)/2):
       #theta = theta[]
    theta[1] = 0
    theta[2] = 0
    bends = plot_points(r,theta) # plot_points converts directions into x, y points
    rot = plot_points3D(r,theta,beta)                                      # bad_points is a way to check the first round of interpolation
                                            # and angle extraction
#uncomment for 2D plot
    plt.subplot(1, 2, 1)
    plt.plot(data[:,0], data[:,1], 'r') # plotting the initial set of points
    plt.title('initial_points')
    plt.axis('equal')
    plt.subplot(1, 2, 2)
    plt.plot(bends[:,0], bends[:,1], 'rx')
    plt.title('bends')
    plt.axis('equal')
    plt.show()
#    
    fig = pyplot.figure()  # uncomment for 3D plot
    x= data[:,0]
    y= data[:,1]
    z= data[:,2]
    bx = Axes3D(fig)
    bx = fig.add_subplot(111, projection= '3d')
    bx.set_xlabel("x axis")
    bx.set_ylabel("y axis")
    bx.set_zlabel("z axis")
    #bx.scatter(x,y,z)
    bx.plot(x,y,z,'-o')
    #bx.show()
    #ax = p
    #bx.plot(rotation[:,0], rotation[:,1], rotation[:,2])
    pyplot.show()
    return np.transpose([r,np.degrees(theta),np.degrees(beta)])



data = pd.read_excel('3D_Test_Curve_Points.xlsx')   
#data = pd.read_csv('trial2.csv')   
#print(data) 

x = pd.to_numeric(data.iloc[1:,0])
y = pd.to_numeric(data.iloc[1:,1])
z = pd.to_numeric(data.iloc[1:,2])
print(type(x[1]))
#[x,y] = svg.svg_to_points("svgs\estimate_test.svg")  # include z dimension when 3 dimensional points are inputed. For now, 0s are appended for the z dimension
len_z =len(x)
append_z = np.zeros(len_z) # 
#data = np.vstack((x,y,append_z)).T #For now, 0s are appended for the z dimension
data = np.vstack((x,y,z)).T
#temp = data   uncomment to read original points
#data = np.array(([10,5,0],[5,3,6],[3,4,0],[2,3,0],[1,4,0],[0,3,0],[0,0,0],))   #testing datapoints
#data = data[::-1]
fig = pyplot.figure()  # uncomment for 3D plot
x= data[:,0]
y= data[:,1]
z= data[:,2]
bx = Axes3D(fig)
bx.set_xlabel("x axis")
bx.set_ylabel("y axis")
bx.set_zlabel("z axis")
bx = fig.add_subplot(111, projection= '3d')
    #bx.scatter(x,y,z)
bx.plot(x,y,z,'-o')
pyplot.show()
    #ax = p
bends = return_bends(data,1) # storing the final array of distances, theta and beta angles
#np.save('longCurve15.npy', np.transpose(bends)) # saving the calculated array into a numpy file




