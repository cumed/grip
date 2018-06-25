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
import matplotlib.pyplot as plt
import math

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
    desiredLength = 3
    nextPoint, aindx = findNextPoint(data, 0, data[0], desiredLength, angles)
    #print("nextpoint is ",nextPoint)
    pointHistory = np.vstack((pointHistory,nextPoint))
    index = 0
    while nextPoint is not None:
        if angles.size==1:      # If angles only contains one value
            desiredLength = 3   # let desiredLength be small and uniform
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
        
def check_rotate(init_z, final_z): # checks if there is a z-dimension change to indicate if catheter needs to be rotated 
    if (abs(final_z - init_z))> 0:
       return 1
    else:
       return 0
   
def rotate(xold, yold, alpha, quadrant):    # rotate to get cur_x,y in frame of
    if (quadrant==1) or (quadrant == 3):    # next_x,y
        alpha = - alpha         # determine direction of rotation
    xnew = xold*np.cos(alpha) - yold*np.sin(alpha)      # rotation is basic trig
    ynew = yold*np.cos(alpha) + xold*np.sin(alpha)
    return xnew, ynew
def rotate_angle(flag,init_x,final_x,init_y,final_y,init_z, final_z): # in case of a 3 dimensional bend, this calculates the rotation angle which brings the rotation plane to the x-y plane
    if (flag == 0):
        return 0
    else:
        dy = final_y - init_y
        dz = final_z - init_z
        angle = np.arctan(abs(dz/dy)) # tentative angle to rotate the catheter by  90 - taninv(z/y)
        return angle

def bend_angle_cos(flag,prev_x, prev_y, curr_x, curr_y, next_x, next_y):
    a = calc_distance(curr_x,next_x,curr_y,next_y,0,0)
    b = calc_distance(prev_x,next_x,prev_y,next_y,0,0) 
    c = calc_distance(curr_x,prev_x,curr_y,prev_y,0,0)
    direction = (prev_x - curr_x)*(next_y - curr_y)-(prev_y - curr_y)*(next_x -curr_x)
    if direction < 0:
        if (a**2 + c**2 - b**2)/(2*a*c) > 1:
            angle = np.pi - np.pi/2
        else:
            angle = np.pi - np.arccos(round((a**2 + c**2 - b**2) /(2*a*c),5))
    else :
        if (a**2 + c**2 - b**2)/(2*a*c) > 1:
            angle = -(np.pi - np.pi/2)
        else:
            angle =-(np.pi - np.arccos(round((a**2 + c**2 - b**2) /(2*a*c),5)))
    if(math.isnan(angle)):
        angle = 0
    if angle == -0:
        angle = 0
    
    return angle

def plot_points(directions, angles):
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
    bloo = np.vstack((xarr, yarr)).T
    return bloo


def return_angles(dat1): # returns the final list of distances, theta and beta angles the shape the catheter
    
    x = dat1[0:]
    initial_points = SpacedInterp(dat1, angles = np.array([0]))
    #print(initial_points)
    x = initial_points[0:]
    rotate_flag = np.zeros(len(x))
    r = np.zeros(len(x))
    theta = np.zeros(len(x))
    beta = np.zeros(len(x))
    dat1 = initial_points[1:]
#    print("data[1] :", dat1[1])
#    print(dat1[2])
    for i in range(0, len(dat1)-1):        
        
        prev_x = dat1[i-1][0]
        prev_y = dat1[i-1][1]
        curr_x = dat1[i][0]
        curr_y = dat1[i][1]
        next_x = dat1[i+1][0]
        next_y = dat1[i+1][1]
        
        rotate_flag[i] = check_rotate(dat1[i-1][2],dat1[i][2])
        r[i] = calc_distance(dat1[i][0],dat1[i+1][0],dat1[i][1],dat1[i+1][1],dat1[i][2],dat1[i+1][2])
         
        if i == 0:
            theta[i] = 0
            beta[i] = 0
        else:
            dat1= translate_data(dat1,i) 
            #print(dat1)
            theta[i] = bend_angle_cos(rotate_flag[i],prev_x, prev_y, curr_x, curr_y, next_x, next_y)
            beta[i] = rotate_angle(rotate_flag[i],dat1[i-1][0],dat1[i][0],dat1[i-1][1],dat1[i][1],dat1[i-1][2],dat1[i][2])
        #theta[i] = bend_angle(rotate_flag[i],dat1[i-1][0],dat1[i][0],dat1[i-1][1],dat1[i][1],dat1[i-1][2],dat1[i][2])
         
#        print("The coordinates for this iteration are:")
#        print(i)
#        print(dat1[i-1][0],dat1[i][0],dat1[i+1][0],dat1[i-1][1],dat1[i][1],dat1[i+1][1])
#        print("angle: ",theta[i])
#            if i == 13:
#                theta[len(dat1) - i] = bend_angle_cos(1,prev_x, prev_y, curr_x, curr_y, next_x, next_y)
#                print("The coordinates for this iteration are:")
#                print(i)
#                print(dat1[i-1][0],dat1[i][0],dat1[i+1][0],dat1[i-1][1],dat1[i][1],dat1[i+1][1])
#                print("angle: ",theta[i])

    bad_points = plot_points(r,theta) # plot_points converts directions into x, y points
                                            # bad_points is a way to check the first round of interpolation
                                            # and angle extraction
    plt.subplot(1, 2, 1)
    plt.plot(dat1[:,0], dat1[:,1], 'r')
    plt.title('initial_points')
    plt.axis('equal')
    plt.subplot(1, 2, 2)
    plt.plot(bad_points[:,1], bad_points[:,0], 'rx')
    plt.title('bends')
    plt.axis('equal')
    plt.show()
    return [r,np.degrees(theta),np.degrees(beta)]


[x,y] = svg.svg_to_points("testOut.txt")  
len_z =len(x)
append_z = np.zeros(len_z)
dat1 = np.vstack((x,y,append_z)).T
temp = dat1
#plt.plot(dat1[:,0], dat1[:,1], 'r')
#dat1 = np.array(([16,5,0],[5,3,0],[3,4,0],[2,3,0],[1,4,0],[0,3,0],[0,0,0]))   
bends = return_angles(dat1)
np.save('bends.npy', bends)














