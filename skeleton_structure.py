# -*- coding: utf-8 -*-
"""
Created on Tue Jun 19 11:43:00

@author: ATI2-Pavan Gurudath
"""
#%% Import statements
import gripper_movements_rpi as gmr


#%% Define directions and thresholds
#directions = np.load('directions.npy')

# Data specific to this main file                                                                    

#directions = np.array([[1,0],[1.,2],[1,3],[1,2],[2,2],[1,1],[1,3],[4,1],[4,20],[1,10],[2,20]])
##directions = np.array([[1,0],[2.,2],[3,3],[4,2],[5,2],[6,1],[1,3],[4,1],[4,20],[1,10],[2,20]])
#zeroes = np.array([0.,0,10,0,0,0,0,0,10,10,0])
#zeroes = zeroes.reshape((11,1))
##zeroes = np.zeros(shape=(11,1))
##zeroes = np.random.randn(11,1)
##zeroes_2 = (zeroes>0.5)*10
#directions = np.append(directions,zeroes,axis=1)
#
#
#servoDist_threshold = 3.1                                                       # Max distance travelled by the servo
#angle_threshold = 7                                                             # Min angle that the catheter needs to be bent by
#incremental_distance = 0                                                        # Keep track of previous distance

#%% Distances

#Split distances into smaller threshold and then send the list of distances
def remaining_distance(servoDist, remDist):                                    
    quotient = int(remDist // servoDist)
    remainder = remDist - servoDist*quotient
    pulseDist = []
    for number in range(quotient):
        pulseDist.append(servoDist)
    pulseDist.append(remainder)
    return pulseDist

# Pushing the catheter in front
def push_catheter(servoDist_threshold, Dist, outer_diameter):
    if Dist > servoDist_threshold:                                              #Check if the pushing distance is more than the servo's threshold distance
        pulse_distance = remaining_distance(servoDist_threshold, Dist)          #Split it up into threshold distances if it is greater
        for rDistances in pulse_distance:
            print('Push catheter by '+str(rDistances) + ' from a total_distance of ' + str(Dist))
            gmr.push_action(rDistances, outer_diameter)
    else:
        print('Push catheter by '+str(Dist))
        gmr.push_action(Dist,outer_diameter)

#%%Bending and rotatin the catheter
def bend_catheter(angle, outer_diameter):
    print('Bend the catheter by '+str(angle))
    flag=1                                                                      #Flag plays no role right now, its there for any future requirement 
    gmr.bending_arm(flag, angle, outer_diameter)

def rotate_catheter(rot_angle):
    #This function has been written keeping in mind that the rotation is done only in one direction, i.e. 0-max angle possible by the servo. 
    #Whenever a negative angle is passed, it means that the call is to move it back to the zeroeth position. 
    #This function would have to be changed to take into account the new mechanism, which is currently put on hold i.e. an incremental movement of a max 
    #of 15 degrees in either direction. 
    print('Turn the plane by ' + str(rot_angle))                                
    if rot_angle>0:
        flag=1                                                                  #Flag is raised once rotation is done in the positive direction
        gmr.back_rotation(flag, rot_angle)
    else:
        flag=0                                                                  #Flag is put down if rotation is in the negative direction 
        gmr.back_rotation(flag, rot_angle)
     

#%% main function

#distances = directions[:,0]
#angles = directions[:,1]
#rotational_angle = directions[:,2]
#
#idx=0
#flag=0
#rotation_flag =0
#
#    
#while idx < np.size(distances,0):
#    present_rot_angle = rotational_angle[idx]                                   #distance, bend angle and rotational angle at the point under 
#    present_angle = angles[idx]                                                 #consideration. 
#    present_distance = distances[idx]
#    
#    if present_rot_angle == 0:                                                  #Do calculations if no rotational angle
#        if present_angle < angle_threshold:                                     #Compare the bend angle with the threshold that we setup
#            incremental_distance = incremental_distance + present_distance      #Remember the incremental distances between points upto a certain bend is approached
#            flag = 1                                                            #Flag is raised to keep note of the incrementation
#        else:
#            if flag:
#                push_catheter(servoDist_threshold,incremental_distance)         #When bend angle is approached, it pushes the catheter by the incremental distance
#                incremental_distance = 0                                        #it had kept in memory so far. 
#                flag = 0                                                        
#            
#            ### now do it for the current position
#            bend_catheter(present_angle)                                        #Bend the catheter by specific angle
#            push_catheter(servoDist_threshold,present_distance)                 #Push the catheter by appropriate distance after the bend
#
#    else:                        
#        if flag:
#            push_catheter(servoDist_threshold,incremental_distance)             #When bend angle is approached, it pushes the catheter by the incremental distance
#            incremental_distance = 0                                            #it had kept in memory so far. 
#            flag = 0
#            
#        rotate_catheter(present_rot_angle)                                      #Rotate the plane of the catheter for the z-axis
#        bend_catheter(present_angle)                                            #Bend it by the bending angle 
#        rotate_catheter(-present_rot_angle)                                     #Rotate it back to its original plane
#        push_catheter(servoDist_threshold, present_distance)                    #Push the catheter by appropriate distance after the bend
#    
#    idx = idx + 1
