#! /usr/bin/env python
''' 
File: nuha_data_sanitizer.py
Description: This file will reorient all hand positions
    for the wine glass to lie along the x-axis. This allow
    the grasps to be interpolated in a consistent and valuable manner.
'''
from object_visualizer import object_visualizer as objvis
import sys
import os
import rospy
import numpy as np
import math
import openravepy

transform_path = '/media/sonny/FA648F24648EE2AD/Old Files/grasping_data/all_obj_transformation'
save_folder = '/home/sonny/grasp_study_2015/grasp_transforms/nuha_reoriented'

def findangle(p1,p2):
    #Find theta of rotation using dot product
    theta = math.acos((p1[0]*p2[0]+p1[1]*p2[1])/(math.hypot(p1[0],p1[1])*math.hypot(p2[0],p2[1])))
    return theta

## Nuha's function goes here!
## Input: A point on the center of the palm (maybe an axis?)
## Output: A transformation that will rotate the hand to
##  lie along that axis.
#find the angle of rotation
def nuha_reorient(pt):
    a = [1,0]
    b= [pt[0],pt[2]]
    print pt
    angle= findangle(a,b)
    if b[1] > 0:
       new_angle = angle
    else:
       new_angle = 2*math.pi - angle

    print 'nuha\'s angle: ', new_angle
    return new_angle

# Function: get_user_input()
# Description: obtains object and subject number for viewing
def get_user_input():
    obj_num = int(raw_input("Please input object number: "))
    sub_num = int(raw_input("Please input subject number: "))
    return obj_num, sub_num

# Function: get_tf_files()
# Description: Given an object, returns all hand/object transformation files to describe the grasp
def get_tf_files(obj_num):
    global transform_path
    files = os.listdir(transform_path+ "/" + 'obj' + str(obj_num) + '/')
    sorted_files = []
    for fname in files:
        if "HandTransformation" in fname:
            sorted_files.append(fname)

    return sorted(sorted_files)

# Function: get_transform()
# Description: Given a file path, the function will return the hand and object transformations as well as the joint angles for the grasp relative to the world frame.
# Parameters: f - a full file path
def get_grasp_parameters(f):
    idx = f.find('_Hand')
    #f = transform_path + "/" + "obj"+str(obj_num)+"_sub"+str(sub_num) + "/" + f[:idx]
    f = transform_path + "/" + "obj"+str(obj_num)+ "/" + f[:idx]
    rospy.loginfo("Showing " + f)
    T_hand = np.genfromtxt(f+"_HandTransformation.txt",delimiter = ',')
    T_obj = np.genfromtxt(f+"_ObjTransformation.txt",delimiter = ',')
    joint_angles = np.genfromtxt(f+"_JointAngles.txt",delimiter = ',')[7:18] #for barrett arm
    return T_hand, T_obj, joint_angles

if __name__ == "__main__":
    ## WHAT SHOULD WE DO HERE? NUHAHAHA?!

    # Create the environment with the hand
    obj_vis =  objvis()
    world_axes = obj_vis.gt.drawTransform(np.eye(4))

    while not rospy.is_shutdown():
        try:
            obj_num, sub_num = get_user_input()
        except:
            rospy.logerr("Exiting program after user input request.")
            sys.exit(1)

        # Show the grasps
        obj_vis.set_obj(obj_num)
        transform_files = get_tf_files(obj_num)
        for f in transform_files:
            t_hand, t_obj, joint_angles = get_grasp_parameters(f)
            print "File: ", f
            print "t_hand: ", t_hand, "t_obj: ", t_obj
            obj_vis.reorient_hand(t_hand, t_obj)
            obj_vis.set_joint_angles(joint_angles)
            raw_input('Grasp set up. Press [Enter] to continue')

            # Fix the grasp
            # TODO: Nuha's code goes here
            palm_pt = obj_vis.get_palm_point()
            print "Palm point: ", palm_pt
            fix_angle = nuha_reorient(palm_pt)
            fix_mat = openravepy.matrixFromAxisAngle(fix_angle * np.array([0,1,0]))
            print "fix transform: ", fix_mat
            t_hand = np.dot(fix_mat, t_hand)
            obj_vis.reorient_hand(t_hand, np.eye(4)) 
            raw_input('Grasp reoriented! Press [Enter] to continue')
            print "Changing finger position to avoid contact."
            points,_ = obj_vis.avoid_hand_collision()

            # Save the output file
            out_f_name = f.replace('HandTransformation', 'prime_HandTransformation')
            t_hand = obj_vis.hand_1.GetTransform()
            np.savetxt('%s/%s' % (save_folder, out_f_name), t_hand, delimiter = ',')
