#!/usr/bin/env python
from object_visualizer import *
from std_msgs.msg import Int32MultiArray
import numpy as np
import time
import scipy
import pyscreenshot
from scipy import misc
from valid_grasp_generator.srv import *
from get_intermediate_points import get_intermediate_points
from angle_format_changer import *
import os
from shared_global import valid_grasp_dir
from retract_finger import get_palm_perpendicular_vector
transform_path = os.path.expanduser("~") + "/grasping_data"

ctrl = None

def view_alignment_cb(msg):
	global transform_path, ctrl
	obj_num = msg.data[0]
	sub_num = msg.data[1]
	grasp_num = msg.data[2]
	idx = msg.data[3]
        folder_name = "obj"+str(obj_num)+"_sub"+str(sub_num) + "/"
	f_name = folder_name+"obj" + str(obj_num) + "_sub" + str(sub_num) + "_grasp" + str(grasp_num) + "_extreme" + str(idx)
	T_hand = np.genfromtxt(f_name+"_HandTransformation.txt",delimiter = ",")
        T_obj = get_transforms(f_name+"_ObjTransformation.txt",delimiter = ",")
	ctrl.set_obj(obj_num)
	_ = ctrl.reorient_hand(T_hand, T_obj)

def GetIntermediateTransformation(Transformations,alpha):
    hand_1 = Transformations[0]
    hand_2 = Transformations[1]
    #
    #vec_1 = mat2euler((hand_1[:3,:3]).tolist())
    #vec_2 = mat2euler((hand_2[:3,:3]).tolist())
    #intermediate_rotation_vec = get_intermediate_points([vec_1,vec_2],alpha)
    #rotation_matrix = euler2mat(intermediate_rotation_vec[0],intermediate_rotation_vec[1],intermediate_rotation_vec[2])
    #
    rotation_matrix = hand_1[0:3,0:3]
    points = np.append([hand_1[0:3,3]],[hand_2[0:3,3]],axis=0)
    translation_vec = get_intermediate_points(points,alpha)
    intermediate_transform = np.append(rotation_matrix, [[0,0,0]],axis=0)
    intermediate_transform = np.append(intermediate_transform,[[translation_vec[0]],[translation_vec[1]],[translation_vec[2]],[1]],axis=1)
    return intermediate_transform


def GetJointAngles(Jointangles,alpha):
    initial_joint_angles = Jointangles[0]
    final_joint_angles = Jointangles[1]
    intermediate_joint_angles = np.add(initial_joint_angles, np.dot(np.subtract(final_joint_angles,initial_joint_angles),alpha))
    return intermediate_joint_angles

def GetGraspFile(folder_name):
    grasp_data = np.empty((2,5),np.int)
    for i in [0,1]:
        print "Enter obj num ",i+1
        obj_num = raw_input(":")
        print "Enter sub num ",i+1
        sub_num = raw_input(":")
        print "Enter grasp num ", i+1
        grasp_num = raw_input(":")
        print "Enter 1 for optimal and 0 for extreme ", i+1
        is_optimal = raw_input(":")
        print "Enter extreme or optimal number ",i+1
        ext_opt_num = raw_input(":")
        grasp_data[i] = [obj_num,sub_num,grasp_num,is_optimal,ext_opt_num]

    files = os.listdir(folder_name)
    for csv_file in files:
        matrix = np.genfromtxt(folder_name+csv_file,delimiter =',')
        comparison_matrix = matrix == grasp_data
        if comparison_matrix.all():
            print "Found csv file! Returning"
            return csv_file

    print
    print "File not found"
    print
    GetGraspData(folder_name)


if __name__=="__main__":
    rospy.init_node('test_on_robot',anonymous = True)
    global transform_path, ctrl
    ctrl = object_visualizer()
    alignment_viewer_sub = rospy.Subscriber("/openrave_grasp_view", Int32MultiArray, view_alignment_cb)
    folder_name = transform_path+"/similar_grasp_extreme_directory/"
    #csv_file = GetGraspFile(folder_name)
    csv_file = '12.csv'
    #alpha_vector = np.array([0,0.2,0.8,1])
    alpha_vector = np.array([1])
    env = ctrl.get_env()
    env.Load(valid_grasp_dir+'/models/robots/barrettwam.robot.xml')
    robot = env.GetRobots()[1]
    new_robot_transform = np.genfromtxt(valid_grasp_dir+ '/essential_files/essential_transform/Original_robot_transform.csv',delimiter = ',')
    robot.SetTransform(new_robot_transform)
    manip = robot.GetActiveManipulator()
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype = IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()
    
    similar_grasp_matrix = np.genfromtxt(folder_name+csv_file,delimiter=',')
    print similar_grasp_matrix
    JointAngles = np.empty((2,10),np.float32)
    HandTransformations = np.empty((2,4,4),np.float32)
    object_names = np.empty((2),dtype = "|S50")
    ContactLinkNames = []
    for i in [0,1]:
        obj_num = int(similar_grasp_matrix[i,0])
        sub_num = int(similar_grasp_matrix[i,1])
        grasp_num = int(similar_grasp_matrix[i,2])
        is_optimal = int(similar_grasp_matrix[i,3])
        ext_opt_num = int(similar_grasp_matrix[i,4])
        ctrl.set_obj(obj_num)
        if is_optimal == 1:
            f = "obj"+str(obj_num)+"_sub"+str(sub_num)+"_grasp"+str(grasp_num)+"_optimal"+str(ext_opt_num)
        else:
            f = "obj"+str(obj_num)+"_sub"+str(sub_num)+"_grasp"+str(grasp_num)+"_extreme"+str(ext_opt_num)
            
        object_names[i] = f 
        f = transform_path +"/"+"obj"+str(obj_num)+"_sub"+str(sub_num) + "/" + f
        rospy.loginfo("Showing " + f)
        T_hand = np.genfromtxt(f+"_HandTransformation.txt",delimiter = ',')
        T_obj = np.genfromtxt(f+"_ObjTransformation.txt",delimiter = ',')
        joint_angles = np.genfromtxt(f+"_JointAngles.txt",delimiter = ',')[7:18]
        JointAngles[i] = joint_angles
        contact_names = np.genfromtxt(f+"_ContactLinkNames.txt",delimiter = ',',dtype="|S")
        ContactLinkNames.append(contact_names)
        ctrl.reorient_hand(T_hand, T_obj,joint_angles)
        user_in = raw_input("Press Enter to continue")
        HandTransformations[i] = ctrl.GetHandTransform()
        print HandTransformations[i]
        ctrl.set_joint_angles(joint_angles)
    
    for alpha in alpha_vector:
        print "File name: ",csv_file
        child_hand_transformation = GetIntermediateTransformation(HandTransformations,alpha)
        child_joint_angles = GetJointAngles(JointAngles,alpha)
        #_ = ctrl.reorient_hand(child_hand_transformation,ObjTransformation)
        ctrl.set_hand_transformation(child_hand_transformation)
        ctrl.set_joint_angles(child_joint_angles)
        points = ctrl.avoid_hand_collision()
        ctrl.PlotPoints(points)
        
        Tgoal = ctrl.hand_1.GetLinkTransformations()[2] 
        print Tgoal
        print
        palm_perpendicular_vector = get_palm_perpendicular_vector()
        print palm_perpendicular_vector
        Tgoal[0:3,3] = np.add(Tgoal[0:3,3], np.dot(-0.062,palm_perpendicular_vector))
        print Tgoal
        sol = manip.FindIKSolution(Tgoal, IkFilterOptions.IgnoreEndEffectorCollisions)
        print sol
        robot.SetDOFValues(sol,manip.GetArmIndices())

    while not rospy.is_shutdown():
        n = 1

