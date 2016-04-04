#!/usr/bin/env python
from openravepy import *
import rospy
import rospkg
import sys,getopt
import csv
from get_all_contact_values import *
from grasp_manager.msg import GraspSnapshot
from shared_global import *
from get_matrix import *
import numpy as np
from angle_format_changer import *
from scipy.interpolate import interp1d
import timeit
import time
import os
from get_principal_axes import GetPrincipalAxes


class valid_grasps():
    def __init__(self,env):
        self.path = rospkg.RosPack().get_path('valid_grasp_generator')
        self.env = env
        self.env.Load(self.path+'/models/robots/adept_barrett_hand.dae')
        self.robot = self.env.GetRobots()[0]
        self.obj_name = 'CerealBox.STL'
        self.part = self.env.ReadKinBodyXMLFile(self.path+'/models/stl_files/'+self.obj_name,{'scalegeometry':'0.001 0.001 0.001'})
        self.env.Add(self.part)
        self.data_saving_folder = "/home/"+user+"/grasping_data/" 
        #is_optimal = raw_input("Is the grasp optimal(y/n): ")
        #self.is_optimal = 1 if is_optimal == 'y' else 0
        #ext_opt_string = 'optimal' if is_optimal == 'y' else 'extreme'
        #self.ext_opt_num = raw_input("Enter the "+ext_opt_string+" number: ")
        self.robot_dof_limits = self.robot.GetDOFLimits()
        self.mapper = interp1d([0,self.robot_dof_limits[1][7]+self.robot_dof_limits[1][8]],[0,self.robot_dof_limits[1][8]])
        #Enter the row of the joint angles
        self.data_vector_arm = [10.70,-63.23,181.82, -162.77,-46.99,63.10]
        self.JointAngles_arm = np.radians(self.data_vector_arm)
        self.JointAngles_hand = [1.36,1.02,1.04,1.49]
        self.data_saving_folder = os.path.expanduser('~')+'/bad_grasps/'

    def get_obj_name(self,obj_num):
        Fid = open(self.path+"/models/stl_files/part_list.csv")
        part_list = csv.reader(Fid,delimiter = ',')
        obj_name = None
        scale = None
        for row in part_list:
            if row[0]==str(obj_num):
                obj_name = row[2]
                scale = row[3]
        Fid.close()
        return obj_name, scale

    def update_environment(self):
        self.obj_num = raw_input("Enter Object Number: ")
        self.sub_num = raw_input("Enter Subject Number: ")
        self.grasp_num = raw_input("Enter Grasp Number: ")
        if not self.obj_name == self.get_obj_name(self.obj_num)[0]:
            self.obj_name = self.get_obj_name(self.obj_num)[0]
            self.env.Remove(self.part)
            self.part = self.env.ReadKinBodyXMLFile(self.path+'/models/stl_files/'+self.obj_name,{'scalegeometry':'0.001 0.001 0.001'})
            self.env.Add(self.part)
        complete_hand_joint_angles = [self.JointAngles_hand[-1], self.JointAngles_hand[0], self.mapper(self.JointAngles_hand[0]),self.JointAngles_hand[-1],self.JointAngles_hand[1],self.mapper(self.JointAngles_hand[1]),self.JointAngles_hand[2],self.mapper(self.JointAngles_hand[2])]
        complete_joint_angles = np.append(self.JointAngles_arm,complete_hand_joint_angles)
        self.robot.SetDOFValues(complete_joint_angles)
        if not os.path.exists(self.data_saving_folder+'obj'+str(self.obj_num)+'_sub'+str(self.sub_num)+'/'):
            os.mkdir(self.data_saving_folder+'obj'+str(self.obj_num)+'_sub'+str(self.sub_num)+'/')
            
        
        current_dir  = self.data_saving_folder+'obj'+str(self.obj_num)+'_sub'+str(self.sub_num)+'/'
        raw_input("Press Enter when you are done with manual alignment!")
        np.savetxt(current_dir+'obj'+str(self.obj_num)+'_sub'+str(self.sub_num)+'_grasp'+str(self.grasp_num)+'_HandTransformation.txt',self.robot.GetLinkTransformations()[7],delimiter=',')
        np.savetxt(current_dir+'obj'+str(self.obj_num)+'_sub'+str(self.sub_num)+'_grasp'+str(self.grasp_num)+'_ObjTransformation.txt',self.part.GetTransform(),delimiter=',')
        np.savetxt(current_dir+'obj'+str(self.obj_num)+'_sub'+str(self.sub_num)+'_grasp'+str(self.grasp_num)+'_JointAngles.txt',self.robot.GetDOFValues(),delimiter=',')
        while not rospy.is_shutdown():
            self.update_environment()
        

if __name__=="__main__":
    rospy.init_node('adept_barrett_hand')
    env = Environment()
    env.SetViewer('qtcoin')
    generate_grasp = valid_grasps(env)
    generate_grasp.update_environment()
    rospy.spin()   
