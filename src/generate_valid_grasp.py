#!/usr/bin/env python

from openravepy import *
import rospy
import rospkg
import sys,getopt
import csv
from grasp_manager.msg import GraspSnapshot
from shared_global import *
from get_matrix import *
import numpy as np

class valid_grasps():
    def __init__(self):
        self.path = rospkg.RosPack().get_path('Valid_grasp_generator')
        self.env = Environment()
        self.env.Load(self.path+'/models/robots/barrett_wam.dae')
        self.env.SetViewer('qtcoin')
        self.robot = self.env.GetRobots()[0]
        self.Table = self.env.ReadKinBodyXMLFile('data/table.kinbody.xml')
        self.env.Add(self.Table)
        self.is_valid_entry = False
        self.obj_name = ''
        self.part = None
        self.obj_num = None
        self.sub_num = None
        self.grasp_num = None
        self.is_optimal = None
        self.ext_opt_num = None # variable for recording the optimal or extreme number
        self.obj_transform = None
        self.previous_obj_name = None
        self.robot_transform = np.genfromtxt(self.path+'/transformation_matrices/essential_transform/Wam_transform.csv',delimiter = ',')
        self.table_transform = np.genfromtxt(self.path+'/transformation_matrices/essential_transform/Table_transform.csv',delimiter = ',')
        self.robot.SetTransform(self.robot_transform)
        self.Table.SetTransform(self.table_transform)

    def get_obj_name(self):
        Fid = open(self.path+"/models/stl_files/part_list.csv")
        part_list = csv.reader(Fid,delimiter = ',')
        obj_name = None
        for row in part_list:
            if row[0]==str(self.obj_num):
                obj_name = row[2]
        Fid.close()
        return obj_name

#    def update_environment(self):
#        
#        try:
#            while True:
#                n=1
#        except KeyboardInterrupt, e:
#            print 'exiting', e
#            sys.exit()
    def robot_updator(self,snapshot):
        T_hand = np.array(snapshot.hand_joints.position)
        rospy.loginfo("Got into robot_updator")
        T_wam = np.array(snapshot.wam_joints.position)
        T_robot = T_wam[0:7]
        T_robot = np.append(T_robot,[0,0])
        T_robot = np.append(T_robot,[T_hand[3],T_hand[0],T_hand[4],T_hand[3],T_hand[1],T_hand[5],T_hand[2],T_hand[6]])
        self.robot.SetDOFValues(T_robot)

    def part_updator(self,get_data):
        self.obj_num = get_data.obj_num
        self.sub_num = get_data.sub_num
        rospy.loginfo("Got into part_updator")
        self.grasp_num = get_data.grasp_num
        self.is_optimal = get_data.is_optimal
        obj_folder = 'obj'+str(self.obj_num)+'_sub'+str(self.sub_num)+'_pointcloud_csvfiles/'
        if self.is_optimal:
            self.ext_opt_num = get_data.optimal_num
            file_name = obj_transform_dir+obj_folder+'obj'+str(self.obj_num)+'_sub'+str(self.sub_num)+'_grasp'+str(self.grasp_num)+'_optimal'+str(self.ext_opt_num)+'_object_transform.txt'
        else:
            self.ext_opt_num = get_data.extreme_num
            file_name = obj_transform_dir+obj_folder+'obj'+str(self.obj_num)+'_sub'+str(self.sub_num)+'_grasp'+str(self.grasp_num)+'_extreme'+str(self.ext_opt_num)+'_object_transform.txt'
        matrix = generate_matrix(file_name)
        self.obj_transform = matrix['obj_matrix']
        self.obj_name = self.get_obj_name()
        if (self.part == None) or (not self.obj_name == self.previous_obj_name):
            if not self.part == None:
                self.env.Remove(self.part)
            self.part = self.env.ReadKinBodyXMLFile(self.path+"/models/stl_files/"+self.obj_name,{'scalegeometry':'0.001 0.001 0.001'})
            self.previous_obj_name = self.obj_name
            self.env.Add(self.part)

        self.part.SetTransform(self.obj_transform)
        

if __name__=="__main__":
    generate_grasp = valid_grasps()
    rospy.init_node('valid_grasp_generator',anonymous = True)
    generate_grasp.sub_robot = rospy.Subscriber("grasp_extremes",GraspSnapshot,generate_grasp.robot_updator)
    generate_grasp.sub_part = rospy.Subscriber("grasp_extremes",GraspSnapshot,generate_grasp.part_updator)
    rospy.spin()
#    generate_grasp.update_environment()
