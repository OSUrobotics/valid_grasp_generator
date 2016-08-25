#!/usr/bin/env python

from openravepy import *
import rospy
import rospkg
import sys,getopt
import csv
from get_all_contact_values import *
from valid_grasp_generator.msg import GraspSnapshot
from shared_global import *
from get_matrix import *
import numpy as np
from angle_format_changer import *
from scipy.interpolate import interp1d
import timeit
import time
import os
from get_principal_axes import GetPrincipalAxes


sys.path.append(catkin_ws_location+"/devel/lib/")
#import libdepth_penetration # This library is generated from depth_penetration.cpp

class ContPointWithDistance():
    def __init__(self,name="Not Specified",MinDist=5,NearestPoint=np.array([0,0,0])):
        self.MinDistance = MinDist
        self.ContactPoint = NearestPoint 
        self.name = name

    def __str__(self):
        return self.name 

class valid_grasps():
    def __init__(self,env):
        self.path = rospkg.RosPack().get_path('valid_grasp_generator')
        self.env = env
        self.env.Load(self.path+'/models/robots/barrett_wam.dae')
        self.robot = self.env.GetRobots()[0]
        #self.Table = self.env.ReadKinBodyXMLFile('data/table.kinbody.xml')
        #self.env.Add(self.Table)
        self.obj_name = ''
        self.part = None
        self.data_saving_folder = "/home/"+user+"/grasping_data/all_obj_transformation/" 
        self.obj_num = None
        self.sub_num = None
        self.grasp_num = None
        self.is_optimal = None
        self.ext_opt_num = None # variable for recording the optimal or extreme number
        self.obj_transform = None
        self.previous_obj_name = None
        #self.contact_point_index = None
        self.robot_transform = np.genfromtxt(self.path+'/essential_files/essential_transform/Wam_transform.csv',delimiter = ',')
        #self.table_transform = np.genfromtxt(self.path+'/essential_files/essential_transform/Table_transform.csv',delimiter = ',')
        self.robot.SetTransform(self.robot_transform)
        #self.Table.SetTransform(self.table_transform)
        #self.contact_matrix = []
        if not self.env.GetCollisionChecker().SetCollisionOptions(CollisionOptions.Distance | CollisionOptions.Contacts):
            collisionChecker = RaveCreateCollisionChecker(self.env,'pqp')
            collisionChecker.SetCollisionOptions(CollisionOptions.Distance|CollisionOptions.Contacts)
            self.env.SetCollisionChecker(collisionChecker)
        self.report = CollisionReport()
        self.links = self.robot.GetLinks()
        self.finger_1_prox = self.links[12]
        self.finger_1_med = self.links[13]
        self.finger_1_dist = self.links[14]
        self.finger_2_prox = self.links[16]
        self.finger_2_med = self.links[17]
        self.finger_2_dist = self.links[18]
        self.finger_3_med = self.links[20]
        self.finger_3_dist = self.links[21]
        self.palm_link = self.links[9]
        self.palm_surface_link = self.links[11]
        self.palm_surface_tranform = self.palm_surface_link.GetTransform()
        #self.plot_points_handler = self.env.plot3(np.array([1,1,1]),2)
        self.robot_dof_limits = list(self.robot.GetDOFLimits())
        self.minDistance_of_finger = 0.0015


        self.palm_contact = ContPointWithDistance("palm contact")
        self.finger_1_prox_contact = ContPointWithDistance("finger_1_prox")
        self.finger_1_med_contact = ContPointWithDistance("finger_1_med")
        self.finger_1_dist_contact = ContPointWithDistance("finger_1_dist")
        self.finger_2_prox_contact = ContPointWithDistance("finger_2_prox")
        self.finger_2_med_contact = ContPointWithDistance("finger_2_med")
        self.finger_2_dist_contact = ContPointWithDistance("finger_2_dist")
        self.finger_3_med_contact = ContPointWithDistance("finger_3_med")
        self.finger_3_dist_contact = ContPointWithDistance("finger_3_dist")
                
        self.ContactPointWithDistance = [self.palm_contact, self.finger_1_prox_contact, self.finger_1_med_contact, self.finger_1_dist_contact, self.finger_2_prox_contact, self.finger_2_med_contact, self.finger_2_dist_contact, self.finger_3_med_contact,self.finger_3_dist_contact]
        self.points = np.array([[0,0,0]])

        # check flags for finger joint retractions

    def get_obj_name(self):
        Fid = open(self.path+"/models/stl_files/part_list.csv")
        part_list = csv.reader(Fid,delimiter = ',')
        obj_name = None
        scale = None
        for row in part_list:
            if row[0]==str(self.obj_num):
                obj_name = row[2]
                scale = row[3]
        Fid.close()
        return obj_name, scale
    
    def get_centroid(self,contact_list):
        if len(contact_list)>1:
            new_array = np.array([[0,0,0]])
            for contact in contact_list:
                new_array = np.append(new_array,[contact.pos],axis=0)
            new_array = np.delete(new_array,0,axis=0)
            return np.mean(new_array,axis=0)

        for contact in contact_list:
            return contact.pos

    def update_environment(self):
        try:
            start_time = time.time()
            self.points = np.array([[0,0,0]])
            # Save everything to file
            if not os.path.exists(self.data_saving_folder):
                os.makedirs(self.data_saving_folder)
            objno_subno = self.data_saving_folder+'obj'+str(self.obj_num)#+'_sub'+str(self.sub_num)
            if not os.path.exists(objno_subno):
                os.makedirs(objno_subno)

            # I have to do below dumb part because there is some error for recoroding contact point.
            palm_vs_part = self.env.CheckCollision(self.palm_link,self.part,report = self.report)
            self.palm_contact.ContactPoint = self.get_centroid(self.report.contacts)
            self.palm_contact.MinDistance = self.report.minDistance
            finger_1_prox_vs_part = self.env.CheckCollision(self.finger_1_prox,self.part,report=self.report)
            self.finger_1_prox_contact.ContactPoint = self.get_centroid(self.report.contacts)
            self.finger_1_prox_contact.MinDistance = self.report.minDistance
            finger_1_med_vs_part = self.env.CheckCollision(self.finger_1_med,self.part,report=self.report)
            self.finger_1_med_contact.ContactPoint = self.get_centroid(self.report.contacts)
            self.finger_1_med_contact.MinDistance = self.report.minDistance
            finger_1_dist_vs_part = self.env.CheckCollision(self.finger_1_dist,self.part,report=self.report)
            self.finger_1_dist_contact.ContactPoint = self.get_centroid(self.report.contacts)
            self.finger_1_dist_contact.MinDistance = self.report.minDistance
            finger_2_prox_vs_part = self.env.CheckCollision(self.finger_2_prox,self.part,report=self.report)
            self.finger_2_prox_contact.ContactPoint = self.get_centroid(self.report.contacts)
            self.finger_2_prox_contact.MinDistance = self.report.minDistance
            finger_2_med_vs_part = self.env.CheckCollision(self.finger_2_med,self.part,report=self.report)
            self.finger_2_med_contact.ContactPoint = self.get_centroid(self.report.contacts)
            self.finger_2_med_contact.MinDistance = self.report.minDistance
            finger_2_dist_vs_part = self.env.CheckCollision(self.finger_2_dist,self.part,report=self.report)
            self.finger_2_dist_contact.ContactPoint = self.get_centroid(self.report.contacts)
            self.finger_2_dist_contact.MinDistance = self.report.minDistance
            finger_3_med_vs_part = self.env.CheckCollision(self.finger_3_med,self.part,report=self.report)
            self.finger_3_med_contact.ContactPoint = self.get_centroid(self.report.contacts)
            self.finger_3_med_contact.MinDistance = self.report.minDistance
            finger_3_dist_vs_part = self.env.CheckCollision(self.finger_3_dist,self.part,report=self.report)
            self.finger_3_dist_contact.ContactPoint = self.get_centroid(self.report.contacts)
            self.finger_3_dist_contact.MinDistance = self.report.minDistance
            
            contact_links_names = np.array([], dtype = "|S")
            for contact in self.ContactPointWithDistance:
                if not contact.ContactPoint.all() == 0 and contact.MinDistance < self.minDistance_of_finger:
                    contact_links_names = np.append(contact_links_names,str(contact))
                    self.points = np.append(self.points,[contact.ContactPoint],axis=0)
            
            self.points = np.delete(self.points,0,axis=0)
            if not (len(self.points) == 0):
                self.plot_points = self.env.plot3(self.points,6,np.array([0,0,0]))
            np.savetxt(objno_subno+'/'+self.file_name+'_JointAngles.txt',self.robot.GetDOFValues(),delimiter=',')
            np.savetxt(objno_subno+'/'+self.file_name+'_HandTransformation.txt',self.robot.GetLinkTransformations()[9],delimiter = ',')
            np.savetxt(objno_subno+'/'+self.file_name+'_ObjTransformation.txt',self.part.GetTransform(),delimiter = ',')
            np.savetxt(objno_subno+'/'+self.file_name+'_ContactLinkNames.txt',contact_links_names,delimiter = ',',fmt = "%s")
            rospy.set_param("Things_done",True)

        except rospy.ROSInterruptException, e:
            print 'exiting', e
            sys.exit()

    def robot_updator(self,snapshot):
        rospy.set_param("Things_done",False)
        rospy.set_param("Ready_for_input",True)
        T_hand = np.array(snapshot.hand_joints.position)
        T_wam = np.array(snapshot.wam_joints.position)
        T_robot = T_wam[0:7]
        T_robot = np.append(T_robot,[0,0])
        T_robot = np.append(T_robot,[T_hand[3],T_hand[0],T_hand[4],T_hand[3],T_hand[1],T_hand[5],T_hand[2],T_hand[6]])
        self.robot.SetDOFValues(T_robot)

    def part_updator(self,get_data):
        fid= open("obj_import_error.txt",'a')
        try:
            self.obj_num = get_data.obj_num
            self.sub_num = get_data.sub_num
            self.grasp_num = get_data.grasp_num
            self.is_optimal = get_data.is_optimal
            obj_folder = ''#'obj'+str(self.obj_num)+'_sub'+str(self.sub_num)+'_pointcloud_csvfiles/'
            grasp_type = None
            if self.is_optimal:
                self.ext_opt_num = get_data.optimal_num
                grasp_type = 'optimal'
                self.file_name = 'obj'+str(self.obj_num)+'_sub'+str(self.sub_num)+'_grasp'+str(self.grasp_num)+'_optimal'+str(self.ext_opt_num)
            else:
                grasp_type = 'extreme'
                self.ext_opt_num = get_data.extreme_num
                self.file_name = 'obj'+str(self.obj_num)+'_sub'+str(self.sub_num)+'_grasp'+str(self.grasp_num)+'_extreme'+str(self.ext_opt_num)
            matrix = get_matrix(obj_transform_dir+obj_folder+self.file_name + '_object_transform.txt')
            self.obj_transform = matrix['obj_matrix']
            self.obj_name,obj_scale = self.get_obj_name()
            if (self.part == None) or (not self.obj_name == self.previous_obj_name):
                if not self.part == None:
                    self.env.Remove(self.part)
                scale_geometry = str(obj_scale)+" "+str(obj_scale)+" "+str(obj_scale)
                self.part = self.env.ReadKinBodyXMLFile(self.path+"/models/stl_files/"+self.obj_name,{'scalegeometry':scale_geometry})
                self.previous_obj_name = self.obj_name
                self.env.Add(self.part)
            self.part.SetTransform(self.obj_transform)

            fid.close()
            self.update_environment()
        except IOError, e:
            fid.writelines("Error while processing {} ".format(self.file_name))
            fid.writelines("\n\n")
            print "There is some error while processing", self.file_name
            print e
            print "continuing"
            rospy.set_param("Things_done",True)
            fid.close()




if __name__=="__main__":
    env = Environment()
    env.SetViewer('qtcoin')
    generate_grasp = valid_grasps(env)
    rospy.init_node('valid_grasp_generator',anonymous = True)
    rospy.loginfo("waiting for topic: grasp_extremes")
    rospy.wait_for_message("grasp_extremes",GraspSnapshot)
    generate_grasp.sub_robot = rospy.Subscriber("grasp_extremes",GraspSnapshot,generate_grasp.robot_updator)
    generate_grasp.sub_part = rospy.Subscriber("grasp_extremes",GraspSnapshot,generate_grasp.part_updator)
    rospy.spin()   
