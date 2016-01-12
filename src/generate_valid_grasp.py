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
    def __init__(self):
        self.path = rospkg.RosPack().get_path('valid_grasp_generator')
        self.env = Environment()
        self.env.Load(self.path+'/models/robots/barrett_wam.dae')
        self.env.SetViewer('qtcoin')
        self.robot = self.env.GetRobots()[0]
        #self.Table = self.env.ReadKinBodyXMLFile('data/table.kinbody.xml')
        #self.env.Add(self.Table)
        self.obj_name = ''
        self.part = None
        self.data_saving_folder = "/home/"+user+"/grasping_data/" 
        self.obj_num = None
        self.sub_num = None
        self.grasp_num = None
        self.is_optimal = None
        self.ext_opt_num = None # variable for recording the optimal or extreme number
        self.obj_transform = None
        self.previous_obj_name = None
        self.contact_point_index = None
        self.robot_transform = np.genfromtxt(self.path+'/essential_files/essential_transform/Wam_transform.csv',delimiter = ',')
        #self.table_transform = np.genfromtxt(self.path+'/essential_files/essential_transform/Table_transform.csv',delimiter = ',')
        self.robot.SetTransform(self.robot_transform)
        #self.Table.SetTransform(self.table_transform)
        self.contact_matrix = []
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
        self.plot_points = self.env.plot3([1,2,3], 2)
        self.plot_points_handler = self.env.plot3(np.array([1,1,1]),2)
        self.COG_part = np.array([])
        self.robot_dof_limits = list(self.robot.GetDOFLimits())
        self.mapper = interp1d([0,self.robot_dof_limits[1][10]+self.robot_dof_limits[1][11]],[0,self.robot_dof_limits[1][10]])
        self.hand_quaternion = np.array([0,0,0,0])
        self.hand_postion = np.array([0,0,0])
        self.file_name = ''
        self.translation_done = False
        self.translational_threshold = 0.0005
        self.joint_retract_threshold = 0.003
        self.finger_retracted = False
        self.flag_palm = False
        self.flag_finger_1 = False
        self.flag_finger_2 = False
        self.robot_all_link_collision_check = np.array([False,False,False,False,False,False,False,False,False],dtype=bool)

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

        self.minDistance_of_finger = 0.001
        
        # check flags for finger joint retractions
        self.flag_finger_1_dist = False
        self.flag_finger_1_med = False
        self.flag_finger_2_dist = False
        self.flag_finger_2_med = False
        self.flag_finger_3_dist = False
        self.flag_finger_3_med = False

    def get_obj_name(self):
        Fid = open(self.path+"/models/stl_files/part_list.csv")
        part_list = csv.reader(Fid,delimiter = ',')
        obj_name = None
        for row in part_list:
            if row[0]==str(self.obj_num):
                obj_name = row[2]
        Fid.close()
        return obj_name
    
    def get_unit_vector(self,pt1,pt2):
        x = pt1[0] - pt2[0]
        y = pt1[1] - pt2[1]
        z = pt1[2] - pt2[2]
        mag = np.sqrt(x**2 + y**2 + z**2)
        return [x/mag,y/mag,z/mag]

    def set_hand_joint_angles(self,joint_angles):
        # This function is specific to this class.
        dist_mapper = interp1d([0,self.robot_dof_limits[1][10]],[0,self.robot_dof_limits[1][11]])
        robot_all_dof_values = self.robot.GetDOFValues()
        robot_all_dof_values[10:18] = [joint_angles[0],dist_mapper(joint_angles[0]),robot_all_dof_values[12],joint_angles[1],dist_mapper(joint_angles[1]),joint_angles[2],dist_mapper(joint_angles[2])]
        self.robot.SetDOFValues(robot_all_dof_values)


    def update_environment(self):
        try:
            #vector = libdepth_penetration.get_penetration(self.part.GetName())
            self.points = np.array([[0,0,0]])
            while not self.finger_retracted:
                self.robot.SetVisible(1)
                #enter_to_continue = raw_input("Press Enter to continue: ")
                print #add line
                self.plot_points_handler.Close()
                part_link = self.part.GetLinks()[0]
                part_points = part_link.GetCollisionData().vertices
                part_link_pose = poseFromMatrix(self.part.GetTransform())
                new_part_points = poseTransformPoints(part_link_pose, part_points)
                self.plot_points_handler = self.env.plot3(new_part_points,3)
                self.COG_part = np.mean(new_part_points,axis =0)
                active_dof = self.robot.GetDOFValues()
                hand_dof = active_dof[9:18]
                finger_1_dof_value = self.mapper(hand_dof[1]+hand_dof[2])
                finger_2_dof_value = self.mapper(hand_dof[4]+hand_dof[5])
                finger_3_dof_value = self.mapper(hand_dof[6]+hand_dof[7])
                finger_spread = hand_dof[0]
                output_dof_vals = np.array([finger_1_dof_value,finger_2_dof_value,finger_3_dof_value,finger_spread])
                current_hand_transform = self.robot.GetLinkTransformations()[9]
                self.hand_position = current_hand_transform[0:3,3]

                euler_angles = mat2euler(current_hand_transform[0:3,0:3])
                self.hand_quaternion = euler2quat(euler_angles[0],euler_angles[1],euler_angles[2])

                rospy.loginfo("Got grasp_extremes")
                self.plot_points.Close()
           

                # variables for changing joint angles values
                finger_1_joint_angles = finger_1_dof_value
                finger_2_joint_angles = finger_2_dof_value
                finger_3_joint_angles = finger_3_dof_value

                # Translation of the object away from the palm

                palm_vs_part = self.env.CheckCollision(self.part,self.palm_link,report = self.report)
                self.palm_contact.MinDistance = self.report.minDistance
                self.robot_all_link_collision_check[0] = palm_vs_part
                print "palm",palm_vs_part
                
                contact_points = self.report.contacts
                contact_points_list = np.array([[0,0,0]])
                if palm_vs_part:
                    self.flag_palm = False
                    for contact in contact_points:
                        contact_points_list = np.append(contact_points_list, [contact.pos],axis = 0)

                    contact_points_list = np.delete(contact_points_list, 0,axis=0)
                    self.palm_contact.ContactPoint = np.mean(contact_points_list,axis=0)

                    translation_unit_vector = self.get_unit_vector(self.COG_part, self.palm_contact.ContactPoint)
                    part_transform = self.part.GetTransform()
                    part_transform[0,3] = (part_transform[0,3] + self.translational_threshold*translation_unit_vector[0])
                    part_transform[1,3] = (part_transform[1,3] + self.translational_threshold*translation_unit_vector[1])
                    part_transform[2,3] = (part_transform[2,3] + self.translational_threshold*translation_unit_vector[2])
                    self.part.SetTransform(part_transform)
                else:
                    self.flag_palm = True
                finger_1_prox_vs_part = self.env.CheckCollision(self.part,self.finger_1_prox,report = self.report)
                self.finger_1_prox_contact.MinDistance = self.report.minDistance
                self.robot_all_link_collision_check[1]=finger_1_prox_vs_part
                contact_points = self.report.contacts
                contact_points_list = np.array([[0,0,0]])

                if finger_1_prox_vs_part:
                    self.flag_finger_1 = False
                    for contact in contact_points:
                        contact_points_list = np.append(contact_points_list, [contact.pos],axis = 0)

                    contact_points_list = np.delete(contact_points_list, 0,axis=0)
                    self.finger_1_prox_contact.ContactPoint = np.mean(contact_points_list,axis = 0)
                    translation_unit_vector = self.get_unit_vector(self.COG_part, self.finger_1_prox_contact.ContactPoint)
                    part_transform = self.part.GetTransform()
                    part_transform[0,3] = (part_transform[0,3] + self.translational_threshold*translation_unit_vector[0])
                    part_transform[1,3] = (part_transform[1,3] + self.translational_threshold*translation_unit_vector[1])
                    part_transform[2,3] = (part_transform[2,3] + self.translational_threshold*translation_unit_vector[2])
                    self.part.SetTransform(part_transform)
                else:
                    self.flag_finger_1 = True
                finger_2_prox_vs_part = self.env.CheckCollision(self.part,self.finger_2_prox,report=self.report)                 
                self.finger_2_prox_contact.MinDistance = self.report.minDistance
                self.robot_all_link_collision_check[2]=finger_2_prox_vs_part
                contact_points_list = np.array([0,0,0])
                if finger_2_prox_vs_part:
                    self.flag_finger_2 = False
                    for contact in contact_points:
                        contact_points_list = np.append(contact_points_list, [contact.pos],axis = 0)

                    contact_points_list = np.delete(contact_points_list, 0,axis=0)
                    self.finger_2_prox_contact.ContactPoint = np.mean(contact_points_list,axis = 0)
                    
                    translation_unit_vector = self.get_unit_vector(self.COG_part, self.finger_2_prox_contact.ContactPoint)
                    part_transform = self.part.GetTransform()
                    part_transform[0,3] = (part_transform[0,3] + self.translational_threshold*translation_unit_vector[0])
                    part_transform[1,3] = (part_transform[1,3] + self.translational_threshold*translation_unit_vector[1])
                    part_transform[2,3] = (part_transform[2,3] + self.translational_threshold*translation_unit_vector[2])
                    self.part.SetTransform(part_transform)
                else:
                    self.flag_finger_2 = True
                

                if self.flag_palm and self.flag_finger_1 and self.flag_finger_2:
                    self.translation_done = True

                # Move the fingers away from the object
                if self.translation_done:
                    finger_1_med_vs_part = self.env.CheckCollision(self.part,self.finger_1_med,report = self.report)
                    self.finger_1_med_contact.MinDistance = self.report.minDistance
                    self.robot_all_link_collision_check[3]=finger_1_med_vs_part
                    print "finger 1 med", finger_1_med_vs_part
                    
                    contact_points = self.report.contacts
                    contact_points_list = np.array([[0,0,0]])
                    if self.finger_1_med_contact.MinDistance < self.minDistance_of_finger and not finger_1_med_vs_part:
                        for contact in contact_points:
                            contact_points_list = np.append(contact_points_list, [contact.pos],axis = 0)

                        contact_points_list = np.delete(contact_points_list, 0,axis=0)
                        self.finger_1_med_contact.ContactPoint = np.mean(contact_points_list, axis=0)
                        self.flag_finger_1_med = True
                    elif not self.flag_finger_1_med and finger_1_med_vs_part:
                        finger_1_joint_angles = finger_1_joint_angles -  self.joint_retract_threshold
                        self.set_hand_joint_angles([finger_1_joint_angles,finger_2_joint_angles,finger_3_joint_angles])

                    finger_1_dist_vs_part = self.env.CheckCollision(self.part,self.finger_1_dist,report = self.report)
                    self.finger_1_dist_contact.MinDistance = self.report.minDistance
                    self.robot_all_link_collision_check[4]=finger_1_dist_vs_part
                    print "finger 1 distal",finger_1_dist_vs_part
                    
                    contact_points = self.report.contacts
                    contact_points_list = np.array([[0,0,0]])
                    if self.finger_1_dist_contact.MinDistance < self.minDistance_of_finger and not finger_1_dist_vs_part:
                        for contact in contact_points:
                            contact_points_list = np.append(contact_points_list, [contact.pos],axis = 0)

                        contact_points_list = np.delete(contact_points_list, 0,axis=0)
                        self.finger_1_dist_contact.ContactPoint = np.mean(contact_points_list,axis=0)
                        self.flag_finger_1_dist = True
                    elif not self.flag_finger_1_dist and finger_1_dist_vs_part:
                        finger_1_joint_angles = finger_1_joint_angles - self.joint_retract_threshold
                        self.set_hand_joint_angles([finger_1_joint_angles,finger_2_joint_angles,finger_3_joint_angles])

                    finger_2_med_vs_part = self.env.CheckCollision(self.part,self.finger_2_med,report = self.report)
                    self.finger_2_med_contact.MinDistance = self.report.minDistance
                    self.robot_all_link_collision_check[5]=finger_2_med_vs_part
                    print "finger 2 med",finger_2_med_vs_part
                    
                    contact_points = self.report.contacts
                    contact_points_list = np.array([[0,0,0]])
                    if self.finger_2_med_contact.MinDistance < self.minDistance_of_finger and not finger_2_med_vs_part:
                        for contact in contact_points:
                            contact_points_list = np.append(contact_points_list, [contact.pos],axis = 0)

                        contact_points_list = np.delete(contact_points_list, 0,axis=0)
                        self.finger_2_med_contact.ContactPoint = np.mean(contact_points_list,axis=0)
                        self.flag_finger_2_med = True
                    elif not self.flag_finger_2_med and finger_2_med_vs_part:
                        finger_2_joint_angles = finger_2_joint_angles - self.joint_retract_threshold
                        self.set_hand_joint_angles([finger_1_joint_angles,finger_2_joint_angles,finger_3_joint_angles])


                    finger_2_dist_vs_part = self.env.CheckCollision(self.part,self.finger_2_dist,report = self.report)
                    self.finger_2_dist_contact.MinDistance = self.report.minDistance
                    self.robot_all_link_collision_check[6]=finger_2_dist_vs_part
                    print "finger 2 dist",finger_2_dist_vs_part
                    
                    contact_points = self.report.contacts
                    contact_points_list = np.array([[0,0,0]])
                    if self.finger_2_dist_contact.MinDistance < self.minDistance_of_finger and not finger_2_dist_vs_part:
                        for contact in contact_points:
                            contact_points_list = np.append(contact_points_list, [contact.pos],axis = 0)

                        contact_points_list = np.delete(contact_points_list, 0,axis=0)
                        self.finger_2_dist_contact.ContactPoint = np.mean(contact_points_list,axis=0)
                        self.flag_finger_2_dist = True
                    elif not self.flag_finger_2_dist and finger_2_dist_vs_part:
                        finger_2_joint_angles = finger_2_joint_angles - self.joint_retract_threshold
                        self.set_hand_joint_angles([finger_1_joint_angles,finger_2_joint_angles,finger_3_joint_angles])

                    finger_3_med_vs_part = self.env.CheckCollision(self.part,self.finger_3_med,report = self.report)
                    self.finger_3_med_contact.MinDistance = self.report.minDistance
                    self.robot_all_link_collision_check[7]=finger_3_med_vs_part
                    print "finger 3 med",finger_3_med_vs_part
                    
                    contact_points = self.report.contacts
                    contact_points_list = np.array([[0,0,0]])
                    if self.finger_3_med_contact.MinDistance < self.minDistance_of_finger and not finger_3_med_vs_part:
                        for contact in contact_points:
                            contact_points_list = np.append(contact_points_list, [contact.pos],axis = 0)

                        contact_points_list = np.delete(contact_points_list, 0,axis=0)
                        self.finger_3_med_contact.ContactPoint = np.mean(contact_points_list,axis=0)
                        self.flag_finger_3_med = True
                    elif not self.flag_finger_3_med and finger_3_med_vs_part:
                        finger_3_joint_angles = finger_3_joint_angles - self.joint_retract_threshold
                        self.set_hand_joint_angles([finger_1_joint_angles,finger_2_joint_angles,finger_3_joint_angles])

                    finger_3_dist_vs_part = self.env.CheckCollision(self.part,self.finger_3_dist,report = self.report)
                    self.finger_3_dist_contact.MinDistance = self.report.minDistance                                                 
                    self.robot_all_link_collision_check[8]=finger_3_dist_vs_part
                    print "finger 3 dist",finger_3_dist_vs_part
                    
                    contact_points = self.report.contacts
                    contact_points_list = np.array([[0,0,0]])
                    if self.finger_3_dist_contact.MinDistance < 0.002 and not finger_3_dist_vs_part:
                        for contact in contact_points:
                            contact_points_list = np.append(contact_points_list, [contact.pos],axis = 0)

                        contact_points_list = np.delete(contact_points_list, 0,axis=0)
                        self.finger_3_dist_contact.ContactPoint = np.mean(contact_points_list,axis=0)
                        self.flag_finger_3_dist = True
                    elif not self.flag_finger_3_dist and finger_3_dist_vs_part:
                        finger_3_joint_angles = finger_3_joint_angles - self.joint_retract_threshold
                        self.set_hand_joint_angles([finger_1_joint_angles,finger_2_joint_angles,finger_3_joint_angles])

                    
                    # I am using palm contact for finger 3 proximal joint. I did this because finger 3 proximal is the part of palm
                    print "finger 3 Proximal",palm_vs_part
                    

                    #overall_collision_check = self.env.CheckCollision(self.robot,report = self.report)
                    print "overall robot collision with part: ",self.robot_all_link_collision_check.any()
                    if not (self.robot_all_link_collision_check.any()):
                        print "Fingers Retracted"
                        self.finger_retracted = True
                    #elif overall_collision_check:
                    #    if self.report.plink2.GetParent().GetName() == ''

           
            

            # Save everything to file
            if not os.path.exists(self.data_saving_folder):
                os.makedirs(self.data_saving_folder)
            objno_subno = self.data_saving_folder+'obj'+str(self.obj_num)+'_sub'+str(self.sub_num)
            if not os.path.exists(objno_subno):
                os.makedirs(objno_subno)

            print "self.palm_contact : ",self.palm_contact.ContactPoint,",  ", self.palm_contact.MinDistance
            print "self.finger_1_prox: ",self.finger_1_prox_contact.ContactPoint,",  ",self.finger_1_prox_contact.MinDistance
            print "self.finger_1_med_: ",self.finger_1_med_contact.ContactPoint ,",  ",self.finger_1_med_contact.MinDistance
            print "self.finger_1_dist: ",self.finger_1_dist_contact.ContactPoint,",  ",self.finger_1_dist_contact.MinDistance
            print "self.finger_2_prox: ",self.finger_2_prox_contact.ContactPoint,",  ",self.finger_2_prox_contact.MinDistance
            print "self.finger_2_med_: ",self.finger_2_med_contact.ContactPoint ,",  ",self.finger_2_med_contact.MinDistance
            print "self.finger_2_dist: ",self.finger_2_dist_contact.ContactPoint,",  ",self.finger_2_dist_contact.MinDistance
            print "self.finger_3_med_: ",self.finger_3_med_contact.ContactPoint ,",  ",self.finger_3_med_contact.MinDistance
            print "self.finger_3_dist: ",self.finger_3_dist_contact.ContactPoint,",  ",self.finger_3_dist_contact.MinDistance

            print "Links that were in contact: "
            for contact in self.ContactPointWithDistance:
                if not contact.ContactPoint.all() == 0 and contact.MinDistance < self.minDistance_of_finger * 10:
                    print str(contact)
                    self.points = np.append(self.points,[contact.ContactPoint],axis=0)
            
            self.points = np.delete(self.points,0,axis=0)
            print
            print "Points that are plotted", self.points
            self.plot_points = self.env.plot3(self.points,6,np.array([0,0,0]))
            np.savetxt(objno_subno+'/'+self.file_name+'_COG.txt',self.COG_part,delimiter=',')
            np.savetxt(objno_subno+'/'+self.file_name+'_hand_position.txt',self.hand_position,delimiter=',')
            np.savetxt(objno_subno+'/'+self.file_name+'_hand_quaternion.txt',self.hand_quaternion,delimiter=',')
            np.savetxt(objno_subno+'/'+self.file_name+'_closeddofvals.txt',output_dof_vals,delimiter=',')
            np.savetxt(objno_subno+'/'+self.file_name+'_contactpoints.txt',self.points)
            self.robot.SetVisible(1)
            
        except rospy.ROSInterruptException, e:
            print 'exiting', e
            sys.exit()

    def robot_updator(self,snapshot):
        T_hand = np.array(snapshot.hand_joints.position)
        T_wam = np.array(snapshot.wam_joints.position)
        T_robot = T_wam[0:7]
        T_robot = np.append(T_robot,[0,0])
        T_robot = np.append(T_robot,[T_hand[3],T_hand[0],T_hand[4],T_hand[3],T_hand[1],T_hand[5],T_hand[2],T_hand[6]])
        self.robot.SetDOFValues(T_robot)

    def part_updator(self,get_data):
        self.obj_num = get_data.obj_num
        self.sub_num = get_data.sub_num
        self.grasp_num = get_data.grasp_num
        self.is_optimal = get_data.is_optimal
        obj_folder = 'obj'+str(self.obj_num)+'_sub'+str(self.sub_num)+'_pointcloud_csvfiles/'
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
        grasp_all_contact_file = obj_transform_dir+obj_folder+'obj'+str(self.obj_num)+'_sub'+str(self.sub_num)+'_all_grasps_contact_points.csv'
        contact_matrix = get_contact_values(grasp_all_contact_file)
        self.contact_matrix = np.array(contact_matrix)
        index_matrix = self.contact_matrix[:,0:5] == [str(self.obj_num),str(self.sub_num),str(self.grasp_num),grasp_type,str(self.ext_opt_num)]
        self.contact_point_index = get_index(index_matrix)
        self.obj_transform = matrix['obj_matrix']
        self.obj_name = self.get_obj_name()
        if (self.part == None) or (not self.obj_name == self.previous_obj_name):
            if not self.part == None:
                self.env.Remove(self.part)
            self.part = self.env.ReadKinBodyXMLFile(self.path+"/models/stl_files/"+self.obj_name,{'scalegeometry':'0.001 0.001 0.001'})
            self.previous_obj_name = self.obj_name
            self.env.Add(self.part)
        self.part.SetTransform(self.obj_transform)
        self.finger_retracted = False
        self.translation_done = False
        self.flag_palm = False
        self.flag_finger_1 = False
        self.flag_finger_2 = False

        self.flag_finger_1_dist = False
        self.flag_finger_1_med = False
        self.flag_finger_2_dist = False
        self.flag_finger_2_med = False
        self.flag_finger_3_dist = False
        self.flag_finger_3_med = False

        self.update_environment()


if __name__=="__main__":

    generate_grasp = valid_grasps()
    rospy.init_node('valid_grasp_generator',anonymous = True)
    rospy.loginfo("waiting for topic: grasp_extremes")
    rospy.wait_for_message("grasp_extremes",GraspSnapshot)
    generate_grasp.sub_robot = rospy.Subscriber("grasp_extremes",GraspSnapshot,generate_grasp.robot_updator)
    generate_grasp.sub_part = rospy.Subscriber("grasp_extremes",GraspSnapshot,generate_grasp.part_updator)
    rospy.spin()   
