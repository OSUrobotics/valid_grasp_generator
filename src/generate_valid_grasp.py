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
import timeit

class valid_grasps():
    def __init__(self):
        self.path = rospkg.RosPack().get_path('valid_grasp_generator')
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
        self.contact_point_index = None
        self.points_inside_obj = None
        self.increment = rospy.get_param('increment_value')
        self.max_translational_limit = rospy.get_param('translational_limit')
        self.robot_transform = np.genfromtxt(self.path+'/essential_files/essential_transform/Wam_transform.csv',delimiter = ',')
        self.table_transform = np.genfromtxt(self.path+'/essential_files/essential_transform/Table_transform.csv',delimiter = ',')
        self.sphere_points = np.genfromtxt(self.path+'/essential_files/sphere_points.csv',delimiter = ',')
        self.robot.SetTransform(self.robot_transform)
        self.Table.SetTransform(self.table_transform)
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
        self.palm_surface_point = self.palm_surface_tranform[0:3,3]
        self.points = np.array([])
        self.count_inside_points = None
        self.part_cdmodel = None
        self.plot_points = self.env.plot3([1,2,3], 2)
        self.satisfactory_finger_position = False

    def get_obj_name(self):
        Fid = open(self.path+"/models/stl_files/part_list.csv")
        part_list = csv.reader(Fid,delimiter = ',')
        obj_name = None
        for row in part_list:
            if row[0]==str(self.obj_num):
                obj_name = row[2]
        Fid.close()
        return obj_name

    def update_environment(self):
        try:
            #self.part_cdmodel =  databases.convexdecomposition.ConvexDecompositionModel(self.part)
            #self.palm_surface_tranform = self.palm_surface_link.GetTransform()
            #self.palm_surface_point = self.palm_surface_tranform[0:3,3]
            #self.points = self.sphere_points + self.palm_surface_point
            #self.plot_points = self.env.plot3(self.points,2)
            #if not self.part_cdmodel.load():
            #    self.part_cdmodel.autogenerate()            
            #start = timeit.default_timer()
            rospy.loginfo("Got grasp_extremes")
            #end = timeit.default_timer()
            #print "time taken for getting points", end-start
            #recommended_transform = self.part.GetTransform()
            #initial_transform = np.dot(recommended_transform,np.eye(4))
            ##self.plot_points = self.env.plot3(self.points_inside_obj,2)
            ##self.plot_points.SetShow(0)
            #self.robot_cdmodel = databases.convexdecomposition.ConvexDecompositionModel(self.robot)
            #if not self.robot_cdmodel.load():
            #    self.robot_cdmodel.autogenerate()
            self.plot_points.Close()
            self.points = np.array([])
            collision_test = self.env.CheckCollision(self.part, self.robot, report = self.report)
            #print "minimum distance between robot and part", self.report.minDistance
            for contact in self.report.contacts:
                #print "contacts between robot and part", contact.pos
                self.points = np.append(self.points,np.array(contact.pos))
                #print "minimum distance between robot and part", contact.depth

            self.plot_points = self.env.plot3(self.points, 2)
            #while not self.satisfactory_finger_position:
            #    finger_1_prox_vs_part = self.env.CheckCollision(self.part,self.finger_1_prox,report = self.report)
            #    dist_finger_1_prox_vs_part = self.report.minDistance
            #    if finger_1_prox_vs_part:
            #for i in range(-1*self.max_translational_limit, self.max_translational_limit, self.increment):#(0,10,1)
            #    for j in range(-1*self.max_translational_limit, self.max_translational_limit, self.increment):
            #        for k in range(-1*self.max_translational_limit,self.max_translational_limit, self.increment):
            #            end = timeit.default_timer()
            #            print "time taken for above operation", (end-start)
            #            previous_transform = recommended_transform
            #
            #            #print "Previous transform ",previous_transform
            #            tested_points_obj= self.part_cdmodel.testPointsInside(self.points) 
            #            self.points_inside_obj = self.points[np.flatnonzero(tested_points_obj),:]
            #            tested_previous_points = self.robot_cdmodel.testPointsInside(self.points_inside_obj)
            #            previous_points = self.points_inside_obj[np.flatnonzero(tested_previous_points),:]
            #            previous_counts = len(previous_points)
            #            
            #            new_transform =np.add(initial_transform,np.zeros((4,4))) 
            #            print "i "+str(i)+" j "+str(j)+" k "+str(k)
            #            new_transform[0,3] = new_transform[0,3] + i/1000.00
            #            new_transform[1,3] = new_transform[1,3] + j/1000.00
            #            new_transform[2,3] = new_transform[2,3] + k/1000.00
            #            self.part.SetTransform(new_transform)
            #            print new_transform


            #            
            #            tested_point_obj = self.part_cdmodel.testPointsInside(self.points)
            #            self.points_inside_obj = self.points[np.flatnonzero(tested_points_obj),:]
            #            tested_new_points = self.robot_cdmodel.testPointsInside(self.points_inside_obj)
            #            new_points = self.points_inside_obj[np.flatnonzero(tested_new_points),:]
            #            new_counts = len(new_points)
            #            
            #            
            #            if new_counts<=previous_counts:
            #                recommended_transform = new_transform
            #            
            #            #print "previous_transform after new_transform",previous_transform


            #            self.part.SetTransform(recommended_transform)
            #            current_joint_value = self.contact_matrix[self.contact_point_index]
            #       
            #            finger_1_prox_vs_part = self.env.CheckCollision(self.part,self.finger_1_prox,report = self.report)
            #            dist_finger_1_prox_vs_part = self.report.minDistance
            #            
            #            print "finger 1 prox", finger_1_prox_vs_part
            #            if (finger_1_prox_vs_part == True and current_joint_value[11] == '1') or (finger_1_prox_vs_part == False and current_joint_value[11] == '0'):
            #                pass
            #            else:
            #                print "continued due to finger 1 proximal link"
            #                recommended_transform = previous_transform
            #                continue

            #            finger_1_med_vs_part = self.env.CheckCollision(self.part,self.finger_1_med,report = self.report)
            #            dist_finger_1_med_vs_part = self.report.minDistance
            #            print "finger 1 med", finger_1_med_vs_part

            #            if (finger_1_med_vs_part == True and current_joint_value[8] == '1') or (finger_1_med_vs_part == False and current_joint_value[8] == '0'):
            #                pass
            #            else:
            #                recommended_transform = previous_transform
            #                print "continued due to finger 1 medial link"
            #                continue

            #            finger_1_dist_vs_part = self.env.CheckCollision(self.part,self.finger_1_dist,report = self.report)
            #            dist_finger_1_dist_vs_part = self.report.minDistance
            #            print "finger 1 distal",finger_1_dist_vs_part

            #            if (finger_1_dist_vs_part == True and current_joint_value[5] == '1') or (finger_1_dist_vs_part == False and current_joint_value[5] == '0'):
            #                pass
            #            else:
            #                recommended_transform = previous_transform
            #                print "continued due to finger 1 distal link"
            #                continue

            #            finger_2_prox_vs_part = self.env.CheckCollision(self.part,self.finger_2_prox,report=self.report)                 
            #            dist_finger_2_prox_vs_part = self.report.minDistance
            #            print "finger 2 proximal",finger_2_prox_vs_part
            #            
            #            if (finger_2_prox_vs_part == True and current_joint_value[12] == '1') or (finger_2_prox_vs_part == False and current_joint_value[12] == '0'):
            #                pass
            #            else:
            #                recommended_transform = previous_transform
            #                print "continued due to finger 2 proximal link"
            #                continue

            #            finger_2_med_vs_part = self.env.CheckCollision(self.part,self.finger_2_med,report = self.report)
            #            dist_finger_2_med_vs_part = self.report.minDistance
            #            print "finger 2 med",finger_2_med_vs_part
            #            
            #            if (finger_2_med_vs_part == True and current_joint_value[9] == '1') or (finger_2_med_vs_part == False and current_joint_value[9] == '0'):
            #                pass
            #            else:
            #                recommended_transform = previous_transform
            #                print "continued due to finger 2 medial link"
            #                continue

            #            finger_2_dist_vs_part = self.env.CheckCollision(self.part,self.finger_2_dist,report = self.report)
            #            dist_finger_1_dist_vs_part = self.report.minDistance
            #            print "finger 2 dist",finger_2_dist_vs_part
            #            
            #            if (finger_2_dist_vs_part == True and current_joint_value[6] == '1') or (finger_2_dist_vs_part == False and current_joint_value[6] == '0'):                       
            #                 pass   
            #            else:
            #                 recommended_transform = previous_transform
            #                 print "continued due to finger 2 distal link"
            #                 continue
            #            
            #            finger_3_med_vs_part = self.env.CheckCollision(self.part,self.finger_3_med,report = self.report)
            #            dist_finger_3_med_vs_part = self.report.minDistance
            #            print "finger 3 med",finger_3_med_vs_part

            #            if (finger_3_med_vs_part == True and current_joint_value[10] == '1') or (finger_3_med_vs_part == False and current_joint_value[10] == '0'):                       
            #                 pass   
            #            else:
            #                recommended_transform = previous_transform
            #                print "continued due to finger 3 medial link"

            #                continue

            #            finger_3_dist_vs_part = self.env.CheckCollision(self.part,self.finger_3_dist,report = self.report)
            #            dist_finger_3_dist_vs_part = self.report.minDistance                                                 
            #            print "finger 3 dist",finger_3_dist_vs_part

            #            if (finger_3_dist_vs_part == True and current_joint_value[7] == '1') or (finger_3_dist_vs_part == False and current_joint_value[7] == '0'):                       
            #                 pass   
            #            else:
            #                recommended_transform = previous_transform
            #                print "continued due to finger 3 distal link"
            #                continue                                                                                                                                                                  
            #            palm_vs_part = self.env.CheckCollision(self.part,self.palm_link,report = self.report)
            #            dist_palm_vs_part = self.report.minDistance
            #            print "palm",palm_vs_part

            #            if (palm_vs_part == True and current_joint_value[14] == '1')  or (palm_vs_part == False and current_joint_value[14] == '0'):
            #                pass
            #            else:
            #                recommended_transform = previous_transform
            #                print 'continued due to palm'
            #                continue
            #           
            #            # I am using palm contact for finger 3 proximal joint. I did this because finger 3 proximal is the part of palm
            #            print "finger 3 Proximal",palm_vs_part
            #            if (palm_vs_part == True and current_joint_value[13] == '1') or (palm_vs_part == False and current_joint_value[13] == '0'):
            #                pass
            #            else:
            #                recommended_transform = previous_transform
            #                print 'continued due to finger 3 proximal joint'
            #            
            #
            #self.part.SetTransform(recommended_transform)
            #end = timeit.default_timer()
            #print "Done !! time taken: ", (end-start)
            #print "output transformation matrix ",recommended_transform
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
            file_name = obj_transform_dir+obj_folder+'obj'+str(self.obj_num)+'_sub'+str(self.sub_num)+'_grasp'+str(self.grasp_num)+'_optimal'+str(self.ext_opt_num)+'_object_transform.txt'
        else:
            grasp_type = 'extreme'
            self.ext_opt_num = get_data.extreme_num
            file_name = obj_transform_dir+obj_folder+'obj'+str(self.obj_num)+'_sub'+str(self.sub_num)+'_grasp'+str(self.grasp_num)+'_extreme'+str(self.ext_opt_num)+'_object_transform.txt'
        matrix = get_matrix(file_name)
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
        self.update_environment()


if __name__=="__main__":
    generate_grasp = valid_grasps()
    rospy.init_node('valid_grasp_generator',anonymous = True)
    rospy.loginfo("waiting for topic: grasp_extremes")
    rospy.wait_for_message("grasp_extremes",GraspSnapshot)
    generate_grasp.sub_robot = rospy.Subscriber("grasp_extremes",GraspSnapshot,generate_grasp.robot_updator)
    generate_grasp.sub_part = rospy.Subscriber("grasp_extremes",GraspSnapshot,generate_grasp.part_updator)
    #generate_grasp.update_environment()
    rospy.spin()   
