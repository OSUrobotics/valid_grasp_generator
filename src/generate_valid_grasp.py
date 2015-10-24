#!/usr/bin/env python
from openravepy import *
import rospy
import rospkg
import sys,getopt
import csv
#from and_gate import *
from get_all_contact_values import *
from grasp_manager.msg import GraspSnapshot
from shared_global import *
from get_matrix import *
import numpy as np

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
        self.no_samples = rospy.get_param('no_of_samples')
        self.sampling_delta = None
        self.robot_transform = np.genfromtxt(self.path+'/transformation_matrices/essential_transform/Wam_transform.csv',delimiter = ',')
        self.table_transform = np.genfromtxt(self.path+'/transformation_matrices/essential_transform/Table_transform.csv',delimiter = ',')
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
            rospy.loginfo("Got grasp_extremes")
            #while True:
            #    finger_1_prox_vs_part = self.env.CheckCollision(self.part,self.finger_1_prox,report=self.report)
            #    dist_finger_1_prox_vs_part = self.report.minDistance
            #    finger_1_med_vs_part = self.env.CheckCollision(self.part,self.finger_1_med,report = self.report)
            #    dist_finger_1_med_vs_part = self.report.minDistance
            #    finger_1_dist_vs_part = self.env.CheckCollision(self.part,self.finger_1_dist,report = self.report)
            #    dist_finger_1_dist_vs_part = self.report.minDistance
            #    finger_2_prox_vs_part = self.env.CheckCollision(self.part,self.finger_2_prox,report=self.report)                 
            #    dist_finger_2_prox_vs_part = self.report.minDistance
            #    finger_2_med_vs_part = self.env.CheckCollision(self.part,self.finger_2_med,report = self.report)
            #    dist_finger_2_med_vs_part = self.report.minDistance
            #    finger_2_dist_vs_part = self.env.CheckCollision(self.part,self.finger_2_dist,report = self.report)
            #    dist_finger_1_dist_vs_part = self.report.minDistance
            #    finger_3_med_vs_part = self.env.CheckCollision(self.part,self.finger_3_med,report = self.report)
            #    dist_finger_3_med_vs_part = self.report.minDistance
            #    finger_3_dist_vs_part = self.env.CheckCollision(self.part,self.finger_3_dist,report = self.report)
            #    dist_finger_3_dist_vs_part = self.report.minDistance                                                 
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
        print "index matrix",index_matrix
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
        self.points_inside_obj = self.get_points()

    def get_points(self):
        cdmodel = databases.convexdecomposition.ConvexDecompositionModel(self.part)
        if not cdmodel.load():
            cdmodel.autogenerate()
        ab = self.part.ComputeAABB()
        boxmin = ab.pos() - ab.extents()
        boxmax = ab.pos()+ ab.extents()
        self.sampling_delta = np.linalg.norm(ab.extents())/self.no_samples
        X,Y,Z = numpy.mgrid[boxmin[0]:boxmax[0]:self.sampling_delta,boxmin[1]:boxmax[1]:self.sampling_delta,boxmin[2]:boxmax[2]:self.sampling_delta]
        points = np.c_[X.flat,Y.flat,Z.flat]
        tested_points = cdmodel.testPointsInside(points)
        inside_points = points[np.flatnonzero(tested_points),:]
        return inside_points
        
        
        

if __name__=="__main__":
    generate_grasp = valid_grasps()
    rospy.init_node('valid_grasp_generator',anonymous = True)
    rospy.loginfo("waiting for topic: grasp_extremes")
    generate_grasp.sub_robot = rospy.Subscriber("grasp_extremes",GraspSnapshot,generate_grasp.robot_updator)
    generate_grasp.sub_part = rospy.Subscriber("grasp_extremes",GraspSnapshot,generate_grasp.part_updator)
    rospy.wait_for_message("grasp_extremes",GraspSnapshot)
    generate_grasp.update_environment()
    rospy.spin()
