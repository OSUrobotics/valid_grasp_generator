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
from Tkinter import *
from symmetricize import reflect_along_x_plane, reflect_along_y_plane, reflect_along_z_plane

transform_path = os.path.expanduser("~") + "/grasping_data"
#transform_path = os.path.expanduser("~") + "/bad_grasps"
ctrl = None

if __name__ == "__main__":
    rospy.init_node('similar_grasp_generator',anonymous = True)
    ctrl = object_visualizer()
    check_contact_links = True
    threshold = 0.5
    threshold_rotation = 0.1
    #take_image = rospy.ServiceProxy('take_snap_shot', SnapShot)
    #alignment_viewer_sub = rospy.Subscriber("/openrave_grasp_view", Int32MultiArray, view_alignment_cb)
    folder_name = transform_path+'/output_data_folder/'
    folder_list = os.listdir(folder_name)
     
    for folder in folder_list:
        fold = folder.split('_')[0]
        cluster_no = folder.split('_')[1]
        obj_num = int(fold[3:])
        if obj_num in [2,4,5,15,17]:
            files = os.listdir(folder_name + "obj" +str(obj_num)+"_"+str(cluster_no) +"/")
            ctrl.set_obj(obj_num)
            sorted_files = []
            prime_name = ''
            for fname in files:
                if "_target_HandTransformation" in fname:
                    sorted_files.append(fname)
                elif "_prime_HandTransformation" in fname:
                    prime_name = fname
            
            new_files = sorted(sorted_files)
            
            sub_idx = prime_name.find('_sub')
            grasp_idx = prime_name.find('_grasp')
            sub_num = int(prime_name[sub_idx+4:grasp_idx])
            idx = prime_name.find('_prime_Hand')
            filename = prime_name[:idx]
            f = folder_name + "obj"+str(obj_num)+'_'+cluster_no+ "/" + filename
            rospy.loginfo("Showing " + f)
            T_hand = np.genfromtxt(f+"_prime_HandTransformation.txt",delimiter = ',')
            T_obj = np.genfromtxt(f+"_prime_ObjTransformation.txt",delimiter = ',')
            prime_joint_angles = np.genfromtxt(f+"_prime_JointAngles.txt",delimiter = ',') #for barrett arm
            prime_contact_links = np.genfromtxt(f+"_prime_ContactLinkNames.txt",delimiter = ',',dtype = '|S')
            ctrl.set_joint_angles(prime_joint_angles)
            _ = ctrl.reorient_hand(T_hand, T_obj)
            points = np.genfromtxt(f+"_prime_contactpoints.txt",delimiter = ',')
            ctrl.PlotPoints(points)

            
            prime_hand = ctrl.set_dummy_hand_transform('1')

            for j in range(len(new_files)):
                file_name = new_files[j]
                sub_idx = file_name.find('_sub')
                grasp_idx = file_name.find('_grasp')
                sub_num = int(file_name[sub_idx+4:grasp_idx])
                idx = file_name.find('_target_Hand')
                filename =file_name[:idx]
                f = folder_name +"obj"+str(obj_num)+'_'+str(cluster_no)+"/" +  filename
                rospy.loginfo("Showing " + f)
                T_hand = np.genfromtxt(f+"_target_HandTransformation.txt",delimiter = ',')
                T_obj = np.genfromtxt(f+"_target_ObjTransformation.txt",delimiter = ',')
                _ = ctrl.reorient_hand(T_hand, T_obj)
                target_joint_angles = np.genfromtxt(f+"_target_JointAngles.txt",delimiter = ',') #for barrett arm
                ctrl.set_joint_angles(target_joint_angles)
                target_contact_links = np.genfromtxt(f+"_target_ContactLinkNames.txt",delimiter = ',',dtype = '|S')
            
                raw_input('Press Enter to continue')


