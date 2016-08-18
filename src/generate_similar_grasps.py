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
    #joint_angle_threshold = 0.25 * ctrl.hand_1.GetDOFLimits()[1]
    joint_angle_threshold = np.array([1,1,0.3,0.4,0.5,0.3,0.4,0.5,0.4,0.5])
    direction_variance = 0.1
    #take_image = rospy.ServiceProxy('take_snap_shot', SnapShot)
    #alignment_viewer_sub = rospy.Subscriber("/openrave_grasp_view", Int32MultiArray, view_alignment_cb)
    output_data_folder = transform_path+'/output_data_folder/'

    folder_name = transform_path+'/all_obj_transformation/'
    folder_list = os.listdir(folder_name)
     
    for folder in folder_list:
        obj_num = int(folder[3:])
        if obj_num == 17:#[2,4,5,15,17]:
            cluster_number = 0
            files = os.listdir(folder_name + "obj" +str(obj_num) +"/")
            ctrl.set_obj(obj_num)
            sorted_files = []
            for fname in files:
                if "HandTransformation" in fname:
                    sorted_files.append(fname)

            new_files = np.array(sorted(sorted_files),dtype = '|S')
            while not (new_files == "Null").all():
                cluster_number += 1

                f = new_files[0]
                print "filename: ", f
                if f == "Null":
                    new_files = np.delete(new_files,0)
                    continue
                    
                sub_idx = f.find('_sub')
                grasp_idx = f.find('_grasp')
                sub_num = int(f[sub_idx+4:grasp_idx])
                idx = f.find('_Hand')
                filename = f[:idx]
                f = folder_name + "obj"+str(obj_num)+ "/" + filename
                rospy.loginfo("Showing " + f)
                T_hand = np.genfromtxt(f+"_HandTransformation.txt",delimiter = ',')
                T_obj = np.genfromtxt(f+"_ObjTransformation.txt",delimiter = ',')
                old_prime_joint_angles = np.genfromtxt(f+"_JointAngles.txt",delimiter = ',')[7:18] #for barrett arm
                old_prime_contact_links = np.genfromtxt(f+"_ContactLinkNames.txt",delimiter = ',',dtype = '|S')
                _ = ctrl.reorient_hand(T_hand, T_obj)
                ctrl.set_joint_angles(old_prime_joint_angles)
                points,prime_contact_links = ctrl.avoid_hand_collision()
                ctrl.PlotPoints(points)
                
                new_files = np.delete(new_files,0)

                
                # Save prime grasp
                data_saving_folder = output_data_folder +'obj'+str(obj_num)+'_cluster'+str(cluster_number)+'/'
                if not os.path.exists(data_saving_folder):
                    os.makedirs(data_saving_folder)

                np.savetxt(data_saving_folder+ filename+'_prime_contactpoints.txt',points,delimiter=',')
                np.savetxt(data_saving_folder+ filename+'_prime_JointAngles.txt',ctrl.hand_1.GetDOFValues(),delimiter=',')
                np.savetxt(data_saving_folder+ filename+'_prime_HandTransformation.txt',ctrl.hand_1.GetTransform(),delimiter = ',')
                np.savetxt(data_saving_folder+ filename+'_prime_ObjTransformation.txt',ctrl.obj.GetTransform(),delimiter = ',')
                np.savetxt(data_saving_folder+ filename+'_prime_ContactLinkNames.txt',prime_contact_links,delimiter = ',',fmt = "%s")
                
                prime_hand = ctrl.set_dummy_hand_transform('1')
                
                for j in range(len(new_files)):
                    file_name = new_files[j]
                    if (new_files == "Null").all():
                        break
                    if file_name == "Null":
                        continue
                    same_contact_links = False
                    same_joint_angles = False
                    same_direction = False
                    ignore_direction = False
                    sub_idx = file_name.find('_sub')
                    grasp_idx = file_name.find('_grasp')
                    sub_num = int(file_name[sub_idx+4:grasp_idx])
                    idx = file_name.find('_Hand')
                    filename =file_name[:idx]
                    f = folder_name +"obj"+str(obj_num)+ "/" +  filename
                    rospy.loginfo("Showing " + f)
                    T_hand = np.genfromtxt(f+"_HandTransformation.txt",delimiter = ',')
                    T_obj = np.genfromtxt(f+"_ObjTransformation.txt",delimiter = ',')
                    _ = ctrl.reorient_hand(T_hand, T_obj)
                    target_joint_angles = np.genfromtxt(f+"_JointAngles.txt",delimiter = ',')[7:18] #for barrett arm
                    target_contact_links = np.genfromtxt(f+"_ContactLinkNames.txt",delimiter = ',',dtype = '|S')
                    ctrl.set_joint_angles(target_joint_angles)
                    rospy.loginfo('Target joint angles and contact links calculated')
                    if (target_contact_links == old_prime_contact_links).all():
                        same_contact_links = True
                    
                    rospy.loginfo('Target has same contact links: ')
                    print same_contact_links
                    print
                    difference_between_joint_angles = np.abs(np.subtract(old_prime_joint_angles,target_joint_angles))

                    print "Similar Joint angles: ",difference_between_joint_angles<joint_angle_threshold
                    if (difference_between_joint_angles<joint_angle_threshold).all():
                        same_joint_angles = True
                    rospy.loginfo('Target has same joint angles: ') 
                    print same_joint_angles
                    print
                    direction_idx = 0
                    all_flipping_combinations = np.array(
                    [[0,0,0],
                     [0,0,1],
                     [0,1,0],
                     [0,1,1],
                     [1,0,0],
                     [1,0,1],
                     [1,1,0],
                     [1,1,1]])

                    
                    if same_joint_angles and same_contact_links:
                        while (not same_direction) and not ignore_direction:
                            direction_axes_prime = np.transpose(np.dot(ctrl.hand_2.GetTransform()[:3,:3],[[1,0,0],[0,1,0],[0,0,1]]))
                            direction_axes_target = np.transpose(np.dot(ctrl.hand_1.GetTransform()[:3,:3],[[1,0,0],[0,1,0],[0,0,1]]))
                            if (np.abs(np.diag((np.dot(direction_axes_prime,np.transpose(direction_axes_target)) - np.array([1,1,1]))))<direction_variance).all():
                                same_direction = True
                                rospy.loginfo('Target has same directions: ') 
                                print same_direction
                                print
                            else:
                                direction_flip_sequence = all_flipping_combinations[direction_idx]
                                print "Flipping vector", direction_flip_sequence
                                if direction_flip_sequence[0] == 1:
                                    reflect_along_x_plane(ctrl.env,ctrl.hand_1)
                                direction_axes_prime = np.transpose(np.dot(ctrl.hand_2.GetTransform()[:3,:3],[[1,0,0],[0,1,0],[0,0,1]]))
                                direction_axes_target = np.transpose(np.dot(ctrl.hand_1.GetTransform()[:3,:3],[[1,0,0],[0,1,0],[0,0,1]]))

                                if (np.abs(np.diag((np.dot(direction_axes_prime,np.transpose(direction_axes_target)) - np.array([1,1,1]))))<direction_variance).all():
                                    same_direction = True
                                    rospy.loginfo('Target has same directions: ') 
                                    print same_direction
                                    print

                                if direction_flip_sequence[1] == 1 and not same_direction:
                                    reflect_along_y_plane(ctrl.env,ctrl.hand_1)

                                direction_axes_prime = np.transpose(np.dot(ctrl.hand_2.GetTransform()[:3,:3],[[1,0,0],[0,1,0],[0,0,1]]))
                                direction_axes_target = np.transpose(np.dot(ctrl.hand_1.GetTransform()[:3,:3],[[1,0,0],[0,1,0],[0,0,1]]))
                                if (np.abs(np.diag((np.dot(direction_axes_prime,np.transpose(direction_axes_target)) - np.array([1,1,1]))))<direction_variance).all():
                                    same_direction = True
                                    rospy.loginfo('Target has same directions: ') 
                                    print same_direction
                                    print

                                if direction_flip_sequence[2] == 1 and not same_direction:
                                    reflect_along_z_plane(ctrl.env,ctrl.hand_1)

                                direction_axes_prime = np.transpose(np.dot(ctrl.hand_2.GetTransform()[:3,:3],[[1,0,0],[0,1,0],[0,0,1]]))
                                direction_axes_target = np.transpose(np.dot(ctrl.hand_1.GetTransform()[:3,:3],[[1,0,0],[0,1,0],[0,0,1]]))
                                if (np.abs(np.diag((np.dot(direction_axes_prime,np.transpose(direction_axes_target)) - np.array([1,1,1]))))<direction_variance).all():
                                    same_direction = True
                                    rospy.loginfo('Target has same directions: ') 
                                    print same_direction
                                    print


                                direction_idx += 1

                            
                            if direction_idx ==8:
                                ignore_direction = True
                            
                            
                    

                    if same_direction:
                        new_files[j] = "Null"
                        ctrl.set_joint_angles(target_joint_angles)
                        points,target_contact_links = ctrl.avoid_hand_collision()

                        np.savetxt(data_saving_folder+ filename+'_target_contactpoints.txt',points,delimiter=',')
                        np.savetxt(data_saving_folder+ filename+'_target_JointAngles.txt',ctrl.hand_1.GetDOFValues(),delimiter=',')
                        np.savetxt(data_saving_folder+ filename+'_target_HandTransformation.txt',ctrl.hand_1.GetTransform(),delimiter = ',')
                        np.savetxt(data_saving_folder+ filename+'_target_ObjTransformation.txt',ctrl.obj.GetTransform(),delimiter = ',')
                        np.savetxt(data_saving_folder+ filename+'_target_ContactLinkNames.txt',target_contact_links,delimiter = ',',fmt = "%s")


                    
                    #break
                #break
            #break                


