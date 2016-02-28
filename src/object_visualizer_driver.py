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
    #vec_1 = mat2euler((hand_1[:3,:3]).tolist())
    #vec_2 = mat2euler((hand_2[:3,:3]).tolist())
    #intermediate_rotation_vec = get_intermediate_points([vec_1,vec_2],alpha)
    #rotation_matrix = euler2mat(intermediate_rotation_vec[0],intermediate_rotation_vec[1],intermediate_rotation_vec[2])
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


def main1():
    global transform_path, ctrl
    ctrl = object_visualizer()
    take_image = rospy.ServiceProxy('take_snap_shot', SnapShot)
    alignment_viewer_sub = rospy.Subscriber("/openrave_grasp_view", Int32MultiArray, view_alignment_cb)
    while not rospy.is_shutdown():
    	obj_num = int(raw_input("Obj num: "))
	sub_num = int(raw_input("Sub num: "))

	files = os.listdir(transform_path+ "/"+ "obj" +str(obj_num)+"_sub"+str(sub_num)+"/")
        print obj_num
        sorted_files = []
        for fname in files:
            if "HandTransformation" in fname:
                sorted_files.append(fname)

	new_files = sorted(sorted_files)
	ctrl.set_obj(obj_num)
	for f in new_files:
            idx = f.find('_Hand')
            f = transform_path + "/" + "obj"+str(obj_num)+"_sub"+str(sub_num) + "/" + f[:idx]
	    rospy.loginfo("Showing " + f)
	    T_hand = np.genfromtxt(f+"_HandTransformation.txt",delimiter = ',')
            T_obj = np.genfromtxt(f+"_ObjTransformation.txt",delimiter = ',')
            joint_angles = np.genfromtxt(f+"_JointAngles.txt",delimiter = ',')[7:18]
	    _ = ctrl.reorient_hand(T_hand, T_obj)
            ctrl.set_joint_angles(joint_angles)
            points = ctrl.avoid_hand_collision()
            #ctrl.PlotPoints(points)
            user_input = raw_input("Do you want to include dummy hands in the environment? (y/n)?") or "x"
            if user_input == "n":
                ctrl.hide_other_hands()
            elif user_input == "y":
                new_user_in = raw_input("Enter the hand you want to move: ")
                ctrl.set_dummy_hand_transform(new_user_in)

            snapshot = raw_input("Do you want to take picture? (y/n)") or "n"
            if snapshot == "y":
                resp = take_image(transform_path+ f.split('_Hand')[0]+'_for_paper.jpg')

def main2():
    global transform_path, ctrl
    ctrl = object_visualizer()
    alignment_viewer_sub = rospy.Subscriber("/openrave_grasp_view", Int32MultiArray, view_alignment_cb)
    folder_name = transform_path+"/similar_grasp_extreme_directory/"
    Image_directory = transform_path+"/Images/"
    rospy.loginfo(" Wating for Service : /take_snap_shot")
    rospy.wait_for_service('take_snap_shot')
    take_image = rospy.ServiceProxy('take_snap_shot', SnapShot)
    print "Reading files from : ", folder_name 
    files = os.listdir(folder_name)
    alpha_vector = np.array([0,0.2,0.8,1])
    for csv_file in files:
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
            user_input = raw_input("Do you want to take image (y/n)") or "y"
            if user_input == "y":
                resp = take_image(Image_directory+object_names[0] + "_alpha_"+str(alpha)+"_"+object_names[1]+".jpg")
                time.sleep(0.5)
                ctrl.Removepoints()
                time.sleep(0.5)
                resp = take_image(Image_directory+object_names[0] + "_alpha_"+str(alpha)+"_"+object_names[1]+"_without_contacts.jpg")
                print resp.output

        os.rename(folder_name+csv_file,transform_path+"/similar_grasp_done/"+csv_file)

if __name__=="__main__":
    rospy.init_node('object_visualizer',anonymous = True)
    user_input = raw_input("What do you want to do? Choose one option: \n1) Visualize grasp\n2) Generate intermediate grasp\n")
    print "You chose: ",user_input
    if user_input == '1':
        main1()
    elif user_input == '2':
        main2()
    else:
        print " Your choice doesn't correspond to any of the option. Please try again"
