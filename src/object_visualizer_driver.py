#!/usr/bin/env python
from object_visualizer import *
from std_msgs.msg import Int32MultiArray
import numpy as np

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
	ctrl.reorient_hand(T_hand, T_obj)


def main1():
    global transform_path, ctrl
    ctrl = object_visualizer()
    rospy.init_node('object_visualizer',anonymous = True)
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
            f = transform_path + "/" + "obj"+str(obj_num)+"_sub"+str(sub_num) + "/" + f[:25]
	    rospy.loginfo("Showing " + f)
	    T_hand = np.genfromtxt(f+"_HandTransformation.txt",delimiter = ',')
            T_obj = np.genfromtxt(f+"_ObjTransformation.txt",delimiter = ',')
            joint_angles = np.genfromtxt(f+"_JointAngles.txt",delimiter = ',')[7:18]
	    ctrl.reorient_hand(T_hand, T_obj)
            ctrl.set_joint_angles(joint_angles)

def main2():
    global transform_path, ctrl
    ctrl = object_visualizer()
    rospy.init_node('object_visualizer',anonymous = True)
    alignment_viewer_sub = rospy.Subscriber("/openrave_grasp_view", Int32MultiArray, view_alignment_cb)
    folder_name = transform_path+"/similar_grasp_extreme_directory/"
    print "Reading files from : ", folder_name 
    while not rospy.is_shutdown():
	files = os.listdir(folder_name)
        alpha_vector = np.arange(1,5,0.1)
        for csv_file in files:
            interpolated_grasps = np.array([])
            similar_grasp_matrix = np.genfromtxt(csv_file,delimiter=',')
            JointAngles = np.empty((2,10),np.float32)
            HandTransformations = np.empty((2,4,4),np.float32)

            for i in [0,1]:
                obj_num = similar_grasp_matrix[i]
                sub_num = similar_grasp_matrix[i]
                grasp_num = similar_grasp_matrix[i]
                is_optimal = similar_grasp_matrix[i]
                ext_opt_num = similar_grasp_matrix[i]
                if is_optimal == 1:
                    f = transform_path + "/" + "obj"+str(obj_num)+"_sub"+str(sub_num) + "/" + "obj"+str(obj_num)+"_sub"+str(sub_num)+"_grasp"+str(grasp_num)+"_optimal"+str(ext_opt_num)
                else:
                    f = transform_path + "/" + "obj"+str(obj_num)+"_sub"+str(sub_num) + "/" + "obj"+str(obj_num)+"_sub"+str(sub_num)+"_grasp"+str(grasp_num)+"_extreme"+str(ext_opt_num)

	        rospy.loginfo("Showing " + f)
	        T_hand = np.genfromtxt(f+"_HandTransformation.txt",delimiter = ',')
                T_obj = np.genfromtxt(f+"_ObjTransformation.txt",delimiter = ',')
                joint_angles = np.genfromtxt(f+"_JointAngles.txt",delimiter = ',')[7:18]
                JointAngles[i] = joint_angles
                contact_names = np.gnefromtxt(f+"_ContactLinkNames.txt",delimiter = ',',dtype="|S")
	        ctrl.reorient_hand(T_hand, T_obj)
                ctrl.set_joint_angles(joint_angles)

if __name__=="__main__":
    user_input = raw_input("What do you want to do? Choose one option: \n1) Visualize grasp\n2) Generate intermediate grasp\n")
    print "You chose: ",user_input
    if user_input == '1':
        main1()
    elif user_input == '2':
        main2()
    else:
        print " Your choice doesn't correspond to any of the option. Please try again"
