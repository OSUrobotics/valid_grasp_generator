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


def main():
    global transform_path, ctrl
    ctrl = object_visualizer()
    rospy.init_node('object_visualizer',anonymous = True)
    alignment_viewer_sub = rospy.Subscriber("/openrave_grasp_view", Int32MultiArray, view_alignment_cb)
    while not rospy.is_shutdown():
    	obj_num = int(raw_input("Obj num: "))
	sub_num = int(raw_input("Sub num: "))

	#transform_path = "/media/eva/FA648F24648EE2AD" + "/csvfiles/obj" + str(obj_num) + "_sub" + str(sub_num) + "_pointcloud_csvfiles"
	#transform_path = os.path.expanduser("~") + "/csvfiles/obj" + str(obj_num) + "_sub" + str(sub_num) + "_pointcloud_csvfiles"
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
	    ctrl.reorient_hand(T_hand, T_obj)

if __name__=="__main__":
    main()
