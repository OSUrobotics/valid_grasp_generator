import threading
from openravepy import *
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
import sys
import rospkg
import time
import os
import string
import math
from openravepy.examples import tutorial_grasptransform


from obj_dict import *

class object_visualizer(object):
    def __init__(self):
        self.path = rospkg.RosPack().get_path('valid_grasp_generator')
	self.stl_path = self.path + "/models/stl_files"
        self.env = Environment()
	self.obj_num = 15
        self.env.Load(self.stl_path + '/Chunk_of_foam.STL',{'scalegeometry':'0.001 0.001 0.001'})
        self.env.SetViewer('qtcoin')
        self.obj = self.env.GetBodies()[0]
        error_1 = self.env.Load(self.path+'/models/robots/bhand.dae')
        self.hand_1 = self.env.GetRobots()[0]
        self.flag = False
        self.hand_1_mats = self.hand_1.GetLinkTransformations()
        self.part_mat = np.array(self.obj.GetTransform())
        self.obj.SetVisible(0)
	self.obj.SetTransform(np.eye(4))
	self.gt = tutorial_grasptransform.GraspTransform(self.env, self.hand_1)

    def set_obj(self, obj_num):
    	global grasp_obj_dict
	self.obj_num = obj_num
    	self.env.Remove(self.obj)
        if obj_num == 19:
	    self.env.Load(self.stl_path + "/" + grasp_obj_dict[obj_num][1], {'scalegeometry':'0.0009 0.0009 0.0009'})
        else:
	    self.env.Load(self.stl_path + "/" + grasp_obj_dict[obj_num][1], {'scalegeometry':'0.001 0.001 0.001'})
	self.obj = self.env.GetKinBody(grasp_obj_dict[obj_num][1].split('.')[0])
	self.obj_y_rotate = grasp_obj_dict[obj_num][2]
	T_cent = self.get_stl_centroid_transform()
	self.apply_link_transform(T_cent, self.obj)
	rospy.loginfo("Loaded " + grasp_obj_dict[obj_num][0])

    def set_joint_angles(self,joint_angles):
        self.hand_1.SetDOFValues(joint_angles)

    def set_hand_joints(self,jnt_dict):
    	hand_jnts = self.hand_1.GetJoints()
	out_dof = self.hand_1.GetDOFValues()
	for j in hand_jnts:
		dof_idx = j.GetDOFIndex()
		n = j.GetName()
		if ("1" in n or "2" in n) and "prox" in n:
			out_dof[dof_idx] = jnt_dict['spread']
		elif "1" in n and "med" in n:
			out_dof[dof_idx] = jnt_dict['inner_f1']
		elif "1" in n and "dist" in n:
			out_dof[dof_idx] = jnt_dict['outer_f1']
		elif "2" in n and "med" in n:
			out_dof[dof_idx] = jnt_dict['inner_f2']
		elif "2" in n and "dist" in n:
			out_dof[dof_idx] = jnt_dict['outer_f2']
		elif "3" in n and "med" in n:
			out_dof[dof_idx] = jnt_dict['inner_f3']
		elif "3" in n and "dist" in n:
			out_dof[dof_idx] = jnt_dict['outer_f3']

	self.hand_1.SetDOFValues(out_dof)

    def reorient_hand(self, T_palm, T_obj, use_joint_angles="False"):
	self.obj.SetVisible(1)
	hand_to_obj = np.dot(np.linalg.inv(T_obj), T_palm)

	self.hand_1.SetTransform(hand_to_obj)

	self.standard_axes = self.gt.drawTransform(np.eye(4))
	self.recenter_from_stl()
	if self.obj_num == 17:
		self.standardize_ball()
	elif self.obj_num == 5 or self.obj_num == 6:
		self.standardize_box()
	elif self.obj_num == 15:
		# The chunk of foam requires a little manual flipping due to its cylindrical top-down symmetry
		self.standardize_foam()
	else:
		self.standardize_axes()

	palm_pt = self.get_palm_point()
	self.palm_plot = self.env.plot3(palm_pt, 10)
        return hand_to_obj
	#print "Final palm point: ", palm_pt[0], "\t", palm_pt[1], "\t", palm_pt[2]
	#raw_input("Finished reorientation. How are we doing?")

   
    # Recenters the robot hand relative to
    #	the centroid of the object
    def recenter_from_stl(self):
	T_cent = self.get_stl_centroid_transform()
	self.apply_link_transform(T_cent, self.hand_1)

    def get_obj_transformation(self):
        return self.obj.GetTransform()

    def get_stl_centroid_transform(self):
    	global obj_centroid_dict
	centroid = obj_centroid_dict[self.obj_num]
	T_cent = np.eye(4)
	for idx, x in enumerate(centroid):
		T_cent[idx][3] = x / 1000.0
	T_cent = np.linalg.inv(T_cent)
	
	return T_cent

    # If the object can freely rotate about the y axis, all grasps should
    #	be reoriented into the same coordinate frame. This function will
    #	standardize the frame of reference.
    def standardize_axes(self):
    	if not self.obj_y_rotate:
		rospy.loginfo("No axis standardization necessary.")
		return
	
	palm_pt = self.get_palm_point()
	palm_approach = [0,0,1] # Here is the palm's direction in its own corrdinate frame
	palm_approach = poseTransformPoints(poseFromMatrix(self.hand_1.GetTransform()), [palm_approach])[0]
	print "palm_approach: ", palm_approach
	try:
		angle = get_angle([0,1,0], palm_approach)
	except:
		rospy.loginfo("Angles too close to consider. Probably dont need to realign.")
		return
	#print "angle: ", angle
	if angle > (math.pi/6) and angle < (2*math.pi/3):
		pass
	else:
		rospy.loginfo("Approach is not along the rotational axis. Skipping.")
		return

	# Find the xz-planar angular difference between the x axis and the palm point
	angle = get_angle([palm_pt[0], palm_pt[2]], [1,0])
	if palm_pt[2] < 0:
		angle = -angle

	# Get the rotation matrix about the y axis
	print "Angle:", angle
	standardize_mat = matrixFromAxisAngle([0,1,0], angle)

	# Reorient things...
	self.apply_scene_transform(standardize_mat)
	rospy.loginfo("Axes standardized.")

    # The ball gets a special procedure because some optimization can be made.
    def standardize_ball(self):
    	palm_pt = self.get_palm_point()
    	axis = np.cross([1,0,0], palm_pt)
	angle = -get_angle([1,0,0], palm_pt)
	standardize_mat = matrixFromAxisAngle(axis, angle)
	self.apply_scene_transform(standardize_mat)

    def standardize_box(self):
    	while True:
		user_input = raw_input("Name the axes to rotate around: (x)(xy)(...): ")
		user_input = user_input.strip().lower()
		standardize_mat = np.eye(4)
		if "x" in user_input:
			standardize_mat = np.dot(matrixFromAxisAngle([1,0,0], math.pi), standardize_mat)
		if "y" in user_input:
			standardize_mat = np.dot(matrixFromAxisAngle([0,1,0], math.pi), standardize_mat)
		if "z" in user_input:
			standardize_mat = np.dot(matrixFromAxisAngle([0,0,1], math.pi), standardize_mat)

		self.apply_link_transform(standardize_mat, self.hand_1)

		if "n" in user_input or user_input == "":
			break

    def standardize_foam(self):
    	self.standardize_axes()
	palm_approach = [0,0,1] # Here is the palm's direction in its own corrdinate frame
	palm_approach = poseTransformPoints(poseFromMatrix(self.hand_1.GetTransform()), [palm_approach])[0]
	angle = get_angle([0,1,0], palm_approach)
	palm_pt = self.get_palm_point()
	if (angle < math.pi/6 or angle > 5 * math.pi / 6) and palm_pt[1] < 0:
		standardize_mat = matrixFromAxisAngle([1,0,0], math.pi)
		self.apply_link_transform(standardize_mat, self.hand_1)
		#raw_input("Flipped the hand around the x axis.")

    def apply_scene_transform(self, T):
    	self.apply_link_transform(T, self.hand_1)
	self.apply_link_transform(T, self.obj)

    def apply_link_transform(self, T, link):
    	T_l = link.GetTransform()
	T_l_new = np.dot(T, T_l)
	link.SetTransform(T_l_new)

    def get_palm_point(self):
	palm_pose = poseFromMatrix(self.hand_1.GetTransform())
	palm_pt = numpy.array(palm_pose[4:])
	palm_pt += self.get_palm_offset()
	#print "Point on palm", palm_pt

	return palm_pt

    # Finds a vector from the base of the wrist to the middle of he palm by following
    #	the apporach vector for the depth of the palm (7.5cm)
    def get_palm_offset(self):
    	palm_approach = [0,0,1]
	palm_approach = poseTransformPoints(poseFromMatrix(self.hand_1.GetTransform()), [palm_approach])[0]
	palm_approach = np.array(palm_approach)
	palm_approach = palm_approach / np.linalg.norm(palm_approach)
	palm_approach *= 0.075
	return palm_approach

    def get_obj_points(self):
    	obj_pts = self.obj.GetLinks()[0].GetCollisionData().vertices
	obj_pose = poseFromMatrix(self.obj.GetTransform())

	return  poseTransformPoints(obj_pose, obj_pts)


# Returns the angular difference between the vectors in radians
def get_angle(v1, v2):
	n1 = float(np.linalg.norm(v1))
	n2 = float(np.linalg.norm(v2))
	if n1 < 0.001 or n2 < 0.001:
		print "The vectors are too small to compare."
		raise ValueError

	return np.arccos(np.dot(v1, v2) / (n1 * n2))

def get_transforms(file_path):
	f = open(file_path, "r")
	lines = f.readlines()
	f.close()
	mats = [0,0]
	mats[0] = "".join(lines[1:5]).replace("[", "").replace("]","").split("\n")
	mats[1] = "".join(lines[7:]).replace("[","").replace("]", "").split("\n")
	
	#print "obj premat", mats[0] 
	#print "hand premat", mats[1]
	obj_mat = []
	hand_mat = []
	for line in mats[0]:
		try:
			obj_mat.append(get_list_from_str(line))
		except:
			continue

	for line in mats[1]:
		try:
			hand_mat.append(get_list_from_str(line))
		except:
			continue
	
	if len(hand_mat) != 4 or len(hand_mat[0]) != 4:
		rospy.logerr("Didnt read in hand matrix properly!: " + str(hand_mat))
		sys.exit(1)
	if len(obj_mat) != 4 or len(obj_mat[0]) != 4:
		rospy.logerr("Didnt read in obj matrix properly: " + str(obj_mat))
		sys.exit(1)

	return numpy.array(hand_mat), numpy.array(obj_mat)

def get_list_from_str(in_str):
	if in_str == '':
		raise ValueError
	l = []
	split_str = in_str.split(" ")
	for n in split_str:
		if n is not "":
			l.append(float(n))
	return l

