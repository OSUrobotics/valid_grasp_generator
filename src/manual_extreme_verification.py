#! /usr/bin/env python
import rospy

from std_msgs.msg import Header, Int32MultiArray
from sensor_msgs.msg import JointState, Image
from valid_grasp_generator.msg import GraspSnapshot
from wam_msgs.msg import HandCommand
from wam_srvs.srv import JointMove, CartPosMove

#from shared_playback import *

import os
import csv
import copy
import time

out_suffix = "_extreme_verification.csv"
similar_dir = os.path.expanduser("~") + "/grasp_similarities"
out_dir = os.path.expanduser("~") + "/grasp_verifications"

def get_testable_objects(similar_dir):
	files = os.listdir(similar_dir)
	obj_sub_tuples = set()
	for f in files:
		with open(similar_dir + "/" + f, "r") as in_file:
			csv_file = csv.reader(in_file, delimiter=",")
			for l in csv_file:
				try:
					obj_sub_tuples.add((int(l[0]), int(l[1])))
				except ValueError:
					pass

	return obj_sub_tuples

def remove_finished(similar_dir, obj_sub_tuples):
	global out_suffix, out_dir
	out_tups = set()
	for t in obj_sub_tuples:
		full_path = mk_out_file_path(t[0], t[1])
		if not os.path.exists(full_path):
			out_tups.add(t)
		else:
			rospy.logwarn(full_path + " already exists. Skipping.")

	return out_tups

def get_hand_command(base_dir, stamp):
	#try:
		hand_bag = rosbag.Bag(base_dir + "/" + "hand_commands.bag", "r")
		hand_cmds = []
		lead_time = rospy.Duration(30)
		play_time = rospy.Duration(1)
		for topic, msg, t in hand_bag.read_messages(topics=["/bhand/hand_cmd"], start_time=(stamp - lead_time), end_time=(stamp + play_time)):
			hand_cmds.append(msg)
		#hand_cmd = msg_from_bag(hand_bag, ["/bhand/hand_cmd"], stamp)
		hand_bag.close()
		if len(hand_cmds) < 1:
			rospy.logerr("Cannot get hand command from this bag, you'll need to manuever the hand manually.")
			return None
		else:
			return hand_cmds[-1]
	#except:
	#	rospy.logerr("Unexpected error handling the hand command bag for " + base_dir)
	#	rospy.logerr("Manual alignment necessary. Use sliders.")
	#	return None

def republish_hand_command(hand_pub, msg):
	if msg == None:
		return
	else:
		for i in range(5):
			hand_pub.publish(msg)

def mk_out_file_path(obj_num, sub_num):
	global out_dir, out_suffix
	return str(out_dir + "/" + "obj" + str(obj_num) + "_sub" + str(sub_num) + out_suffix)

def run_trial(ros_dict, snapshot, hand_cmd):
	for i in range(5):
		ros_dict['rgb_show'].publish(snapshot.rgb_image)
	# Move the arm
	#raw_input("Press [Enter] to move the arm.")
	ros_dict['wam_jnt_srv'](snapshot.wam_joints.position)
	
	raw_input("Place the object in the proper location and press [Enter]")
	republish_hand_command(ros_dict['hand_cmd_pub'], hand_cmd)
	raw_input("Press [Enter] to execute grasp.")
	shake_wam(ros_dict)

def shake_wam(ros_dict):
	rospy.loginfo("Initiating shake.")
	base_pose = [-.7, -.1, .15]
	end_pose = [-.7, -.1, .35]

	velocity = rospy.get_param("/wam/velocity")
	sleep_time = 1 / velocity

	for i in range(4):
		rospy.loginfo("Shaking once.")
		ros_dict['cart_move_srv'](base_pose)
		time.sleep(sleep_time)
		ros_dict['cart_move_srv'](end_pose)
		time.sleep(sleep_time)

def reset_trial(ros_dict):
	# Open the hand
	republish_hand_command(ros_dict['hand_cmd_pub'], HandCommand(Header(0, rospy.Time.now(), ''), 0,0,0,0))

if __name__ == "__main__":
	rospy.init_node("manual_extreme_verification")
	base_dict = {"object":'', "subject":'', "grasp":'', "idx":'', "trial":'', "success":''}
	base_fields = ["object", "subject", "grasp", "idx", "trial", "success"]

	obj_sub_tuples = get_testable_objects(similar_dir)
	obj_sub_tuples = remove_finished(similar_dir, obj_sub_tuples)
	
	ros_dict = {}
	ros_dict['rgb_show'] = rospy.Publisher("/grasp_rgb", Image, queue_size=1, latch=True)
	ros_dict['hand_cmd_pub'] = rospy.Publisher("/bhand/hand_cmd", HandCommand, queue_size=1, latch=True)
	ros_dict['openrave_show'] = rospy.Publisher("/openrave_grasp_view", Int32MultiArray, queue_size=1, latch=True)

	rospy.loginfo("Waiting for joint motion service.")
	rospy.wait_for_service('/wam/joint_move')
	ros_dict['wam_jnt_srv'] = rospy.ServiceProxy("/wam/joint_move", JointMove)
	ros_dict['cart_move_srv'] = rospy.ServiceProxy("/wam/cart_move", CartPosMove)

	if not os.path.exists(out_dir):
		os.makedirs(out_dir)

	for t in obj_sub_tuples:
		base_path = grasp_data_directory + "/good/obj" + str(t[0]) + "_sub" + str(t[1]) + "_defrag"
		if not os.path.exists(base_path):
			base_path = "_".join(base_path.split("_")[:-1])
			if not os.path.exists(base_path):
				rospy.logerr("Cannot find snapshot directory for tuple: " + str(t))
				continue

		# Open the Snapshot file
		b = None
		snapshot_path = base_path + "/" + "grasp_extreme_snapshots.bag"
		try:
			b = try_bag_open(snapshot_path)
		except:
			rospy.logerr("Cannot open bag file: " + snapshot_path)
			continue


		rospy.loginfo("Processing " + base_path)
		# Find those darn extremes
		out_file = open(mk_out_file_path(t[0], t[1]), "a")
		out_csv = csv.DictWriter(out_file, base_fields, delimiter=",")
		out_csv.writeheader()
		for topic, msg, t in b.read_messages():
			if msg.is_optimal == True:
				rospy.loginfo("Skipping optimal")
				continue
		
			user_input = raw_input("About to start obj %d sub %d grasp %d idx %d (s to skip, anything else to continue):" % (msg.obj_num, msg.sub_num, msg.grasp_num, msg.extreme_num))
			if "s" in user_input.lower():
				continue

			out_row = copy.deepcopy(base_dict)
			out_row['object'] = msg.obj_num
			out_row['subject'] = msg.sub_num
			out_row['grasp'] = msg.grasp_num
			out_row['idx'] = msg.extreme_num
			hand_cmd = get_hand_command(base_path, msg.stamp)
			
			i = 0
			while i < 5:
				reset_trial(ros_dict)
				rospy.loginfo("Testing trial %d OF obj %d sub %d grasp %d idx %d" % (i, msg.obj_num, msg.sub_num, msg.grasp_num, msg.extreme_num))
				out_row['trial'] = i

				vis_msg = Int32MultiArray()
				vis_msg.data.extend([msg.obj_num, msg.sub_num, msg.grasp_num, msg.extreme_num])
				ros_dict['openrave_show'].publish(vis_msg)
				try:
					run_trial(ros_dict, msg, hand_cmd)
				except:
					rospy.logerr("Could not finish trial.")
					user_input = raw_input("b to break trial and r to retry: ")
					if "b" in user_input.lower():
						break
					if "r" in user_input.lower():
						continue
				while True:
					user_input = raw_input("Success? (y/n): ")
					if "y" in user_input.lower():
						out_row['success'] = 1
					elif "n" in user_input.lower():
						out_row['success'] = 0
					else:
						rospy.logwarn("Unrecognized input")
						continue
					break
				i += 1
				out_csv.writerow(out_row)

		out_file.close()

