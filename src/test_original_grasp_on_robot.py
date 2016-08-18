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
import os
from shared_global import valid_grasp_dir
from retract_finger import get_palm_perpendicular_vector
transform_path = os.path.expanduser("~") + "/grasping_data"
from wam_msgs.msg import HandCommand
from wam_srvs.srv import JointMove, CartPosMove
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
    #
    vec_1 = mat2euler((hand_1[:3,:3]).tolist())
    vec_2 = mat2euler((hand_2[:3,:3]).tolist())
    intermediate_rotation_vec = get_intermediate_points([vec_1,vec_2],alpha)
    rotation_matrix = euler2mat(intermediate_rotation_vec[0],intermediate_rotation_vec[1],intermediate_rotation_vec[2])
    #
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

def GetGraspFile(folder_name):
    grasp_data = np.empty((2,5),np.int)
    for i in [0,1]:
        print "Enter obj num ",i+1
        obj_num = raw_input(":")
        print "Enter sub num ",i+1
        sub_num = raw_input(":")
        print "Enter grasp num ", i+1
        grasp_num = raw_input(":")
        print "Enter 1 for optimal and 0 for extreme ", i+1
        is_optimal = raw_input(":")
        print "Enter extreme or optimal number ",i+1
        ext_opt_num = raw_input(":")
        grasp_data[i] = [obj_num,sub_num,grasp_num,is_optimal,ext_opt_num]

    files = os.listdir(folder_name)
    for csv_file in files:
        matrix = np.genfromtxt(folder_name+csv_file,delimiter =',')
        comparison_matrix = matrix == grasp_data
        if comparison_matrix.all():
            print "Found csv file! Returning"
            return csv_file

    print
    print "File not found"
    print

def republish_hand_command(hand_pub, msg):
    if msg == None:
        return
    else:
        for i in range(5):
            hand_pub.publish(msg)


def run_trial(ros_dict,wam_joints_position_1, wam_joints_position_2, hand_cmd):
    print "executing"
    ros_dict['wam_jnt_srv']([0,0,0,0,0,0,0])
    if wam_joints_position_1 == None:
        wam_joints_position_1 = [0,0,0,0,0,0,0]
    raw_input("Wait for robot to reach ZERO position. Then Press [Enter]")
    null_hand_commands = HandCommand()
    null_hand_commands.f1 =0
    null_hand_commands.f2 =0
    null_hand_commands.f3 =0
    null_hand_commands.spread = 0
    republish_hand_command(ros_dict['hand_cmd_pub'], null_hand_commands)
    null_hand_commands.spread = hand_cmd.spread
    successful_grasp = np.array([])
    for i in range(5):
        # Move the arm
        #raw_input("Press [Enter] to move the arm.")
        ros_dict['wam_jnt_srv'](wam_joints_position_1)
        user_in = raw_input("Wait for the robot to move then press [Enter]. Satisfied with this position(y/n)") or 'y'
        if user_in == 'n':
            ros_dict['wam_jnt_srv']([0,0,0,0,0,0,0])
            wam_joints_position_1 = wam_joints_position_2
            raw_input('Position changed! Press [Enter] to continue')
            
        ros_dict['wam_jnt_srv'](wam_joints_position_2)
        raw_input("Press [Enter] to close fingers")
        republish_hand_command(ros_dict['hand_cmd_pub'],hand_cmd) 
        raw_input("Press [Enter] to execute shake test")
        shake_wam(ros_dict)
        raw_input("Press [Enter] to move the object back")
        ros_dict['wam_jnt_srv'](wam_joints_position_2)
        raw_input("Press [Enter] to open the fingers")
        republish_hand_command(ros_dict['hand_cmd_pub'], null_hand_commands)
        raw_input("Press [Enter] to move the hand back little bit")
        ros_dict['wam_jnt_srv'](wam_joints_position_1)

        success = raw_input("Was the shake test successful?(0/1)") or '1'
        successful_grasp = np.append(successful_grasp,int(success))

    return successful_grasp




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


if __name__=="__main__":
    rospy.init_node('test_on_robot',anonymous = True )
    # Jackson's code
    
    ros_dict = {}
    ros_dict['hand_cmd_pub'] = rospy.Publisher("/bhand/hand_cmd", HandCommand, queue_size=1, latch=True)
    #rospy.wait_for_service('/wam/joint_move')
    ros_dict['wam_jnt_srv'] = rospy.ServiceProxy("/wam/joint_move", JointMove)
    ros_dict['cart_move_srv'] = rospy.ServiceProxy("/wam/cart_move", CartPosMove)

    ctrl = object_visualizer()
    alignment_viewer_sub = rospy.Subscriber("/openrave_grasp_view", Int32MultiArray, view_alignment_cb)
    folder_name = transform_path+"/similar_grasp_extreme_directory/"
    similar_grasp_matrix = np.genfromtxt(transform_path+'/grasp_for_testing.csv',delimiter=',')
    done_grasp_matrix = np.genfromtxt(transform_path+'/done_grasp_testing.csv',delimiter=',')
    env = ctrl.get_env()
    env.Load(valid_grasp_dir+'/models/robots/barrettwam.robot.xml')
    robot = env.GetRobots()[3]
    print robot 
    manip = robot.GetActiveManipulator()
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype = IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()
    user_in_obj_num = int(raw_input("Select the object number from this list (15,17,2,4,5): ")) or 17
    for grasp_vector in similar_grasp_matrix:
        print "grasp_vector {} is done:  {}".format(grasp_vector,grasp_vector.tolist() in done_grasp_matrix[:,0:5].tolist())
        if not (grasp_vector.tolist() in done_grasp_matrix[:,0:5].tolist()):
            obj_num = int(grasp_vector[0])
            robot_transform = np.genfromtxt(valid_grasp_dir+ '/essential_files/essential_transform/obj'+str(obj_num)+'_robot_transform.csv',delimiter = ',')
            robot.SetTransform(robot_transform)
            if obj_num == user_in_obj_num:
                sub_num = int(grasp_vector[1])
                grasp_num = int(grasp_vector[2])
                is_optimal = int(grasp_vector[3])
                ext_opt_num = int(grasp_vector[4])
                ctrl.set_obj(obj_num)

                if is_optimal == 1:
                    f = "obj"+str(obj_num)+"_sub"+str(sub_num)+"_grasp"+str(grasp_num)+"_optimal"+str(ext_opt_num)
                else:
                    f = "obj"+str(obj_num)+"_sub"+str(sub_num)+"_grasp"+str(grasp_num)+"_extreme"+str(ext_opt_num)
                    
                f = transform_path +"/"+"obj"+str(obj_num)+"_sub"+str(sub_num) + "/" + f
                rospy.loginfo("Showing " + f)
                T_hand = np.genfromtxt(f+"_HandTransformation.txt",delimiter = ',')
                T_obj = np.genfromtxt(f+"_ObjTransformation.txt",delimiter = ',')
                joint_angles = np.genfromtxt(f+"_JointAngles.txt",delimiter = ',')[7:18]
                hand_command = np.genfromtxt(f+"_closeddofvals.txt",delimiter=',')
                hand_cmd = HandCommand()
                hand_cmd.f1 = hand_command[0]
                hand_cmd.f2 = hand_command[1]
                hand_cmd.f3 = hand_command[2]
                hand_cmd.spread = hand_command[3]
                
                
                ctrl.reorient_hand(T_hand, T_obj,joint_angles)
                user_in = raw_input("Press Enter to continue")
                HandTransformation = ctrl.GetHandTransform()
                ctrl.set_joint_angles(joint_angles)
                
                print "grasp vector:  ",grasp_vector
                
                Tgoal = ctrl.hand_1.GetLinkTransformations()[2] 
                palm_perpendicular_vector = get_palm_perpendicular_vector(ctrl.hand_1)
                Tgoal[0:3,3] = np.add(Tgoal[0:3,3], np.dot(0.062,palm_perpendicular_vector))
                sol_1 = manip.FindIKSolution(Tgoal, IkFilterOptions.IgnoreEndEffectorCollisions)
                print "solution 1", sol_1
                Tgoal[0:3,3] = np.add(Tgoal[0:3,3], np.dot(-2*0.062,palm_perpendicular_vector))
                sol_2 = manip.FindIKSolution(Tgoal, IkFilterOptions.IgnoreEndEffectorCollisions)
                print "solution 2", sol_2
                robot.SetDOFValues(sol_2,manip.GetArmIndices())
                user_in = raw_input("Do you want to move practical robot? (y/n)")or "y"
                if user_in == "y":
                    success_vector = run_trial(ros_dict, sol_1, sol_2, hand_cmd)
                else:
                    success_vector = [-1]
                raw_input("Press [Enter] to continue")
                    #rospy.loginfo( "solution not found")
                    #robot.SetDOFValues([0,0,0,0,0,0,0],manip.GetArmIndices())
                
                
                done_grasp_matrix = np.append(done_grasp_matrix,np.array([[obj_num,sub_num,grasp_num,is_optimal,ext_opt_num, sum(success_vector)]]),axis = 0)

                np.savetxt(transform_path+'/done_grasp_testing.csv',done_grasp_matrix,delimiter=',')
                
    while not rospy.is_shutdown():
        n = 1

