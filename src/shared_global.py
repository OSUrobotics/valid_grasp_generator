import rospy
import rospkg
import os
import getpass

valid_grasp_dir = rospkg.RosPack().get_path('valid_grasp_generator')
catkin_ws_location = valid_grasp_dir[:-26]
#obj_transform_dir = valid_grasp_dir+'/transformation_matrices/test_transform'
user = getpass.getuser()

obj_transform_dir = '/home/'+user+'/csvfiles/'


