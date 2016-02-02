import rospy
import rospkg
import os
import getpass

valid_grasp_dir = rospkg.RosPack().get_path('valid_grasp_generator')
catkin_ws_location = valid_grasp_dir[:-26]
user = getpass.getuser()

#obj_transform_dir = '/home/'+user+'/csvfiles/'
obj_transform_dir = '/media/'+user+'/FA648F24648EE2AD/obj_transforms/'


grasping_data_folder = '/home/'+user+'/grasping_data/'
