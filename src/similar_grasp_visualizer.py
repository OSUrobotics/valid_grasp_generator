#!/usr/bin/env python
from object_visualizer import *
from std_msgs.msg import Int32MultiArray,String
import numpy as np
import time
import scipy
import pyscreenshot
from scipy import misc
from valid_grasp_generator.srv import *
from Tkinter import *
from symmetricize import reflect_along_x_plane, reflect_along_y_plane, reflect_along_z_plane
import rospkg
import getpass
from stlwriter import *


class VisualizeSimilarGrasps(object):
    def __init__(self):
        self.transform_path = os.path.expanduser("~")+ "/grasping_data"
        self.pkg_path = rospkg.RosPack().get_path('valid_grasp_generator')
        self.camera_transform_path = self.pkg_path+'/essential_files/essential_transform/camera_transform.csv'
        self.camera_transform = np.genfromtxt(self.camera_transform_path,delimiter= ',')
        self.ctrl = object_visualizer()
        self.viewer = self.ctrl.env.GetViewer()
        self.folder_name = self.transform_path+'/output_data_folder/'
        self.folder_list = os.listdir(self.folder_name)
        self.obj_num = 0
        self.filename = None
        self.points = np.array([])
        self.point_plotted = False

    def update_environment(self):
        user_obj = raw_input('Please Enter object number you want to visualize: ') 
        for folder in self.folder_list:
            fold = folder.split('_')[0]
            self.obj_num = int(fold[3:])
            cluster_no = folder.split('_')[1]
            if self.obj_num  == int(user_obj):
                files = os.listdir(self.folder_name + "obj" +str(self.obj_num)+"_"+str(cluster_no) +"/")
                self.ctrl.set_obj(self.obj_num)
                sorted_files = []
                for fname in files:
                    if "_HandTransformation" in fname:
                        sorted_files.append(fname)
                
                new_files = sorted(sorted_files)
                
                for j in range(len(new_files)):
                    self.point_plotted = False
                    file_name = new_files[j]
                    sub_idx = file_name.find('_sub')
                    grasp_idx = file_name.find('_grasp')
                    sub_num = int(file_name[sub_idx+4:grasp_idx])
                    idx = file_name.find('_Hand')
                    filename =file_name[:idx]
                    f = self.folder_name +"obj"+str(self.obj_num)+'_'+str(cluster_no)+"/" +  filename
                    rospy.loginfo("Showing " + f)
                    T_hand = np.genfromtxt(f+"_HandTransformation.txt",delimiter = ',')
                    T_obj = np.genfromtxt(f+"_ObjTransformation.txt",delimiter = ',')
                    _ = self.ctrl.reorient_hand(T_hand, T_obj)
                    joint_angles = np.genfromtxt(f+"_JointAngles.txt",delimiter = ',') #for barrett arm
                    self.ctrl.set_joint_angles(joint_angles)
                    contact_links = np.genfromtxt(f+"_ContactLinkNames.txt",delimiter = ',',dtype = '|S')
                    self.points = np.genfromtxt(f+"_contactpoints.txt",delimiter = ',')
                    self.ctrl.PlotPoints(self.points)
                    self.point_plotted = True
                    self.filename = 'obj'+str(self.obj_num) +'_'+cluster_no+file_name[sub_idx:]
                    print
                    raw_input('Press Enter to continue')
                    #user_inp = raw_input('Do you want to write stl(y/n)?') or 'y' 
                    #if user_inp == 'y':
                    #    self.generate_stl(f+'.stl')
                    
                        

    def execute_command(self,data):
        rospy.loginfo('Command Received: %s',data.data)
        if data.data == 'reflect_x':
            reflect_along_x_plane(self.ctrl.env,self.ctrl.hand_1)
        elif data.data == 'reflect_y':
            reflect_along_y_plane(self.ctrl.env,self.ctrl.hand_1)
        elif data.data == 'reflect_z':
            reflect_along_z_plane(self.ctrl.env,self.ctrl.hand_1)
        elif data.data == 'plot_contact_points':
            if self.point_plotted:
                self.ctrl.remove_points()
                self.point_plotted = False
            else:
                self.ctrl.PlotPoints(self.points)
                self.point_plotted = True
        elif data.data == 'reorient_camera':
            self.camera_transform = np.genfromtxt(self.camera_transform_path,delimiter= ',')
            self.viewer.SetCamera(self.camera_transform)
        elif data.data == 'retract_fingers':
            self.points,_ = self.ctrl.avoid_hand_collision()
        elif data.data == 'take_picture':
            take_image = rospy.ServiceProxy('take_snap_shot',SnapShot)
            take_image('/home/'+getpass.getuser()+'/similar_grasp_images/'+self.filename+'.jpg')
        elif data.data == 'save_new_camera_transform':
            np.savetxt(self.camera_transform_path,self.viewer.GetCameraTransform(),delimiter=',')

    def get_robot_points(self,robot):
        robot.SetTransform(np.eye(4))
        links = [l for l in robot.GetLinks() if 'bhand' in l.GetName()]
        all_vertices = []
        all_faces = []
        ind = 0
        for link in links:
            vertices = link.GetCollisionData().vertices
            faces = link.GetCollisionData().indices
            #print "faces: ", len(faces), " vertices: ", len(vertices)
            if ind == 0:
                faces = np.add(faces,ind)
            else:
                faces = np.add(faces,ind+1)
            
            try:
                ind = faces[-1][-1]
            except:
                pass
            link_pose = poseFromMatrix(link.GetTransform())
            transform_vertices = poseTransformPoints(link_pose, vertices)
            all_vertices.extend(transform_vertices)
            all_faces.extend(faces.tolist())

        return all_vertices, all_faces

    def generate_stl(self,fname,robot=None,env=None):
        if robot == None:
            robot = self.ctrl.hand_1
        if env == None:
            env = self.ctrl.env

        [points, faces] = self.get_robot_points(robot)
        matrix = []
        for i in range(len(faces)):
            matrix.append([points[faces[i][0]],points[faces[i][1]],points[faces[i][2]]])

        with open(fname, 'wb') as fp:
            writer = Binary_STL_Writer(fp)
            writer.add_faces(matrix)
            writer.close()




if __name__ == "__main__":
    rospy.init_node('visualize_similar_grasp',anonymous = True)
    visualize_grasps = VisualizeSimilarGrasps()
    subscriber_commander = rospy.Subscriber('Modify_and_snapshot', String, visualize_grasps.execute_command)
    visualize_grasps.update_environment()


