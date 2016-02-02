#!/usr/bin/env python

from openravepy import *
from shared_global import grasping_data_folder, user
import numpy as np
import rospkg

class VisualizeGrasp:
    def __init__(self):
        self.path = rospkg.RosPack.get_path('valid_grasp_generator')
        self.env = Environment()
        self.env.Load(self.path+'/models/robots/bhand.dae')
        self.hand = self.env.GetRobots()[0]
        self.obj_num = None
        self.sub_num = None
        self.grasp_num = None
        self.ext_opt_num = None
        self.obj_name = None
        self.grasp_data_folder = grasping_data_folder

if __name__=="__main__":
    GraspShow = VisualizeGrasp()

