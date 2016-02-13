#!/usr/bin/env python
import pyscreenshot
import rospy
from valid_grasp_generator.srv import *

def TakeSnapShot(file_name):
    img = pyscreenshot.grab(bbox = (100,30,1250,980))
    img.save(file_name.file_name)
    return SnapShotResponse(output = ("Saved Image: " + file_name.file_name))

if __name__ == "__main__":
    rospy.init_node('photographer')
    rospy.loginfo("photographer node online !")
    s = rospy.Service('take_snap_shot', SnapShot, TakeSnapShot)
    rospy.spin()
    

