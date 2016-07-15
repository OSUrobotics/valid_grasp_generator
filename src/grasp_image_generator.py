#!/usr/bin/env python

import rospy
from Tkinter import *
from std_msgs.msg import String


def retract_fingers_commander():
    global pub
    pub.publish('retract_fingers')


def reflect_along_x_axis():
    global pub
    pub.publish('reflect_x')

def reflect_along_y_axis():
    global pub
    pub.publish('reflect_y')

def reflect_along_z_axis():
    global pub
    pub.publish('reflect_z')

def plot_contact_points():
    global pub
    pub.publish('plot_contact_points')

def set_camera_transform():
    global pub
    pub.publish('reorient_camera')

if __name__ == "__main__":
    rospy.init_node('image_generator',anonymous=True)
    rospy.loginfo('Image generator node online')
    pub= rospy.Publisher('Modify_and_snapshot',String, queue_size=1)
    master = Tk()
    retract_fingers = Button(master,text = "Retract Fingers", height=10,width=20, command = retract_fingers_commander)
    retract_fingers.grid(row=0,column=0)
    plot_contact_points = Button(master,text = "Plot contact points", height = 10,width = 20,command = plot_contact_points)
    plot_contact_points.grid(row=0,column=1)
    set_camera_transform = Button(master,text = "Reorient Camera", height = 10, width = 20, command = set_camera_transform)
    set_camera_transform.grid(row=0,column=2)
    reflect_along_x_axis = Button(master,text = "Reflect: X", height=10,width=20, command = reflect_along_x_axis)
    reflect_along_x_axis.grid(row=1,column=0)
    reflect_along_y_axis = Button(master,text = "Reflect: Y", height=10,width=20, command = reflect_along_y_axis)
    reflect_along_y_axis.grid(row=1,column=1)
    reflect_along_z_axis = Button(master,text = "Reflect: Z", height=10,width=20, command = reflect_along_z_axis)
    reflect_along_z_axis.grid(row=1,column=2)
    while not rospy.is_shutdown():
        master.update()

