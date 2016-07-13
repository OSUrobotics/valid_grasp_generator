#!/usr/bin/env python

import numpy as np
from openravepy import *
import transformations
from math import *
from stl_generator import *

def get_centroid(part):
    part_link = part.GetLinks()[0]
    part_points = part_link.GetCollisionData().vertices
    part_link_pose = poseFromMatrix(part.GetTransform())
    new_part_points = poseTransformPoints(part_link_pose,part_points)
    Centroid = np.mean(new_part_points,axis=0)
    return Centroid

def get_unit_vector(pt2,pt1):
    x = pt1[0] - pt2[0]
    y = pt1[1] - pt2[1]
    z = pt1[2] - pt2[2]
    mag = np.sqrt(x**2 + y**2 + z**2)
    return [x/mag,y/mag,z/mag]

def get_direction_vector(pt2,pt1):
    x = pt1[0] - pt2[0]
    y = pt1[1] - pt2[1]
    z = pt1[2] - pt2[2]
    return [x,y,z]

def get_angle(normal_1,normal_2):
    angle = np.arccos(np.clip(np.dot(normal_1,normal_2), -1, 1))
    return angle

def generate_rotation_matrix(normal_1,normal_2):
    M = np.eye(3)
    M[:,0] = normal_1
    M[:,2] = np.cross(normal_1,normal_2)
    #M[:,2] *= -1
    M[:,1] = np.cross(M[:,2],normal_1)
    return M

def reflect_along_y_plane(env,robot):
    '''
    Note: Only readable by author. No gaurantees for other programmer !!. Ha Ha: Mail me to saurabh.dxt21@gmail.com if you have trouble
    Uses implementation of householder transformation for mirroring some points
    Only for barrett hand when the initial position of the barrett hand is known. Initially in OpenRAVE, Barrett hand is facing towards [0,0,1]. This code mirrors the quaternion for the barrett hand then flips the quaternions by changing signs of the vector. This new quaternions is used to generate a transformation matrix. There is 180 degrees phase difference in the original position and virtual mirrored position. Generate the transformation from the [0,0,1] (which is the starting location ) and mirrored transformation of the barrett hand to get the current direction of the barrett hand face. Generate a rotation matrix from rotationMatrixFromAxisAngle command with inputs as direction vector and angle as pi to generate rotation matrix. Perform matrix multiplication of new rotation matrix and 
current mirrored transformation to get virtual mirrored location   '''
    robot_transform = robot.GetTransform()
    mirror_matrix = transformations.reflection_matrix([0,0,0],[0,1,0])
    mirror_transform = np.dot(mirror_matrix,robot_transform)
    robot_pose = poseFromMatrix(robot_transform)
    initial_points = np.array([robot_pose[1:4],robot_pose[4:]])
    initial_points = np.transpose(initial_points)
    new_points = np.dot(mirror_matrix[:3,:3],initial_points)
    new_points = np.transpose(new_points)
    new_points[0] *= -1
    rotation_matrix = rotationMatrixFromQuat(np.append(robot_pose[0],new_points.reshape(-1)))
    
    # Initial direction of hand face is [0,0,1]
    direction_vector = np.dot(rotation_matrix,[0,0,1])
    flip_matrix = rotationMatrixFromAxisAngle(direction_vector,pi)
    mirror_transform[:3,:3] = np.dot(flip_matrix,rotation_matrix)
    robot.SetTransform(mirror_transform)

def reflect_along_x_plane(env,robot):
    robot_transform = robot.GetTransform()
    mirror_matrix = transformations.reflection_matrix([0,0,0],[1,0,0])
    mirror_transform = np.dot(mirror_matrix,robot_transform)
    robot_pose = poseFromMatrix(robot_transform)
    initial_points = np.array([robot_pose[1:4],robot_pose[4:]])
    initial_points = np.transpose(initial_points)
    new_points = np.dot(mirror_matrix[:3,:3],initial_points)
    new_points = np.transpose(new_points)
    new_points[0] *= -1
    rotation_matrix = rotationMatrixFromQuat(np.append(robot_pose[0],new_points.reshape(-1)))
    
    # Initial direction of hand face is [0,0,1]
    mirror_transform[:3,:3] = rotation_matrix
    robot.SetTransform(mirror_transform)


def reflect_along_z_plane(env,robot):
    robot_transform = robot.GetTransform()
    mirror_matrix = transformations.reflection_matrix([0,0,0],[0,0,1])
    mirror_transform = np.dot(mirror_matrix,robot_transform)
    robot_pose = poseFromMatrix(robot_transform)
    initial_points = np.array([robot_pose[1:4],robot_pose[4:]])
    initial_points = np.transpose(initial_points)
    new_points = np.dot(mirror_matrix[:3,:3],initial_points)
    new_points = np.transpose(new_points)
    new_points[0] *= -1
    rotation_matrix = rotationMatrixFromQuat(np.append(robot_pose[0],new_points.reshape(-1)))
    
    # Initial direction of hand face is [0,0,1]
    direction_vector = np.dot(rotation_matrix,[0,1,0])
    flip_matrix = rotationMatrixFromAxisAngle(direction_vector,pi)
    mirror_transform[:3,:3] = np.dot(flip_matrix,rotation_matrix)
    robot.SetTransform(mirror_transform)


if __name__ == "__main__":
    env = Environment()
    env.SetViewer('qtcoin')
    env.Load('trial_env_1.dae')
    user_in = raw_input("Plese press Enter to proceed: ")
    obj = env.GetBodies()[1]
    robot = env.GetRobots()[0]

    reflect_along_z_plane(env,robot)
