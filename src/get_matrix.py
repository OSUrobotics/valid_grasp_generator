import numpy as np
from StringIO import *

def get_obj_matrix(transform):
    string_1 = transform[0][2:-2]
    string_2 = transform[1][2:-2]
    string_3 = transform[2][2:-2]
    string_4 = transform[3][2:-3]
    all_combined = [string_1,string_2,string_3,string_4]
    all_int_values = []
    for row in all_combined:
        all_int_values.append(np.genfromtxt(StringIO(row)))
    return all_int_values

def get_hand_matrix(transform):
    string_1 = transform[0][2:-2]
    string_2 = transform[1][2:-2]
    string_3 = transform[2][2:-2]
    string_4 = transform[3][2:-2]
    all_combined = [string_1,string_2,string_3,string_4]
    all_int_values = []
    for row in all_combined:
        all_int_values.append(np.genfromtxt(StringIO(row)))
    return all_int_values


def get_matrix(filename):
    fileid = open(filename,'r')
    lines = fileid.readlines()
    fileid.close()
    obj_transformation = lines[1:5]
    hand_transformation = lines[7:11]
    obj_matrix = get_obj_matrix(obj_transformation)
    hand_matrix = get_hand_matrix(hand_transformation)
    return {'obj_matrix':obj_matrix, 'hand_matrix':hand_matrix}

def get_index(index_matrix):
    for i in range(len(index_matrix)):
        if index_matrix[i][0] == True:
            if index_matrix[i][1] == True:
                if index_matrix[i][2] == True:
                    if index_matrix[i][3] == True:
                        if index_matrix[i][4] == True:
                            return i



