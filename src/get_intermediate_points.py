#!/usr/bin/env python

import numpy as np

def get_intermediate_points(points,alpha):
    first_point = points[0]
    second_point = points[1]
    intermediate_point = np.add(first_point, np.dot(np.subtract(second_point,first_point),alpha))
    return intermediate_point

if __name__ == "__main__":
    points = np.array([[0,0,0],[1,1,1]],dtype = np.float32)
    alpha_vector = np.arange(0,1.1,0.1)
    for alpha in alpha_vector:
        print get_intermediate_points(points,alpha)
    

