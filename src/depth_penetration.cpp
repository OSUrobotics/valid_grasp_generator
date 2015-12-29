/*
 * =====================================================================================
 *
 *       Filename:  generate_valid_grasp.cpp
 *
 *    Description:  code for opening grasp in openrave and making required changes in 
 *                  object position, object scale and joint angles of Barrett Hand
 *
 *        Version:  1.0
 *        Created:  12/21/2015 05:14:33 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Saurabh Dixit (mn), dixits@oregonstate.edu
 *        Company:  Robotics, Oregon State University
 *
 * =====================================================================================
*/

#include "openrave-core.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "valid_grasp_generator/GraspSnapshot.h"
#include <iostream>
#include <vector>
#include <boost/python.hpp>
using namespace OpenRAVE;
using namespace std;

boost::python::list get_penetration_depth(string pointer)
{
    vector<double> vec(9,9);
    cout << "Value of the pointer: " << pointer << endl;
//code for getting penetration value from the openrave


    typename std::vector<double>::iterator iter;
    boost::python::list list;
    for (iter = vec.begin(); iter != vec.end(); ++iter) {
    	list.append(*iter);
    }
    return list;
}

BOOST_PYTHON_MODULE(libdepth_penetration){
    using namespace boost::python;
    def("get_penetration", get_penetration_depth);
}

