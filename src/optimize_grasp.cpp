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
#include "generate_valid_grasp/GraspSnapshot.h"


using namespace OpenRAVE;
using namespace std;

void import_data(const generate_valid_grasp::GraspSnapshot::ConstPtr& grasp){
    ROS_INFO("Got Grasp Extremes");
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"optimize_grasp");
    ros::NodeHandle n;
    ros::Subscriber grasp_extreme_sub = n.subscribe("grasp_extremes", 1000, import_data);
    ros::spin();
    return 0;
}
