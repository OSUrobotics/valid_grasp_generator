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
#include <openrave/openrave.h>
#include <openrave/utils.h>
#include <openrave/interface.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "valid_grasp_generator/GraspSnapshot.h"
#include <iostream>
#include <vector>
#include <boost/python.hpp>
#include <sstream>
#include <typeinfo>
using namespace OpenRAVE;
using namespace std;

GraphHandlePtr point_handler;

boost::python::list get_penetration_depth(string part_name)
{
    vector<double> vec(0);
    std::list<EnvironmentBasePtr> envs;
    RaveGetEnvironments(envs);
    EnvironmentBasePtr penv = envs.front();
    vector<RobotBasePtr> robots;
    cout << "Just before getting part: " << part_name << endl;
    KinBodyPtr part_kinbody = penv->GetKinBody(part_name);
    cout << "part_kinbody: " << part_kinbody << endl;
    KinBody::LinkConstPtr part = part_kinbody->GetLinks()[0];
    cout << "Just after getting part" << endl;
    penv->GetRobots(robots);
    RobotBasePtr Barrett = robots[0];
    vector<KinBody::LinkPtr> links = Barrett->GetLinks();
    cout << "part ptr: " << part << endl;
    cout << "part name: " << part->GetName() << endl;
    //vector<vector<double>> point_list;
    CollisionReportPtr report (new CollisionReport());
    for (int i=0; i<links.size(); i++){
        cout << "link " << i << " :" << links[i]->GetName() << endl;
        bool link_collision = penv->CheckCollision(links[i], part, report) ;
        //cout << "contact link depth" << report->contacts[0].depth << endl;
        //cout << "link " << i << "collision with part is " << link_collision << endl;
        for(int j=0; j<report->contacts.size() ; j++){
            cout << "link " << i << " penetration: " << report->contacts[j].depth << endl;
            vec.push_back((double) report->contacts[j].pos.x);
            vec.push_back((double) report->contacts[j].pos.y);
            vec.push_back((double) report->contacts[j].pos.z);
        }
    }
    
    //point_handler = penv->plot3(points.data(),points.size(),12,5);
    //delete<CollisionReportPtr> report;
    
           // cout << "vector of links" << links << endl;
    cout << "size of robots: " << robots.size() << endl;
    cout << "value of penv: " << penv << endl;

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

