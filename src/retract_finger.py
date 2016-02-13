#!/usr/bin/env python

from angle_format_changer import *
from openravepy import *
import numpy as np
from scipy.interpolate import interp1d
import rospy
import time
import os

class ContPointWithDistance():
    def __init__(self,name="Not Specified",MinDist=5,NearestPoint=np.array([0,0,0])):
        self.MinDistance = MinDist
        self.ContactPoint = NearestPoint 
        self.name = name

    def __str__(self):
        return self.name 
    
def get_unit_vector(pt2,pt1):
    x = pt1[0] - pt2[0]
    y = pt1[1] - pt2[1]
    z = pt1[2] - pt2[2]
    mag = np.sqrt(x**2 + y**2 + z**2)
    return [x/mag,y/mag,z/mag]
    
def set_hand_joint_angles(hand,joint_angles):
    # This function is specific to this class.
    hand_dof_limits = hand.GetDOFLimits()
    try:
        dist_mapper = interp1d([0,hand_dof_limits[1][3]],[0,hand_dof_limits[1][4]])
        hand_dof_values = hand.GetDOFValues()
        #print hand_dof_values
        hand_dof_values[3:11] = [joint_angles[0],dist_mapper(joint_angles[0]),hand_dof_values[5],joint_angles[1],dist_mapper(joint_angles[1]),joint_angles[2],dist_mapper(joint_angles[2])]
        hand.SetDOFValues(hand_dof_values)

    except ValueError:
        pass

def get_perpendicular_vector(vec1,vec2):
    return np.cross(vec2,vec1)

def get_centroid(contact_list):
    if len(contact_list)>1:
        new_array = np.array([[0,0,0]])
        for contact in contact_list:
            new_array = np.append(new_array,[contact.pos],axis=0)
        new_array = np.delete(new_array,0,axis=0)
        return np.mean(new_array,axis=0)

    for contact in contact_list:
        return contact.pos

    
 
def retract_fingers(env,hand,part):
    if not env.GetCollisionChecker().SetCollisionOptions(CollisionOptions.Distance | CollisionOptions.Contacts):
        collisionChecker = RaveCreateCollisionChecker(env,'pqp')
        collisionChecker.SetCollisionOptions(CollisionOptions.Distance|CollisionOptions.Contacts)
        env.SetCollisionChecker(collisionChecker)
    report = CollisionReport()
    links = hand.GetLinks()
    finger_1_prox = links[3]
    finger_1_med = links[4]
    finger_1_dist = links[5]
    finger_2_prox = links[7]
    finger_2_med = links[8]
    finger_2_dist = links[9]
    finger_3_med = links[11]
    finger_3_dist = links[12]
    palm_link = links[0]
    palm_surface_link = links[11]

    palm_contact = ContPointWithDistance("palm contact")
    finger_1_prox_contact = ContPointWithDistance("finger_1_prox")
    finger_1_med_contact = ContPointWithDistance("finger_1_med")
    finger_1_dist_contact = ContPointWithDistance("finger_1_dist")
    finger_2_prox_contact = ContPointWithDistance("finger_2_prox")
    finger_2_med_contact = ContPointWithDistance("finger_2_med")
    finger_2_dist_contact = ContPointWithDistance("finger_2_dist")
    finger_3_med_contact = ContPointWithDistance("finger_3_med")
    finger_3_dist_contact = ContPointWithDistance("finger_3_dist")

    ContactPointWithDistance = [palm_contact, finger_1_prox_contact, finger_1_med_contact, finger_1_dist_contact, finger_2_prox_contact, finger_2_med_contact, finger_2_dist_contact, finger_3_med_contact,finger_3_dist_contact]

    minDistance_of_finger = 0.0015
    flag_palm  = False
    flag_finger_1 = False
    flag_finger_2 = False

    flag_finger_1_dist = False
    flag_finger_1_med = False
    flag_finger_2_dist = False
    flag_finger_2_med = False
    flag_finger_3_dist = False
    flag_finger_3_med = False

    flag_finger_1_closure = False
    flag_finger_2_closure = False
    flag_finger_3_closure = False

    hand_dof_limits = hand.GetDOFLimits()
    mapper = interp1d([0,hand_dof_limits[1][3]+hand_dof_limits[1][4]],[0,hand_dof_limits[1][3]])


    translation_done = False
    translational_threshold = 0.0005
    joint_retract_threshold = 0.003
    finger_retracted = False

    hand_all_link_collision_check = np.array([False,False,False,False,False,False,False,False,False])
    user_input = raw_input("Do you want to retract it? (y/n)") or "y"
    points = np.array([[0,0,0]])
    try:
        if user_input == "y":
            current_hand_transform = hand.GetTransform()
            hand_position = current_hand_transform[0:3,3]
            euler_angles = mat2euler(current_hand_transform[0:3,0:3])
            hand_quaternion = euler2quat(euler_angles[0],euler_angles[1],euler_angles[2])
            hand_link = hand.GetLinks()[0]
            #print hand_link
            palm_points = hand_link.GetCollisionData().vertices
            palm_link_pose = poseFromMatrix(palm_link.GetTransform())
            transformed_points = poseTransformPoints(palm_link_pose, palm_points)
            palm_center = np.mean(np.array([transformed_points[3687],transformed_points[3875],transformed_points[3605],transformed_points[3782]]), axis = 0)
            palm_perpendicular_vector = get_perpendicular_vector(get_unit_vector(transformed_points[3687],palm_center),get_unit_vector(transformed_points[3605],palm_center))
            point_along_palm_perpendicular_vector = np.add(palm_center,np.dot(-0.1,palm_perpendicular_vector))
            palm_link = hand.GetLinks()[0]
            start_time = time.time()
            while not finger_retracted:
                rospy.set_param("Ready_for_input",False)
                hand_dof = hand.GetDOFValues()
                finger_1_dof_value = mapper(hand_dof[3]+hand_dof[4])
                finger_2_dof_value = mapper(hand_dof[6]+hand_dof[7])
                finger_3_dof_value = mapper(hand_dof[8]+hand_dof[9])
                finger_spread = hand_dof[2]
                output_dof_vals = np.array([finger_1_dof_value,finger_2_dof_value,finger_3_dof_value,finger_spread])
           

                # variables for changing joint angles values
                finger_1_joint_angles = finger_1_dof_value
                finger_2_joint_angles = finger_2_dof_value
                finger_3_joint_angles = finger_3_dof_value

                # Translation of the object away from the palm

                palm_vs_part = env.CheckCollision(part,palm_link,report = report)
                palm_contact.MinDistance = report.minDistance
                hand_all_link_collision_check[0] = palm_vs_part
                #print "palm",palm_vs_part
                
                contact_points = report.contacts
                contact_points_list = np.array([[0,0,0]])
                if palm_vs_part:
                    flag_palm = False
                    for contact in contact_points:
                        contact_points_list = np.append(contact_points_list, [contact.pos],axis = 0)

                    contact_points_list = np.delete(contact_points_list, 0,axis=0)
                    palm_contact.ContactPoint = np.mean(contact_points_list,axis=0)

                    translation_unit_vector = get_unit_vector(point_along_palm_perpendicular_vector, palm_contact.ContactPoint)
                    hand_transform = hand.GetTransform()
                    hand_transform[0,3] = (hand_transform[0,3] + translational_threshold*translation_unit_vector[0])
                    hand_transform[1,3] = (hand_transform[1,3] + translational_threshold*translation_unit_vector[1])
                    hand_transform[2,3] = (hand_transform[2,3] + translational_threshold*translation_unit_vector[2])
                    hand.SetTransform(hand_transform)
                else:
                    flag_palm = True
                finger_1_prox_vs_part = env.CheckCollision(part,finger_1_prox,report = report)
                finger_1_prox_contact.MinDistance = report.minDistance
                hand_all_link_collision_check[1]=finger_1_prox_vs_part
                contact_points = report.contacts
                contact_points_list = np.array([[0,0,0]])

                if finger_1_prox_vs_part:
                    flag_finger_1 = False
                    for contact in contact_points:
                        contact_points_list = np.append(contact_points_list, [contact.pos],axis = 0)

                    contact_points_list = np.delete(contact_points_list, 0,axis=0)
                    finger_1_prox_contact.ContactPoint = np.mean(contact_points_list,axis = 0)
                    translation_unit_vector = get_unit_vector(point_along_palm_perpendicular_vector, finger_1_prox_contact.ContactPoint)
                    hand_transform = hand.GetTransform()
                    hand_transform[0,3] = (hand_transform[0,3] + translational_threshold*translation_unit_vector[0])
                    hand_transform[1,3] = (hand_transform[1,3] + translational_threshold*translation_unit_vector[1])
                    hand_transform[2,3] = (hand_transform[2,3] + translational_threshold*translation_unit_vector[2])
                    hand.SetTransform(hand_transform)
                else:
                    flag_finger_1 = True
                finger_2_prox_vs_part = env.CheckCollision(part,finger_2_prox,report=report)                 
                finger_2_prox_contact.MinDistance = report.minDistance
                hand_all_link_collision_check[2]=finger_2_prox_vs_part
                contact_points_list = np.array([[0,0,0]])
                if finger_2_prox_vs_part:
                    flag_finger_2 = False
                    for contact in contact_points:
                        contact_points_list = np.append(contact_points_list, [contact.pos],axis = 0)

                    contact_points_list = np.delete(contact_points_list, 0,axis=0)
                    finger_2_prox_contact.ContactPoint = np.mean(contact_points_list,axis = 0)
                    
                    translation_unit_vector = get_unit_vector(point_along_palm_perpendicular_vector, finger_2_prox_contact.ContactPoint)
                    hand_transform = hand.GetTransform()
                    hand_transform[0,3] = (hand_transform[0,3] + translational_threshold*translation_unit_vector[0])
                    hand_transform[1,3] = (hand_transform[1,3] + translational_threshold*translation_unit_vector[1])
                    hand_transform[2,3] = (hand_transform[2,3] + translational_threshold*translation_unit_vector[2])
                    hand.SetTransform(hand_transform)
                else:
                    flag_finger_2 = True
                

                if flag_palm and flag_finger_1 and flag_finger_2:
                    translation_done = True

                # Move the fingers away from the object
                if translation_done:
                    finger_1_med_vs_part = env.CheckCollision(part,finger_1_med,report = report)
                    finger_1_med_contact.MinDistance = report.minDistance
                    hand_all_link_collision_check[3]=finger_1_med_vs_part
                    #print "finger 1 med", finger_1_med_vs_part
                    
                    contact_points = report.contacts
                    contact_points_list = np.array([[0,0,0]])
                    if finger_1_med_contact.MinDistance < minDistance_of_finger and not finger_1_med_vs_part:
                        for contact in contact_points:
                            contact_points_list = np.append(contact_points_list, [contact.pos],axis = 0)

                        contact_points_list = np.delete(contact_points_list, 0,axis=0)
                        finger_1_med_contact.ContactPoint = np.mean(contact_points_list, axis=0)
                        flag_finger_1_med = True
                        flag_finger_1_closure = True
                    elif not flag_finger_1_med and finger_1_med_vs_part:
                        finger_1_joint_angles = finger_1_joint_angles -  joint_retract_threshold
                        set_hand_joint_angles(hand,[finger_1_joint_angles,finger_2_joint_angles,finger_3_joint_angles])

                    finger_1_dist_vs_part = env.CheckCollision(part,finger_1_dist,report = report)
                    finger_1_dist_contact.MinDistance = report.minDistance
                    hand_all_link_collision_check[4]=finger_1_dist_vs_part
                    #print "finger 1 distal",finger_1_dist_vs_part
                    
                    contact_points = report.contacts
                    contact_points_list = np.array([[0,0,0]])
                    if finger_1_dist_contact.MinDistance < minDistance_of_finger and not finger_1_dist_vs_part:
                        for contact in contact_points:
                            contact_points_list = np.append(contact_points_list, [contact.pos],axis = 0)

                        contact_points_list = np.delete(contact_points_list, 0,axis=0)
                        finger_1_dist_contact.ContactPoint = np.mean(contact_points_list,axis=0)
                        flag_finger_1_dist = True
                    elif not flag_finger_1_dist and finger_1_dist_vs_part:
                        finger_1_joint_angles = finger_1_joint_angles - joint_retract_threshold
                        set_hand_joint_angles(hand,[finger_1_joint_angles,finger_2_joint_angles,finger_3_joint_angles])

                    finger_2_med_vs_part = env.CheckCollision(part,finger_2_med,report = report)
                    finger_2_med_contact.MinDistance = report.minDistance
                    hand_all_link_collision_check[5]=finger_2_med_vs_part
                    #print "finger 2 med",finger_2_med_vs_part
                    
                    contact_points = report.contacts
                    contact_points_list = np.array([[0,0,0]])
                    if finger_2_med_contact.MinDistance < minDistance_of_finger and not finger_2_med_vs_part:
                        for contact in contact_points:
                            contact_points_list = np.append(contact_points_list, [contact.pos],axis = 0)

                        contact_points_list = np.delete(contact_points_list, 0,axis=0)
                        finger_2_med_contact.ContactPoint = np.mean(contact_points_list,axis=0)
                        flag_finger_2_med = True
                        flag_finger_2_closure = True
                    elif not flag_finger_2_med and finger_2_med_vs_part:
                        finger_2_joint_angles = finger_2_joint_angles - joint_retract_threshold
                        set_hand_joint_angles(hand,[finger_1_joint_angles,finger_2_joint_angles,finger_3_joint_angles])


                    finger_2_dist_vs_part = env.CheckCollision(part,finger_2_dist,report = report)
                    finger_2_dist_contact.MinDistance = report.minDistance
                    hand_all_link_collision_check[6]=finger_2_dist_vs_part
                    #print "finger 2 dist",finger_2_dist_vs_part
                    
                    contact_points = report.contacts
                    contact_points_list = np.array([[0,0,0]])
                    if finger_2_dist_contact.MinDistance < minDistance_of_finger and not finger_2_dist_vs_part:
                        for contact in contact_points:
                            contact_points_list = np.append(contact_points_list, [contact.pos],axis = 0)

                        contact_points_list = np.delete(contact_points_list, 0,axis=0)
                        finger_2_dist_contact.ContactPoint = np.mean(contact_points_list,axis=0)
                        flag_finger_2_dist = True
                    elif not flag_finger_2_dist and finger_2_dist_vs_part:
                        finger_2_joint_angles = finger_2_joint_angles - joint_retract_threshold
                        set_hand_joint_angles(hand,[finger_1_joint_angles,finger_2_joint_angles,finger_3_joint_angles])

                    finger_3_med_vs_part = env.CheckCollision(part,finger_3_med,report = report)
                    finger_3_med_contact.MinDistance = report.minDistance
                    hand_all_link_collision_check[7]=finger_3_med_vs_part
                    #print "finger 3 med",finger_3_med_vs_part
                    
                    contact_points = report.contacts
                    contact_points_list = np.array([[0,0,0]])
                    if finger_3_med_contact.MinDistance < minDistance_of_finger and not finger_3_med_vs_part:
                        for contact in contact_points:
                            contact_points_list = np.append(contact_points_list, [contact.pos],axis = 0)

                        contact_points_list = np.delete(contact_points_list, 0,axis=0)
                        finger_3_med_contact.ContactPoint = np.mean(contact_points_list,axis=0)
                        flag_finger_3_med = True
                        flag_finger_3_closure = True
                    elif not flag_finger_3_med and finger_3_med_vs_part:
                        finger_3_joint_angles = finger_3_joint_angles - joint_retract_threshold
                        set_hand_joint_angles(hand,[finger_1_joint_angles,finger_2_joint_angles,finger_3_joint_angles])

                    finger_3_dist_vs_part = env.CheckCollision(part,finger_3_dist,report = report)
                    finger_3_dist_contact.MinDistance = report.minDistance                                                 
                    hand_all_link_collision_check[8]=finger_3_dist_vs_part
                    #print "finger 3 dist",finger_3_dist_vs_part
                    
                    contact_points = report.contacts
                    contact_points_list = np.array([[0,0,0]])
                    if finger_3_dist_contact.MinDistance < 0.002 and not finger_3_dist_vs_part:
                        for contact in contact_points:
                            contact_points_list = np.append(contact_points_list, [contact.pos],axis = 0)

                        contact_points_list = np.delete(contact_points_list, 0,axis=0)
                        finger_3_dist_contact.ContactPoint = np.mean(contact_points_list,axis=0)
                        flag_finger_3_dist = True
                    elif not flag_finger_3_dist and finger_3_dist_vs_part:
                        finger_3_joint_angles = finger_3_joint_angles - joint_retract_threshold
                        set_hand_joint_angles(hand,[finger_1_joint_angles,finger_2_joint_angles,finger_3_joint_angles])

                    
                    # I am using palm contact for finger 3 proximal joint. I did this because finger 3 proximal is the part of palm
                    #print "finger 3 Proximal",palm_vs_part
                    

                    #overall_collision_check = env.CheckCollision(hand,report = report)
                    #print "overall hand collision with part: ",hand_all_link_collision_check.any()

                    if not (hand_all_link_collision_check.any()):
                        #print "Fingers Retracted"
                        finger_retracted = True

                    new_time = time.time()
                    #print "time difference",new_time-start_time
                    if (new_time-start_time)>40:
                        #print "timeout"
                        finger_retracted = True
                    #elif overall_collision_check:
                    #    if report.plink2.GetParent().GetName()el== ''

            finger_closure = False
            finger_closure_start = time.time()
            while not finger_closure:
                finger_1_dist_vs_part = env.CheckCollision(part,finger_1_dist,report = report)
                finger_1_dist_contact.MinDistance = report.minDistance
                ##print flag_finger_1_closure, flag_finger_2_closure, flag_finger_3_closure
                ##print hand.GetDOFValues()[11],hand.GetDOFValues()[14],hand.GetDOFValues()[16]
                if hand.GetDOFValues()[4] >= 0.82:
                    flag_finger_1_closure = False
                if hand.GetDOFValues()[7] >= 0.82:
                    flag_finger_2_closure = False
                if hand.GetDOFValues()[9] >= 0.82:
                    flag_finger_3_closure = False
                hand_dof_values = hand.GetDOFValues()
                if finger_1_dist_contact.MinDistance > minDistance_of_finger and flag_finger_1_closure:
                    hand_dof_values[4] = hand_dof_values[4] + joint_retract_threshold/5
                elif finger_1_dist_contact.MinDistance <= minDistance_of_finger:
                    flag_finger_1_closure = False
                
                finger_2_dist_vs_part = env.CheckCollision(part,finger_2_dist,report = report)
                finger_2_dist_contact.MinDistance = report.minDistance
                if finger_2_dist_contact.MinDistance > minDistance_of_finger and flag_finger_2_closure:
                    hand_dof_values[7] = hand_dof_values[7] + joint_retract_threshold/5
                elif finger_2_dist_contact.MinDistance <= minDistance_of_finger:
                    flag_finger_2_closure = False
                    
                finger_3_dist_vs_part = env.CheckCollision(part,finger_3_dist,report = report)
                finger_3_dist_contact.MinDistance = report.minDistance
                if finger_3_dist_contact.MinDistance > minDistance_of_finger and flag_finger_3_closure:
                    hand_dof_values[9] = hand_dof_values[9] + joint_retract_threshold/5
                elif finger_3_dist_contact.MinDistance <= minDistance_of_finger:
                    flag_finger_3_closure = False
                
                finger_closure_end = time.time()
                hand.SetDOFValues(hand_dof_values)
                if not flag_finger_1_closure and not flag_finger_2_closure and not flag_finger_3_closure:
                    #print "Executed finger closure"
                    finger_closure = True


            # Save everything to file
            #if not os.path.exists(data_saving_folder):
            #    os.makedirs(data_saving_folder)
            #objno_subno = data_saving_folder+'obj'+str(obj_num)+'_sub'+str(sub_num)
            #if not os.path.exists(objno_subno):
            #    os.makedirs(objno_subno)

            # I have to do below dumb part because there is some error for recoroding contact point.
        palm_vs_part = env.CheckCollision(palm_link,part,report = report)
        palm_contact.ContactPoint = get_centroid(report.contacts)
        palm_contact.MinDistance = report.minDistance
        finger_1_prox_vs_part = env.CheckCollision(finger_1_prox,part,report=report)
        finger_1_prox_contact.ContactPoint = get_centroid(report.contacts)
        finger_1_prox_contact.MinDistance = report.minDistance
        finger_1_med_vs_part = env.CheckCollision(finger_1_med,part,report=report)
        finger_1_med_contact.ContactPoint = get_centroid(report.contacts)
        finger_1_med_contact.MinDistance = report.minDistance
        finger_1_dist_vs_part = env.CheckCollision(finger_1_dist,part,report=report)
        finger_1_dist_contact.ContactPoint = get_centroid(report.contacts)
        finger_1_dist_contact.MinDistance = report.minDistance
        finger_2_prox_vs_part = env.CheckCollision(finger_2_prox,part,report=report)
        finger_2_prox_contact.ContactPoint = get_centroid(report.contacts)
        finger_2_prox_contact.MinDistance = report.minDistance
        finger_2_med_vs_part = env.CheckCollision(finger_2_med,part,report=report)
        finger_2_med_contact.ContactPoint = get_centroid(report.contacts)
        finger_2_med_contact.MinDistance = report.minDistance
        finger_2_dist_vs_part = env.CheckCollision(finger_2_dist,part,report=report)
        finger_2_dist_contact.ContactPoint = get_centroid(report.contacts)
        finger_2_dist_contact.MinDistance = report.minDistance
        finger_3_med_vs_part = env.CheckCollision(finger_3_med,part,report=report)
        finger_3_med_contact.ContactPoint = get_centroid(report.contacts)
        finger_3_med_contact.MinDistance = report.minDistance
        finger_3_dist_vs_part = env.CheckCollision(finger_3_dist,part,report=report)
        finger_3_dist_contact.ContactPoint = get_centroid(report.contacts)
        finger_3_dist_contact.MinDistance = report.minDistance
        
        hand_dof = hand.GetDOFValues()
        finger_1_dof_value = mapper(hand_dof[3]+hand_dof[4])
        finger_2_dof_value = mapper(hand_dof[6]+hand_dof[7])
        finger_3_dof_value = mapper(hand_dof[8]+hand_dof[9])
        finger_spread = hand_dof[0]
        output_dof_vals = np.array([finger_1_dof_value,finger_2_dof_value,finger_3_dof_value,finger_spread])
        
        part_link = part.GetLinks()[0]
        part_points = part_link.GetCollisionData().vertices
        part_link_pose = poseFromMatrix(part.GetTransform())
        new_part_points = poseTransformPoints(part_link_pose, part_points)
        COG_part = np.mean(new_part_points,axis =0)

        #print "palm_contact : ",palm_contact.ContactPoint,",  ", palm_contact.MinDistance
        #print "finger_1_prox: ",finger_1_prox_contact.ContactPoint,",  ",finger_1_prox_contact.MinDistance
        #print "finger_1_med_: ",finger_1_med_contact.ContactPoint ,",  ",finger_1_med_contact.MinDistance
        #print "finger_1_dist: ",finger_1_dist_contact.ContactPoint,",  ",finger_1_dist_contact.MinDistance
        #print "finger_2_prox: ",finger_2_prox_contact.ContactPoint,",  ",finger_2_prox_contact.MinDistance
        #print "finger_2_med_: ",finger_2_med_contact.ContactPoint ,",  ",finger_2_med_contact.MinDistance
        #print "finger_2_dist: ",finger_2_dist_contact.ContactPoint,",  ",finger_2_dist_contact.MinDistance
        #print "finger_3_med_: ",finger_3_med_contact.ContactPoint ,",  ",finger_3_med_contact.MinDistance
        #print "finger_3_dist: ",finger_3_dist_contact.ContactPoint,",  ",finger_3_dist_contact.MinDistance

        #print "Links that were in contact: "
        contact_links_names = np.array([], dtype = "|S")
        for contact in ContactPointWithDistance:
            if not contact.ContactPoint.all() == 0 and contact.MinDistance < minDistance_of_finger:
                #print str(contact)
                contact_links_names = np.append(contact_links_names,str(contact))
                points = np.append(points,[contact.ContactPoint],axis=0)
        
        points = np.delete(points,0,axis=0)
        print "Points that are plotted", points
#        np.savetxt(objno_subno+'/'+file_name+'_COG.txt',COG_part,delimiter=',')
#        np.savetxt(objno_subno+'/'+file_name+'_hand_position.txt',hand_position,delimiter=',')
#        np.savetxt(objno_subno+'/'+file_name+'_hand_quaternion.txt',hand_quaternion,delimiter=',')
#        np.savetxt(objno_subno+'/'+file_name+'_closeddofvals.txt',output_dof_vals,delimiter=',')
#        np.savetxt(objno_subno+'/'+file_name+'_contactpoints.txt',points,delimiter=',')
#        np.savetxt(objno_subno+'/'+file_name+'_JointAngles.txt',hand.GetDOFValues(),delimiter=',')
#        np.savetxt(objno_subno+'/'+file_name+'_HandTransformation.txt',current_hand_transform,delimiter = ',')
#        np.savetxt(objno_subno+'/'+file_name+'_ObjTransformation.txt',part.GetTransform(),delimiter = ',')
#        np.savetxt(objno_subno+'/'+file_name+'_ContactLinkNames.txt',contact_links_names,delimiter = ',',fmt = "%s")
#        if is_optimal:
#            csv_writer.writerow(["obj"+str(obj_num),"sub"+str(sub_num),"grasp"+str(grasp_num),"optimal"+str(ext_opt_num), output_dof_vals.tolist(), COG_part.tolist(), points.tolist(),hand_position.tolist(),hand_quaternion.tolist()])
#        else:
#            csv_writer.writerow(["obj"+str(obj_num),"sub"+str(sub_num),"grasp"+str(grasp_num),"extreme"+str(ext_opt_num), output_dof_vals.tolist(), COG_part.tolist(), points.tolist(),hand_position.tolist(),hand_quaternion.tolist()])
#        output_file_id.close()
#        rospy.set_param("Things_done",True)
        return points
#        
    except rospy.ROSInterruptException, e:
        #print 'exiting', e
        sys.exit()

