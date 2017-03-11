This repository consist of programs to generate valid grasp in openrave. This package test the grasp on BarrettWAM and BarrettHand on physically. 
Dependencies:
	barrett_wam_grasp_capture_host



Definition of Valid grasp: Valid grasp is the grasp by robot hand in the simulation without any collision.

What does this package do?
This package retract the finges until the collision is avoided. If the grasp leverages the outside surface of fingers this package doesn't work as expected. It will retract the finger but give you an entirely different grasps. I am trying to fix that but it is not fixed yet

Folders Description:
1) essential_files: This folder consist of the initial transformation of the Table, Robot and transformation of the robot with respect to the some objects.


Before running the similar_grasp_generator.launch please run the command roscore and rosrun grasp_manager replay_extraction.py. 

You have to run similar_grasp_generator.launch before running object_visualizer_driver.py as the similar_grasp_generator will save the transformation of the BarrettHand and object a folder.


Roslaunch valid_grasp_generator generate_valid_grasp.launch
1) This requires a folder names grasping_data on the Home location. This folder is backed up somewhere on the robot grasp folder. I can't remember exact location now.
2) What it does?
	It looks through all the grasps that we collected in summer 2014. It identifies the collision between the robot hand and the 3D objects we have. After detecting the collision, It moves the fingers and robot hand little bit backwards in order to avoid the collision in Openrave simulator.

Rosrun valid_grasp_generator cluster_similar_grasp.py
1) This program clusters the similar grasp together. 

Roslaunch valid_grasp_generator similar_grasp_image_generator.launch
1) this program can be used to save the image generated from the similar grasp cluster



