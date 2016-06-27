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

