/*********************************************************************

 * Software License Agreement (BSD License)

 *

 *  Copyright (c) 2013, SRI International

 *  All rights reserved.

 *

 *  Redistribution and use in source and binary forms, with or without

 *  modification, are permitted provided that the following conditions

 *  are met:

 *

 *   * Redistributions of source code must retain the above copyright

 *     notice, this list of conditions and the following disclaimer.

 *   * Redistributions in binary form must reproduce the above

 *     copyright notice, this list of conditions and the following

 *     disclaimer in the documentation and/or other materials provided

 *     with the distribution.

 *   * Neither the name of SRI International nor the names of its

 *     contributors may be used to endorse or promote products derived

 *     from this software without specific prior written permission.

 *

 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS

 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT

 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS

 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE

 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,

 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,

 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;

 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER

 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT

 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN

 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE

 *  POSSIBILITY OF SUCH DAMAGE.

 *********************************************************************/



/* Author: Oliver Bosin*/


#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2/LinearMath/Vector3.h>

#include <moveit_msgs/DisplayRobotState.h>

#include <moveit_msgs/DisplayTrajectory.h>



#include <moveit_msgs/AttachedCollisionObject.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>



int main(int argc, char** argv){

  ros::init(argc, argv, "move_group_interface");

  ros::NodeHandle node_handle;

  tf2::Vector3 old_torso_to_hand;
  old_torso_to_hand[0] = 0.0;
  old_torso_to_hand[1] = 0.0;
  old_torso_to_hand[2] = 0.0;
  tf2::Vector3 new_torso_to_hand;

  ros::AsyncSpinner spinner(1);

  spinner.start();


  static const std::string PLANNING_GROUP_ARM = "roboter_arm";
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";



  moveit::planning_interface::MoveGroupInterface move_group_arm(PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(PLANNING_GROUP_GRIPPER);

  // Raw pointers are frequently used to refer to the planning group for improved performance.

  const robot_state::JointModelGroup* joint_model_group_arm =

      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

  const robot_state::JointModelGroup* joint_model_group_gripper =

        move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);

    move_group_arm.clearPoseTargets();
	move_group_arm.setStartStateToCurrentState();
	move_group_arm.setPoseReferenceFrame("base_link");

	move_group_arm.setGoalTolerance(0.005);
	move_group_arm.setPlanningTime(2);

	move_group_gripper.clearPoseTargets();
	move_group_gripper.setStartStateToCurrentState();
	move_group_gripper.setPoseReferenceFrame("base_link");

	move_group_gripper.setGoalTolerance(0.005);
	move_group_gripper.setPlanningTime(2);

	tf2_ros::Buffer tfBuffer;

	/* Here a TransformListener object is created.
			 * Once the listener is created it starts receiving tf2 transformations over the wire
			 * and buffers them for up to 10 seconds*/

	tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::TransformStamped transformStamped_left_hand_1;
	geometry_msgs::TransformStamped transformStamped_right_hand_1;
	geometry_msgs::TransformStamped transformStamped_shoulder_to_elbow;
    geometry_msgs::TransformStamped transformStamped_elbow_to_hand;

	/*get transform from shoulder to elbow and elbow to hand for armlength*/
    while (node_handle.ok()){

    	    try{

    	    	transformStamped_shoulder_to_elbow = tfBuffer.lookupTransform("left_shoulder_1", "left_elbow_1",
    	    			                               ros::Time(0));

    	    	transformStamped_elbow_to_hand = tfBuffer.lookupTransform("left_elbow_1","left_hand_1",
    	    			                               ros::Time(0));
    	    }
    	    catch (tf2::TransformException &ex) {
    	      ROS_WARN("%s",ex.what());


    	      continue;
    	    }
    	    break;
    }

    /*convert from geometry_msgs::Vector3 to tf2::Vector3 so the use of the length() function is possible*/

    tf2::Vector3 shoulder_to_elbow_vector;
	tf2::Vector3 elbow_to_hand_vector;

	shoulder_to_elbow_vector[0] =  transformStamped_shoulder_to_elbow.transform.translation.x;
	shoulder_to_elbow_vector[1] =  transformStamped_shoulder_to_elbow.transform.translation.y;
	shoulder_to_elbow_vector[2] =  transformStamped_shoulder_to_elbow.transform.translation.z;

	elbow_to_hand_vector[0] =  transformStamped_elbow_to_hand.transform.translation.x;
	elbow_to_hand_vector[1] =  transformStamped_elbow_to_hand.transform.translation.y;
	elbow_to_hand_vector[2] =  transformStamped_elbow_to_hand.transform.translation.z;

	/*calculate armlength of user*/

	double armlength = shoulder_to_elbow_vector.length() + elbow_to_hand_vector.length();

	/*calculate conversion factor for position target*/

	double conversion_factor = 0.40 / armlength;


	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	moveit::core::RobotStatePtr current_state;
	std::vector<double> joint_group_positions;

	ros::Rate rate(20.0);

	/*main loop*/

	while (node_handle.ok()){

		try{

	      transformStamped_left_hand_1 = tfBuffer.lookupTransform("torso_1", "left_hand_1",
	                               ros::Time(0));
	      transformStamped_right_hand_1 = tfBuffer.lookupTransform("right_shoulder_1", "right_hand_1",
	      	                               ros::Time(0));
	    }
	    catch (tf2::TransformException &ex) {
	      ROS_WARN("%s",ex.what());


	      continue;
	    }



	    new_torso_to_hand[0] = transformStamped_left_hand_1.transform.translation.x;
	    new_torso_to_hand[1] = transformStamped_left_hand_1.transform.translation.y;
	    new_torso_to_hand[2] = transformStamped_left_hand_1.transform.translation.z;


  // Planning to a position target

  // ^^^^^^^^^^^^^^^^^^^^^^^

  // We can plan a motion for this group to a desired position target for the

  // end-effector.
 



  if((old_torso_to_hand.distance(new_torso_to_hand)) > 0.025){

	  move_group_arm.setPositionTarget(conversion_factor * transformStamped_left_hand_1.transform.translation.x - 0.12, conversion_factor * transformStamped_left_hand_1.transform.translation.z*(-1), conversion_factor * transformStamped_left_hand_1.transform.translation.y);


	  ROS_INFO_NAMED("planning","x : %lf y: %lf z: %lf",conversion_factor * transformStamped_left_hand_1.transform.translation.x, conversion_factor * transformStamped_left_hand_1.transform.translation.z*(-1), conversion_factor * transformStamped_left_hand_1.transform.translation.y );
	  ROS_INFO_NAMED("distance","distance to target: %lf", old_torso_to_hand.distance(new_torso_to_hand));

	  bool succes = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

	  if(succes){
		  succes = false;
		  joint_group_positions = my_plan.trajectory_.joint_trajectory.points.back().positions;
		  joint_group_positions[4] = -0.0;
		  move_group_arm.setJointValueTarget(joint_group_positions);
		  succes = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		  if(succes){
			  move_group_arm.execute(my_plan);
			  old_torso_to_hand = new_torso_to_hand;
		  }
	  }
  }



  //current_state->copyJointGroupPositions(joint_model_group_arm, joint_group_positions);







  if(true){

	  //joint_group_positions[0] = -0.300;

	  //move_group_gripper.setJointValueTarget(joint_group_positions);

	  //move_group_gripper.move();
  }

  rate.sleep();
	  }
  return 0;

}
