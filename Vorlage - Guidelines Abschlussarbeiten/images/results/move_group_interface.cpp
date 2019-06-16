/**
 *********************************************************************************
 * @file       move_grop_interface.cpp
 * @author     Oliver Bosin
 * @version    V1.0.0
 * @date       13.06.2019
 * @copyright  2011 - 2019 UniBw M - ETTI - Institute 4
 * @brief      Node to control the roboticarm rob_arm_small
 * @details    This Node listen to the tf of skeleton broadcasted by openni_tracker
 * 			   and calculate a goal for the roboticarm which is
 *             		   then executed by the framework MoveIt!
 *			  
 * 			   1. Listeners
 *              		- tfListener
 *
 **********************************************************************************
 *  @par History:
 *
 *  @details V1.0.0 13.06.2019 Oliver Bosin
 *         - Initial Release
 ***********************************************************************************
 * @todo   Optimize tolerances, try asyncExecute() funktion
 ***********************************************************************************
 * @bug  none
 ***********************************************************************************
 */

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Vector3.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

/**
  * @brief   Main function for move_group_interface
  * @details In this function tf-listener listen to tf broadcasted by openni_tracker
  * 		 and calculate a goal for the roboticarm which is
  *              then executed by MoveIt!
  * @param   [in] argc: Non-negative value representing the number of
  *                     arguments passed to the program from the environment
  *                     in which the program is run.
  * @param   [in] argv: Pointer to an array of pointers to null-terminated
  *                     multibyte strings that represent the arguments passed
  *                     to the program from the execution environment
  * @retval  If the return statement is used, the return value is used as the
  *          argument to the implicit call to exit(). This value can be:
  *                     @arg EXIT_SUCCESS [indicate successful termination]
  *                     @arg EXIT_FAILURE [indicate unsuccessful termination]
  */
int main(int argc, char** argv){

  	ros::init(argc, argv, "move_group_interface");

  	ros::NodeHandle node_handle;

  	ros::AsyncSpinner spinner(1);

  	spinner.start();

  	/* MoveIt! declarations and initializations */
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

	move_group_arm.setGoalPositionTolerance(0.02);
	move_group_arm.setGoalJointTolerance(0.07);
	move_group_arm.setPlanningTime(0.08);

	move_group_gripper.clearPoseTargets();
	move_group_gripper.setStartStateToCurrentState();
	move_group_gripper.setPoseReferenceFrame("base_link");

	move_group_gripper.setGoalTolerance(0.025);
	move_group_gripper.setPlanningTime(0.08);

	/* Here a TransformListener object is created.
			 * Once the listener is created it starts receiving tf2 transformations over the wire
			 * and buffers them for up to 10 seconds*/

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

	/* Declarations of messages to store Transforms

    	geometry_msgs::TransformStamped transformStamped_left_hand_1;
	geometry_msgs::TransformStamped transformStamped_right_hand_1;
	geometry_msgs::TransformStamped transformStamped_shoulder_to_elbow;
    	geometry_msgs::TransformStamped transformStamped_elbow_to_hand;

	/*loop to get transform from shoulder to elbow and elbow to hand for armlength*/

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
	tf2::Vector3 rotate = {0.0,0.0,1.0};
	tf2::Vector3 distance;

	shoulder_to_elbow_vector[0] =  transformStamped_shoulder_to_elbow.transform.translation.x;
	shoulder_to_elbow_vector[1] =  transformStamped_shoulder_to_elbow.transform.translation.y;
	shoulder_to_elbow_vector[2] =  transformStamped_shoulder_to_elbow.transform.translation.z;

	elbow_to_hand_vector[0] =  transformStamped_elbow_to_hand.transform.translation.x;
	elbow_to_hand_vector[1] =  transformStamped_elbow_to_hand.transform.translation.y;
	elbow_to_hand_vector[2] =  transformStamped_elbow_to_hand.transform.translation.z;

	/*calculate armlength of user*/

	double armlength = shoulder_to_elbow_vector.length() + elbow_to_hand_vector.length();

	/*calculate conversion factor for position target*/

	double conversion_factor = 0.38 / armlength;


	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	moveit::core::RobotStatePtr current_state;
	std::vector<double> joint_group_positions;

	ros::Rate rate(60.0);

	/*main loop*/

	while (node_handle.ok()){

		try{

			transformStamped_left_hand_1 = tfBuffer.lookupTransform("torso_1", "left_hand_1",
	                               						ros::Time(0));

			/* optional Transform calculation to implement grab functionality of the gripper

	      		transformStamped_right_hand_1 = tfBuffer.lookupTransform("torso_1", "right_hand_1",
	      	                               					ros::Time(0));
			*/
	    	}
	    	catch (tf2::TransformException &ex) {

	      		ROS_WARN("%s",ex.what());
			continue;

	    	}

 	 /* calculate new position target for endeffector */
	 distance[0] = conversion_factor * (transformStamped_left_hand_1.transform.translation.x - 0.15);
	 distance[1] = conversion_factor * (transformStamped_left_hand_1.transform.translation.z*(-1.0));
	 distance[2] = conversion_factor * (transformStamped_left_hand_1.transform.translation.y - 0.15);
	 
	 distance = distance.rotate(rotate, 0.800);
	  move_group_arm.setPositionTarget(distance[0], distance[1], distance[2]);

	  ROS_INFO_NAMED("planning","x : %lf y: %lf z: %lf", distance[0], distance[1], distance[2]);
	  ROS_INFO_NAMED("distance","distance: %lf", distance.length());
	  bool succes = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

	  if(succes){

		  succes = false;
		  joint_group_positions = my_plan.trajectory_.joint_trajectory.points.back().positions;
		  joint_group_positions[4] = -0.0;
		  move_group_arm.setJointValueTarget(joint_group_positions);
		  succes = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

		  if(succes){

			  move_group_arm.execute(my_plan);

		  }
	  }

	/* optional gripper control */

  	if(true){

		//joint_group_positions[0] = -0.300;

	  	//move_group_gripper.setJointValueTarget(joint_group_positions);

	  	//move_group_gripper.move();
  	}
  	rate.sleep();
	}
  	return 0;

}