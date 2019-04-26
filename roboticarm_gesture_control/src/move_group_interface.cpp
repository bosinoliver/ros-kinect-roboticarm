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



#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>



#include <moveit_msgs/DisplayRobotState.h>

#include <moveit_msgs/DisplayTrajectory.h>



#include <moveit_msgs/AttachedCollisionObject.h>

#include <moveit_msgs/CollisionObject.h>



#include <moveit_visual_tools/moveit_visual_tools.h>



int main(int argc, char** argv)

{

  ros::init(argc, argv, "move_group_interface_tutorial");

  ros::NodeHandle node_handle;

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
	move_group_arm.setPlanningTime(10);

	move_group_gripper.clearPoseTargets();
	move_group_gripper.setStartStateToCurrentState();
	move_group_gripper.setPoseReferenceFrame("base_link");

	move_group_gripper.setGoalTolerance(0.005);
	move_group_gripper.setPlanningTime(10);






  // Getting Basic Information

  // ^^^^^^^^^^^^^^^^^^^^^^^^^

  //

  // We can print the name of the reference frame for this robot.

  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group_arm.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group_gripper.getPlanningFrame().c_str());


  // We can also print the name of the end-effector link for this group.

  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_arm.getEndEffectorLink().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_gripper.getEndEffectorLink().c_str());



  // Planning to a Pose goal

  // ^^^^^^^^^^^^^^^^^^^^^^^

  // We can plan a motion for this group to a desired pose for the

  // end-effector.
 


  move_group_arm.setPositionTarget(0.0, 0.317388, 0.231109);



  move_group_arm.move();


  moveit::core::RobotStatePtr current_state = move_group_arm.getCurrentState();

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group_arm, joint_group_positions);

  joint_group_positions[4] = -0.385;

  move_group_arm.setJointValueTarget(joint_group_positions);

  move_group_arm.move();

  if(true){

	  joint_group_positions[0] = -0.100;

	  move_group_gripper.setJointValueTarget(joint_group_positions);

	  move_group_gripper.move();
  }


  ros::shutdown();

  return 0;

}
