/**
 *********************************************************************************
 * @file       gesture_control.cpp
 * @author     Oliver Bosin
 * @version    V1.0.0
 * @date       22.02.2019
 * @copyright  2011 - 2019 UniBw M - ETTI - Institute 4
 * @brief      Node to control the turtle in turtlesim
 * @details    This Node listen to the tf of skeleton broadcasted by openni_tracker
 * 			   and then publish messages to change velocity and angle of the turtle
 *             Therefore following publishers are created.
 *             
 * 			   1. PUBLISHERS
 *              -#  turtle_vel (publish on "/UDPcommand" topic)
 *
 **********************************************************************************
 *  @par History:
 *
 *  @details V1.0.0 22.02.2019 Oliver Bosin
 *         - Initial Release
 ***********************************************************************************
 * @todo   
 ***********************************************************************************
 * @bug  none
 ***********************************************************************************
 */
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>


/**
  * @brief   Main function for gesture_control
  * @details In this function tf-listener listen to tf broadcasted by openni_tracker
  * 		 and calculates the values to be send by the publisher turtle_vel to control
  * 		 the turtle in turtlesim
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
  ros::init(argc, argv, "gesture_control");

  ros::NodeHandle node;

  ros::service::waitForService("spawn");
  ros::ServiceClient spawner =
    node.serviceClient<turtlesim::Spawn>("spawn");
      system("rosservice call /kill 'turtle1'");

  turtlesim::Spawn turtle;
  turtle.request.x = 4;
  turtle.request.y = 2;
  turtle.request.theta = 0;
  turtle.request.name = "turtle1";

  spawner.call(turtle);


  ros::Publisher turtle_vel =
    node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);

	/* Here a TransformListener object is created. 
	 * Once the listener is created it starts receiving tf2 transformations over the wire 
	 * and buffers them for up to 10 seconds*/
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (node.ok()){
    geometry_msgs::TransformStamped transformStamped;
    try{
		/*here the transform between openni_link and left_hand_1 is calculated
		 *openni_link -> kinect , left_hand_1 -> right hand of person*/
      transformStamped = tfBuffer.lookupTransform("openni_link", "left_hand_1",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::Twist vel_msg;
	/*if right hand is right of the kinect from the perspective of the person the
	 *turtle will move forward*/
    if(transformStamped.transform.translation.y > 0.2)
    {
			vel_msg.linear.x = 1.0;
	}
    else{ 
		/*if right hand is left of the kinect from the perspective of the person the
	     *turtle will move backward*/
		if(transformStamped.transform.translation.y < -0.2){
			vel_msg.linear.x = - 1.0;
		}
		else{
			/*if right hand is in front of the kinect from the perspective of the person the
			 *turtle will stop*/
			vel_msg.linear.x = 0.0;
			}
	}
	/*if right hand is above the kinect from the perspective of the person the
	 *turtle will turn left*/
    if(transformStamped.transform.translation.z > 0.3){
		vel_msg.angular.z = 1.0;
	}
    else{
		/*if right hand is below the kinect from the perspective of the person the
		 *turtle will turn right*/
		if(transformStamped.transform.translation.z < -0.3){
		vel_msg.angular.z = - 1.0;
		}
		else{
			/*if right hand is in front of the kinect from the perspective of the person the
			 *turtle will not turn*/
			vel_msg.angular.z = 0.0;
		}
	}
    turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
};
