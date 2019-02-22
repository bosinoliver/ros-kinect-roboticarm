turtlesim_gesture_control readme


Commands to be executed before launch start_demo.launch:
					
		1. roslaunch openni_launch openni.launch camera:=openni
		2. rosrun openni_tracker openni_tracker
					
			optional to visualize skeleton and pointcloud:
						
				1. rosrun rviz rviz 
					1.1 in rviz add tf
					1.2 in rviz add depthcloud
						Depth Map Topic choose /openni/depth/image
						Color Image Topic choose /openni/rgb/image_color
