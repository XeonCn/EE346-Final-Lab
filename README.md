# EE346-Final-Lab
##  Part I Camera Calibration

Intrinsic matrix Camera Calibration YAML is in calibration folder

##  Part II Competition

1. Remote PC  

        roscore

2. SBC  

        roslaunch turtlebot3_bringup turtlebot3_robot.launch  

        roslaunch turtlebot3_autorace_traffic_light_camera turtlebot3_autorace_camera_pi.launch

3. Launch the intrinsic camera calibration on Remote PC  

        export AUTO_IN_CALIB=action  

        export GAZEBO_MODE=false  

        roslaunch turtlebot3_autorace_camera turtlebot3_autorace_intrinsic_camera_calibration.launch  

* Check images  

        rqt_image_view  

* Reduce ISO to reduce image stroboflash  

        rosrun rqt_reconfigure rqt_reconfigure  

4. Run the program  

        cd ~/catkin_ws/src/EE346-Final-Lab/src  

        python lane_following_part3b.py  

* Path: racetrack + point2, repeat 4 times in 5 minutes
