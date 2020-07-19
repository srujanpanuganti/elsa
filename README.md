# ELSA - Explore, Localize, Map Simultaeneously and Act

<p align="center">
<img src="images/elsabot.jpeg" alt="elsa" width="640">
</p>
<p align="center">
<em>ELSA Robot</em>
</p>

## Project information
* The project has two packages `elsa` and `ros_arduino_bridge`
* Follow the below instructions to build the project
  * `cd ~/catkin_ws`
  * `source devel/setup.bash` 
  * `catkin_make --only-pkg-with-deps elsa`
  * `catkin_make --only-pkg-with-deps ros_arduino_bridge`
* Follow the below commands to execute the project
  * `roslaunch elsa elsabot.launch`


## Docs:
* Coordinate Transformations and Robot Dimensions can be found at `docs/Robot dimensions and coordinate transformations.pdf`
* More info about the references used is provided in the reference section

## Finished tasks

* Publish imu to `/imu/raw_data` topic 
* Publish encoder ticks to `/ticks_pub/lwheel_ticks` and `/ticks_pub/lwheel_ticks` topics
* Publish Odometry data to `/odom` topic
* Configured the ROS_Arduino_Bridge
* Debug the ROS_Arduino_Bridge
  * Solder Arduino Nano
  * Connect the Arduino Nano to the Motor_Driver
  * Edit the scripts for the second USB port (/dev/ttyUSB*)
* Publish encoder ticks to `/arduino/lwheel_ticks` and `/arduino/lwheel_ticks` topics

## To Do

* Publish velocoties to `/cmd_vel` topic 
  * Publish sample Twist msg -- Seems like there's an error/lag
  * Debug the motor_driving  
* Finish the base_controller to subscribe to `/cmd_vel` topic to move the robot_base
* Tune the PID controller
* Use robot_localization package to filter the Odometry msg using ekf_lozalisation node
* Setup either RPLidar A1 or YDLidar X2 for 2D map creation
* Use slam techniques for mapping
* Use camera data to obtain the images to do SFM and build the 3D_point_cloud_map

## Results


<p align="center">
<img src="images/tf.png" alt="tf" width="640">
</p>
<p align="center">
<em>Coordinate Transformations (TF) in rviz</em>
</p>

<p align="center">
<img src="images/odom.png" alt="odom" width="640">
</p>
<p align="center">
<em>Odometry of the ELSA Robot in rviz</em>
</p>


## References:
* The project uses the [ROSArduinoBridge](https://github.com/hbrobotics/ros_arduino_bridge) Package. However, many modifications are made according to the requirements
* The Project also uses the [SparkFun_RedBot_Arduino_Library](https://github.com/sparkfun/SparkFun_RedBot_Arduino_Library) Arduino Library
* The Projects also uses few insights from the package [diff_drive](https://github.com/merose/diff_drive)
* The project refers several tutorials from [ROS wiki](http://wiki.ros.org/ROS/Tutorials) 
* The topics are published according to the coordinates frames as per [REP103 standards](https://www.ros.org/reps/rep-0103.html) and [REP105 standards](https://www.ros.org/reps/rep-0105.html)
