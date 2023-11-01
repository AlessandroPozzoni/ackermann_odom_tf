# ackermann-odom-tf

## Task
Set up and configure a node to provide the odometry of an autonomous vehicle modeled with Ackermann Steering.

![LasersScan](https://github.com/AlessandroPozzoni/ackermann-odom-tf/blob/main/images/lasers_scan.png)

## Vehicle
Experimental autonomous shuttle EasyMile EZ10.

![EasyMile EZ10](https://github.com/AlessandroPozzoni/ackermann-odom-tf/blob/main/images/EZMile.png)

### Available data:
* Wheelbase: 2.80m
* Displacement of the lasers from the center of the vehicle (converted in quaternion in [parameters.yaml](https://github.com/AlessandroPozzoni/ackermann-odom-tf/blob/main/launcher.launch))
  Sensor | x | y | z | yaw | pitch | roll
  -------|---|---|---|-----|-------|----- 
  sick_front_left | 1.85 | 0.93 | 0.0 | 0.81 | 0.0 | 3.14
  sick_front_right | 1.85 | -0.93 | 0.0 | -0.76 | 0.0 | 3.14
  sick_rear_left | -1.85 | 0.93 | 0.0 | 2.38 | 0.0 | 3.14
  sick_rear_right | -1.75 | -0.8 | 0.0 | -2.30 | 0.0 | 3.14
* [📁 Bags Google Drive folder](https://drive.google.com/drive/folders/1up9J7SY8vHblapu4eD_WjhbXlN9l7I8v?usp=drive_link)
  
  /speed_steer &emsp;  &emsp; geometry_msgs::Quaternion
  
  &emsp;x : speed of the rear wheels
  
  &emsp;y : angular displacement of the steering wheels
    ```bash
    rosbag play --clock -l first.bag
    ```
## Layout:
### ROS package: project1_22
* ### Nodes:
  * [odom_node](https://github.com/AlessandroPozzoni/ackermann-odom-tf/blob/main/src/odom_node.cpp):
    
    * Publications:
      * /custom_odometry&emsp;  &emsp; [project1_22/Odom]
      * /odometry &emsp;  &emsp;&emsp;&emsp;&emsp;&ensp;&nbsp;[nav_msgs/Odometry]
      * /tf &emsp;  &emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&nbsp;[tf2_msgs/TFMessage]
        
    * Subscriptions:
      * /speed_steer&emsp;  &emsp;&emsp;&emsp;&nbsp; [geometry_msgs::Quaternion]
     
    * Services: 
      * /reset_odom
     
  * [reset_odom_client](https://github.com/AlessandroPozzoni/ackermann-odom-tf/blob/main/src/reset_odom_client.cpp)
 
* ### Custom messages:
  * [Odom](https://github.com/AlessandroPozzoni/ackermann-odom-tf/blob/main/msg/Odom.msg)
 
* ### Services:
  * [ResetOdom](https://github.com/AlessandroPozzoni/ackermann-odom-tf/blob/main/srv/ResetOdom.srv)
    
   
### ROS package: tf
* ### Nodes:
  * /front_left
  * /front_right
  * /rear_left
  * /rear_right
 
## Parameters:
* starting_x = 0&emsp;  &emsp;initial x value
* starting_y = 0&emsp;  &emsp;initial y value
* starting_th = 0&emsp;  &ensp; initial theta value

## tf tree 

![TFTree](https://github.com/AlessandroPozzoni/ackermann-odom-tf/blob/main/images/tf_tree.png)

## Published TF + static transformations

![StaticTF](https://github.com/AlessandroPozzoni/ackermann-odom-tf/blob/main/images/static_tf.png)

## Build and run
### Build the packages
In your catkin_ws
```bash
catkin_make
```
```bash
source devel/setup.bash
```
### Start a roscore
```bash
roscore
```
### Use the launcher to start the nodes and initalize the service server
```bash
roslaunch project1_22 launcher.launch
```
### Play the bag
Use the path to the directory containing the bags. Play first.bag or second.bag.
```bash
rosbag play path/to/bags/first.bag --clock -l
```
### View tf_tree
```bash
rosrun rqt_tf_tree rqt_tf_tree
```
While the bag is playing, press refresh (top left corner of the window, ↪️). This will update the tree. If the bag is paused before this command no value will be displayed.
### Start rviz
```bash
rviz
```
__Settings__: 
* Change ```global options > fixed frame``` to ```odom```
* Press Add > By display type > TF > Ok
* Press Add > By topic and select one LaserScan for each of the topic ```sick_front_left``` ... > Ok
  * If the LaserScan is not visible, increase the size by expanding the LaserScan submenus.
