This repository is for providing a quick start on using the [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros) to launch a F1tenth virtual compeition.

# Quick demonstration by using example code
## Preparation:
```
Install Docker see https://docs.docker.com/engine/install/
$ mkdir -p ~/f110_ws/src
$ cd ~/f110_ws/src/
$ git clone https://github.com/cosynus-lix/f1tenth_quickstart.git
$ git clone https://github.com/f1tenth/f1tenth_gym_ros.git
```

## Single vehicle mode:
### Step 1 - Launch the simulator:
```
$ cd ~/f110_ws/src/f1tenth_gym_ros/
$ git checkout master
$ sudo ./build_docker.sh (it takes several minutes)
$ sudo ./docker.sh
```
### Step 2 - Launch the controller
In another temrinal do :
```
$ cd ~/f110_ws/
$ catkin_make
$ source devel/setup.bash
$ roslaunch f1tenth_controller_example wall_following_agent_node.launch
```

## Head-to-head mode, ego + opp:
### Step 1 - Launch the controller
```
$ cd ~/f110_ws/src/f1tenth_gym_ros/
$ git checkout multi_node
$ sudo ./build_docker.sh (it takes several minutes)
$ sudo ./docker.sh
```

### Step 2 - Launch the controller
In another temrinal do :
```
edit the class Agent() in file wall_following_agent_node.py: comment the part about “single vehicle racing” and uncomment the part about “head-to-head racing”
$ cd ~/f110_ws/
$ catkin_make
$ roslaunch f1tenth_controller_example wall_following_agent_node.launch
```

# Change the track map
You can use the python script `change_map.py` to easily change track map by doing:
```
$ cd ~/f110_ws/
$ sudo python src/f1tenth_quickstart/change_map.py src/f1tenth_quickstart/maps/[map_name] (replace map_name by yourself, see paragraph below)
$ cd ~/f110_ws/src/f1tenth_gym_ros/
$ sudo ./build_docker.sh
```
We provided two different tracks, with / without obstacles, which is used in F1tenth virtual competition edition IFAC2020 and IROS2020. Their names are: `map_name` = `berlin.png`,`berlin_OFFICIAL_obstacles.png`,`vegas.png`,`vegas_OFFICIAL_obstacles.png`. You could find them in the folder `maps/`.

For more maps, you can have a look [here](https://github.com/f1tenth/f1tenth_simulator/tree/master/maps) and download chosed ones (with the corresponding `.yaml`) to `maps/` for usage.
You can also DIY a map (design a new one or add obstacles on an old one) by drawing pixels on map image (png/pgm/... files)! Just remember: white for free space and black for obstacles. Moreover, in the `.yaml` file, choose a small value for `resolution` and set the third coordinate of `origin` to a null value (e.g. `0`).

It should just work perfertly! For more details on changing maps, we refer to the description [here](https://github.com/f1tenth/f1tenth_gym_ros#changing-maps).


# Write the controller code by yourself!
## What to change?
1. in `src/`: modify the file `wall_following_agent_node.py` or write an independent file with similiar structure; you can also write the controller in C++, refer to [ROS Tutorial](http://wiki.ros.org/ROS/Tutorials).
2. the main point is to get environment information from the topic `/scan`(Message type - [`sensor_msgs/LaserScan`](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html), [more details](http://wiki.ros.org/laser_pipeline/Tutorials/IntroductionToWorkingWithLaserScannerData)) and(or) `/odom`(Message type - [`nav_msgs/Odometry`](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html), [more details](http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom)), calculate the controll command and send to the topic `/drive` (Message type - [`ackermann_msgs/AckermannDrive`](http://docs.ros.org/en/api/ackermann_msgs/html/msg/AckermannDrive.html), [more details](http://wiki.ros.org/Ackermann%20Group)).
2. in `Launch/`: in case that you write the controller in a new file, write a launch file similiar to `wall_following_agent_node.launch`.
3. file `package.xml`: in case that you use new dependancies (such as `ackermann_msgs` etc), add them in this file.
4. file `CMakeLists.txt`: in case that you write the controller in C++, you need to modify this file.

## What is missing in the example code?
1. in the example code, we only use the information of Lidar via the topic `/scan` but not `/odom`. You can use `/odom` to obtain the ego agent's odometry for your own algorithm!
2. we have not use the information from the topic `/race_info`. You can subscript to it for getting both agents' elapsed runtimes, lap count, and the collsion info.
3. for head-to-head mode, we suppose that the ego and opp vehicle use the same controller and put their controller inside a same file. Of course you can use different controllers for each! In practice, it is better to write their controllers in different files.
