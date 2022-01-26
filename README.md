## F110 Workspace Description 
We are a group of students from Ecole Polytechnique working on a collective scientific project:
[PSC f1tenth (Tianyang HUANG, Maroun Rami, El Mallah Rim, Bouillé Aude)]. 

More on *f1tenth* here : https://f1tenth.org/. 

Environment : catkin_ws on `Ubuntu 20.04 arm64` with `ROS noetic` 

Environment Gym : catkin_ws on `Ubuntu 18.04.6 amd64` with `ROS melodic`

Author of source code & ReadMe : **Tianyang HUANG**

Some overviews:

- [1:10 racing car (Nvidia Jetson, Hkuyo Lidar, RGBD camera)](#jump)

- [Real-time racing simulation (f1tenth_gym_ros & f1tenth_quickstart)](#jump2)

## Implementations : 

### Navigation
- [Pure Pursuit----⭐️⭐️](#jump5)  
  --> High speed path following 
  
- [RRT------------⭐️⭐️⭐️](#jump3)   
  --> Dynamic Obstacle Avoidance + Randomized path planning
  
- [RRT+BFS-------⭐️⭐️⭐️⭐️](#jump3)  
  --> Dynamic Obstacle Avoidance + Rapide path planning
  
- [TEB------------⭐️⭐️⭐️⭐️⭐️](#jump16)
  --> Dynamic Obstacle Avoidance + Time Optimal path planning + Backward Enabled
  
- [Gap Follow-----⭐️⭐️](#jump12)   
  --> Dynamic Obstacle Avoidance

### SLAM
- [Cartographer - SLAM](#jump6)
- [Cartographer - Adaptation to real environment Mapping](#jump7)
- [Cartographer - Pure Localization](#jump8)
- [Localization with amcl](#jump9)
- [Occupancy Grid](#jump10)
- [Scan Matching](#jump11)

### Turtlebot3
- [Turtlebot3 Navigation](#jump13)
- [Turtlebot3 SLAM + Random Walk](#jump14)

****

- <span id="jump"> 1:10 racing car (Nvidia, Hkuyo Lidar, RGBD camera)</span>
<img width="500" alt="截屏2022-01-25 09 26 38" src="https://user-images.githubusercontent.com/57991090/150939766-7dea024a-10d0-48da-9d79-92e76d8f93d5.png">

- <span id="jump2"> Real-time racing simulation (f1tenth_gym_ros & f1tenth_quickstart) </span>
<img src="https://user-images.githubusercontent.com/57991090/149190984-b1d64572-6465-4cac-bc3a-35b36e396169.png" width="500">
<img src="https://user-images.githubusercontent.com/57991090/149200842-425aea3b-7aa5-464a-864a-201b8ec8a60e.png" width="500">

## Navigation

- <span id="jump5"> Pure Pursuit: High speed path following </span>

https://user-images.githubusercontent.com/57991090/143314357-4c70dbf4-5b17-4286-8e66-a9be53fd4848.mp4

- <span id="jump3"> RRT+BFS: Dynamic Obstacle Avoidance + Rapide path planning </span>

https://user-images.githubusercontent.com/57991090/146034581-74e46914-e8d1-4864-a195-5547195d206d.mp4

- <span id="jump16"> TEB planner：Backward planning Enabled + Dynamic Obstacle Avoidance + Time Optimal path planning  </span>


https://user-images.githubusercontent.com/57991090/151223147-1c30c54f-6cd2-4c04-8fd8-6eb6de403c5b.mp4


https://user-images.githubusercontent.com/57991090/150957168-7f51dea4-ab0d-456c-bfc7-58612290fa2d.mp4


## SLAM (simultaneous localization and mapping)

- <span id="jump6"> Cartographer - SLAM:  </span>

https://user-images.githubusercontent.com/57991090/149562722-cf9809b6-cd23-4d33-83c4-7de7ddcc727c.mp4

- <span id="jump7"> Cartographer - Adaptation to real environment Mapping:  </span>

![tuning](https://user-images.githubusercontent.com/57991090/150939433-b279e31f-48d8-4d69-a5dc-ddc47d7f9413.jpg)


- <span id="jump8"> Cartographer - Pure Localization:   </span>

https://user-images.githubusercontent.com/57991090/149562825-0bf85e47-1546-4b40-ae24-98e737a39235.mp4

- <span id="jump9"> Localization with amcl:  </span>

https://user-images.githubusercontent.com/57991090/150185599-335dd249-5710-4106-9d0d-5e2b3bdb37b3.mov

- <span id="jump10"> Occupancy Grid  </span>

https://user-images.githubusercontent.com/57991090/143580321-4ed67070-22cb-4867-8cff-11c65a2d77b8.mp4

- <span id="jump11"> Scan Matching  </span>

https://user-images.githubusercontent.com/57991090/143314509-1f561a2b-a9e6-47b8-983f-9a1ecc386bc5.mp4

- <span id="jump12"> Gap follow </span>

https://user-images.githubusercontent.com/57991090/143314487-2c1119ae-832c-4e69-89e0-3080c8b5ce0e.mp4


## Turtlebot3

- <span id="jump13"> Turtlebot3 Navigation </span>

https://user-images.githubusercontent.com/57991090/144667026-d02580a6-1520-42c8-875c-eda40cd15aa5.mp4

- <span id="jump14"> Turtlebot3 SLAM + Random Walk</span>

https://user-images.githubusercontent.com/57991090/144667100-c815c7c6-3710-4f7e-98a2-852090395f0b.mp4


