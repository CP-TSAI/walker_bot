# ROS Beginner Tutorials


## Overview
The repo presents a turtlebot that can repeatedly execute the following scenarios:
(1) move forward until it reaches an obstacle (but not colliding),

(2) then rotate in place until the way ahead is clear, 

(3) then move forward again.


## Dependencies
- Ubuntu Xenial (16.04)  
- ROS Kinetic  


## Setup ROS Environment with Terminal
- Before executing any ROS program
```
$ roscore
```

- Everytime when open a new terminal
```
$ source ~/catkin_ws/devel/setup.bash
```





## How to build

```
cd to catkin_ws/src
git clone https://github.com/CP-TSAI/walker_bot.git
cd ..
catkin_make --only-pkg-with-deps walker_bot
```









## How to run the package
- Use roslaunch command for the package
```
- $ roscd walker_bot

- $ roslaunch walker_bot all.launch
```

- if you want to use rosbag for the topics, the the **record** flag, the rosbag file would be saved in **result** file.

```
- $ roslaunch walker_bot all.launch record:=enable
```


## How to play the rosbag

- Once we have the rosbag, it can be used for testing

- cd to the rosbag path

```
$ rosbag play topics.bag
```

- Then you can use **rostopic list** to see if it is executed.

- The content of rosbag can be seen by the command

```
$ rosbag info topics.bag
```
