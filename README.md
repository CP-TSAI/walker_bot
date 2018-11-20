# ROS Beginner Tutorials


## Overview
The repo presents a turtlebot that can repeatedly execute the following scenarios:

(1) move forward until it reaches an obstacle (but not colliding),

(2) then rotate in place until the way ahead is clear, 

(3) then move forward again.


## Dependencies
- Ubuntu Xenial (16.04)  
- ROS Kinetic  




<!-- 

## How to build


```
cd to catkin_ws
source devel/setup.bash
cd src
git clone https://github.com/CP-TSAI/walker_bot.git
cd ..
catkin_make --only-pkg-with-deps walker_bot
```




## Setup ROS Environment with Terminal
- Before executing any ROS program
```
$ roscore
```

- Everytime when open a new terminal
```
$ source ~/catkin_ws/devel/setup.bash
```



## How to run the package

Use either **(1) roslaunch** or **(2) rosrun** command.


(1) **roslaunch**

```
- $ roscd beginner_tutorials

- $ roslaunch beginner_tutorials all.launch freq:=1
```

- The default publisher frequency is 10, you can change it by the roslaunch command.  


(2) **rosrun**

- cd to catkin_ws

- Use 2 terminals and execute the following commands. 

- terminal1

```
$ rosrun beginner_tutorials talker
```

- terminal2

```
$ rosrun beginner_tutorials listener
```


## How to use the service call

- Once the publisher and subscriber is running, we can use a **service call** to change the string message. 

- Open a new terminal, then run

```
$ rosservice call /change_string "DESIRED_MESSAGE"
``` 


## How to use tf frames

- When the talker node is running, the /tf is also being broadcasting

- The tf information can be viewed by the following command

```
$ rosrun tf tf_echo /world /talk
```

- We can produced a PDF file with the command

```
$ rosrun tf view_frames
```

- We can view the PDF by the command

```
$ evince frames.pdf
```



## How to run rostest

- Go to the catkin_ws

```
$ catkin_make run_tests_beginner_tutorials
```

- After the compilation, execute the command and wait for the result

```
$ rostest beginner_tutorials test.launch
```


## How to record with rosbag

- We can use the roslaunch flag to enable the rosbag (do not use "record" flag if you don't want to use rosbag)

```
$ roslaunch beginner_tutorials all.launch record:=enable
```

- The rosbag would be saved in the "result" file


## How to play the rosbag

- Once we have the rosbag, it can be used for testing

- cd to the rosbag file

```
$ rosbag play pub.bag
```

- Then you can rosrun listener node to see the result

- The content of rosbag can be seen by the command

```
$ rosbag info pub.bag
```



## Example Showcase

### Change the publish string

- make the frequency of the publisher 5hz.

```
$ roslaunch beginner_tutorials all.launch freq:=5
```

- You'll see something like
```
[ INFO] [1541551721.920444997]: I heard: [wakanda forever  85]
[ INFO] [1541551722.119970793]: wakanda forever  86
[ INFO] [1541551722.120464329]: I heard: [wakanda forever  86]
[ INFO] [1541551722.319971642]: wakanda forever  87
[ INFO] [1541551722.320449526]: I heard: [wakanda forever  87]
[ INFO] [1541551722.519969315]: wakanda forever  88
[ INFO] [1541551722.520364922]: I heard: [wakanda forever  88]
```

- If you want to change the published message, then do

```
$ rosservice call /change_string "wakanda is doomed"
```	

- The terminal would shows something like ...

```
[ INFO] [1541551856.720157134]: I heard: [wakanda forever  759]
[ INFO] [1541551856.919995812]: wakanda forever  760
[ WARN] [1541551856.920119453]: Changing the output String
[ INFO] [1541551856.920420078]: I heard: [wakanda forever  760]
[ INFO] [1541551857.119965528]: wakanda is doomed 761
[ INFO] [1541551857.120448497]: I heard: [wakanda is doomed 761]
[ INFO] [1541551857.319967544]: wakanda is doomed 762
[ INFO] [1541551857.320436805]: I heard: [wakanda is doomed 762]
```


## Reference
- http://wiki.ros.org/ROS/Tutorials
 -->