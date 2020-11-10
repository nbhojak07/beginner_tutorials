## beginner_tutorials
## OVERVIEW 
This package demonstrates about the use of ROS publisher and subscriber. For more information on this, following tutorials can be referred,

http://wiki.ros.org/ROS/Tutorials/NavigatingTheWiki (Links to an external site.)

http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem (Links to an external site.)

http://wiki.ros.org/ROS/Tutorials/CreatingPackage (Links to an external site.)

http://wiki.ros.org/ROS/Tutorials/BuildingPackages (Links to an external site.)

http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes (Links to an external site.)

http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics (Links to an external site.)

http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29 (Links to an external site.)

http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber

## DEPENDENCIES 

Following are the dependencies required to run this package:
```
-ROS Melodic 
-Catkin
-Ubuntu 18.04
```
## INSTALLATION
```
cd ~/catkin_ws/src
git clone --recursive https://github.com/nbhojak07/beginner_tutorials.git
cd ..
catkin_make
```
In three different terminals run the following:
#### Terminal 1
```
roscore
```
#### Terminal 2
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials talker
```
#### Terminal 3
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials listener 
```
## Using ROS Service
#### New Terminal 
```
rosservice call /add_two_ints 1 3
```

## Using ROS Launch
#### New Terminal 
```
roslaunch beginner_tutorials AddTwoInts.launch a:=1 b:=3
```
## Using RQT_CONSOLE and RQT_LOGGER_LEVEL
#### Terminal 1
```
rosrun rqt_console rqt_console
```
#### Terminal 2
```
rosrun rqt_logger_level rqt_logger_level
```
Run the server and client service after running rqt_console

## Inspecting TF Frames
```
cd catkin_ws
source devel/setup.bash
rosrun rqt_tf_tree rqt_tf_tree
```
## Running Tests
```
cd catkin_ws
source devel/setup.bash
roslaunch talker_test.launch a:=5 b:=4
```
## Recording bag files
```
cd catkin_ws
source devel/setup.bash
roslaunch beginner_tutorials AddTwoInts.launch record:=true
```
## Inspecting rosbag files
```
cd catkin_ws/src/beginner_tutorials/results
rosbag info beginner_tutorials.bag
```
## Playback rosbag
#### Terminal 1:
```
roscore
```
#### Terminal 2:
```
cd catkin_ws/src/beginner_tutorials/results
rosbag play beginner_tutorials.bag
```
#### Terminal 3:
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials listener
```
To stop the program, press Ctrl+C.
