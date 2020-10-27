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

