sample-return-robot
===================

set up the environment:
source /opt/ros/groovy/stacks/simulator_gazebo/gazebo/gazebo/share/gazebo-1.2/setup.sh
source catkin_ws/devel/setup.bash

run Gazebo:
roslaunch heightmap stlmap_world.launch

view camera output:
rosrun image_view image_view image:=/fancySLR/image

move robot using arrow keys:
rosrun teleop1 teleop1

run blob detection:
rosrun sample_detection blobdetection input_image:=/fancySLR/image
