# performance_tests
Exercise 1- Performance Test - ROS Package

How does this package work?

This package follows the ROS package format as expected, the cpp nodes are on the src folder while the py nodes are on the scripts folder. The custom SuperAwesome message is 
on the msg folder. The dynamic reconfiguration cfg file is on the cfg folder and finally, obviously, the launch files are on the 
launch folder.

You can run the exercise by running the launch files. There are 4 different launch files, they launch the 4 different publisher/
subscriber combinations. These files launch the proper publisher and subscriber nodes (indicated on the file name), the rqt_reconfigure in order to choose the loop_late value by GUI; and a rqt_plot showing how the real value is changing through time against the desired value.

If the real rate value is changed from 1 on the rqt_reconfigure GUI, then the user can visualize in which desired rate the real 
rate becomes slower (lower frequency) and unstable (several peaks on the plot).

If you want to try this package, remember to make executable the python files, including the dynparam.cfg.

Steps to build and run:- 
1) Create a catkin workspace and clone/copy this package in the src folder. Example:-  "catkin_ws/src"
2) Build the package from the root folder of catkin workspace with the command "catkin_make".
3) Source the setup.bash files generated as needed for every terminal by ROS.
4) Launch the launch file to see the results. Example:- roslaunch performance_tests cpp_pub_cpp_sub.launch

You can now the rqt_plot showing how the real value is changing through time against the desired value. You can also configure the loop_rate by rqt_reconfigure GUI window using the slider.


Code Quality Tools used:- 
1) roslint for linting of the code.
2) cppcheck for syntex errors.

Refrences:-
http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
http://wiki.ros.org/dynamic_reconfigure/Tutorials/SettingUpDynamicReconfigureForANode

