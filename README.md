HL Navigation
=============

> [!WARNING] 
> This repository has been discontinued. Please have a look at https://github.com/idsia_robots/navground and https://github.com/idsia_robots/navground_ros that provide similar functionality.

The repository contains a [ROS](http://www.ros.org) node that performs local navigation.

The navigation node implements the three navigation behaviors described in the [article](http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=6696726&url=http%3A%2F%2Fieeexplore.ieee.org%2Fxpls%2Fabs_all.jsp%3Farnumber%3D6696726)

    Guzzi, J.; Giusti, A.; Gambardella, L.M.; Di Caro, G.A., "Local reactive robot navigation: A comparison between reciprocal velocity obstacle variants and human-like behavior," Intelligent Robots and Systems (IROS), 2013 IEEE/RSJ International Conference on , vol., no., pp.2622,2629, 3-7 Nov. 2013

One behavior, inspired by how pedestrian move, has been originally presented in

    Guzzi, J.; Giusti, A.; Gambardella, L.M.; Theraulaz, G.; Di Caro, G.A., "Human-friendly robot navigation in dynamic environments," Robotics and Automation (ICRA), 2013 IEEE International Conference on , vol., no., pp.423,430, 6-10 May 2013

The other 2 are based on the following implementations

* [RVO2 library](http://gamma.cs.unc.edu/RVO2/)
* [HRVO library](http://gamma.cs.unc.edu/HRVO/)
