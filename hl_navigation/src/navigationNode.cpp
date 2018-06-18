/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "ros/ros.h"
#include "Navigator.h"


int main(int argc, char **argv)
{
        ros::init(argc, argv, "navigation_node");
        Navigator localNavigation;
        ROS_INFO("Navigation initialized");
        ros::spin();
}
