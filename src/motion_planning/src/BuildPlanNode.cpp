#include <ros/ros.h>

#include <motion_planning/AStar.hpp>

int main(int argc, char** argv){

	ROS_INFO("Starting path planning");
    ros::init(argc, argv, "path_plan_node");
 
    ros::NodeHandle nh_("~");

    motion_planning::AStar AStar(nh_);

	ros::spin();
	ros::waitForShutdown();
	return 0;
}
