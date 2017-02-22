#include "ros/ros.h"

#include <string>

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include <stdlib.h>
#include <stdio.h>




int main(int argc, char** argv)
{
  ros::init(argc, argv, "pathplanz");
  ROS_INFO("Starting Cell List");
  ros::NodeHandle nh;
  ros::Publisher pubpath = nh.advertise<nav_msgs::Path>("/POINTZ", 10);

	ros::Rate loop_rate(0.5);
	geometry_msgs::PoseStamped cell1, cell2, cell3, cell4, cell5, cell6, cell7, cell8, cell9, cell0;
	std::vector<geometry_msgs::PoseStamped> ListCell;
	while(ros::ok()){
	cell1.pose.position.x = 1.0;
	cell1.pose.position.y = 0.0;
	cell2.pose.position.x = 2.0;
	cell2.pose.position.y = 0.0;
	cell3.pose.position.x = 2.0;
	cell3.pose.position.y = 1.0;
	cell4.pose.position.x = 2.0;
	cell4.pose.position.y = 2.0;

	cell5.pose.position.x = 1.0;
	cell5.pose.position.y = 2.0;
	cell6.pose.position.x = 0.0;
	cell6.pose.position.y = 2.0;
	cell7.pose.position.x = 0.0;
	cell7.pose.position.y = 3.0;
	cell8.pose.position.x = 0.0;
	cell8.pose.position.y = 4.0;

	cell9.pose.position.x = 1.0;
	cell9.pose.position.y = 4.0;
	cell0.pose.position.x = 2.0;
	cell0.pose.position.y = 4.0;








	cell1.header.seq = 0;
	cell2.header.seq = 1;
	cell3.header.seq = 2;
	cell4.header.seq = 3;
	cell5.header.seq = 4;
	cell6.header.seq = 5;
	cell7.header.seq = 6;
	cell8.header.seq = 7;
	cell9.header.seq = 6;
	cell0.header.seq = 7;
	
	cell1.header.frame_id = "/base_link";
	cell2.header.frame_id = "/base_link";
	cell3.header.frame_id = "/base_link";
	cell4.header.frame_id = "/base_link";
	cell5.header.frame_id = "/base_link";
	cell6.header.frame_id = "/base_link";
	cell7.header.frame_id = "/base_link";
	cell8.header.frame_id = "/base_link";
	cell9.header.frame_id = "/base_link";
	cell0.header.frame_id = "/base_link";
	
	ListCell.push_back(cell1);
	ListCell.push_back(cell2);
	ListCell.push_back(cell3);
	ListCell.push_back(cell4);
	ListCell.push_back(cell5);
	ListCell.push_back(cell6);
	ListCell.push_back(cell7);
	ListCell.push_back(cell8);
	ListCell.push_back(cell9);
	ListCell.push_back(cell0);

	nav_msgs::Path PlanPath;
	PlanPath.poses = ListCell;
	//PlanPath.header.stamp= ros::Time::now();
	PlanPath.header.frame_id = "/base_link";


	pubpath.publish(PlanPath);
	ListCell.clear();
	ros::spinOnce();
	loop_rate.sleep();
	
}
  return 0;

}