#include <ros/ros.h>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <turtlebot_actions/TurtlebotMoveAction.h>
#include <actionlib/client/simple_action_client.h>
typedef actionlib::SimpleActionClient<turtlebot_actions::TurtlebotMoveAction> MoveBaseClient;

class MotionActionClient
{
private:
    std::string action_name;
    ros::NodeHandle nh;
    ros::Subscriber pts_sub;
    nav_msgs::Path msg;
    turtlebot_actions::TurtlebotMoveGoal goal;
    std::vector<turtlebot_actions::TurtlebotMoveGoal> plan;
    geometry_msgs::Point oldpt, currentpt, nextpt;
    MoveBaseClient ac;
    float Res_of_cell = 1;
    float Left = 1.5701;
    float Right = -1.5701;
public:
    MotionActionClient(const std::string name) : ac(nh, name, true), action_name(name){
      ROS_INFO("before While loop");
      while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for action server");
      }
      ROS_INFO("after while loop");
     
      pts_sub = nh.subscribe("/path_plan", 1000, &MotionActionClient::MotionCB, this);

    }

void MotionCB(const nav_msgs::Path msg){

  for(int i = 0; i < msg.poses.size()-1; i++){

    if(i==0){
      goal.forward_distance = Res_of_cell;
      goal.turn_distance = 0.0;
      ROS_INFO("Straight");
    }
    else{
      oldpt.x = msg.poses[i-1].pose.position.x;
      oldpt.y = msg.poses[i-1].pose.position.y;
      currentpt.x = msg.poses[i].pose.position.x;
      currentpt.y = msg.poses[i].pose.position.y;
      nextpt.x = msg.poses[i+1].pose.position.x;
      nextpt.y = msg.poses[i+1].pose.position.y;
      goal.forward_distance = Res_of_cell;
      goal.turn_distance = DirectiontoTurn(oldpt, currentpt, nextpt);
      ;
    }
    
   plan.push_back(goal);
  }


  for(int it = 0; it < plan.size(); it++){
    ROS_INFO("Sending goal");
    ac.sendGoal(plan.at(it));
    ac.waitForResult();

  }
  plan.clear();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray base moved");
  else
    ROS_INFO("FAILURE!!");
  }

//plan.position.x = 1, 2, 2, 2, 1, 0, 0, 0, 1, 2,
//plan.position.y = 0, 0, 1, 2, 2, 2, 3, 4, 4, 4,
float DirectiontoTurn(geometry_msgs::Point old, geometry_msgs::Point current, geometry_msgs::Point next){
 float Left = 1.5701;
 float Right = -1.5701;
 float turn_distance;
//std::cout << current.x <<", " << current.y << '\n';
 if((old.x==current.x && current.x  == next.x) || (old.y == current.y && current.y == next.y)){
      turn_distance = 0.0;
      ROS_INFO("Straight");
    }
//Determining direction facing
    //North
  else if(current.x - old.x < 0 && current.y == old.y){

      if(current.y - next.y < 0){
      turn_distance = Left;
      ROS_INFO("Left");
      }
      else{
      turn_distance = Right;
       ROS_INFO("Right");
      }
  }  
    //East
  else if(current.y - old.y < 0 && current.x == old.x){

      if(current.x - next.x > 0){
      turn_distance = Left;
      ROS_INFO("Left");
      }
      else{
      turn_distance = Right;
      ROS_INFO("Right");
      }
  }  
  //South
  else if(current.x - old.x > 0 && current.y == old.y){

      if(current.y - next.y > 0){
      turn_distance = Left;
      ROS_INFO("Left");
      }
      else{
      turn_distance = Right;
      ROS_INFO("Right");
      }
  }  
   //West
  else{

      if(current.x - next.x < 0){
      turn_distance = Left;
      ROS_INFO("Left");
      }
      else{
      turn_distance = Right;
      ROS_INFO("Right");
      }
  }  
std::cout << turn_distance<<'\n';
return turn_distance;
}

};

// //TURNS THEN GOES FORWARD
    
// goal.forward_distance = 1.0;
// goal.turn_distance = 0.0;

// plan.push_back(goal);
// goal.forward_distance = 1.0; // IN FRONT OF ROBOT
// goal.turn_distance = 1.57; //IN RADIANS
// plan.push_back(goal);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motionClient");
  ROS_INFO("Starting motion client");
 
  MotionActionClient ac("turtlebot_move");
  ros::spin();

  return 0;
}
