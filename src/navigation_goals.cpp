#include <ros/ros.h>
#include "Robot.cpp"
#include <vector>

std::vector<Robot*> apeHolder;

void callback(ar_track_alvar_msgs::AlvarMarkers);

int main(int argc, char** argv){

  ros::init(argc, argv, "navigation_goals");
  ros::NodeHandle nh;

  Robot harambe = Robot();
  apeHolder.push_back(&harambe);
  apeHolder[0]->hello();

  ros::Subscriber sub = nh.subscribe("/ar_pose_marker", 1000, callback);

  while(!apeHolder[0]->localized()){
    ros::spinOnce();
  }

  apeHolder[0]->run();

  ROS_INFO("Program Terminated");
  return 0;
}

void callback(ar_track_alvar_msgs::AlvarMarkers msg){

  apeHolder[0]->localize(msg);

}

