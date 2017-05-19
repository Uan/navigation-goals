#ifndef ROBOT_H
#define ROBOT_H

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
//change "oaktobotics to $(HOME)"
#include "/home/oaktobotics/catkin_ws/devel/include/motor_controller/IntList.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

class Robot
{
private:
  int weight;
  double armPos;
  double STAGES[4] = {0,3,5,5};//empty prep defaultentdig currentdig
  const int MAX_WEIGHT = (20000);//20000kg
  const int ARM_SPEED = 40;//digging speed thingy
  const int DRIVE_SPEED = 50;
  const int MAX_DEPTH = 20;
  const int MAX_HOPPER = 20;
  const int HOPPER_SPEED = 50;
  int leftPos;
  int rightPos;
  geometry_msgs::PoseStamped digZone;
  geometry_msgs::PoseStamped bin;
  tf::StampedTransform tfRef;
  bool isLocalized = false;

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<motor_controller::IntList>("motor", 1000);
  
  int getWeight();
  bool checkWeight();

  double getArmPos();
  bool checkArmPos();

  void moveArm(int, bool);
  void move(int, int, char);

  void dig();
  void scoop(bool); //this does scoopin'

  //initialize parameters
  void initialize();

  void dump();
  bool notMaxHeight();

  //goals
  void setGoal();

  //hardcoded movement
  void drive(bool);
  void dock();

public:
  Robot();
  void run();

  //localization
  bool localized();
  void localize(ar_track_alvar_msgs::AlvarMarkers);

  //TESTING
  void hello();
};

#endif
