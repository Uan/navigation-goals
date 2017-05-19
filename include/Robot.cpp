#include <ros/ros.h>
#include <unistd.h>
#include "Robot.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
//change "oaktobotics to $(HOME)"
#include "/home/oaktobotics/catkin_ws/devel/include/motor_controller/IntList.h"
#include <vector>
#include <math.h>

Robot::Robot()
{
  initialize();
}
void Robot::initialize()
{
	if(checkWeight() && checkArmPos())
    {
		ROS_INFO("Robot has been created");
    }
    else 
	{
		ROS_INFO("One of the checkers has failed. Check the xistance of the Weight/ArmPos parameters");
	}

	//initialize digZone pose
	digZone.pose.position.x = 0;
	digZone.pose.position.y = 0;
	digZone.pose.position.z = 0;
	digZone.pose.orientation.x = 0;
	digZone.pose.orientation.y = 0;
	digZone.pose.orientation.z = 0;
	digZone.pose.orientation.w = 1;

}

void Robot::run(){
	//AUTONOMY
	ROS_INFO("Begin Autonomous Logic");
	while(ros::ok()){
		setGoal();
	}
}

bool Robot::checkWeight()
{
  if(nh.hasParam("/weight"))
  {
    nh.getParam("/weight", weight);
    return true;
  }

  return false;
}

bool Robot::checkArmPos()
{
  if(nh.hasParam("/armPos"))
  {
    nh.getParam("/armPos", armPos);
    return true;
  }

  return false;
}

int Robot::getWeight()
{
  checkWeight();
  return weight;
}

double Robot::getArmPos()
{
  checkArmPos();
  return armPos;
}

void Robot::dig()
{
      while(getWeight() <= MAX_WEIGHT)
      {
		moveArm(0, false);
		scoop(false);
		moveArm(1, false);
		move(0, 100, 'b');
		usleep(1000000);

        if(STAGES[3] == MAX_DEPTH)
        {
          STAGES[3] = STAGES[2];
		  move(1, DRIVE_SPEED, 'w');
		  usleep(100000); // wait 0.2s [usleep() + move()]
		  move(1, 0, 'w');
        }

		moveArm(3, false);
		scoop(true);

        STAGES[3]++;
        usleep(1000000);
      }
}

void Robot::moveArm(int index, bool atBin)
{
	int direction;
	switch(index)
	{
	case 0:
	  direction = 0;
	  break;
	case 1:
	  if(atBin)
	  {
		direction = 1;
		break;
	  }
	  direction = 0;
	  break;
	case 2:
	  direction = 1;
	  break;
	default:
	  ROS_INFO("WRONG CHOICE");
	  return;
	}

	while(getArmPos() != STAGES[index])//add tolerance
	{
		if(getArmPos() <= (STAGES[index] + 1) && getArmPos() >= (STAGES[index] - 1))
			break;
		else if(getArmPos() < STAGES[index])
			move(0, 100, 'a');
		else
			move(1, 100, 'a'); 
	}
	move(0, 0, 'a');
}

void Robot::move(int direction, int speed, char limb)
{  
  motor_controller::IntList msg;

  ROS_INFO("Sending..");
  // 1 = b, 0 = f, 2 = r, 3 = l
  if (limb == 'w'){
	if (direction == 1){
   		msg.wheel = {1, 5, speed, 1, 5};
	}
	else if (direction == 0){
		msg.wheel = {0, 4, speed, 0, 4};
	}
	else if (direction == 2){
		msg.wheel = {0, 5, speed, 0, 5};
	}
	else if (direction == 3){
    	msg.wheel = {1, 4, speed, 1, 4};
	}
  }
  if (limb == 'a'){
    msg.arm = {direction, speed};
  }
  if (limb == 'b'){
	if (direction == 0)
    	msg.bucket = {4, speed};
	else
		msg.bucket = {5, speed};
  }
  if (limb == 'h'){
    msg.hopper = {direction, speed, direction + 4, speed};
  }
  if (limb == 'l'){
    if (direction == 0){
      msg.hopper = {0, speed, 4, 0};
    }
    else
      msg.hopper = {1, speed, 5, 0};
  }
  if (limb == 'r'){
    if (direction == 0){
      msg.hopper = {0, 0, 4, speed};
    }
    else
      msg.hopper = {1, 0, 5, speed};
  }

  pub.publish(msg);
  usleep(100000);
  ROS_INFO("Moved");
}


void Robot::scoop(bool dig = true)
{
	if(dig)
	{
		move(1, 100, 'b');
		usleep(10000000);
		move(0, 100, 'b');
		usleep(10000000);
	}
	else
	{
		move(0, 100, 'b');
		usleep(10000000);
		move(1, 100, 'b');
		usleep(10000000);
	}
}

bool Robot::localized(){
	return isLocalized;
}

void Robot::localize(ar_track_alvar_msgs::AlvarMarkers msg){
	int size = msg.markers.size();

	move(2, DRIVE_SPEED, 'w');
	ROS_INFO("Markers[] size: %d", msg.markers.size());
	if(size > 4)
	{
		tf::TransformListener tfListener;
		usleep(5000000);
		ROS_INFO("Localizing..");
		move(1, 0, 'w');
		usleep(100000);
		ros::Time t = ros::Time(0);
		try{
			//tfListener.lookupTransform("marker", "head_camera", t, tfBin);
			tfListener.lookupTransform("base_link", "map", t, tfRef);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s", ex.what());
			ROS_WARN("Inside Catch Block");
			ros::Duration(5.0).sleep();
		}
 
		for(int i = 0; i < size; i++)
		{
			bin.pose.position.x += msg.markers[i].pose.pose.position.x;
			bin.pose.position.y += msg.markers[i].pose.pose.position.y;
			bin.pose.position.z += msg.markers[i].pose.pose.position.z;
			bin.pose.orientation.x += msg.markers[i].pose.pose.orientation.x;
			bin.pose.orientation.y += msg.markers[i].pose.pose.orientation.y;
			bin.pose.orientation.z += msg.markers[i].pose.pose.orientation.z;
		}
		//Set Bin Pose
		bin.pose.position.x /= size;
		bin.pose.position.y /= size;
		bin.pose.position.z /= size;
		bin.pose.orientation.x /= size;
		bin.pose.orientation.y /= size;
		bin.pose.orientation.z /= size;
		bin.pose.orientation.w = 1;	


		ROS_INFO("TF DATA:");
		ROS_INFO("Position X: %lf", tfRef.getOrigin().x());
		ROS_INFO("Position Y: %lf", tfRef.getOrigin().y());
		ROS_INFO("Position Z: %lf", tfRef.getOrigin().z());
		ROS_INFO("Rotation X: %lf", tfRef.getRotation().x());
		ROS_INFO("Rotation Y: %lf", tfRef.getRotation().y());
		ROS_INFO("Rotation Z: %lf", tfRef.getRotation().z());
		ROS_INFO("END TF DATA");

		ROS_INFO("POSE DATA:");
		ROS_INFO("Position X: %lf", bin.pose.position.x);
		ROS_INFO("Position Y: %lf", bin.pose.position.y);
		ROS_INFO("Position Z: %lf", bin.pose.position.z);
		ROS_INFO("Rotation X: %lf", bin.pose.orientation.x);
		ROS_INFO("Rotation Y: %lf", bin.pose.orientation.y);
		ROS_INFO("Rotation Z: %lf", bin.pose.orientation.z);
		ROS_INFO("END POSE DATA");

		//calculate digzone coordinates
		double binRefY = sqrt(pow(bin.pose.position.z, 2) - pow(bin.pose.position.x, 2));
		double digRefZ = sqrt(pow(bin.pose.position.x, 2) + pow((4.5 - binRefY), 2));
		double deltaZ = bin.pose.orientation.z;

		ROS_INFO("Position X: %lf", bin.pose.orientation.z);
		ROS_INFO("BinRefY: %lf", binRefY);
		ROS_INFO("DigRefZ: %lf", digRefZ);
		ROS_INFO("DeltaZ: %lf", deltaZ);

		//Set DigZone Pose
		digZone.pose.position.x = bin.pose.position.x;
		digZone.pose.position.y = 0;
		digZone.pose.position.z = -digRefZ;
		digZone.pose.orientation.x = 0;
		digZone.pose.orientation.y = 0;
		digZone.pose.orientation.z = 0;
		digZone.pose.orientation.w = 1;

		//calculate digZone (Pose or TF)
		//set digZone pose

		ROS_INFO("..Localized");
		isLocalized = true;
	}
}

void Robot::hello(){
	ROS_INFO("HELLO FROM OUR SAVIOR HARAMBE");
}

void Robot::setGoal(){
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0)) && ros::ok()){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	//we'll send a goal to the robot to move 1 meter forward
	goal.target_pose.header.frame_id = "/map";
	goal.target_pose.header.stamp = ros::Time::now();


	//use data info to set pose
	goal.target_pose.pose.position.x = 1.0;
	goal.target_pose.pose.orientation.w = 1.0;

	ROS_INFO("Sending goal");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Success");
	else
		ROS_INFO("Failed to reach goal");

	ac.cancelGoal();
}

bool Robot::notMaxHeight(){
    nh.getParam("/leftPos", leftPos);
    nh.getParam("/rightPos", rightPos);
    return ((leftPos + rightPos) / 2 < MAX_HOPPER);
}

void Robot::dump(){
    while (notMaxHeight()){
        if (leftPos < rightPos){
            move(0, HOPPER_SPEED, 'l');
        }
        else if (leftPos > rightPos){
            move(0, HOPPER_SPEED, 'r');
        }
        else
            move(0, HOPPER_SPEED, 'h');
    }
    usleep(3000000);
    move(1, HOPPER_SPEED, 'h');
    usleep(5000000);
}

