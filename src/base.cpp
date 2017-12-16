#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "ardrone_autonomy/Navdata.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
//#include "ros/ros.h"
#include "tf/transform_datatypes.h"
//#include "ar_track_alvar_msgs/AlvarMarkers.h"
//#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "Command.hpp"
#include "PIDcontroller.hpp"
#include <sstream>
#include <string>
#include <cmath>
#include <vector>

//rostopic pub /ardrone/reset std_msgs/Empty
int marklength;
double avgX, avgY, avgZ, roll, pitch, yaw, sumRoll, sumPitch, sumYaw;
int markSIZE = 1;

double convertDegree(double rad) {
  double degree = rad*180.0/M_PI;
  if (degree < 0)
    degree += 360;
  return degree;
}
//yall increases +rotz to 90 degree
void markerCallback(const ar_track_alvar_msgs::AlvarMarkers& msg)
{
  //int length = -1;
  avgX = 0;
  avgY = 0;
  avgZ = 0;
  sumRoll = 0;
  sumPitch = 0;
  sumYaw = 0;
  roll = 0;
  pitch = 0;
  yaw = 0;

  marklength = msg.markers.size();
  if (marklength == markSIZE) {
    for (int i = 0; i < marklength; i ++) {
      avgX += msg.markers[i].pose.pose.position.x;
      avgY += msg.markers[i].pose.pose.position.y;
      avgZ += msg.markers[i].pose.pose.position.z;
      tf::Quaternion q(msg.markers[i].pose.pose.orientation.x,
        msg.markers[i].pose.pose.orientation.y,
        msg.markers[i].pose.pose.orientation.z,
        msg.markers[i].pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      m.getRPY(roll, pitch, yaw);
      roll = convertDegree(roll);
      pitch = convertDegree(pitch);
      yaw = convertDegree(yaw);
      if (yaw > 180) {
        yaw -= 180;
      }
      sumRoll += roll;
      sumPitch += pitch;
      sumYaw += yaw;
    }
    avgX = avgX/markSIZE;
    avgY = avgY/markSIZE;
    avgZ = avgZ/markSIZE;
    sumRoll = sumRoll/markSIZE;
    sumPitch = sumPitch/markSIZE;
    sumYaw = sumYaw/markSIZE;
  }
  //ROS_INFO("%d sign detected!\nCenterX:%f\nCenterY:%f\nCenterZ:%f\n", marklength, avgX, avgY, avgZ);
  //ROS_INFO("ROll:%f\nPITCH:%f\nYALL:%f\n",sumRoll,sumPitch,sumYaw);
}

geometry_msgs::Twist generateTwist(std::string axis, float value, float value2, float value3) {
  geometry_msgs::Twist msg;
  msg.linear.x = 0;
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = 0;

  if (axis.compare("lx") == 0) {
    msg.linear.x = value;
  }
  else if (axis.compare("ly") == 0) {
    msg.linear.y = value;
    ROS_INFO("LZ:%f",msg.linear.y);
  }
  else if (axis.compare("lz") == 0) {
    msg.linear.z = value;
    ROS_INFO("LZ:%f",msg.linear.z);
  }
  else if (axis.compare("lylz")==0 ) {
    msg.linear.y = value;
    msg.linear.z = value2;
    //ROS_INFO("LY:%f\nLZ:%f\n",value,value2);
  }
  else if (axis.compare("lylzaz")==0 ) {
  msg.linear.y = value;
  msg.linear.z = value2;
  msg.angular.z = value3;
  //ROS_INFO("LY:%f\nLZ:%f\n",value,value2);
  }
  else if (axis.compare("az") == 0) {
    msg.angular.z = value;
    ROS_INFO("LZ:%f",msg.angular.z);
  }
  return msg;  
}

float readjust(Command &cmd) {
  float diff;
  if (cmd.diffRot() < 0) {
    diff = cmd.diffRot() + 360; 
  }
  else {
    diff = cmd.diffRot();
  }
  return diff; 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "base");
  ros::NodeHandle n;
  Command cmd;
  ros::Publisher takeOffPub = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 100);
  ros::Publisher landPub = n.advertise<std_msgs::Empty>("/ardrone/land",100);
  ros::Publisher velPub = n.advertise<geometry_msgs::Twist>("/cmd_vel",100);
  ros::Subscriber markerSub = n.subscribe("/ar_pose_marker", 100, markerCallback);
  ros::Subscriber stateSub = n.subscribe("/ardrone/navdata",100, &Command::stateCallback,&cmd);
  double linMax = 0.46;
  double angMax = 0.6;
  ros::Rate loop_rate(20);


  // Takeoff
  ros::Duration(0.5).sleep();
  std_msgs::Empty eMsg;
  takeOffPub.publish(eMsg);

  int count = 0;
  bool vertFlag = false;
  bool turnFlag = true;
  float diff = 0.0;
  int vertCount = 0;
 
    // Wait for hovering
  while(cmd.getState() != 4){
     ROS_INFO("STATE:%d",cmd.getState());
     ros::spinOnce();
     loop_rate.sleep();
  }

  ros::Duration(1).sleep();

  // Search for Tags 
    /*
  while (ros::ok() && marklength < markSIZE && vertCount < 4)
  {
    //Turning around;
    ROS_INFO("diff value is %6.4f", diff);
    cmd.rotReset();
    diff = 0;
    while(diff < 300) {
      ROS_INFO("diff value is %6.4f", diff);
      velPub.publish(generateTwist("az",0.5,0,0));
      ros::spinOnce();
      // All tags found
      if (marklength >= markSIZE) {
        break;
      }
      loop_rate.sleep();
      diff = readjust(cmd);
    }

    ros::Duration(0.5).sleep();

    // Vertical shift
    if (marklength < markSIZE) {
	    count = 0;
	    while (count < 10) {
	  	  velPub.publish(generateTwist("lz",0.1,0,0));
		    loop_rate.sleep();
		    ++count;		    
      }
      ++vertCount; 		
    }
    ros::Duration(0.5).sleep();
  }
*/

  // Aligning Y left Z tommand
  PIDcontroller linCtr;
  linCtr.setP(0.05);
  linCtr.setD(0.05);
  linCtr.setI(0);
  linCtr.setT(1);
 
  PIDcontroller angCtr;
  angCtr.setP(0.001);
  angCtr.setD(0.001);
  angCtr.setI(0);
  angCtr.setT(1);

  int Count = 0; //For rotating when losing target
  std::vector<double>angVec;
  double angVecSum;

  while(ros::ok()) {
    //Translation  CHANGE HERE WHEN USING 4  
    if (marklength > 0) {
      ROS_INFO("Detected");
      double velY = linCtr.calculatePID(avgY);
      double velZ = linCtr.calculatePID(avgZ);
      
      if (abs(velY) > linMax) {
        if (velY > 0)
          velY = linMax;
        else
          velY = -linMax;
      }
      if (abs(velZ) > linMax) {
        if (velZ > 0)
          velZ = linMax;
        else
          velZ = -linMax;
      }

      double angZ = angCtr.calculatePID(sumYaw-90);
      if (abs(angZ) > angMax) {
        if (angZ > 0){
          angZ = angMax;
        }
        else
          angZ = -angMax;
      }

      ROS_INFO("\nAVGY:%f\nAVGZ:%f\nYaw:%f\n\nY:%f\nZ:%f\nAngZ:%f",avgY,avgZ,(sumYaw-90),velY,velZ,angZ);
      // Storing value;
      angVec.push_back(sumYaw-90);
      //velPub.publish(generateTwist("lylz",velY,velZ,0));
      velPub.publish(generateTwist("lylzaz",velY,velZ,angZ));
    }
    else{
      ROS_INFO("Nothing found.");
      ros::Duration(1).sleep();      
      //Begin Searching
      if (angVec.empty()) {
        ROS_INFO("ERROR OCCURED, SHUTDOWN");
        break;
      }
      for (int i = 0; i < 20; i++) {
        angVecSum += angVec.back();
        angVec.pop_back();
      }
      angVecSum = angVecSum/20;
      if (angVecSum > 0) {
        Count = 0;
        while (Count < 20){ // 2 Sec
          ROS_INFO("ROTATE +az");
          velPub.publish(generateTwist("az",0.1,0,0));
          loop_rate.sleep();
          Count ++;
        }
      }
      else {
        Count = 0;
        while (Count < 20){ // 2 Secs
          ROS_INFO("Rotate -az");
          velPub.publish(generateTwist("az",-0.1,0,0));
          loop_rate.sleep();
          Count ++;
        }
      }
      angVec.clear();
      angVecSum = 0;     
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  //if (marklength == 4)
  //  ROS_INFO("Four tags found");
  //landPub.publish(eMsg);
  return 0;
}
