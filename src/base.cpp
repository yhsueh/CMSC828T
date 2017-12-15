#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "ardrone_autonomy/Navdata.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
//#8nclude "ar_track_alvar_msgs/AlvarMarker.h"
#include "geometry_msgs/Twist.h"
#include "Command.hpp"

#include <sstream>
#include <string>
int marklength;
float avgY, avgZ;

void markerCallback(const ar_track_alvar_msgs::AlvarMarkers& msg)
{
  avgY = 0;
  avgZ = 0;
  marklength = msg.markers.size();
  if (marklength == 4) {
    for (int i = 0; i < 4; i ++) {
      avgY += msg.markers[i].pose.pose.position.y;
      avgZ += msg.markers[i].pose.pose.position.z;
    }
    avgY = avgY/4;
    avgZ = avgZ/4;
  }
  ROS_INFO("%d sign detected! \n CenterY:%f\nCenterZ:%f\n", marklength, avgY, avgZ);
}


geometry_msgs::Twist generateTwist(std::string axis, float value) {
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

  if (axis.compare("lz") == 0) {
    msg.linear.z = value;
    ROS_INFO("LZ:%f",msg.linear.z);
  }

  if (axis.compare("az") == 0) {
    msg.angular.z = value;
    ROS_INFO("LZ:%f",msg.linear.z);
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
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "base");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  Command cmd;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher takeOffPub = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 100);
  ros::Publisher landPub = n.advertise<std_msgs::Empty>("/ardrone/land",100);
  ros::Publisher velPub = n.advertise<geometry_msgs::Twist>("/cmd_vel",100);
  ros::Subscriber markerSub = n.subscribe("/ar_pose_marker", 100, markerCallback);
  ros::Subscriber stateSub = n.subscribe("/ardrone/navdata",100, &Command::stateCallback,&cmd);
 
  // Takeoff
  ros::Duration(0.5).sleep();
  std_msgs::Empty eMsg;
  takeOffPub.publish(eMsg);

  ros::Rate loop_rate(10);
  int count = 0;
  bool vertFlag = false;
  bool turnFlag = true;
  float diff = 0.0;
  int vertCount = 0;
 
    // Wait for hovering
    while(cmd.getState() != 4){
      ros::spinOnce();
      loop_rate.sleep();
    }
  while (ros::ok() && marklength < 4 && vertCount < 4)
  {
    //Turning around;
    ROS_INFO("diff value is %6.4f", diff);
    cmd.rotReset();
    diff = 0;
    while(diff < 300) {
      ROS_INFO("diff value is %6.4f", diff);
      velPub.publish(generateTwist("az",0.5));
      ros::spinOnce();
      // All tags found
      if (marklength >= 4) {
        break;
      }
      loop_rate.sleep();
      diff = readjust(cmd);
    }

    ros::Duration(0.5).sleep();

    // Vertical shift
    if (marklength < 4) {
	    count = 0;
	    while (count < 10) {
	  	  velPub.publish(generateTwist("lz",0.1));
		    loop_rate.sleep();
		    ++count;		    
      }
      ++vertCount; 		
    }
    ros::Duration(0.5).sleep();
  }

  if (marklength == 4)
    ROS_INFO("Four tags found");
  landPub.publish(eMsg);
  return 0;
}
