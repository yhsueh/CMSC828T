#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "std_msgs/String.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

int marklength;
double avgX, avgY, avgZ, roll, pitch, yaw, sumRoll, sumPitch, sumYaw;

double convertDegree(double rad) {
  double degree = rad*180.0/M_PI;
  if (degree < 0)
    degree += 360;
  return degree;
}



//yall increases +rotz to 90 degree
void chatterCallback(const ar_track_alvar_msgs::AlvarMarkers& msg)
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
  if (marklength == 1) {
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
      sumRoll += roll;
      sumPitch += pitch;
      sumYaw += yaw;
    }
    avgX = avgX/4;
    avgY = avgY/4;
    avgZ = avgZ/4;
  }
  ROS_INFO("%d sign detected!\nCenterX:%f\nCenterY:%f\nCenterZ:%f\n", marklength, avgX, avgY, avgZ);
  ROS_INFO("ROll:%f\nPITCH:%f\nYALL:%f\n",sumRoll,sumPitch,sumYaw);
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
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("/ar_pose_marker", 1000, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
