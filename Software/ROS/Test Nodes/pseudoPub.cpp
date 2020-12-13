#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pseudoPub");
  ros::NodeHandle n("/NAV0");

  ros::Publisher pseudo_pub = n.advertise<std_msgs::String>("mode"    , 1000);

  //ros::Rate loop_rate(10);
  ros::Rate loop_rate(1);

  int count = 0;

  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;

    if   (count % 25 == 0)   
    {
        ss << "Beacon";
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        pseudo_pub.publish(msg);
    } 
        
        ros::spinOnce();

    loop_rate.sleep();
    count++;
  }

  return 0;
}

