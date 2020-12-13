#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt8MultiArray.h"

void statusCallback(const std_msgs::String::ConstPtr& msg);
void signalCallback(const std_msgs::String::ConstPtr& msg);
void STTCallback(const std_msgs::String::ConstPtr& msg);
void DOACallback(const std_msgs::Int32::ConstPtr& msg);
void confidenceCallback(const std_msgs::Float32::ConstPtr& msg);
void audioCallback(const std_msgs::UInt8MultiArray& msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pseudoSub");

  ros::NodeHandle ASR0("/ASR0");

  ros::Subscriber subStatus = ASR0.subscribe("Status",       1000, statusCallback);
  ros::Subscriber subSignal = ASR0.subscribe("Signal",       1000, signalCallback);
  ros::Subscriber subSTT    = ASR0.subscribe("STT",          1000, STTCallback);
  ros::Subscriber subDOA    = ASR0.subscribe("DOA",          1000, DOACallback);
  ros::Subscriber subConfid = ASR0.subscribe("Confidence",   1000, confidenceCallback);
  ros::Subscriber subAudio  = ASR0.subscribe("Audio",        1000, audioCallback);

  ros::spin();

  return 0;
}

void statusCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Pseudo Sub - Status - Collected: [%s]",msg->data.c_str());
}

void signalCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Pseudo Sub - Signal - Collected: [%s]",msg->data.c_str());
}


void STTCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Pseudo Sub - STT - Collected: [%s]",msg->data.c_str());

}

void DOACallback(const std_msgs::Int32::ConstPtr& msg)
{
    int DOA = msg->data;
    ROS_INFO("Pseudo Sub - DOA - Collected: [%i]",DOA);

}

void confidenceCallback(const std_msgs::Float32::ConstPtr& msg)
{
    float confidence = msg->data;
    ROS_INFO("Pseudo Sub - Confidence - Collected: [%f]",confidence);
}

void audioCallback(const std_msgs::UInt8MultiArray& msg)
{
    uint8_t audio[1000];
    int i =0;
    std::vector<uint8_t>::const_iterator it;
    
    for( it = msg.data.begin(); it != msg.data.end(); it++)
    {
        if(i % 1000 == 0) ROS_INFO("Pseudo Sub - Audio - Byte#:  [%i] Collected: [%i]",i,*it);
        i++;
    }
    
    ROS_INFO("Pseudo Sub - Audio - Collected #Bytes: [%i]",i);

}




