#!/bin/bash

# Setup ROS Environment Variables
  source /opt/ros/melodic/setup.bash

# Allow OS to Intialize (10 seconds)
  sleep 10

  # Configure Network Specific Environment Varibles
  # ------------------- WIFI ---------------------
  export ROS_IP=192.168.1.115
  export ROS_HOSTNAME=192.168.1.115
  export ROS_MASTER_URI=http://localhost:11311

  export ROS_PACKAGE_PATH=/home/jesse/catkin_ws:$ROS_PACKAGE_PATH
  source /home/jesse/catkin_ws/devel/setup.bash

# Set Working Directory
 cd /home/jesse/workspace/exec/

# Generate Main Log File Header
  #cat /dev/null > main.log
  echo "-----------------------------------" >>  main.log
  echo "--------Main Startup Script--------" >>  main.log
  echo "-------------TEST MODE-------------" >>  main.log
  echo "-----------------------------------" >>  main.log
  date >> main.log
  echo "-----------------------------------" >>  main.log

# Generate Core Log File
  echo "-- ROS Core --->STARTING<---" >>  main.log
  #cat /dev/null > core.log
  echo "-----------------------------------" >>  core.log
  echo "------------ Core Log -------------" >>  core.log
  echo "-----------------------------------" >>  core.log
  date >> core.log
  echo "-----------------------------------" >>  core.log

# Start a a ROS Core for Test Environemnts
  roscore &>> core.log &
  echo "-- ROS Core --->RUNNING<---" >>  main.log

# Wait for Core to Intiailzie
  sleep 4

# Initialize  Publisher Node Log File
  echo "-- Pseudo Publisher --->STARTING<---" >>  main.log
  #cat /dev/null > pub.log
  echo "-----------------------------------" >>  pub.log
  echo "---------- Publisher Log ----------" >>  pub.log
  echo "-----------------------------------" >>  pub.log
  date > pub.log
  echo "-----------------------------------" >>  pub.log

# Start a Publisher Node for Test Environemnts
  rosrun speech_recog pseudoPub  &>> pub.log &
  echo "-- Pseudo Publisher --->RUNNING<---" >>  main.log

# Wait for Publisher Node to Initialize
  sleep 2

# Initialize the ASR Manager Log File
  echo "-- ASR Manager --->STARTING<---" >>  main.log
  #cat /dev/null > asr.log
  echo "-----------------------------------" >>  asr.log
  echo "-------- ASR Manager Log  ---------" >>  asr.log
  echo "-----------------------------------" >>  asr.log
  date >> asr.log
  echo "-----------------------------------" >>  asr.log
# Start the ASR Manager Node
  rosrun speech_recog asrManager &>> asr.log &
  echo "-- ASR Manager --->RUNNING<---" >> main.log

# Wait for ASR Manager Node to Intialize
  sleep 2

# Initialize the Pseudo Subscriber Log File
echo "-- Pseudo Subscriber --->STARTING<---" >>  main.log
  #cat /dev/null > sub.log
  echo "-----------------------------------" >>  sub.log
  echo "---------- Subscriber Log ---------" >>  sub.log
  echo "-----------------------------------" >>  sub.log
  date >> sub.log
  echo "-----------------------------------" >>  sub.log
  echo "--- PSEUDO SUBSCRIBER LOG ---" >>  sub.log

# Start the Pseudo Subscriber Node
  rosrun speech_recog pseudoSub  &>> sub.log &
  echo "-- Pseudo Subscriber --->RUNNIG<---" >>  main.log

