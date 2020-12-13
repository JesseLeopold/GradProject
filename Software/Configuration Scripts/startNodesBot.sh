#!/bin/bash

# Setup ROS Environment Variables
  source /opt/ros/melodic/setup.bash

# Allow OS to Intialize (10 seconds)
  sleep 10

# Configure Network Specific Environment Varibles
  export ROS_IP=192.168.1.34
  export ROS_HOSTNAME=192.168.1.34
  export ROS_MASTER_URI=http://192.168.1.10:11311

  export ROS_PACKAGE_PATH=/home/jesse/catkin_ws:$ROS_PACKAGE_PATH
  source /home/jesse/catkin_ws/devel/setup.bash

# Set Working Directory
 cd /home/jesse/workspace/exec/

# Generate Main Log File Header
  #cat /dev/null > main.log
  echo "-----------------------------------" >>  main.log
  echo "--------Main Startup Script--------" >>  main.log
  echo "--------------BOT MODE-------------" >>  main.log
  echo "-----------------------------------" >>  main.log
  date >> main.log
  echo "-----------------------------------" >>  main.log

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

