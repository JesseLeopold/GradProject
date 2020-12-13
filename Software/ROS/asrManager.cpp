// LIBRARY INCLUDES
#include "ros/ros.h"                    // Standard ROS Library
#include "std_msgs/String.h"            // ROS Data Type - String
#include "std_msgs/Float32.h"           // ROS Data Type - Float32
#include "std_msgs/Int32.h"             // ROS Data Type - Unsigned 32-Bit Integer
#include "std_msgs/UInt8.h"             // ROS Data Type - Unsigned  8-Bit Integer 
#include "std_msgs/ByteMultiArray.h"    // ROS Data Type - Byte Array
#include "std_msgs/UInt8MultiArray.h"   // ROS Data Type - Unsigned Integer Array
#include <iostream>                     // Input/Output Stream Library
#include <sstream>                      // String String Library
#include <string>                       // Data Type - String
#include <stdio.h>                      // Standard IO Library
#include <stdlib.h>                     // Variable Type Library
#include <cstdio>                       // Standard IO Library                    
#include <memory>                       // Memory Library
#include <stdexcept>                    // Error Handling Library
#include <array>                        // Array Library
#include <fstream>                      // File Stream Library
#include <vector>                       // Vector Library

using namespace std;

// FUNCTION PROTOTYPES
void listenCallback(const std_msgs::String::ConstPtr& msg); 
void sendStatus();
void sendSTT();
void sendSignal();
void sendConfidence();
void sendDOA();
void sendAudio();
std::string exec(const char* cmd);


// LOCAL VARIABLES

// ROS Declaration
ros::Publisher pubStatus;
ros::Publisher pubSTT;
ros::Publisher pubSignal;
ros::Publisher pubDOA;
ros::Publisher pubConfidence;
ros::Publisher pubAudio;
ros::Subscriber sub;

// Overall Autonomous Robot System Status Variable
enum SysState {SYS_IDLE, SYS_MOVING, SYS_BEACON, SYS_LISTEN, SYS_CHILL};
SysState systemState  = SYS_IDLE;

// ASR Manager Node Status Variable (FSM Control)
enum ASRState {ASR_IDLE, ASR_BEACON, ASR_LISTEN, ASR_CHILL};
ASRState currentState = ASR_IDLE;

//General Use Variables: Iterators, flags, etc. 
bool idleFlag = false, beaconingFlag = false, listeningFlag = false, chillingFlag = false;
int delayCounter = 0;
string speechText, confidText, scoreText, directionText;
float confidence;
int score, direction;

int main(int argc, char **argv)
{
    // Setup ROS Node Parameters
    ros::init(argc, argv, "ASR0");
    ros::NodeHandle ASR0("/ASR0");
    ros::NodeHandle NAV0("/NAV0");

    // Configure all ROS Topics to Publish
    pubStatus      = ASR0.advertise<std_msgs::String>          ("Status"    , 100);
    pubSTT         = ASR0.advertise<std_msgs::String>          ("STT"       , 100);
    pubSignal      = ASR0.advertise<std_msgs::String>          ("Signal"    , 100);
    pubDOA         = ASR0.advertise<std_msgs::Int32>           ("DOA"       , 100);
    pubConfidence  = ASR0.advertise<std_msgs::Float32>         ("Confidence", 100);
    pubAudio       = ASR0.advertise<std_msgs::UInt8MultiArray> ("Audio"     , 100);

  
    // Set Publishing Rate
    ros::Rate loop_rate(1);   // Publish Once a Second
  
    // Configure all ROS Topic Subscriptions 
    sub = NAV0.subscribe("mode", 1000, listenCallback);

    ROS_INFO("ASR Manager - System Initialized");

    while(ros::ok())
    {
        // Show the ASR's FSM status
        //sendStatus();

        // ASR Manager FSM
        switch(currentState) 
        {
            // WAIT ON THE SYSTEM TO BE IN A STOPPED STATE
            case ASR_IDLE:
                
                // On Idle, wait for the system comand to change
                if     (systemState  == SYS_IDLE)    currentState = ASR_IDLE;
                else if(systemState  == SYS_MOVING)  currentState = ASR_IDLE;
                else if(systemState  == SYS_BEACON)  currentState = ASR_BEACON;
                else if(systemState  == SYS_LISTEN)  currentState = ASR_LISTEN;
                else if(systemState  == SYS_CHILL)   currentState = ASR_CHILL;
                
            break;
           
            // ONCE THE SYSTEM IS IN A STOPPED STATE, BROADCAST AN AUDITORY BEACON
            case ASR_BEACON:
                if(!beaconingFlag)
                {
                    // Execute Beacon Command
                    system("/home/jesse/workspace/BeaconSounds/playBeacon.sh");
                    beaconingFlag = true;
                    delayCounter = 0;
                    currentState = ASR_BEACON;
                }
                else
                {
                    // Wait until the Beacon has had time to broadcast before entering the Listening State
                    if (delayCounter > 3) 
                    {
                        currentState = ASR_LISTEN;
                        beaconingFlag = false;
                    }
                }
                delayCounter++;

            break;
            
            // RUN AN ASR ROUTINE TO CAPTURE VOICE CONTENT
            case ASR_LISTEN:
                if(!listeningFlag)
                {
                    // Transmit a signal to ASR1 to trigger a listening event
                    sendSignal();
                    
                    // Execute ASR Listening Command
                    exec("/home/jesse/workspace/autorun/rec5SecAnalyze.sh");
                       
                    // Set FSM Parameters 
                    listeningFlag = true;
                    currentState = ASR_LISTEN;
                    
                    // Publish to a topic specifically for Olga to consume so that she doesn't have to figure out a flag issue
                }
                else
                {
                    // Collect Recog Parameters and Data
                    exec("/home/jesse/workspace/autorun/retrieveLogs.sh");
                    confidText      = exec("/home/jesse/workspace/autorun/getConfidence.sh");
                    scoreText       = exec("/home/jesse/workspace/autorun/getScore.sh");
                    speechText      = exec("/home/jesse/workspace/autorun/getText.sh");
                    directionText   = exec("/home/jesse/workspace/autorun/getDirection.sh");
                    //exec("/home/jesse/workspace/autorun/getFile.sh");
                    

                    // Convert Parameters from Text into integers and floats
                    ROS_INFO("ASR Manager - STT Length - %i",speechText.length());
                        
                    if        ( (speechText.length() <= 3) || (speechText.compare(0,7," (null)") == 0) )    confidence = 0;
                    else if     (speechText.length() >  3)                                                  confidence = 1;
                    score      = std::atoi(scoreText.c_str());
                    direction  = std::atoi(directionText.c_str());

                    // Transmit Collected Parameters to ROS
                    sendSTT();
                    sendDOA();
                    sendConfidence();

                    // Play a chime Based upon findings
                    if(confidence >= 0.7)  system("/home/jesse/workspace/BeaconSounds/playPositiveChime.sh");
                    else                   system("/home/jesse/workspace/BeaconSounds/playNegativeChime.sh");

                    // Transmit Collected Audio Data to ROS
                    //sendAudio();

                    // Set FSM Parameters 
                    listeningFlag = false;
                    delayCounter = 0;
                    currentState = ASR_IDLE;
                    systemState = SYS_IDLE;
                }

                delayCounter++;
            
            break;

            case ASR_CHILL:
            if(!chillingFlag)
            {
                system("/home/jesse/workspace/BeaconSounds/playChill.sh");
            
                chillingFlag = true;
                delayCounter = 0;
                currentState = ASR_CHILL;
            }
            else 
            {
                if(delayCounter > 1000)
                {
                    chillingFlag = false;
                    delayCounter = 0;
                    currentState = ASR_IDLE;
                }
            }
            
            delayCounter++;
            break;
        }
        
        // Show the ASR's FSM status
        sendStatus();

        // General ROS Sleep Commands.
        ros::spinOnce();
        loop_rate.sleep();
  }

  return 0;
}


// Listener - Listen for System Status Changes and update System Status Register Accordingly
void listenCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("ASR Manager - System Mode  - Collected: [%s]", msg->data.c_str());
    
    // Convert incoming Status Mesasge into the Internal System State Variable
    if( strcmp( msg->data.c_str(),"Idle") == 0)          systemState = SYS_IDLE;
    else if( strcmp( msg->data.c_str(),"Navigate") == 0) systemState = SYS_MOVING;
    else if( strcmp( msg->data.c_str(),"Beacon") == 0)   systemState = SYS_BEACON;
    else if( strcmp( msg->data.c_str(),"Listen") == 0)   systemState = SYS_LISTEN;
    else if( strcmp( msg->data.c_str(),"Chill") == 0)    systemState = SYS_CHILL;
}


// Publish System Status to ROS Topic
void sendStatus()
{
    std_msgs::String msgStatus;
    std::stringstream ss;

    switch(currentState)
    {
        case ASR_IDLE:    ss << "Idle";   break;
        case ASR_BEACON:  ss << "Beacon"; break;
        case ASR_LISTEN:  ss << "Listen"; break;
    }
    
    msgStatus.data = ss.str();  
    ROS_INFO("ASR Manager - Sending Status - %s",msgStatus.data.c_str());
    pubStatus.publish(msgStatus);
}

// Publish Computed Speech to Text Data to ROS Topic 
void sendSTT()
{
      std_msgs::String msgSTT;
      std::stringstream ss;
      ss << speechText; 
      msgSTT.data = ss.str();  
      ROS_INFO("ASR Manager - Sending STT - %s",msgSTT.data.c_str());
      pubSTT.publish(msgSTT);
}

// Push a Flag to ROS as a Flag for ASR1 to synchronize with 
void sendSignal()
{
      std_msgs::String msgSignal;
      std::stringstream ss;
      ss << "Listening"; 
      msgSignal.data = ss.str();  
      ROS_INFO("ASR Manager - Sending Signal - %s",msgSignal.data.c_str());
      pubSignal.publish(msgSignal);
}

// Publish Computed Confidence Interval
void sendConfidence()
{
    std_msgs::Float32 msgConfid;
    msgConfid.data = confidence;
      ROS_INFO("ASR Manager - Sending Confidence - %f",msgConfid.data);
      pubConfidence.publish(msgConfid);
}


// Publish Detection of Arrival Data to ROS Topic
void sendDOA()
{
    std_msgs::Int32 msgDOA;
    msgDOA.data = direction;
    ROS_INFO("ASR Manager - Sending DOA - %i",msgDOA.data);
    pubDOA.publish(msgDOA);
}

// Safely Run an External Sript Command
std::string exec(const char* cmd) 
{
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}


// Serialize and Transmit Audio Data to ROS 
void sendAudio()
{
    std_msgs::UInt8MultiArray audioData;
    int i = 0;

    audioData.data.clear();
    
    // Pull in Audio Data From File
    std::ifstream       file("/home/jesse/workspace/autorun/out/fileHexCropped.txt");
    if (file)
    {
        file.seekg(0,std::ios::end);
        std::streampos          length = file.tellg();
        file.seekg(0,std::ios::beg);
        std::vector<char>       buffer(length);
        file.read(&buffer[0],length);
        
        // Convert Data into Data Array
        for (auto& it : buffer) 
        { 
            if( i % 1000 == 0)  ROS_INFO("ASR Manager - Sending Audio - Byte#: [%i], Data: [%i]",i,it);
            audioData.data.push_back(it);
            i++;
        } 

        // Publish Audio Data
        pubAudio.publish(audioData);
    
    }
}
