#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <math.h>
#include "../include/v_repConst.h"

// Used data structures:
//#include "vrep_common/ProximitySensorData.h"
#include "vrep_common/VrepInfo.h"
//#include "vrep_common/JointSetStateData.h"
#include "sensor_msgs/Joy.h"

// Used API services:
//#include "vrep_common/simRosEnablePublisher.h"
//#include "vrep_common/simRosEnableSubscriber.h"


class AckermannSteering
{
    public:
    //constructor
    AckermannSteering();
    //destructor
    virtual ~AckermannSteering();

    // Topic subscriber callbacks:
    //void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
   // {
    //    simulationTime=info->simulationTime.data;
    //    simulationRunning=(info->simulatorState.data&1)!=0;
   // }

    void joyCallback(const sensor_msgs::JoyConstPtr& joyMsg);

    virtual void spin();
    virtual void spinonce();
    void process();

    protected:
    ros::NodeHandle node;
    ros::Subscriber joy_sub_;
    //ros::Subscriber subInfo;

    ros::Publisher steeringPub;
    ros::Publisher drivePub;
    ros::Publisher signalPub;
    ros::Publisher brakePub;

    std_msgs::Float32 steer;
    std_msgs::Float32 drive;
    std_msgs::Float32 signal;
    std_msgs::Float32 brakeSignal;

    int mode,sig;
    bool joyflag;
    //ros::ServiceClient client_enablePublisher;

    //vrep_common::simRosEnablePublisher srv_enablePublisher;
    //vrep_common::simRosEnableSubscriber srv_enableSubscriber;
    //bool simulationRunning;
    //bool sensorTrigger;
    //float simulationTime;

    sensor_msgs::JoyConstPtr joyin;
};

void AckermannSteering::joyCallback(const sensor_msgs::JoyConstPtr& joyMsg)
{
   joyin = joyMsg;
   //std::cout<<"received "<<signal<<std::endl;
   joyflag = true;
}

AckermannSteering::AckermannSteering()
{
    joy_sub_ = node.subscribe<sensor_msgs::Joy>("joy",10,&AckermannSteering::joyCallback,this);
    steeringPub = node.advertise<std_msgs::Float32>("vrep/steer",10);
    drivePub = node.advertise<std_msgs::Float32>("vrep/drive",10);
    signalPub = node.advertise<std_msgs::Float32>("vrep/signal",10);
    brakePub = node.advertise<std_msgs::Float32>("vrep/break",10);
}

AckermannSteering::~AckermannSteering(){

}

void AckermannSteering::spin()
{
    ros::Rate rate(10);
    while (ros::ok())
    {
        AckermannSteering::spinonce();
        rate.sleep();
    }
}

void AckermannSteering::spinonce()
{
    process();
    ros::spinOnce();
}

void AckermannSteering::process()
{
    //std::cout<<"publish drive and steer to vrep"<<std::endl;
    //std::cout<<"steering angle = "<<steer<<std::endl;
    if(joyin != NULL)
    {
      if(joyin->buttons[20]==1){
        steer.data = joyin->axes[0] * 450;
        steeringPub.publish(steer);

        brakeSignal.data = joyin->axes[2];
        brakePub.publish(brakeSignal);
        //Controlling the gear
        if(joyin->buttons[13]==1)   //Forward drive
        {
           mode = 1;
        }
        if(joyin->buttons[12]==1)   //Reverse Drive
        {
           mode = 2;
        }
        if (mode==0)               //Neutral
        {
           drive.data = 0;
        }

        //Control the brake and drive
        if (mode == 1)      //drive forward
        {
           if(joyin->axes[1] < 0)
           {
                drive.data = (joyin->axes[1] * 10);// - (joyin->axes[2] * 10 +10);
           }
           else
           {
                drive.data = (joyin->axes[1] * 10);// - (joyin->axes[2] * 10 +10);
           }

           if(drive.data<0)
           {
                drive.data=0;
           }
        }

        if(mode==2)     //reverse
        {
           if(joyin->axes[1] < 0)
           {
                drive.data = ((joyin->axes[1] * 10)*(-1));// - ((joyin->axes[2] * 10 +10)*(-1));
           }
           else
           {
                drive.data = ((joyin->axes[1] * 10)*(-1));// - ((joyin->axes[2] * 10 +10)*(-1));
           }

           if(drive.data>0)
           {
                drive.data=0;
           }
        }

        drivePub.publish(drive);

        //controlling signal
        if(joyin->buttons[11]==1)   //left signal
        {
//            if(sig==1)
//            {
//                sig=0;
//            }
//           else
//            {
                sig = 1;
            }
//        }
        if(joyin->buttons[10]==1)   //right signal
        {
//            if(sig==2)
//            {
//                sig=0;
//            }
//            else
//            {
                sig = 2;
//            }
        }
        if(joyin->buttons[7]==1 || joyin->buttons[6]==1)
        {
            sig=0;
        }

        if(sig==0)
        {
            signal.data=0;
        }
        if(sig==1)
        {
          signal.data=1;

        }
        if(sig==2)
        {
          signal.data=2;
        }
        signalPub.publish(signal);

    }
 }

}


// Main code:
int main(int argc,char* argv[])
{

    //char** _argv = NULL;
     std::string nodeName("joyAckermannSteering");
    ros::init(argc,argv,nodeName.c_str());


        AckermannSteering* ackermann_steering;
        ackermann_steering = new AckermannSteering ();
      ackermann_steering->spin();
    return(0);
}
