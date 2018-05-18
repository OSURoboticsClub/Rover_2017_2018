#include <string>
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>

#include "simplemotion/simplemotion.h"


using namespace std;

int device_address = 1;

////////// Global Variables //////////
// ROS Parameter Defaults
const string default_port = "/dev/ttyUSB0";


// Axis Defaults
const smint32 axis1_max_count = 100000;
float axis1_max_degrees = 180.0;

const smint32 axis2_max_count = 100000;

const smint32 axis3_max_count = 100000;

const smint32 axis4_max_count = 100000;

const smint32 axis5_max_count = 320000;

const smint32 axis6_max_count = 100000;


class RoverArm{
public:
    RoverArm(int argc, char** argv){
        ros::init(argc, argv, "rover_arm");

        node_handle = new ros::NodeHandle("~");

        node_handle->param("port", arm_port, default_port);


        // Connect to arm, exit if failed
        arm_bus_handle = smOpenBus(arm_port.c_str());

        if(arm_bus_handle < 0){
            ROS_ERROR("Could not connect to arm");
        }
    }

    void run(){
        char dir = 0;

        printf("OK?: %d", ros::ok());

        while(ros::ok()){
            smSetParameter(arm_bus_handle, device_address, SMP_FAULTS, 0);
            smSetParameter(arm_bus_handle, device_address, SMP_CONTROL_BITS1, SMP_CB1_ENABLE);

            smint32 status = 0;
            while(!(status & STAT_TARGET_REACHED)){
                smRead1Parameter(arm_bus_handle, device_address, SMP_STATUS, &status);
            }

            dir = !dir;

            if(dir){
                smSetParameter(arm_bus_handle, device_address, SMP_ABSOLUTE_SETPOINT, 0);
            }else{
                smSetParameter(arm_bus_handle, device_address, SMP_ABSOLUTE_SETPOINT, 330000);
            }

            ros::spinOnce();
        }

        ROS_ERROR("Shutting down.");
    }
private:
    ros::NodeHandle *node_handle;

    string arm_port;
    smbus arm_bus_handle;



};



int main(int argc, char** argv){
    RoverArm rover_arm(argc, argv);
    rover_arm.run();
}