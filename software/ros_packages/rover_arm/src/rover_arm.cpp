#include <string>
#include <iostream>
#include <unistd.h>
#include <algorithm>

#include <ros/ros.h>
#include <ros/console.h>

#include "simplemotion/simplemotion.h"
#include <rover_arm/ArmControlMessage.h>
#include <rover_arm/ArmStatusMessage.h>


using namespace std;

int device_address = 1;

////////// Global Variables //////////
// ROS Parameter Defaults
const string default_port = "/dev/ttyUSB0";

const string default_absolute_position_control_topic = "control/absolute";
const string default_relative_position_control_topic = "control/relative";
const string default_arm_status_topic = "status";

////////// Axis limits and conversions //////////
// Base
const smuint8 base_address = 1;
const smint32 base_counts_per_rev = 5725807;
const double base_min = -0.5;
const double base_max_rev = 0.5;

// Shoulder
const smuint8 shoulder_address = 2;
const smint32 shoulder_counts_per_rev = 5725807; //FIXME
const double shoulder_min_rev = -0.25;
const double shoulder_max_rev = 0.25;

//Elbow
const smuint8 elbow_address = 3;
const smint32 elbow_counts_per_rev = 5725807;  //FIXME
const double elbow_min_rev = -0.5;
const double elbow_max_rev = 0.5;

//Roll
const smuint8 roll_address = 4;
const smint32 roll_counts_per_rev = 5725807; //FIXME
const double roll_min_rev = -0.25;
const double roll_max_rev = 0.25;

//Wrist Pitch
const smuint8 wrist_pitch_address = 5;
const smint32 wrist_pitch_counts_per_rev = 5725807; //FIXME
const double wrist_pitch_min_rev = -0.25;
const double wrist_pitch_max_rev = 0.25;

//Wrist Roll
const smuint8 wrist_roll_address = 6;
const smint32 wrist_roll_counts_per_rev = 5725807; //FIXME


class RoverArm{
public:
    RoverArm(int argc, char** argv){
        ros::init(argc, argv, "rover_arm");

        node_handle = new ros::NodeHandle("~");

        node_handle->param("port", arm_port, default_port);

        arm_bus_handle = smOpenBus(arm_port.c_str());

        if(arm_bus_handle < 0){
            ROS_ERROR("Could not connect to arm");
            return;
        }else{
            arm_sucessfully_connected = true;
        }

        absolute_position_control_subscriber = node_handle->subscribe(default_absolute_position_control_topic, 1, &RoverArm::absolute_position_callback, this);
        relative_position_control_subscriber = node_handle->subscribe(default_relative_position_control_topic, 1, &RoverArm::relative_position_callback, this);

        status_publisher = node_handle->advertise<rover_arm::ArmStatusMessage>(default_arm_status_topic, 1);

    }

    void run(){
        char dir = 0;



        while(ros::ok()){
            if(!arm_sucessfully_connected){ return; }

            clear_faults();
            get_joint_statuses();
            set_joint_positions();
            reset_controllers();

            ros::spinOnce();
            sleep(1);
        }


        ROS_ERROR("Shutting down.");
    }

    void reset_controllers(){
        // Check if should reset
        // Reset all, then kill thread
    }

    void clear_faults(){
        if(should_clear_faults) {
            smSetParameter(arm_bus_handle, base_address, SMP_FAULTS, 0);
            smSetParameter(arm_bus_handle, base_address, SMP_CONTROL_BITS1, SMP_CB1_ENABLE);

            should_clear_faults = false;
        }

    }

    void get_joint_statuses(){

        // Check if joint states valid
        // SMP_STATUS
        // SMP_FAULTS

        // SMP_ACTUAL_POSITION

        if(!received_first_joint_position_update) {
            base_comm_state = smRead3Parameters(arm_bus_handle, base_address, SMP_STATUS, &base_status, SMP_FAULTS,
                                                &base_faults, SMP_ACTUAL_POSITION_FB, &base_position);
        }

        smint32 base_current_position;
        smRead3Parameters(arm_bus_handle, base_address, SMP_STATUS, &base_status, SMP_FAULTS,
                                                       &base_faults, SMP_ACTUAL_POSITION_FB, &base_current_position);

        smint32 shoulder_current_position;
        smRead3Parameters(arm_bus_handle, base_address, SMP_STATUS, &base_status, SMP_FAULTS,
                          &base_faults, SMP_ACTUAL_POSITION_FB, &shoulder_current_position);
//        shoulder_comm_state = smRead3Parameters(arm_bus_handle, shoulder_address, SMP_STATUS, &shoulder_status, SMP_FAULTS, &shoulder_faults, SMP_ACTUAL_POSITION_FB, &shoulder_position);
//        elbow_comm_state = smRead3Parameters(arm_bus_handle, elbow_address, SMP_STATUS, &elbow_status, SMP_FAULTS, &elbow_faults, SMP_ACTUAL_POSITION_FB, &elbow_position);
//        roll_comm_state = smRead3Parameters(arm_bus_handle, roll_address, SMP_STATUS, &roll_status, SMP_FAULTS, &roll_faults, SMP_ACTUAL_POSITION_FB, &roll_position);
//        wrist_pitch_comm_state = smRead3Parameters(arm_bus_handle, wrist_pitch_address, SMP_STATUS, &wrist_pitch_status, SMP_FAULTS, &wrist_pitch_faults, SMP_ACTUAL_POSITION_FB, &wrist_pitch_position);
//        wrist_roll_comm_state = smRead3Parameters(arm_bus_handle, wrist_roll_address, SMP_STATUS, &wrist_roll_status, SMP_FAULTS, &wrist_roll_faults, SMP_ACTUAL_POSITION_FB, &wrist_roll_position);
        ROS_INFO("Base position: %ld", base_position);
        // If so, broadcast
        status_message.base = base_current_position / base_counts_per_rev;
        status_message.shoulder = shoulder_current_position / shoulder_counts_per_rev;
        status_message.elbow = elbow_position / elbow_counts_per_rev;
        status_message.roll = roll_position / roll_counts_per_rev;
        status_message.wrist_pitch = wrist_pitch_position / wrist_pitch_counts_per_rev;
        status_message.wrist_roll = wrist_roll_position / wrist_roll_counts_per_rev;

//        ROS_INFO("BASE:%d\tSHOULDER: %d\tELBOW: %d\tROLL: %d\tW_PITCH: %d\tW_ROLL: %d\t", base_comm_state, shoulder_comm_state, elbow_comm_state, roll_comm_state, wrist_pitch_comm_state, wrist_roll_comm_state );

        //float64 base
        //float64 shoulder
        //float64 elbow
        //float64 roll
        //float64 wrist_pitch
        //float64 wrist_roll
        //
        //bool base_present
        //bool shoulder_present
        //bool elbow_present
        //bool roll_present
        //bool wrist_pitch_present
        //bool wrist_roll_present
        //
        //int32 base_status
        //int32 shoulder_status
        //int32 elbow_status
        //int32 roll_status
        //int32 wrist_pitch_status
        //int32 wrist_roll_status
        //
        //int32 base_faults
        //int32 shoulder_faults
        //int32 elbow_faults
        //int32 roll_faults
        //int32 wrist_pitch_faults
        //int32 wrist_roll_faults

        status_publisher.publish(status_message);

        // If first run, set received first message
        if(!received_first_joint_position_update){ received_first_joint_position_update = true; }

    }

    void set_joint_positions(){
        if(new_positions_received){
            set_base_position();
            set_shoulder_position();
            set_elbow_position();
            set_roll_position();
            set_wrist_positions();

         new_positions_received = false ;
        }
    }

    void set_base_position(){
        smSetParameter(arm_bus_handle, base_address, SMP_FAULTS, 0);
        smSetParameter(arm_bus_handle, base_address, SMP_CONTROL_BITS1, SMP_CB1_ENABLE);
        smSetParameter(arm_bus_handle, base_address, SMP_ABSOLUTE_SETPOINT, base_position);
        ROS_INFO("SENT TO BASE: %ld", base_position);
    }

    void set_shoulder_position(){
//        smSetParameter(arm_bus_handle, shoulder_address, SMP_FAULTS, 0);
//        smSetParameter(arm_bus_handle, shoulder_address, SMP_CONTROL_BITS1, SMP_CB1_ENABLE);
//        smSetParameter(arm_bus_handle, shoulder_address, SMP_ABSOLUTE_SETPOINT, shoulder_position);

    }

    void set_elbow_position(){

    }

    void set_roll_position(){

    }

    void set_wrist_positions(){

    }

    void absolute_position_callback(const rover_arm::ArmControlMessage::ConstPtr& msg)
    {
        if(!received_first_joint_position_update){ return; }



//        base_position = smint32(min(max(msg->base, base_min), base_max_rev) * base_counts_per_rev);
//        shoulder_position = smint32(min(max(msg->base, base_min), base_max_rev) * base_counts_per_rev);;
//        elbow_position = smint32(min(max(msg->elbow, elbow_min_rev), elbow_max_rev) * elbow_counts_per_rev);;
//        roll_position = smint32(min(max(msg->roll, roll_min_rev), roll_max_rev) * roll_counts_per_rev);;
//        wrist_pitch_position = smint32(min(max(msg->wrist_pitch, wrist_pitch_min_rev), wrist_pitch_max_rev) * wrist_pitch_counts_per_rev);
//        wrist_roll_position = smint32(msg->wrist_roll * wrist_roll_counts_per_rev);

        base_position = msg->base * base_counts_per_rev;
        shoulder_position = msg->base * base_counts_per_rev;

//        shoulder_position = smint32(min(max(msg->base* base_counts_per_rev);;
//        elbow_position = smint32(min(max(msg->elbow, elbow_min_rev), elbow_max_rev) * elbow_counts_per_rev);;
//        roll_position = smint32(min(max(msg->roll, roll_min_rev), roll_max_rev) * roll_counts_per_rev);;
//        wrist_pitch_position = smint32(min(max(msg->wrist_pitch, wrist_pitch_min_rev), wrist_pitch_max_rev) * wrist_pitch_counts_per_rev);
//        wrist_roll_position = smint32(msg->wrist_roll * wrist_roll_counts_per_rev);

        should_clear_faults = (msg->clear_faults);
        should_reset = (msg->reset_controllers);

        new_positions_received = true;

        ROS_INFO("BASE:%d\tSHOULDER: %d\tELBOW: %d\tROLL: %d\tW_PITCH: %d\tW_ROLL: %d\t", base_position, shoulder_position, elbow_position, roll_position, wrist_pitch_position, wrist_roll_position );
//        ROS_INFO("ABS::::Base: %f\tshoulder: %f", msg->base, msg->shoulder);
    }

    void relative_position_callback(const rover_arm::ArmControlMessage::ConstPtr& msg)
    {
        if(!received_first_joint_position_update){ return; }

        base_position += msg->base;
        shoulder_position += msg->shoulder;
        elbow_position += msg->elbow;
        roll_position += msg->roll;
        wrist_pitch_position += msg->wrist_pitch;
        wrist_roll_position += msg->wrist_roll;

        should_clear_faults += msg->clear_faults;
        should_reset += msg->reset_controllers;

        new_positions_received = true;

        ROS_INFO("REL::::Base: %f\tshoulder: %f", msg->base, msg->shoulder);
    }

private:
    ros::NodeHandle *node_handle;

    string arm_port;
    smbus arm_bus_handle;
    bool arm_sucessfully_connected = false;

    ros::Publisher status_publisher;
    rover_arm::ArmStatusMessage status_message;

    ros::Subscriber absolute_position_control_subscriber;
    ros::Subscriber relative_position_control_subscriber;

    bool received_first_joint_position_update = false;

    bool new_positions_received = false;

    smint32 base_position = 0;
    smint32 base_comm_state = 0;
    smint32 base_status = 0;
    smint32 base_faults = 0;

    smint32 shoulder_position = 0;
    smint32 shoulder_comm_state = 0;
    smint32 shoulder_status = 0;
    smint32 shoulder_faults = 0;

    smint32 elbow_position = 0;
    smint32 elbow_comm_state = 0;
    smint32 elbow_status = 0;
    smint32 elbow_faults = 0;

    smint32 roll_position = 0;
    smint32 roll_comm_state = 0;
    smint32 roll_status = 0;
    smint32 roll_faults = 0;

    smint32 wrist_pitch_position = 0;
    smint32 wrist_pitch_comm_state = 0;
    smint32 wrist_pitch_status = 0;
    smint32 wrist_pitch_faults = 0;

    smint32 wrist_roll_position = 0;
    smint32 wrist_roll_comm_state = 0;
    smint32 wrist_roll_status = 0;
    smint32 wrist_roll_faults = 0;



    bool should_clear_faults = true;
    bool should_reset = false;



};


int main(int argc, char** argv){
    RoverArm rover_arm(argc, argv);
    rover_arm.run();
}