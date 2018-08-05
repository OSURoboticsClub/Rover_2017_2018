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
////////// Simplemotion variables and defines /////
#define SMP_SERIAL_ENC_OFFSET 575 //This was missing from simplemotion_defs.h

////////// ROS Variables and Defines /////
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
const smint32 shoulder_counts_per_rev = 2620130;
const double shoulder_min_rev = -0.25;
const double shoulder_max_rev = 0.25;

//Elbow
const smuint8 elbow_address = 3;
const smint32 elbow_counts_per_rev = 4917661;
const double elbow_min_rev = -0.5;
const double elbow_max_rev = 0.5;

//Roll
const smuint8 roll_address = 4;
const smint32 roll_counts_per_rev = 1637581;
const double roll_min_rev = -0.25;
const double roll_max_rev = 0.25;

//Wrist Pitch
const smuint8 wrist_pitch_address = 5;
const smint32 wrist_pitch_counts_per_rev = 3799492;
const double wrist_pitch_min_rev = -0.25;
const double wrist_pitch_max_rev = 0.25;

//Wrist Roll
const smuint8 wrist_roll_address = 6;
const smint32 wrist_roll_counts_per_rev = 3799492;


class RoverArm {
public:
    RoverArm(int argc, char **argv) {
        ros::init(argc, argv, "rover_arm");

        node_handle = new ros::NodeHandle("~");

        node_handle->param("port", arm_port, default_port);

        arm_bus_handle = smOpenBus(arm_port.c_str());

        if (arm_bus_handle < 0) {
            ROS_ERROR("Could not connect to arm");
            return;
        } else {
            arm_successfully_connected = true;
        }

        absolute_position_control_subscriber = node_handle->subscribe(default_absolute_position_control_topic, 1,
                                                                      &RoverArm::absolute_position_callback, this);
        relative_position_control_subscriber = node_handle->subscribe(default_relative_position_control_topic, 1,
                                                                      &RoverArm::relative_position_callback, this);

        status_publisher = node_handle->advertise<rover_arm::ArmStatusMessage>(default_arm_status_topic, 1);

    }

    void run() {
        char dir = 0;


        while (ros::ok()) {
            if (!arm_successfully_connected) { return; }

            clear_faults();
            set_calibration();
            get_joint_statuses();
            set_joint_positions();
            reset_controllers();

            ros::spinOnce();
//            sleep(1);
        }


        ROS_ERROR("Shutting down.");
    }

    void reset_controllers() {
        if (should_reset) {
            smSetParameter(arm_bus_handle, base_address, SMP_SYSTEM_CONTROL, SMP_SYSTEM_CONTROL_RESTART);
            received_first_joint_position_update = false;
            should_clear_faults = true;
            should_reset = false;
        }
    }

    void clear_faults() {
        if (should_clear_faults) {
            smSetParameter(arm_bus_handle, base_address, SMP_FAULTS, 0);
            smSetParameter(arm_bus_handle, base_address, SMP_CONTROL_BITS1, SMP_CB1_ENABLE);

            smSetParameter(arm_bus_handle, shoulder_address, SMP_FAULTS, 0);
            smSetParameter(arm_bus_handle, shoulder_address, SMP_CONTROL_BITS1, SMP_CB1_ENABLE);

            smSetParameter(arm_bus_handle, elbow_address, SMP_FAULTS, 0);
            smSetParameter(arm_bus_handle, elbow_address, SMP_CONTROL_BITS1, SMP_CB1_ENABLE);

            smSetParameter(arm_bus_handle, roll_address, SMP_FAULTS, 0);
            smSetParameter(arm_bus_handle, roll_address, SMP_CONTROL_BITS1, SMP_CB1_ENABLE);

            smSetParameter(arm_bus_handle, wrist_pitch_address, SMP_FAULTS, 0);
            smSetParameter(arm_bus_handle, wrist_pitch_address, SMP_CONTROL_BITS1, SMP_CB1_ENABLE);

            smSetParameter(arm_bus_handle, wrist_roll_address, SMP_FAULTS, 0);
            smSetParameter(arm_bus_handle, wrist_roll_address, SMP_CONTROL_BITS1, SMP_CB1_ENABLE);

            should_clear_faults = false;
        }

    }

    void set_calibration() {
        if (should_calibrate) {

            // Base
            smint32 base_current_offset;
            smint32 base_position_to_offset;
            smRead2Parameters(arm_bus_handle, base_address, SMP_SERIAL_ENC_OFFSET, &base_current_offset,
                              SMP_ACTUAL_POSITION_FB, &base_position_to_offset);

            smSetParameter(arm_bus_handle, base_address, SMP_SERIAL_ENC_OFFSET, base_current_offset - base_position_to_offset);

            // Shoulder
            smint32 shoulder_current_offset;
            smint32 shoulder_position_to_offset;
            smRead2Parameters(arm_bus_handle, shoulder_address, SMP_SERIAL_ENC_OFFSET, &shoulder_current_offset,
                              SMP_ACTUAL_POSITION_FB, &shoulder_position_to_offset);

            smSetParameter(arm_bus_handle, shoulder_address, SMP_SERIAL_ENC_OFFSET, shoulder_current_offset - shoulder_position_to_offset);


            // Elbow

            // Roll

            // Wrist Pitch

            // Wrist Roll

            // Save config through reboots
            smSetParameter(arm_bus_handle, shoulder_address, SMP_SYSTEM_CONTROL, SMP_SYSTEM_CONTROL_SAVECFG);

            should_reset = true;
            should_clear_faults = true;
            should_calibrate = false;
        }
    }

    void get_joint_statuses() {

        base_comm_state = smRead3Parameters(arm_bus_handle, base_address, SMP_STATUS, &base_status, SMP_FAULTS,
                                            &base_faults, SMP_ACTUAL_POSITION_FB, &base_current_position);


        shoulder_comm_state = smRead3Parameters(arm_bus_handle, shoulder_address, SMP_STATUS, &shoulder_status,
                                                SMP_FAULTS,
                                                &shoulder_faults, SMP_ACTUAL_POSITION_FB, &shoulder_current_position);


        elbow_comm_state = smRead3Parameters(arm_bus_handle, elbow_address, SMP_STATUS, &elbow_status, SMP_FAULTS,
                                             &elbow_faults, SMP_ACTUAL_POSITION_FB, &elbow_current_position);


        roll_comm_state = smRead3Parameters(arm_bus_handle, roll_address, SMP_STATUS, &roll_status, SMP_FAULTS,
                                            &roll_faults, SMP_ACTUAL_POSITION_FB, &roll_current_position);


        wrist_pitch_comm_state = smRead3Parameters(arm_bus_handle, wrist_pitch_address, SMP_STATUS, &wrist_pitch_status,
                                                   SMP_FAULTS,
                                                   &wrist_pitch_faults, SMP_ACTUAL_POSITION_FB,
                                                   &wrist_pitch_current_position);


        wrist_roll_comm_state = smRead3Parameters(arm_bus_handle, wrist_roll_address, SMP_STATUS, &wrist_roll_status,
                                                  SMP_FAULTS,
                                                  &wrist_roll_faults, SMP_ACTUAL_POSITION_FB,
                                                  &wrist_roll_current_position);

        if (!received_first_joint_position_update) {
            base_set_position = base_current_position;
            shoulder_set_position = shoulder_current_position;
            elbow_set_position = elbow_current_position;
            roll_set_position = roll_current_position;
            wrist_pitch_last_set_position = wrist_pitch_current_position;
            wrist_pitch_set_position = wrist_pitch_current_position;
            wrist_roll_set_position = wrist_roll_current_position;

            received_first_joint_position_update = true;
        }

        status_message.base = base_current_position / double(base_counts_per_rev);
        status_message.shoulder = shoulder_current_position / double(shoulder_counts_per_rev);
        status_message.elbow = elbow_current_position / double(elbow_counts_per_rev);
        status_message.roll = roll_set_position / double(roll_counts_per_rev);
        status_message.wrist_pitch = wrist_pitch_set_position / double(wrist_pitch_counts_per_rev);
        status_message.wrist_roll = wrist_roll_set_position / double(wrist_roll_counts_per_rev);

        status_message.base_comm_status = base_comm_state;
        status_message.shoulder_comm_status = shoulder_comm_state;
        status_message.elbow_comm_status = elbow_comm_state;
        status_message.roll_comm_status = roll_comm_state;
        status_message.wrist_pitch_comm_status = wrist_pitch_comm_state;
        status_message.wrist_roll_comm_status = wrist_roll_comm_state;

        status_message.base_status = base_status;
        status_message.shoulder_status = shoulder_status;
        status_message.elbow_status = elbow_status;
        status_message.roll_status = roll_status;
        status_message.wrist_pitch_status = wrist_pitch_status;
        status_message.wrist_roll_status = wrist_roll_status;

        status_message.base_faults = base_faults;
        status_message.shoulder_faults = shoulder_faults;
        status_message.elbow_faults = elbow_faults;
        status_message.roll_faults = roll_faults;
        status_message.wrist_pitch_faults = wrist_pitch_faults;
        status_message.wrist_roll_faults = wrist_roll_faults;

        status_publisher.publish(status_message);

    }

    void set_joint_positions() {
        if (new_positions_received) {
            set_base_position();
            set_shoulder_position();
            set_elbow_position();
            set_roll_position();
            set_wrist_positions();

            new_positions_received = false;
        }
    }

    void set_base_position() {
        smSetParameter(arm_bus_handle, base_address, SMP_ABSOLUTE_SETPOINT, base_set_position);
    }

    void set_shoulder_position() {
        smSetParameter(arm_bus_handle, shoulder_address, SMP_ABSOLUTE_SETPOINT, shoulder_set_position);

    }

    void set_elbow_position() {
        smSetParameter(arm_bus_handle, elbow_address, SMP_ABSOLUTE_SETPOINT, elbow_set_position);

    }

    void set_roll_position() {
        smSetParameter(arm_bus_handle, roll_address, SMP_ABSOLUTE_SETPOINT, roll_set_position);
    }

    void set_wrist_positions() {
        smSetParameter(arm_bus_handle, wrist_roll_address, SMP_ABSOLUTE_SETPOINT, wrist_roll_set_position);
        smSetParameter(arm_bus_handle, wrist_pitch_address, SMP_ABSOLUTE_SETPOINT, wrist_pitch_set_position);

    }

    void absolute_position_callback(const rover_arm::ArmControlMessage::ConstPtr &msg) {
        if (!received_first_joint_position_update) { return; }


        base_set_position = msg->base * base_counts_per_rev;
        shoulder_set_position = msg->shoulder * shoulder_counts_per_rev;
        elbow_set_position = msg->elbow * elbow_counts_per_rev;
        roll_set_position = msg->roll * roll_counts_per_rev;

        wrist_roll_set_position = msg->wrist_roll * wrist_roll_counts_per_rev;

        wrist_pitch_set_position = msg->wrist_pitch * wrist_pitch_counts_per_rev;

        smint32 roll_pitch_position_difference = (wrist_pitch_set_position - wrist_pitch_last_set_position) / 2;
//        wrist_roll_set_position -= roll_pitch_position_difference;
        wrist_pitch_last_set_position = wrist_pitch_set_position;


        should_clear_faults = (msg->clear_faults);
        should_reset = (msg->reset_controllers);
        should_calibrate = msg->calibrate;

        if(!should_calibrate && !should_reset){
            new_positions_received = true;
        }
    }

    void relative_position_callback(const rover_arm::ArmControlMessage::ConstPtr &msg) {
        if (!received_first_joint_position_update) { return; }

        ROS_INFO("Shoulder before: %ld", shoulder_set_position);

        base_set_position += msg->base * base_counts_per_rev;
        shoulder_set_position += msg->shoulder * shoulder_counts_per_rev;
        elbow_set_position += msg->elbow * elbow_counts_per_rev;
        roll_set_position += msg->roll * roll_counts_per_rev;

        wrist_roll_set_position += msg->wrist_roll * wrist_roll_counts_per_rev;

        wrist_pitch_set_position += msg->wrist_pitch * wrist_pitch_counts_per_rev;

        smint32 roll_pitch_position_difference = (wrist_pitch_set_position - wrist_pitch_last_set_position) / 2;
//        wrist_roll_set_position -= roll_pitch_position_difference;
        wrist_pitch_last_set_position = wrist_pitch_set_position;


        should_clear_faults = (msg->clear_faults);
        should_reset = (msg->reset_controllers);
        should_calibrate = msg->calibrate;

        if(!should_calibrate && !should_reset) {
            new_positions_received = true;
        }

        ROS_INFO("Shoulder after: %ld", shoulder_set_position);
    }

private:
    ros::NodeHandle *node_handle;

    string arm_port;
    smbus arm_bus_handle;
    bool arm_successfully_connected = false;

    ros::Publisher status_publisher;
    rover_arm::ArmStatusMessage status_message;

    ros::Subscriber absolute_position_control_subscriber;
    ros::Subscriber relative_position_control_subscriber;

    bool received_first_joint_position_update = false;

    bool new_positions_received = false;

    smint32 base_set_position = 0;
    smint32 base_current_position = 0;
    smint32 base_comm_state = 0;
    smint32 base_status = 0;
    smint32 base_faults = 0;

    smint32 shoulder_set_position = 0;
    smint32 shoulder_current_position = 0;
    smint32 shoulder_comm_state = 0;
    smint32 shoulder_status = 0;
    smint32 shoulder_faults = 0;

    smint32 elbow_set_position = 0;
    smint32 elbow_current_position = 0;
    smint32 elbow_comm_state = 0;
    smint32 elbow_status = 0;
    smint32 elbow_faults = 0;

    smint32 roll_set_position = 0;
    smint32 roll_current_position = 0;
    smint32 roll_comm_state = 0;
    smint32 roll_status = 0;
    smint32 roll_faults = 0;

    smint32 wrist_pitch_last_set_position = 0;
    smint32 wrist_pitch_set_position = 0;
    smint32 wrist_pitch_current_position = 0;
    smint32 wrist_pitch_comm_state = 0;
    smint32 wrist_pitch_status = 0;
    smint32 wrist_pitch_faults = 0;

    smint32 wrist_roll_set_position = 0;
    smint32 wrist_roll_current_position = 0;
    smint32 wrist_roll_comm_state = 0;
    smint32 wrist_roll_status = 0;
    smint32 wrist_roll_faults = 0;

    bool should_calibrate = false;
    bool should_clear_faults = true;
    bool should_reset = false;


};


int main(int argc, char **argv) {
    RoverArm rover_arm(argc, argv);
    rover_arm.run();
}