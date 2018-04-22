#include <string>
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>

#include "simplemotion/simplemotion.h"


using namespace std;

int device_address = 1;

int main(int argc, char** argv){
        smbus busHandle = smOpenBus("/dev/ttyUSB0");

    if (busHandle >= 0) {
        cout << "Successfully connected bus" << endl;
//        deviceAddress=ui->deviceAddress->value();
    } else
        cout << "Couldn't connect to bus";

    while(1){
        smSetParameter(busHandle, device_address, SMP_FAULTS, 0);
        smSetParameter(busHandle, device_address, SMP_CONTROL_BITS1, SMP_CB1_ENABLE);
        smSetParameter(busHandle, device_address, SMP_ABSOLUTE_SETPOINT, 1000);
        cout << "Test" << endl;
    }

}