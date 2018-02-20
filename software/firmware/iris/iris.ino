#include "SBUS.h"
#include <ModbusRtu.h>

SBUS x8r(Serial3);

const char num_modbus_channels = 16; //Being explicit so I get errors if I try to do something stupid
uint16_t modbus_data[num_modbus_channels] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0 , 0 , 0, 0, 0, 0, 0};

Modbus slave(1, 2, 0);

uint16_t channels[16];
uint8_t failSafe;
uint16_t lostFrames = 0;

const char bad_pins[] = {3, 4, 23, 24,};

void setup() {
  for (int i = 0 ; i < sizeof(bad_pins) / sizeof(bad_pins[0]) ; i++) {
    pinMode(bad_pins[i], INPUT);
  }

  pinMode(8, INPUT_PULLUP);

  //  analogWriteResolution(16);
  x8r.begin();
  slave.begin(2000000); // baud-rate at 19200
}

void loop() {
  x8r.read(&channels[0], &failSafe, &lostFrames);
  slave.poll( channels, num_modbus_channels );

}
