////////// Includes //////////
#include <ModbusRtu.h>
#include <Adafruit_BNO055_t3.h>

/*
   Imu/data (Imu)
   Imu/raw (Imu)
   Imu/mag (MagneticField)
   Imu/temp (Temperature)

   ---Raw Data (Imu/raw) ---
   LinearAccel (x, y, z)
   AngularVel (x, y, z);

   --- Filtered Data (Imu/data) ---
   Orientation(z, y, z, w)
   LinearAccel(x, y, z)
   AngularVel(x, y, z)

   --- Mag Data (Imu/mag) ---
   MagField(x, y, z)

   --- IMU Temp (Imu/temp)
   temp (deg c)
*/

////////// Hardware / Data Enumerations //////////
enum HARDWARE {
  GPS_IMU_RS485_EN = 3,
  GPS_IMU_RS485_RX = 9,
  GPS_IMU_RS485_TX = 10,

  COMMS_RS485_EN = 2,
  COMMS_RS485_RX = 0,
  COMMS_RS485_TX = 1,

  GPS_UART_RX = 7,
  GPS_UART_TX = 8,

  IMU_SDA = 18,
  IMU_SCL = 19,

  WS2812_DATA = 11,
  C02_SENSOR = A7,
  MISC_PIN = A8,

  LED_BLUE_EXTRA = 13
};

enum MODBUS_REGISTERS {
  DIRECTION = 0,  // Input
  SPEED = 1,      // Input
  SLEEP = 2,      // Input

  CURRENT = 3,    // Output
  FAULT = 4,      // Output

  TEMPERATURE = 5        // Output
};

#define GPS_SERIAL_PORT Serial3
#define GPS_IMU_STREAMING_PORT Serial2

////////// Global Variables //////////
///// Modbus
const uint8_t node_id = 1;
const uint8_t mobus_serial_port_number = 1;

uint16_t modbus_data[] = {0, 0};
uint8_t num_modbus_registers = 0;

int8_t poll_state = 0;
bool communication_good = false;
uint8_t message_count = 0;

///// IMU
imu::Vector<3> linear_accel;
imu::Vector<3>  angular_vel;
imu::Quaternion quat;

char float_decimal_places = 8;

///// GPS
char current_byte = '$';
String nmea_sentence = "";

////////// Class Instantiations //////////
Modbus slave(node_id, mobus_serial_port_number, HARDWARE::COMMS_RS485_EN);

Adafruit_BNO055 bno = Adafruit_BNO055(WIRE_BUS, -1, BNO055_ADDRESS_A, I2C_MASTER, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_100, I2C_OP_MODE_ISR);

void setup() {
  // Debugging
  Serial.begin(9600);

  // Raw pin setup
  setup_hardware();

  // Setup modbus serial communication
  num_modbus_registers = sizeof(modbus_data) / sizeof(modbus_data[0]);
  slave.begin(115200); // baud-rate at 19200
  slave.setTimeOut(150);

  // GPS & IMU serial streaming setup
  GPS_IMU_STREAMING_PORT.begin(115200);
  GPS_IMU_STREAMING_PORT.transmitterEnable(HARDWARE::GPS_IMU_RS485_EN);

  // IMU Setup
  Serial.println("Setting up IMU");
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  Serial.println("IMU Online. Setting to external crystal.");
  bno.setExtCrystalUse(true);
  Serial.println("IMU Configured.");

  // GPS Setup
  GPS_SERIAL_PORT.begin(9600);

}

void loop() {
  poll_modbus();
  set_leds();
  send_imu_stream_line();
  process_gps_and_send_if_ready();

}

void setup_hardware() {
  // Setup pins as inputs / outputs
  pinMode(HARDWARE::WS2812_DATA, OUTPUT);
  pinMode(HARDWARE::C02_SENSOR, INPUT);
  pinMode(HARDWARE::MISC_PIN, OUTPUT);

  pinMode(HARDWARE::LED_BLUE_EXTRA, OUTPUT);

  // Set default pin states
  digitalWrite(HARDWARE::LED_BLUE_EXTRA, LOW);

  // Set teensy to increased analog resolution
  analogReadResolution(13);

}

void send_imu_stream_line() {
  quat = bno.getQuat();
  linear_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  angular_vel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  GPS_IMU_STREAMING_PORT.print("ox:");
  GPS_IMU_STREAMING_PORT.print(quat.x(), float_decimal_places);
  GPS_IMU_STREAMING_PORT.print(",");
  GPS_IMU_STREAMING_PORT.print("oy:");
  GPS_IMU_STREAMING_PORT.print(quat.y(), float_decimal_places);
  GPS_IMU_STREAMING_PORT.print(",");
  GPS_IMU_STREAMING_PORT.print("oz:");
  GPS_IMU_STREAMING_PORT.print(quat.z(), float_decimal_places);
  GPS_IMU_STREAMING_PORT.print(",");
  GPS_IMU_STREAMING_PORT.print("ow:");
  GPS_IMU_STREAMING_PORT.print(quat.w(), float_decimal_places);

  GPS_IMU_STREAMING_PORT.print(",");
  GPS_IMU_STREAMING_PORT.print("lax:");
  GPS_IMU_STREAMING_PORT.print(linear_accel.x(), float_decimal_places);
  GPS_IMU_STREAMING_PORT.print(",");
  GPS_IMU_STREAMING_PORT.print("lay:");
  GPS_IMU_STREAMING_PORT.print(linear_accel.y(), float_decimal_places);
  GPS_IMU_STREAMING_PORT.print(",");
  GPS_IMU_STREAMING_PORT.print("laz:");
  GPS_IMU_STREAMING_PORT.print(linear_accel.z(), float_decimal_places);


  GPS_IMU_STREAMING_PORT.print(",");
  GPS_IMU_STREAMING_PORT.print("avx:");
  GPS_IMU_STREAMING_PORT.print(angular_vel.x(), float_decimal_places);
  GPS_IMU_STREAMING_PORT.print(",");
  GPS_IMU_STREAMING_PORT.print("avy:");
  GPS_IMU_STREAMING_PORT.print(angular_vel.y(), float_decimal_places);
  GPS_IMU_STREAMING_PORT.print(",");
  GPS_IMU_STREAMING_PORT.print("avz:");
  GPS_IMU_STREAMING_PORT.print(angular_vel.z(), float_decimal_places);

  GPS_IMU_STREAMING_PORT.println();

  //
  //  /* Display calibration status for each sensor. */
  //  uint8_t system, gyro, accel, mag = 0;
  //  bno.getCalibration(&system, &gyro, &accel, &mag);
  //  Serial.print("CALIBRATION: Sys=");
  //  Serial.print(system, DEC);
  //  Serial.print(" Gyro=");
  //  Serial.print(gyro, DEC);
  //  Serial.print(" Accel=");
  //  Serial.print(accel, DEC);
  //  Serial.print(" Mag=");
  //  Serial.print(mag, DEC);
}

void process_gps_and_send_if_ready() {
  if (GPS_SERIAL_PORT.available() > 0 ) {
    current_byte = GPS_SERIAL_PORT.read();

    if (current_byte != '\r' && current_byte != '\n') {
      nmea_sentence += current_byte;
    }

    if (current_byte == '\r') {
      GPS_IMU_STREAMING_PORT.println(nmea_sentence);
      nmea_sentence = "";
    }
  }
}

void poll_modbus() {
  poll_state = slave.poll(modbus_data, num_modbus_registers);
  communication_good = !slave.getTimeOutState();
}

void set_leds() {
  if (poll_state > 4) {
    message_count++;
    if (message_count > 2) {
      digitalWrite(HARDWARE::LED_BLUE_EXTRA, !digitalRead(HARDWARE::LED_BLUE_EXTRA));
      message_count = 0;
    }
  } else if (!communication_good) {
    digitalWrite(HARDWARE::LED_BLUE_EXTRA, LOW);
  }
}
