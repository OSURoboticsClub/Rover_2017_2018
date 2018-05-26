#include <NMEAGPS.h>
#include <ublox/ubxGPS.h>
#include <NeoGPS_cfg.h>
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

Adafruit_BNO055 bno = Adafruit_BNO055(WIRE_BUS, -1, BNO055_ADDRESS_A, I2C_MASTER, I2C_PINS_29_30, I2C_PULLUP_INT, I2C_RATE_100, I2C_OP_MODE_ISR);


#define gpsPort Serial2
#define commsPort Serial3
#define GPS_PORT_NAME "Serial2"

const unsigned char ubxRate5Hz[] = { 0x06, 0x08, 0x06, 0x00, 200, 0x00, 0x01, 0x00, 0x01, 0x00 };
const unsigned char ubxRate10Hz[] = { 0x06, 0x08, 0x06, 0x00, 100, 0x00, 0x01, 0x00, 0x01, 0x00 };

const char baud38400 [] = "PUBX,41,1,3,3,38400,0";
const char baud57600 [] = "PUBX,41,1,3,3,57600,0";
const char baud115200[] = "PUBX,41,1,3,3,115200,0";

imu::Vector<3> linear_accel;
imu::Vector<3>  angular_vel;
imu::Quaternion quat;

char current_byte = '$';
String nmea_sentence = "";
String output_sentence;

char float_decimal_places = 8;

void sendUBX( const unsigned char *progmemBytes, size_t len )
{
  gpsPort.write( 0xB5 ); // SYNC1
  gpsPort.write( 0x62 ); // SYNC2

  uint8_t a = 0, b = 0;
  while (len-- > 0) {
    uint8_t c = pgm_read_byte( progmemBytes++ );
    a += c;
    b += a;
    gpsPort.write( c );
  }

  gpsPort.write( a ); // CHECKSUM A
  gpsPort.write( b ); // CHECKSUM B

}

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Booting up");
  // put your setup code here, to run once:
  commsPort.begin(115200);
  commsPort.transmitterEnable(3);

  delay(250);
  gpsPort.begin(9600);
  //  sendUBX( ubxRate5Hz, sizeof(ubxRate5Hz) );

  Serial.println("Setting up IMU");
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  Serial.println("IMU Online. Setting to external crystal.");
  bno.setExtCrystalUse(true);
  Serial.println("IMU Configured.");


}

void loop() {

  if (gpsPort.available() > 0 ) {
    nmea_sentence = "";
    do {
      if (gpsPort.available() > 0) {
        current_byte = gpsPort.read();

        if (current_byte != '\r' and current_byte != '\n') {
          nmea_sentence += current_byte;
        }
      }
    } while (current_byte != '\r');
    commsPort.println(nmea_sentence);
  }

  quat = bno.getQuat();
  linear_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  angular_vel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  commsPort.print("ox:");
  commsPort.print(quat.x(), float_decimal_places);
  commsPort.print(",");
  commsPort.print("oy:");
  commsPort.print(quat.y(), float_decimal_places);
  commsPort.print(",");
  commsPort.print("oz:");
  commsPort.print(quat.z(), float_decimal_places);
  commsPort.print(",");
  commsPort.print("ow:");
  commsPort.print(quat.w(), float_decimal_places);

  commsPort.print(",");
  commsPort.print("lax:");
  commsPort.print(linear_accel.x(), float_decimal_places);
  commsPort.print(",");
  commsPort.print("lay:");
  commsPort.print(linear_accel.y(), float_decimal_places);
  commsPort.print(",");
  commsPort.print("laz:");
  commsPort.print(linear_accel.z(), float_decimal_places);


  commsPort.print(",");
  commsPort.print("avx:");
  commsPort.print(angular_vel.x(), float_decimal_places);
  commsPort.print(",");
  commsPort.print("avy:");
  commsPort.print(angular_vel.y(), float_decimal_places);
  commsPort.print(",");
  commsPort.print("avz:");
  commsPort.print(angular_vel.z(), float_decimal_places);

  commsPort.println();

  //
  //  /* Display calibration status for each sensor. */
  //  uint8_t system, gyro, accel, mag = 0;
  //  bno.getCalibration(&system, &gyro, &accel, &mag);
  //  commsPort.print("CALIBRATION: Sys=");
  //  commsPort.print(system, DEC);
  //  commsPort.print(" Gyro=");
  //  commsPort.print(gyro, DEC);
  //  commsPort.print(" Accel=");
  //  commsPort.print(accel, DEC);
  //  commsPort.print(" Mag=");
  //  commsPort.print(mag, DEC);

}
