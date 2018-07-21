////////// Includes //////////
#include <PID_v1.h>
#include <Encoder.h>

////////// Hardware / Data Enumerations //////////
enum HARDWARE {
    // RS485_EN = 2,
    // RS485_RX = 7,
    // RS485_TX = 8,

    MOTOR_PWM_1 = 25,
    MOTOR_PWM_2 = 22,
    MOTOR_CURRENT_SENSE = A2,

    ENCODER_A = 2,
    ENCODER_B = 30,

    LED_RED = 6,
    LED_GREEN = 32,
    LED_BLUE = 13,
};

////////// Global Variables //////////
// define user words for motor states
#define BRAKE     0
#define FWD       1
#define REV       2
#define COAST     3

long oldPosition  = -999;
float velocity = 0;
long count = 0;
long oldposition = 0;
long newposition;
int interval = 10;
long prev_time = 0;
float conversion_factor = 1500/31.25; //60.0*1000.0/cpr;

////////// Class Instantiations //////////
// Encoder
Encoder enc(HARDWARE::ENCODER_A, HARDWARE::ENCODER_B);

// Velocity
double setpoint_vel, input_vel, output_vel; //PID velocity variables
PID motor_vel(&input_vel, &output_vel, &setpoint_vel, 10, 1, 0, DIRECT); //velocity PID
double signed_setpoint_vel = -200;

// Position
double setpoint_pos = 0; //PID position variables
double input_pos, output_pos;
PID motor_pos(&input_pos, &output_pos, &setpoint_pos, 10, 0, 0, DIRECT); //position PID
long current_pos = 1000;

// Homing
int trip_threshold = 650;
bool home_flag = true;
long prev_millis = 0;

////////// Setup //////////
void setup() {
  setup_hardware();
  Serial.begin(9600);
  motor_vel.SetMode(AUTOMATIC);
  motor_vel.SetSampleTime(1);
  motor_pos.SetMode(AUTOMATIC);
  motor_pos.SetSampleTime(1);

  Serial.println("init");
}

////////// Loop //////////
void loop() {

  // Position based homing
  // if (home_flag == true){
  //   Serial.println("homing");
  //   Serial.println(analogRead(HARDWARE::MOTOR_CURRENT_SENSE));
  //
  //   unsigned long curr_millis = millis();
  //   // if it is time to move the motor and we haven't tripped the current yet:
  //   if (curr_millis - prev_millis > interval){
  //     prev_millis = curr_millis;
  //     current_pos += 5; // increment by counts
  //
  //     if (analogRead(HARDWARE::MOTOR_CURRENT_SENSE) < trip_threshold){
  //       current_pos = 0;
  //       enc.write(0);
  //       home_flag = false;
  //     }
  //   }
  // }

  // Position movement test
  // unsigned long curr_millis = millis();
  // // if it is time to move the motor and we haven't tripped the current yet:
  // if (curr_millis - prev_millis > interval){
  //   prev_millis = curr_millis;
  //   current_pos += 1; // increment by counts
  // }
  //
  // set_position(10000);
  //
  // Serial.print("Set Pos:  ");
  // Serial.println(current_pos);
  // Serial.print("Curr Pos: ");
  // Serial.println(enc.read());
  // delay(1);

  // Velocity based non blocking homing
  if (home_flag == true){

    set_velocity(-150);

    Serial.println("Homing");

    Serial.print("Current: ");
    Serial.println(analogRead(HARDWARE::MOTOR_CURRENT_SENSE));

    if ((analogRead(HARDWARE::MOTOR_CURRENT_SENSE) > trip_threshold) && (millis() > 500)){
      Serial.println("Tripped current sensor!");
      set_motor_output(BRAKE, 0); //stop moving
      home_flag = false; // no more homing
      //set_motor_output(COAST, 0); //stop moving
      enc.write(0); // reset encoder
    }
  }
  else {
    set_position(current_pos); // hold
    Serial.print("Set Pos:  ");
    Serial.println(current_pos);
    Serial.print("Curr Pos: ");
    Serial.println(enc.read());
  }

  delay(1);

}

void setup_hardware(){
  // setup IO
  pinMode(HARDWARE::MOTOR_PWM_1, OUTPUT);
  pinMode(HARDWARE::MOTOR_PWM_2, OUTPUT);
  pinMode(HARDWARE::MOTOR_CURRENT_SENSE, INPUT);

  pinMode(HARDWARE::LED_RED, OUTPUT);
  pinMode(HARDWARE::LED_GREEN, OUTPUT);
  pinMode(HARDWARE::LED_BLUE, OUTPUT);

  // setup default states
  set_motor_output(COAST, 0);

  digitalWrite(HARDWARE::LED_RED, HIGH);
  digitalWrite(HARDWARE::LED_GREEN, HIGH);
  digitalWrite(HARDWARE::LED_BLUE, HIGH);

  analogReadAveraging(32);
}

void home(int trip_threshold) {

  // change to position based homing
  while (analogRead(HARDWARE::MOTOR_CURRENT_SENSE) > trip_threshold) {
    current_pos = enc.read();
    set_position(current_pos + 10);
    Serial.print("Current: ");
    Serial.println(analogRead(HARDWARE::MOTOR_CURRENT_SENSE));
  }

  set_motor_output(COAST, 0); //stop moving
  enc.write(0);
  Serial.println("Home complete");
}

void set_position(double setpoint_input){
  input_pos = enc.read(); //get actual current position
  setpoint_pos = (double)setpoint_input;
  //setpoint_pos = setpoint_input;

  //move to new position
  if (input_pos > setpoint_pos) { //positive
    motor_pos.SetControllerDirection(REVERSE);
    motor_pos.Compute();
    set_motor_output(REV, output_pos);
  }
  else { //negative
    motor_pos.SetControllerDirection(DIRECT);
    motor_pos.Compute();
    set_motor_output(FWD, output_pos);
  }
}

void set_velocity(double signed_setpoint_vel){
  // PID Velocity Control
  if (signed_setpoint_vel > 0){ //positive
    input_vel = calc_velocity();
    setpoint_vel = signed_setpoint_vel;
    motor_vel.Compute();
    set_motor_output(FWD, output_vel);
  }
  else{ //negative
    input_vel = abs(calc_velocity());
    setpoint_vel = abs(signed_setpoint_vel);
    motor_vel.Compute();
    set_motor_output(REV, output_vel);
  }
  // Serial.print("Measured Velocity: ");
  // Serial.println(input_vel);
}

float calc_velocity(){
  unsigned long current_time = millis();
  int delta_t = current_time - prev_time;

  if (delta_t > interval) {
    prev_time = current_time; //reset time
    newposition = enc.read(); //find the new position
    count = newposition - oldposition; //find the count since the last interval
    velocity = (float(count) / float(delta_t))*conversion_factor; //calculate velocity
    oldposition = newposition; //set the old position
  }
  return velocity;
}

void set_motor_output(uint8_t direct, uint8_t pwm){
  /*
  Control Logic
                A  |  B
  0) BRAKE:     1     1
  1) FWD:      1/0    0
  2) REV:       0    1/0
  3) COAST:     0     0
  */

  // make sure the PWM doesn't go too high
  pwm = map(pwm, 0, 255, 0, 110);

  if (direct <= 4){
    switch (direct){
      case 0:
      digitalWrite(HARDWARE::MOTOR_PWM_1, HIGH);
      digitalWrite(HARDWARE::MOTOR_PWM_2, HIGH);
      break;
      case 1:
      analogWrite(HARDWARE::MOTOR_PWM_1, pwm);
      digitalWrite(HARDWARE::MOTOR_PWM_2, LOW);
      break;
      case 2:
      digitalWrite(HARDWARE::MOTOR_PWM_1, LOW);
      analogWrite(HARDWARE::MOTOR_PWM_2, pwm);
      break;
      case 3:
      digitalWrite(HARDWARE::MOTOR_PWM_1, LOW);
      digitalWrite(HARDWARE::MOTOR_PWM_2, LOW);
      break;
      default:
      digitalWrite(HARDWARE::MOTOR_PWM_1, LOW);
      digitalWrite(HARDWARE::MOTOR_PWM_2, LOW);
    }
  }
}
