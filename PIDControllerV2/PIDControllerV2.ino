#include "DualVNH5019MotorShield.h"
#include <SharpIR.h>
#include <PinChangeInt.h>
#include <RunningMedian.h>

/////////////////////////////////////////////////////////////////////////
///////////////////////////////HARDWARE SETUP////////////////////////////
/////////////////////////////////////////////////////////////////////////

// SETUP MOTOR PINS 
// Motor encoders are taking the DIGITAL pins
#define LEFT_MOTOR_PIN 3
#define RIGHT_MOTOR_PIN 5

// DECLARE VARIABLES
DualVNH5019MotorShield md;
volatile float encoder_left = 0;
volatile float encoder_right = 0;
double error = 0.0, integralError = 0.0, target_ticks = 0.0;
float left_straight_speed, right_straight_speed;
float left_rotate_speed, right_rotate_speed, left_rotate_slow_speed, right_rotate_slow_speed;
float left_brake_speed, right_brake_speed, left_ramp_brake_speed, right_ramp_brake_speed, left_rotate_brake_speed, right_rotate_brake_speed;
int angle;
int left_distance = 0, center_left_distance = 0, center_distance = 0, center_right_distance = 0, right_distance = 0, right_long_distance = 0;
char command[64];

// SETUP SENSORS PINS 
// sensors are taking the ANALOG pins
#define SENSOR_LEFT_PIN 0
#define SENSOR_RIGHT_PIN 1
#define SENSOR_RIGHT_LONG_PIN 2
#define SENSOR_CB_PIN 3
#define SENSOR_CL_PIN 4
#define SENSOR_CR_PIN 5

#define FRONT_SHORT_OFFSET 7
#define SIDE_SHORT_OFFSET 12
#define SIDE_LONG_OFFSET 11
#define WALL_DISTANCE 12

// 1080 => short range senor GP2Y0A21YK
// 20150 => long range sensor GP2Y0A02YK
SharpIR SENSOR_LEFT (SENSOR_LEFT_PIN, 1080); // left, short range sensor
SharpIR SENSOR_RIGHT (SENSOR_RIGHT_PIN, 1080); // right, short range sensor
SharpIR SENSOR_RIGHT_LONG (SENSOR_RIGHT_LONG_PIN, 20150); // right, long range sensor
SharpIR SENSOR_C_BOT (SENSOR_CB_PIN, 1080); // center bottom, short range sensor
SharpIR SENSOR_C_LEFT (SENSOR_CL_PIN, 1080);  // center left, short range sensor
SharpIR SENSOR_C_RIGHT (SENSOR_CR_PIN, 1080);  // center right, short range sensor

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////SETUP///////////////////////////////////
/////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  Serial.flush();
  md.init();
  pinMode(LEFT_MOTOR_PIN, INPUT);
  pinMode(RIGHT_MOTOR_PIN, INPUT);
  // attach interrputs to the encoders output pins with PinChangeInt
  PCintPort::attachInterrupt(LEFT_MOTOR_PIN, left_encoder_rising, HIGH);
  PCintPort::attachInterrupt(RIGHT_MOTOR_PIN, right_encoder_rising, HIGH);
  left_straight_speed = 400; //250
  right_straight_speed = 400; //250
  left_rotate_speed = 350; //150
  right_rotate_speed = 350; //150
  left_rotate_slow_speed = 275; // for fastest path exploration
  right_rotate_slow_speed = 275; // for fastest path exploration
  left_brake_speed = 385;
  right_brake_speed = 400;
  left_ramp_brake_speed = 400;
  right_ramp_brake_speed = 400;
  left_rotate_brake_speed = 400;
  right_rotate_brake_speed = 400;
}

void loop() {
//rotate_right_ramp(90);
//delay(100);
// rotate_left_ramp(90);
// delay(100);
// while(1){
//   simulate();
//   move_forward_ramp(10);

// }

  // while(1){
  //   simulate();
  //   move_forward_ramp(10);
  // }

  

  char* rpiMsg = get_rpi_message();
  if(strlen(rpiMsg)<=0) {
    return;
  }
  else if (strlen(rpiMsg) == 1) {
    move_robot(rpiMsg[0]);
    read_sensor_readings();
    //set_rpi_message(left_distance, center_left_distance, center_distance, center_right_distance, right_distance);
    memset(rpiMsg, 0, sizeof(rpiMsg));
  }
  else {
    int n = strlen(rpiMsg);
    for (int i = 0; i < n; i++) {
      Serial.println(rpiMsg[i]);
      move_robot(rpiMsg[i]);
      read_sensor_readings();
      //set_rpi_message(left_distance, center_left_distance, center_distance, center_right_distance, right_distance);
    }
    memset(rpiMsg, 0, sizeof(rpiMsg));
  }
  Serial.flush();
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////RASPBERRY PI COMMUNICATION//////////////////////
/////////////////////////////////////////////////////////////////////////

// Read message from Serial sent by RPi

char* get_rpi_message() {
  memset(command, 0, sizeof(command));
  Serial.readBytes(command, 64);
  return command;
}

// Print to Serial for RPi to read messages

void set_rpi_message(int left, int center_left, int center, int center_right, int right, int right_long){
  // For distances greater than 20 cm, use long 
  if ((right_long - SIDE_LONG_OFFSET) >= 20) 
    right = right_long - SIDE_LONG_OFFSET + SIDE_SHORT_OFFSET;
  Serial.print("AR2PC|");
  Serial.print(left-SIDE_SHORT_OFFSET);
  Serial.print(":");
  Serial.print(center_left-FRONT_SHORT_OFFSET);
  Serial.print(":");
  Serial.print(center-FRONT_SHORT_OFFSET);
  Serial.print(":");
  Serial.print(center_right-FRONT_SHORT_OFFSET);
  Serial.print(":");
  Serial.print(right-SIDE_SHORT_OFFSET);
  Serial.print("\0");
  Serial.print("\n");
  Serial.flush();
}

/////////////////////////////////////////////////////////////////////////
///////////////////////////ROBOT MOVEMENT CONTROL////////////////////////
/////////////////////////////////////////////////////////////////////////

// Cases for commands sent by Raspberry Pi to move

void move_robot(char command) {
  //Serial.print("XXXXXXXXXXXXRPI command received by Arduino: ");
  //Serial.println(command);
  switch(command) {
    case 'E': break;
    case 'F': move_forward_ramp(10); break;
    case 'L': rotate_left_ramp(90); break;
    case 'R': align_distance(); rotate_right_ramp(90); break;
    case 'B': move_backward_ramp(10); break;
    case 'H': stop_robot(); break;
    case 'X': fastest_path(); break;
  }
}

void fastest_path() {
  int multiplication;
  char* rpi_message = get_rpi_message();
  String rpiMsg(rpi_message);
  Serial.println(rpiMsg);
  int strlength = rpiMsg.length();
  for ( int i = 0; i < strlength-1; i++)  {
    Serial.print(i);
    Serial.print(":");
    Serial.println(rpiMsg[i]);
    if (rpiMsg[i]=='F') {
      while (rpiMsg[i] == rpiMsg[i+1]) {
        multiplication++;
        i++;
      }
      move_forward(10*multiplication);
      Serial.print("F multiplication: ");
      Serial.println(multiplication);
      multiplication=1;
    }
    else if (rpiMsg[i] == 'R') {
      while (rpiMsg [i] == rpiMsg [i+1]) {
        multiplication++;
        i++;
      }
      Serial.print("R multiplication: ");
      Serial.println(multiplication);
      rotate_right(90*multiplication);
      multiplication=1;
    }
    else if (rpiMsg[i] == 'L') {
      while (rpiMsg [i] == rpiMsg [i+1]) {
        multiplication++;
        i++;
      }
      Serial.print("L multiplication: ");
      Serial.println(multiplication);
      rotate_left(90*multiplication);
      multiplication=1;
    }
  }
  //End after path is finished
  while(1){
    
  }
}

// Straight line movement

void move_forward_ramp(int distance_cm) {
  encoder_left = 0;
  encoder_right = 0;
  double compensation = 0;
  error = 0.0;
  integralError = 0.0;
  if (distance_cm <= 10) target_ticks = distance_cm * 58.02;
  else if(distance_cm<=20) target_ticks = distance_cm * 58; // calibration done
  else if(distance_cm<=30) target_ticks = distance_cm * 58.5;
  else if(distance_cm<=40) target_ticks = distance_cm * 59.0;
  else if(distance_cm<=50) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=60) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=70) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=80) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=90) target_ticks = distance_cm * 59.6; //calibration done
  else if(distance_cm<=100) target_ticks = distance_cm * 60.2; //calibration redone on 26/9
  else if(distance_cm<=110) target_ticks = distance_cm * 59.8; 
  else if(distance_cm<=120) target_ticks = distance_cm * 60.3; //calibration redone on 26/9
  else if(distance_cm<=130) target_ticks = distance_cm * 60.3; 
  else if(distance_cm<=140) target_ticks = distance_cm * 60.5; 
  else if(distance_cm<=150) target_ticks = distance_cm * 60.5; 
  else if(distance_cm<=160) target_ticks = distance_cm * 60.5; //calibration redone on 26/9
  else if(distance_cm<=170) target_ticks = distance_cm * 60.5; 
  else target_ticks = distance_cm * 60.5;
  
  while (encoder_right < target_ticks * 0.1)
  {
    compensation = tune_pid();
    md.setSpeeds(100 + compensation, 100 - compensation);
  }
  
  while (encoder_right < target_ticks * 0.3)
  {
    compensation = tune_pid();
    md.setSpeeds(125 + compensation, 125 - compensation);
  }
  
  while (encoder_right < target_ticks * 0.8)
  {
    compensation = tune_pid();
    md.setSpeeds(200 + compensation, 200 - compensation);
  }
  
  while (encoder_right < target_ticks)
  {
    compensation = tune_pid();
    md.setSpeeds(125 + compensation, 125 - compensation);
  }
  md.setBrakes(left_ramp_brake_speed, right_ramp_brake_speed);
  delay(80);
  md.setSpeeds(0, 0);
  delay(50);
}

void move_backward_ramp(int distance_cm) {
  encoder_left = 0;
  encoder_right = 0;
  double compensation = 0;
  error = 0.0;
  integralError = 0.0;
  //target_ticks = distance_cm * 58.5;
  if (distance_cm<= 10) target_ticks = distance_cm * 58.3;
  else if(distance_cm<=20) target_ticks = distance_cm * 58.3;
  else if(distance_cm<=30) target_ticks = distance_cm * 58.5;
  else if(distance_cm<=40) target_ticks = distance_cm * 59.0;
  else if(distance_cm<=50) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=60) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=70) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=80) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=90) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=100) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=110) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=120) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=130) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=140) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=150) target_ticks = distance_cm * 72.5;
  else if(distance_cm<=160) target_ticks = distance_cm * 72.5;
  else if(distance_cm<=170) target_ticks = distance_cm * 72.5;
  else target_ticks = distance_cm * 72.5;
  
  while (encoder_right < target_ticks * 0.1)
  {
    compensation = tune_pid();
    md.setSpeeds(-(100 + compensation), -(100 - compensation));
  }
  
  while (encoder_right < target_ticks * 0.3)
  {
    compensation = tune_pid();
    md.setSpeeds(-(125 + compensation), -(125 - compensation));
  }
  
  while (encoder_right < target_ticks * 0.8)
  {
    compensation = tune_pid();
    md.setSpeeds(-(200 + compensation), -(200 - compensation));
  }
  
  while (encoder_right < target_ticks)
  {
    compensation = tune_pid();
    md.setSpeeds(-(125 + compensation), -(125 - compensation));
  }
  //Okay at HPL
  //md.setBrakes(375, 400);

  // at HWL2
  md.setBrakes(left_brake_speed, right_brake_speed);
  delay(80);
  md.setSpeeds(0, 0);
  delay(50);
}

void move_forward (int distance_cm) {
  encoder_left = 0;
  encoder_right = 0;
  double compensation = 0;
  error = 0.0;
  integralError = 0.0;
  if (distance_cm <= 10) target_ticks = distance_cm * 58.3;
  else if(distance_cm<=20) target_ticks = distance_cm * 58; // calibration done
  else if(distance_cm<=30) target_ticks = distance_cm * 58.5;
  else if(distance_cm<=40) target_ticks = distance_cm * 59.0;
  else if(distance_cm<=50) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=60) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=70) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=80) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=90) target_ticks = distance_cm * 59.6; //calibration done
  else if(distance_cm<=100) target_ticks = distance_cm * 60; //calibration redone on 26/9
  else if(distance_cm<=110) target_ticks = distance_cm * 59.8; 
  else if(distance_cm<=120) target_ticks = distance_cm * 60.3; //calibration redone on 26/9
  else if(distance_cm<=130) target_ticks = distance_cm * 60.3; 
  else if(distance_cm<=140) target_ticks = distance_cm * 60.5; 
  else if(distance_cm<=150) target_ticks = distance_cm * 60.5; 
  else if(distance_cm<=160) target_ticks = distance_cm * 60.5; //calibration redone on 26/9
  else if(distance_cm<=170) target_ticks = distance_cm * 60.5; 
  else target_ticks = distance_cm * 60.5;
  
  while (encoder_right < 100)
  {
    compensation = tune_pid();
    md.setSpeeds(100 + compensation, 100 - compensation);
  }
  
  while (encoder_right < 200)
  {
    compensation = tune_pid(); 
    md.setSpeeds(200 + compensation, 200 - compensation);
  }

  while (encoder_right < 250)
  {
    compensation = tune_pid();
    md.setSpeeds(300 + compensation, 300 - compensation);
  }

  while (encoder_right < target_ticks - 200)
  {
    compensation = tune_pid();
    md.setSpeeds(left_straight_speed + compensation, right_straight_speed - compensation);
  }
  
  while (encoder_right < target_ticks - 100)
  {
    compensation = tune_pid();
    md.setSpeeds(200 + compensation, 200 - compensation);
  }

  while (encoder_right < target_ticks)
  {
    compensation = tune_pid();
    md.setSpeeds(100 + compensation, 100 - compensation);
  }
  md.setBrakes(left_brake_speed, right_brake_speed);
  delay(80);
  md.setSpeeds(0, 0);
  delay(50);
}

void move_backward (int distance_cm) {
  encoder_left = 0;
  encoder_right = 0;
  double compensation = 0;
  error = 0.0;
  integralError = 0.0;
  //target_ticks = distance_cm * 58.5;
  if (distance_cm<= 10) target_ticks = distance_cm * 58.3;
  else if(distance_cm<=20) target_ticks = distance_cm * 58.3;
  else if(distance_cm<=30) target_ticks = distance_cm * 58.5;
  else if(distance_cm<=40) target_ticks = distance_cm * 59.0;
  else if(distance_cm<=50) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=60) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=70) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=80) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=90) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=100) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=110) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=120) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=130) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=140) target_ticks = distance_cm * 59.3;
  else if(distance_cm<=150) target_ticks = distance_cm * 72.5;
  else if(distance_cm<=160) target_ticks = distance_cm * 72.5;
  else if(distance_cm<=170) target_ticks = distance_cm * 72.5;
  else target_ticks = distance_cm * 72.5;
  
  while (encoder_right < 100)
  {
    compensation = tune_pid();
    md.setSpeeds(-(100 + compensation), -(100 - compensation));
  }
  
  while (encoder_right < 200)
  {
    compensation = tune_pid(); 
    md.setSpeeds(-(200 + compensation), -(200 - compensation));
  }

  while (encoder_right < 250)
  {
    compensation = tune_pid();
    md.setSpeeds(-(300 + compensation), -(300 - compensation));
  }

  while (encoder_right < target_ticks - 200)
  {
    compensation = tune_pid();
    md.setSpeeds(-(left_straight_speed + compensation), -(right_straight_speed - compensation));
  }
  
  while (encoder_right < target_ticks - 100)
  {
    compensation = tune_pid();
    md.setSpeeds(-(200 + compensation), -(200 - compensation));
  }

  while (encoder_right < target_ticks)
  {
    compensation = tune_pid();
    md.setSpeeds(-(100 + compensation), -(100 - compensation));
  }
  md.setBrakes(left_brake_speed, right_brake_speed);
  delay(80);
  md.setSpeeds(0, 0);
  delay(50);
}

// Rotation
// Accelerating and decelerating slowly
void rotate_right_ramp(int angle) {
  encoder_right = 0;
  encoder_left = 0;
  double compensation = 0;
  error = 0;
  integralError = 0;
  if (angle <= 5) target_ticks = angle * 5.2;
  else if (angle <= 10) target_ticks = angle * 6.3;
  else if (angle <= 15) target_ticks = angle * 6.4;
  else if (angle <= 30) target_ticks = angle * 7.7; //7.72
  else if (angle <= 45) target_ticks = angle * 8.01; //8.635
  else if (angle <= 60) target_ticks = angle * 8.3;
  else if (angle <= 90) target_ticks = angle * 8.77; // calibration redone on 11/10
  else if (angle <= 180) target_ticks = angle * 9.1; // calibration redone on 26/9
  else if (angle <= 360) target_ticks = angle * 9.12; // calibration redone on 26/9
  else if (angle <= 540) target_ticks = angle * 9.11; // calibration redone on 26/9
  else if (angle <= 720) target_ticks = angle * 9.12; // calibration redone on 26/9
  else if (angle <= 900) target_ticks = angle * 9.11; // calibration redone on 26/9
  else if (angle <= 1080) target_ticks = angle * 9.1; // calibration redone on 26/9
  else target_ticks = angle * 9.1;
  
  // ramping up
 while (encoder_right < target_ticks * 0.1 )
  {
    compensation = tune_pid();
    md.setSpeeds(100 + compensation, -(100 - compensation));
  }

  while (encoder_right < target_ticks * 0.3 )
  {
    compensation = tune_pid();
    md.setSpeeds(125 + compensation, -(125 - compensation));
  }

  // achieving max speed
  while (encoder_right < target_ticks * 0.8) 
  {
    compensation = tune_pid();
    md.setSpeeds(170 + compensation, -(170 - compensation));
  }

  // ramping down
  while (encoder_right < target_ticks) 
  {
    compensation = tune_pid();
    md.setSpeeds(125 + compensation, -(125 - compensation));
  }
  md.setBrakes(left_brake_speed, right_brake_speed);
  delay(80);
  md.setSpeeds(0, 0);
  delay(50);
}

void rotate_left_ramp(int angle) {
  encoder_right = 0;
  encoder_left = 0;
  double compensation = 0;
  error = 0;
  integralError = 0;
  if (angle <= 5) target_ticks = angle * 5.2;
  else if (angle <= 10) target_ticks = angle * 6.3;
  else if (angle <= 15) target_ticks = angle * 6.4;
  else if (angle <= 30) target_ticks = angle * 7.7; //7.72
  else if (angle <= 45) target_ticks = angle * 8.01; //8.635
  else if (angle <= 60) target_ticks = angle * 8.25;
  else if (angle <= 90) target_ticks = angle * 8.75; //8.643
  else if (angle <= 180) target_ticks = angle * 9.75;    //tune 180
  else if (angle <= 360) target_ticks = angle * 9.37;
  else if (angle <= 720) target_ticks = angle * 9.15;
  else if (angle <= 900) target_ticks = angle * 9.16;
  else if (angle <= 1080) target_ticks = angle * 9.06;
  else target_ticks = angle * 9.0;

  // ramping up
  while (encoder_right < target_ticks * 0.1 )
  {
    compensation = tune_pid();
    md.setSpeeds(-(100 + compensation), (100 - compensation));
  }

  while (encoder_right < target_ticks * 0.3 )
  {
    compensation = tune_pid();
    md.setSpeeds(-(125 + compensation), (125 - compensation));
  }

  // achieving max speed
  while (encoder_right < target_ticks * 0.8) 
  {
    compensation = tune_pid();
    md.setSpeeds(-(170 + compensation), (170 - compensation));
  }

  // ramping down
  while (encoder_right < target_ticks) 
  {
    compensation = tune_pid();
    md.setSpeeds(-(125 + compensation), (125 - compensation));
  }
  md.setBrakes(left_brake_speed, right_brake_speed);
  delay(80);
  md.setSpeeds(0, 0);
  delay(50);
}

// Without ramp
void rotate_left(int angle) {
  encoder_right = 0;
  encoder_left = 0;
  double compensation = 0;
  error = 0;
  integralError = 0;
  if (angle <= 5) target_ticks = angle * 5.2;
  else if (angle <= 10) target_ticks = angle * 6.3;
  else if (angle <= 15) target_ticks = angle * 6.4;
  else if (angle <= 30) target_ticks = angle * 7.7; //7.72
  else if (angle <= 45) target_ticks = angle * 8.01; //8.635
  else if (angle <= 60) target_ticks = angle * 8.3;
  else if (angle <= 90) target_ticks = angle * 8.522; // calibrated on 28/09
  else if (angle <= 180) target_ticks = angle * 9.1; 
  else if (angle <= 360) target_ticks = angle * 9.12; 
  else if (angle <= 540) target_ticks = angle * 9.11; 
  else if (angle <= 720) target_ticks = angle * 9.12; 
  else if (angle <= 900) target_ticks = angle * 9.11; 
  else if (angle <= 1080) target_ticks = angle * 9.1; 
  else target_ticks = angle * 9.1;

  while (encoder_right < target_ticks) 
  {
    compensation = tune_pid();
    md.setSpeeds(-(left_rotate_slow_speed + compensation), (right_rotate_slow_speed - compensation));
  }
  md.setBrakes(left_rotate_brake_speed, right_rotate_brake_speed); 
  delay(80);
  md.setSpeeds(0, 0);
  delay(50);
}

void rotate_right(int angle) {
  encoder_right = 0;
  encoder_left = 0;
  double compensation = 0;
  error = 0;
  integralError = 0;
  if (angle <= 5) target_ticks = angle * 5.2;
  else if (angle <= 10) target_ticks = angle * 6.3;
  else if (angle <= 15) target_ticks = angle * 6.4;
  else if (angle <= 30) target_ticks = angle * 7.7; //7.72
  else if (angle <= 45) target_ticks = angle * 8.01; //8.635
  else if (angle <= 60) target_ticks = angle * 8.3;
  else if (angle <= 90) target_ticks = angle * 8.47; //8.643
  else if (angle <= 180) target_ticks = angle * 9.75;    //tune 180
  else if (angle <= 360) target_ticks = angle * 9.37;
  else if (angle <= 720) target_ticks = angle * 9.15;
  else if (angle <= 900) target_ticks = angle * 9.16;
  else if (angle <= 1080) target_ticks = angle * 9.06;
  else target_ticks = angle * 9.0;

  while (encoder_right < target_ticks) 
  {
    compensation = tune_pid();
    md.setSpeeds(left_rotate_speed + compensation, -(right_rotate_speed - compensation));
  }  
  md.setBrakes(left_rotate_brake_speed, right_rotate_brake_speed); 
  delay(80);
  md.setSpeeds(0, 0);
  delay(50);
}

void stop_robot() {
  md.setBrakes(left_brake_speed, right_brake_speed);
  md.setSpeeds(0, 0);
}

/////////////////////////////////////////////////////////////////////////
///////////////////////////////PID TUNING////////////////////////////////
/////////////////////////////////////////////////////////////////////////

// interrupts to be called for counting the number of ticks from encoder output 
// when motor moves, used in tuning the PID, then translated into distance

void left_encoder_rising () {
  encoder_left++;
}

void right_encoder_rising () {
  encoder_right++;
}

double tune_pid () {
  double compensation, previous_encoder_right;
  //double Kp, Ki, Kd, p, i, d;
   double Kp, Ki, Kd, p, i;
  // Okay at HPL
//  Kp = 50;
//  Ki = 0.1;
//  Kd = 0.01;
  // Okay at HWL2
  // Kp = 54 for rotation
  Kp = 49; // increase in case it is going left, decrease in case it is going right
  Ki = 0.1;
  //Kd = 0.01;
  error = encoder_right - encoder_left;
  integralError += error;
  p = error * Kp;
  i = integralError * Ki;
  //d = (previous_encoder_right - encoder_right) * Kd;
  //compensation = p + i + d;
  compensation = p + i;
  previous_encoder_right = encoder_right;
  return compensation;
}

/////////////////////////////////////////////////////////////////////////
///////////////////////////ROBOT SENSOR CONTROL//////////////////////////
/////////////////////////////////////////////////////////////////////////

//Takes a median of the distance for accurate reading 
int get_distance (SharpIR sensor) {
 int distance = sensor.distance();
 // if any snesor reading is more than 70, return as 70
 if (distance > 70) {
  return 70;
 }
 return distance;
}

int get_median_distance (SharpIR sensor) {
  RunningMedian buffer = RunningMedian(100);
  for (int i = 0; i < 25; i ++)
  {
      delay(10);
      buffer.add(get_distance(sensor)); 
  }
  return buffer.getMedian();
}

//bot calibration

void bot_calibration()
{
  close_avoidance();
  align_angle();
  move_forward_ramp(1);
}

void close_avoidance()
{
  int initial_right_distance = get_median_distance(SENSOR_C_RIGHT);
  int initial_left_distance = get_median_distance(SENSOR_C_LEFT);
  if ( (initial_left_distance > 14) || (initial_right_distance > 14) ) move_backward_ramp(2);
}

// Align the distances of the left and right wheel to the arena grids
void align_distance(){
  int center_left_distance, center_distance, center_right_distance;
  while(1) {
    center_right_distance = get_median_distance(SENSOR_C_RIGHT);
    Serial.println (center_right_distance);
    if (center_right_distance < FRONT_SHORT_OFFSET) {
      move_backward_ramp(1);
    }
    else if (center_right_distance > FRONT_SHORT_OFFSET) {
      move_forward_ramp(1);
      //md.setSpeeds(80,80);
    }
    else 
      break;
  }
  
  while(1) {
    center_left_distance = get_median_distance(SENSOR_C_LEFT);
     Serial.println (center_left_distance);
    if (center_left_distance < FRONT_SHORT_OFFSET) {
      move_backward_ramp(1);
    }
    else if (center_left_distance > FRONT_SHORT_OFFSET) {
      move_forward_ramp(1);//md.setSpeeds(80,80);
    }
    else 
      break;
  }
  
  while(1) {
    center_distance = get_median_distance(SENSOR_C_BOT);
     Serial.println (center_distance);
    if (center_distance < FRONT_SHORT_OFFSET) {
      move_backward_ramp(1);
    }
    else if (center_distance > FRONT_SHORT_OFFSET) {
      move_forward_ramp(1);
    }
    else 
      break;
  }
  
  center_left_distance = get_median_distance(SENSOR_C_LEFT);
  center_right_distance = get_median_distance(SENSOR_C_RIGHT);
  int error = center_left_distance - center_right_distance; 
  if (error >= 1 || error <= -1) {
    align_angle();
  }
}

// Correct the angle of the robot such that left and right parts of the robot are at equal distances from an obstacle
void align_angle(){
  int center_distance, left_distance, right_distance;  
  left_distance = get_median_distance(SENSOR_C_LEFT);
  right_distance = get_median_distance(SENSOR_C_RIGHT);
  center_distance = get_median_distance(SENSOR_C_BOT);
  int difference = 2*center_distance - left_distance - right_distance;
  if ((difference < 4) || (difference > -4)){ 
    while(1)
    {  
      Serial.println("left_distance");
      Serial.println(left_distance);
      Serial.println("right_distance");
      Serial.println(right_distance);
      left_distance = get_median_distance(SENSOR_C_LEFT);
      right_distance = get_median_distance(SENSOR_C_RIGHT);
      int error = left_distance - right_distance;
      if (error >= 1) {
        rotate_right(2);
      }
      else if (error <= -1) {
        rotate_left(2);
      }
      else 
        break;
    }
  }
}

void read_sensor_readings(){
  left_distance = get_median_distance(SENSOR_LEFT);
  center_left_distance = get_median_distance(SENSOR_C_LEFT);
  center_distance = get_median_distance(SENSOR_C_BOT);
  center_right_distance = get_median_distance(SENSOR_C_RIGHT);
  right_distance = get_median_distance(SENSOR_RIGHT);
  right_long_distance = get_median_distance(SENSOR_RIGHT_LONG);
  if ((right_long_distance - SIDE_LONG_OFFSET) >= 20) 
    right_distance = right_long_distance - SIDE_LONG_OFFSET + SIDE_SHORT_OFFSET;
  Serial.print("AR2PC|");
  Serial.print(left_distance-SIDE_SHORT_OFFSET);
  Serial.print(":");
  Serial.print(center_left_distance-FRONT_SHORT_OFFSET);
  Serial.print(":");
  Serial.print(center_distance-FRONT_SHORT_OFFSET);
  Serial.print(":");
  Serial.print(center_right_distance-FRONT_SHORT_OFFSET);
  Serial.print(":");
  Serial.print(right_distance-SIDE_SHORT_OFFSET);
  Serial.print("\0");
  Serial.print("\n");
  Serial.flush();
}

void simulate(){
  left_distance = get_median_distance(SENSOR_LEFT);
  center_left_distance = get_median_distance(SENSOR_C_LEFT);
  center_distance = get_median_distance(SENSOR_C_BOT);
  center_right_distance = get_median_distance(SENSOR_C_RIGHT);
  right_distance = get_median_distance(SENSOR_RIGHT);
  right_long_distance = get_median_distance(SENSOR_RIGHT_LONG);
  if ((center_left_distance<=15)&&(center_right_distance<=15)&&(center_distance<=15)) {
    align_angle();
    align_distance();
    rotate_right_ramp(90);
  }

}

// Checklist task: avoid one obstacle placed on a 150 cm path
void avoid_obstacle(){
  int distance_moved = 0;
  int ditance_turned = 0;
  int obst_height = 0;
  int distance_turned = 0;
  int moving_distance = 0;
  Serial.println("Before Loop 1");
  Serial.print("SENSOR_C_RIGHT: ");
  Serial.println(get_median_distance(SENSOR_C_RIGHT));
  Serial.print("SENSOR_C_LEFT: ");
  Serial.println(get_median_distance(SENSOR_C_LEFT));
  Serial.print("SENSOR_C_BOT: ");
  Serial.println(get_median_distance(SENSOR_C_BOT));
  // going straight before robot collides into the obstacle
  Serial.println();
  // As long as all the distance readings are more than 10 cm, move forward
  // Once it is less than 10, obstacle is detected and call for obstacle avoidance
  while ((get_median_distance(SENSOR_C_RIGHT)- SIDE_SHORT_OFFSET) > 10 && ((get_median_distance(SENSOR_C_LEFT)-SIDE_SHORT_OFFSET) > 10)) { 
    // if ((get_median_distance(SENSOR_C_RIGHT)-SIDE_SHORT_OFFSET)>(get_median_distance(SENSOR_C_LEFT)-SIDE_SHORT_OFFSET))
    // {
    //   //move the shorter distance sensed by one of the sensors
    //   moving_distance = get_median_distance(SENSOR_C_LEFT);
    // }
    // else 
    // {
    //   moving_distance = get_median_distance(SENSOR_C_RIGHT);
    // }
    move_forward_ramp(10);
    distance_moved += 10 ;
  }
  // Encountered obstacle, turn for avoidance 
  // First turn to the left
  rotate_left(90);
  // Move along the length of the obstance until the obstacle is detected cleared by the right sensor
  while ((get_median_distance(SENSOR_RIGHT) - SIDE_SHORT_OFFSET) < 15) {
    move_forward_ramp(10);
    distance_turned += 10;
  }
  // Get one block clearance of the obstacle
  move_forward_ramp(20);
  // Second turn to the right
  rotate_right(90);
  // Get one block clearance of the obstacle
  move_forward_ramp(20);
  // Move along the breadth of the obstacle until the obstacle is detected cleared by the right sensor
  while ((get_median_distance(SENSOR_RIGHT)-SIDE_SHORT_OFFSET)<15) {
    Serial.println("Loop 3");
    Serial.print("SENSOR_LEFT");
    Serial.println(get_median_distance(SENSOR_LEFT));
    move_forward_ramp(10);
    obst_height +=10;
  }
  // Get one block clearance of the obstacle
  move_forward_ramp(20);
  // Third turn to the right
  rotate_right(90);
  // Moves back on the original line
  move_forward_ramp(20);
  // Fourth turn to get back to the original path and complete the rest of the straight lines 
  rotate_left(90);
  // 150 cm is the full length to be moved. Move straight through the rest of maze.
  move_forward_ramp(150 - distance_moved - obst_height - 40);
  while(1) 
  {
    // End of avoidance
  }
}

// Checklist task: Useless curve without pivoting
void avoid_obstacle_curve() {
  delay(50);
  double comp;
  int right_distance = get_median_distance(SENSOR_C_RIGHT);
  int left_distance = get_median_distance(SENSOR_C_LEFT);
  // before it turns
  while(right_distance >= 25 && left_distance >= 25)
  {
    md.setM2Speed(200);
    delay(2);
    md.setM1Speed(200);
    comp = tune_pid();
    // md.setSpeeds(200, 200);
    delay(50);
    // move_forward_ramp(10);
    right_distance = SENSOR_C_RIGHT.distance();
    left_distance = SENSOR_C_LEFT.distance();
  }
  //start of the first turn
  md.setM2Speed(0);
  delay(1500);
  md.setM2Speed(200);
  md.setM1Speed(0);
  delay(1500);
  md.setM1Speed(200);
  delay(1200);
  md.setM1Speed(0);
  delay(1500);
  md.setM1Speed(200);
  md.setM2Speed(0);
  delay(1500);
  md.setBrakes(400,400);
}
