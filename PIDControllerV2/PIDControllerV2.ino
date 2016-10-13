#include "DualVNH5019MotorShield.h"
#include <SharpIR.h>
#include <PinChangeInt.h>
#include <RunningMedian.h>

/////////////////////////////////////////////////////////////////////////
/////INDEX///////////////////////////////////////////////////////////////
/////SECTION 1 - HARDWARE SETUP//////////////////////////////////////////
/////SECTION 2 - RASPBERRY PI COMMUNICATION//////////////////////////////
/////SECTION 3 - ROBOT MOVEMENT CONTROL EXPLORATION//////////////////////
/////SECTION 4 - ROBOT MOVEMENT CONTROL FASTEST PATH/////////////////////
/////SECTION 5 - PID TUNING//////////////////////////////////////////////
/////SECTION 6 - ROBOT SENSOR CONTROL////////////////////////////////////
/////SECTION 7 - ROBOT SELF CALIBRATION//////////////////////////////////
/////SECTION 7 - DUBUGGING///////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////
/////////////////////SECTION 1 - HARDWARE SETUP//////////////////////////
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
int move_counter;
int initial_left_obstacle_distance = 0;
char command[64];

// SETUP SENSORS PINS 
// sensors are taking the ANALOG pins
#define SENSOR_LEFT_PIN 0
#define SENSOR_RIGHT_PIN 1
#define SENSOR_RIGHT_LONG_PIN 2
#define SENSOR_CB_PIN 3
#define SENSOR_CL_PIN 4
#define SENSOR_CR_PIN 5

#define FRONT_SHORT_OFFSET 8
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
  move_counter = 0; // Used by align_angle()
  initial_left_obstacle_distance = get_median_distance(SENSOR_LEFT) - SIDE_SHORT_OFFSET;
}

void loop() {
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
/////////////////SECTION 2 - RASPBERRY PI COMMUNICATION//////////////////
/////////////////////////////////////////////////////////////////////////

// Read message from Serial sent by RPi

char* get_rpi_message() {
  memset(command, 0, sizeof(command));
  Serial.readBytes(command, 64);
  return command;
}

// // Print to Serial for RPi to read messages

// void set_rpi_message(int left, int center_left, int center, int center_right, int right, int right_long){
//   // For distances greater than 20 cm, use long 
//   if ((right_long - SIDE_LONG_OFFSET) >= 20) 
//     right = right_long - SIDE_LONG_OFFSET + SIDE_SHORT_OFFSET;
//   Serial.print("AR2PC|");
//   Serial.print(left-SIDE_SHORT_OFFSET);
//   Serial.print(":");
//   Serial.print(center_left-FRONT_SHORT_OFFSET);
//   Serial.print(":");
//   Serial.print(center-FRONT_SHORT_OFFSET);
//   Serial.print(":");
//   Serial.print(center_right-FRONT_SHORT_OFFSET);
//   Serial.print(":");
//   Serial.print(right-SIDE_SHORT_OFFSET);
//   Serial.print("\0");
//   Serial.print("\n");
//   Serial.flush();
// }

// Cases for commands sent by Raspberry Pi to move

void move_robot(char command) {
  //Serial.print("XXXXXXXXXXXXRPI command received by Arduino: ");
  //Serial.println(command);
  switch(command) {
    case 'E': break;
    case 'F': move_forward_ramp(10); break;
    case 'L': rotate_left_ramp(90); break;
    case 'R': bot_calibration(); rotate_right_ramp(90); break;
    case 'B': move_backward_ramp(10); break;
    case 'H': stop_robot(); break;
    case 'X': fastest_path(); break;
  }
}

/////////////////////////////////////////////////////////////////////////
///////////SECTION 3 - ROBOT MOVEMENT CONTROL EXPLORATION////////////////
/////////////////////////////////////////////////////////////////////////



void move_forward_ramp (int distance_cm) {
  encoder_left = 0;
  encoder_right = 0;
  double compensation = 0;
  error = 0.0;
  integralError = 0.0;
  if (distance_cm <= 10) target_ticks = distance_cm * 54.1; // calibration redone on 12/10
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
  delay(25);
  md.setSpeeds(0, 0);
  //delay(50);
  move_counter++;
}

void move_backward_ramp (int distance_cm) {
  encoder_left = 0;
  encoder_right = 0;
  double compensation = 0;
  error = 0.0;
  integralError = 0.0;
  //target_ticks = distance_cm * 58.5;
  if (distance_cm<= 10) target_ticks = distance_cm * 54.4;
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
  delay(25);
  md.setSpeeds(0, 0);
  //delay(50);
}



// Rotation
// Accelerating and decelerating slowly
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
  else if (angle <= 60) target_ticks = angle * 8.3;
  else if (angle <= 90) target_ticks = angle * 8.5; // calibration redone on 11/10
  else if (angle <= 180) target_ticks = angle * 9.1; // calibration redone on 26/9
  else if (angle <= 360) target_ticks = angle * 9.12; // calibration redone on 26/9
  else if (angle <= 540) target_ticks = angle * 9.11; // calibration redone on 26/9
  else if (angle <= 720) target_ticks = angle * 9.12; // calibration redone on 26/9
  else if (angle <= 900) target_ticks = angle * 9.11; // calibration redone on 26/9
  else if (angle <= 1080) target_ticks = angle * 9.1; // calibration redone on 26/9
  else target_ticks = angle * 9.1;
  
  // ramping up
  while (encoder_right < target_ticks * 0.2 )
  {
    compensation = tune_pid();
    md.setSpeeds(- (150 + compensation), 150 - compensation);
  }

  // achieveing max speed  
  while (encoder_right < target_ticks * 0.7) 
  {
    compensation = tune_pid();
    md.setSpeeds(-(left_rotate_speed + compensation), (right_rotate_speed - compensation));
  }

  // ramping down
  while (encoder_right < target_ticks) 
  {
    compensation = tune_pid();
    md.setSpeeds(-(150 + compensation), (150 - compensation));
  }
  md.setBrakes(left_brake_speed, right_brake_speed);
  delay(80);
  md.setSpeeds(0, 0);
  delay(50);
}

void rotate_right_ramp(int angle) {
  encoder_right = 0;
  encoder_left = 0;
  double compensation = 0;
  error = 0;
  integralError = 0;
  if (angle <= 5) target_ticks = angle * 3.7;
  else if (angle <= 10) target_ticks = angle * 6.3;
  else if (angle <= 15) target_ticks = angle * 6.4;
  else if (angle <= 30) target_ticks = angle * 7.7; //7.72
  else if (angle <= 45) target_ticks = angle * 8.01; //8.635
  else if (angle <= 60) target_ticks = angle * 8.3;
  else if (angle <= 90) target_ticks = angle * 8.51; //8.643
  else if (angle <= 180) target_ticks = angle * 9.75;    //tune 180
  else if (angle <= 360) target_ticks = angle * 9.37;
  else if (angle <= 720) target_ticks = angle * 9.15;
  else if (angle <= 900) target_ticks = angle * 9.16;
  else if (angle <= 1080) target_ticks = angle * 9.06;
  else target_ticks = angle * 9.0;

  // ramping up
  while (encoder_right < target_ticks * 0.2 )
  {
    compensation = tune_pid();
    md.setSpeeds(150 + compensation, -(150 - compensation));
  }

  // achieving max speed
  while (encoder_right < target_ticks * 0.7) 
  {
    compensation = tune_pid();
    md.setSpeeds(left_rotate_speed + compensation, -(right_rotate_speed - compensation));
  }

  // ramping down
  while (encoder_right < target_ticks) 
  {
    compensation = tune_pid();
    md.setSpeeds(150 + compensation, -(150 - compensation));
  }
  md.setBrakes(left_brake_speed, right_brake_speed);
  delay(80);
  md.setSpeeds(0, 0);
  delay(50);
}


void stop_robot() {
  md.setBrakes(left_brake_speed, right_brake_speed);
  md.setSpeeds(0, 0);
}






/////////////////////////////////////////////////////////////////////////
/////////////////SECTION 4 - ROBOT MOVEMENT CONTROL FASTEST PATH/////////
/////////////////////////////////////////////////////////////////////////


void fastest_path() {
  int multiplication;
  char* rpi_message = get_rpi_message();
  String rpiMsg(rpi_message);
  Serial.println(rpiMsg);
  int strlength = rpiMsg.length();
  for ( int i = 0; i < strlength-1; i++) {
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
/////////////////////////////////////////////////////////////////////////
/////////////////////////SECTION 5 - PID TUNING//////////////////////////
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
/////////////////SECTION 6 - ROBOT SENSOR CONTROL////////////////////////
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
  for (int i = 0; i < 25; i++)
  {
      delay(0.5);
      buffer.add(get_distance(sensor)); 
  }
  return buffer.getMedian();
}

// Robot calibration


void read_sensor_readings(){
  left_distance = get_median_distance(SENSOR_LEFT);
  center_left_distance = get_median_distance(SENSOR_C_LEFT);
  center_distance = get_median_distance(SENSOR_C_BOT);
  center_right_distance = get_median_distance(SENSOR_C_RIGHT);
  right_distance = get_median_distance(SENSOR_RIGHT);
  right_long_distance = get_median_distance(SENSOR_RIGHT_LONG);
  if ((right_distance - SIDE_SHORT_OFFSET) > 15) 
    right_distance = right_long_distance - SIDE_LONG_OFFSET + 4 + SIDE_SHORT_OFFSET;
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


/////////////////////////////////////////////////////////////////////////
/////////////////SECTION 7 - ROBOT SELF CALIBRATION//////////////////////
/////////////////////////////////////////////////////////////////////////


void bot_calibration()
{
  align_angle();
  align_distance();
}

// Align the position of the robot to the arena grids
void align_distance(){
  int center_left_distance, center_distance, center_right_distance;  
  center_left_distance = get_median_distance(SENSOR_C_LEFT)-FRONT_SHORT_OFFSET;
  if (center_left_distance < 5 && center_left_distance !=0) {
    if (center_left_distance > 0)
      move_forward_ramp(center_left_distance);
    else if (center_left_distance < 0)
      move_backward_ramp(-center_left_distance);
      
  }
  center_distance = get_median_distance(SENSOR_C_BOT)-FRONT_SHORT_OFFSET;
  if (center_distance < 5 && center_distance !=0) {
    if (center_distance < 5)
      move_forward_ramp(center_distance);
    else if (center_distance < 0)
      move_backward_ramp(-center_distance);
  }
  center_right_distance = get_median_distance(SENSOR_C_RIGHT)-FRONT_SHORT_OFFSET;
  if (center_right_distance < 5 && center_right_distance !=0) {
    if (center_right_distance < 5)
      move_forward_ramp(center_right_distance);   
    else if (center_right_distance < 0)
      move_backward_ramp(-center_right_distance);   
  }
}

// Correct the angle of the robot so the robot can be straightened
void align_angle(){
  int current_left_obstacle_distance = get_median_distance(SENSOR_LEFT)-SIDE_SHORT_OFFSET;
  double error = current_left_obstacle_distance - initial_left_obstacle_distance;
  double error_angle = asin(abs(error)/(move_counter*10))*(180/3.1459);
  //Serial.print("error_angle: ");
  //Serial.println(error_angle);
  if (error > 0)
    rotate_left_ramp(error_angle);
  else if (error < 0)
    rotate_right_ramp(error_angle);  
  move_counter = 0;
  initial_left_obstacle_distance = current_left_obstacle_distance;
}


//////////////////////////////////////////////////
///////////////SECTON 8 - DUBUGGING///////////////
//////////////////////////////////////////////////


