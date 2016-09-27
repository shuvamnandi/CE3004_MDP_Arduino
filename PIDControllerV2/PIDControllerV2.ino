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
float left_rotate_speed, right_rotate_speed;
float left_brake_speed, right_brake_speed;
int angle;

// SETUP SENSORS PINS 
// sensors are taking the ANALOG pins
#define SENSOR_LEFT_PIN 0
#define SENSOR_RIGHT_PIN 1
#define SENSOR_CT_PIN 2
#define SENSOR_CB_PIN 3
#define SENSOR_CL_PIN 4
#define SENSOR_CR_PIN 5
#define WALL_DISTANCE 12

#define FRONT_SHORT_OFFSET 2
#define SIDE_SHORT_OFFSET 5
//#define front_center_offset

// 1080 => short range senor GP2Y0A21YK
// 20150 => long range sensor GP2Y0A02YK
SharpIR SENSOR_LEFT (SENSOR_LEFT_PIN, 1080); // left, short range sensor
SharpIR SENSOR_RIGHT (SENSOR_RIGHT_PIN, 1080); // right, short range sensor
SharpIR SENSOR_C_TOP (SENSOR_CT_PIN, 20150); // center top, long range sensor
SharpIR SENSOR_C_BOT (SENSOR_CB_PIN, 20150); // center bottom, long range sensor
SharpIR SENSOR_C_LEFT (SENSOR_CL_PIN, 1080);  // center left, short range sensor
SharpIR SENSOR_C_RIGHT (SENSOR_CR_PIN, 1080);  // center right, short range sensor

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////SETUP///////////////////////////////////
/////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  md.init();
  pinMode(LEFT_MOTOR_PIN, INPUT);
  pinMode(RIGHT_MOTOR_PIN, INPUT);
  // attach interrputs to the encoders output pins with PinChangeInt
  PCintPort::attachInterrupt(LEFT_MOTOR_PIN, left_encoder_rising, HIGH);
  PCintPort::attachInterrupt(RIGHT_MOTOR_PIN, right_encoder_rising, HIGH);
  left_straight_speed = 400;    //250
  right_straight_speed = 400;    //250
  left_rotate_speed = 350;    //150
  right_rotate_speed = 350;    //150
  left_brake_speed = 385;    //250
  right_brake_speed = 400;
}

void loop() {
//  Serial.print("median distance from left=");
//  Serial.println (get_median_distance (SENSOR_LEFT));
//  
//  Serial.print("median Distance from right=");
//  Serial.println (get_median_distance (SENSOR_RIGHT));
//  Serial.print("median Distance from center low=");
//  Serial.println (get_median_distance (SENSOR_C_BOT));  
  // Serial.print("median Distance from center =");
  // Serial.println (get_median_distance (SENSOR_C_TOP)); 
//   
//  Serial.print("median Distance from center right=");
//  Serial.println (get_median_distance (SENSOR_C_RIGHT));  
//  
//  Serial.print("median Distance from center left=");
//  Serial.println (get_median_distance (SENSOR_C_LEFT));
//  while(1){
//  }
  align_distance();
  delay(500);
}

/////////////////////////////////////////////////////////////////////////
///////////////////////////ROBOT MOVEMENT CONTROL////////////////////////
/////////////////////////////////////////////////////////////////////////

// Straight line movement

void move_forward_ramp (int distance_cm) {
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
  else if(distance_cm<=100) target_ticks = distance_cm * 60.2; //calibration redone on 26/9
  else if(distance_cm<=110) target_ticks = distance_cm * 59.8; 
  else if(distance_cm<=120) target_ticks = distance_cm * 60.3; //calibration redone on 26/9
  else if(distance_cm<=130) target_ticks = distance_cm * 60.3; 
  else if(distance_cm<=140) target_ticks = distance_cm * 60.5; 
  else if(distance_cm<=150) target_ticks = distance_cm * 60.5; 
  else if(distance_cm<=160) target_ticks = distance_cm * 60.5; //calibration redone on 26/9
  else if(distance_cm<=170) target_ticks = distance_cm * 60.5; 
  else target_ticks = distance_cm * 60.5;
  
  Serial.print("Distance_cm: ");
  Serial.println(distance_cm);
  Serial.print("target tick: ");
  Serial.println(target_ticks);
  
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
  Serial.print("encoder_left: ");
  Serial.println(encoder_left);
  Serial.print("encoder_right: ");
  Serial.println(encoder_right);
  md.setBrakes(left_brake_speed, right_brake_speed);
  delay(1000);
  md.setBrakes(0, 0);
}

void move_backward_ramp (int distance_cm) {
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
  
  Serial.print("Distance_cm: ");
  Serial.println(distance_cm);
  Serial.print("target tick: ");
  Serial.println(target_ticks);
  
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
  Serial.print("encoder_left: ");
  Serial.println(encoder_left);
  Serial.print("encoder_right: ");
  Serial.println(encoder_right);
  //Okay at HPL
  //md.setBrakes(375, 400);

  // at HWL2
  md.setBrakes(left_brake_speed, right_brake_speed);
  delay(100);
  //md.setBrakes(0, 0);
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
  else if (angle <= 90) target_ticks = angle * 8.877; // calibration redone on 26/9
  else if (angle <= 180) target_ticks = angle * 9.1; // calibration redone on 26/9
  else if (angle <= 360) target_ticks = angle * 9.12; // calibration redone on 26/9
  else if (angle <= 540) target_ticks = angle * 9.11; // calibration redone on 26/9
  else if (angle <= 720) target_ticks = angle * 9.12; // calibration redone on 26/9
  else if (angle <= 900) target_ticks = angle * 9.11; // calibration redone on 26/9
  else if (angle <= 1080) target_ticks = angle * 9.1; // calibration redone on 26/9
  else target_ticks = angle * 9.1;
  
  // ramping up
  while (encoder_right < target_ticks* 0.2 )
  {
    compensation = tune_pid();
    md.setSpeeds(- (150 + compensation), 150 - compensation);
  }

  // achieveing max speed  
  while (encoder_right < target_ticks*0.7) 
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
  md.setBrakes(0, 0);
}

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
  else if (angle <= 90) target_ticks = angle * 8.47; //8.643
  else if (angle <= 180) target_ticks = angle * 9.75;    //tune 180
  else if (angle <= 360) target_ticks = angle * 9.37;
  else if (angle <= 720) target_ticks = angle * 9.15;
  else if (angle <= 900) target_ticks = angle * 9.16;
  else if (angle <= 1080) target_ticks = angle * 9.06;
  else target_ticks = angle * 9.0;

  // ramping up
  while (encoder_right < target_ticks*0.2 )
  {
    compensation = tune_pid();
    md.setSpeeds(150 + compensation, -(150 - compensation));
  }

  // achieving max speed
  while (encoder_right < target_ticks*0.7) 
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
  md.setBrakes(0, 0);
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
  else if (angle <= 90) target_ticks = angle * 8.8; 
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
    md.setSpeeds(-(left_rotate_speed + compensation), (right_rotate_speed - compensation));
  }
  md.setBrakes(left_brake_speed, right_brake_speed); 
  delay(80);
  md.setBrakes(0, 0);
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
  md.setBrakes(left_brake_speed, right_brake_speed); 
  delay(80);
  md.setBrakes(0, 0);
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
  Kp = 54; // increase in case it is going left, decrease in case it is going right
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
float get_distance (SharpIR sensor) {
 return (sensor.distance());
}

float get_median_distance (SharpIR sensor) {
RunningMedian buffer = RunningMedian(100);
for (int i = 0; i < 100; i ++)
  {
    delay(20);
    buffer.add(get_distance(sensor)); 
  }
  return buffer.getMedian();
}

//bot calibration

void bot_calibration()
{
  close_avoidance();
  align_angle();
  move_forward_ramp(1.3);
}

void close_avoidance()
{
  int initial_right_distance = get_median_distance(SENSOR_C_RIGHT);
  int initial_left_distance = get_median_distance(SENSOR_C_LEFT);
  if ( (initial_left_distance > 14) || (initial_right_distance > 14) ) move_backward_ramp(2);
}

// Align the distances of the left and right wheel to the arena grids
void align_distance(){
  while(1) {
    int right_distance = get_median_distance(SENSOR_C_RIGHT);
    if (right_distance > WALL_DISTANCE) {
      move_forward_ramp(0.8); 
      //md.setSpeeds(80,80);
    }
    else if (right_distance < WALL_DISTANCE) {
      move_backward_ramp(0.8);
    }
    else 
      break;
  }
  md.setBrakes(left_brake_speed, right_brake_speed);

  while(1) {
    int left_distance = get_median_distance(SENSOR_C_LEFT);
    if (left_distance > WALL_DISTANCE) {
      move_forward_ramp(0.8);//md.setSpeeds(80,80);
    }
    else if (left_distance < WALL_DISTANCE) {
      move_backward_ramp(0.8);
    }
    else 
      break;
  }
  md.setBrakes(left_brake_speed, right_brake_speed);
  int left_distance = get_median_distance(SENSOR_C_LEFT);
  int right_distance = get_median_distance(SENSOR_C_RIGHT);
  int error = left_distance - right_distance; 
  // if (error >= 1 || error <= -1) {
  //   align_angle();
  // }
}



// Correct the angle of the robot such that left and right parts of the robot are at equal distances from an obstacle
void align_angle(){
  int left_distance, right_distance;
  int initial_left_distance, initial_right_distance;
  initial_left_distance = get_median_distance(SENSOR_C_LEFT);
  initial_right_distance = get_median_distance(SENSOR_C_RIGHT);
  while(1)
  {  
    left_distance = get_median_distance(SENSOR_C_LEFT);
    right_distance = get_median_distance(SENSOR_C_RIGHT);
    int error = left_distance - right_distance;
    if (error >= 1) {
      rotate_right(0.5);
    }
    else if (error <= -1) {
      rotate_left(0.5);
    }
    else break;
  }
  if (initial_left_distance < initial_right_distance){
    rotate_left(2);
  }
  right_distance = get_median_distance(SENSOR_C_RIGHT);
  if (right_distance > WALL_DISTANCE || right_distance < WALL_DISTANCE) {
    align_distance();
  }
}
