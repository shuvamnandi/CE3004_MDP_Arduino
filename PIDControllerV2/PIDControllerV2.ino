#include <SharpIR.h>
#include <PinChangeInt.h>
#include "DualVNH5019MotorShield.h"
DualVNH5019MotorShield md;


/////////////////////////////////////////////////////////////////////////
///////////////////////////////////PIN CONFIG////////////////////////////
/////////////////////////////////////////////////////////////////////////

// SETUP THE MOTOR ENCODERS - USE DIGITAL PINS

#define LEFT_MOTOR_PIN 3
#define RIGHT_MOTOR_PIN 5

// SETUP THE SENSORS - USE ANALOG PINS

#define sensor_L_pin 1
#define sensor_R_pin 2
#define sensor_C_pin 3
#define sensor_CL_pin 4
#define sensor_CR_pin 5


// 1080 => short range senor GP2Y0A21Y
// 20150 => long range sensor GP2Y0A02Y
// these are taking the ANALOG pins
SharpIR sensor_L (sensor_L_pin, 1080);
SharpIR sensor_R (sensor_R_pin, 1080);
SharpIR sensor_C (sensor_C_pin, 20150);   //kong range sensor
SharpIR sensor_CL (sensor_CL_pin, 1080);  //center left
SharpIR sensor_CR (sensor_CR_pin, 1080);  //center right 

volatile float encoder_left = 0;
volatile float encoder_right = 0;
double error = 0.0, integralError = 0.0, target_tick = 0.0;
float left_straight_speed, right_straight_speed;
float left_rotate_speed, right_rotate_speed;
float left_break_speed, right_break_speed;
int angle;

/////////////////////////////////////////////////////////////////////////
///////////////////////////////////SETUP/////////////////////////////////
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
  left_break_speed = 370;    //250
  right_break_speed = 400;
}

void loop() {
  int left = sensor_CL.distance();
  int right = sensor_CR.distance();
  Serial.print("left distance = ");
  Serial.println(left);
  Serial.print("right distance = ");
  Serial.println(right);

}

/////////////////////////////////////////////////////////////////////////
///////////////////////////ROBOT MOVEMENT CONTROL////////////////////////
/////////////////////////////////////////////////////////////////////////

// Straight line movement

void move_forward_ramp_up (int distance_cm) {
  encoder_left = 0;
  encoder_right = 0;
  double compensation = 0;
  error = 0.0;
  integralError = 0.0;
  //target_tick = distance_cm * 58.5;
  if (distance_cm<= 10) target_tick = distance_cm * 58.3;
  else if(distance_cm<=20) target_tick = distance_cm * 58.5;
  else if(distance_cm<=30) target_tick = distance_cm * 58.5;
  else if(distance_cm<=40) target_tick = distance_cm * 59.0;
  else if(distance_cm<=50) target_tick = distance_cm * 59.3;
  else if(distance_cm<=60) target_tick = distance_cm * 59.3;
  else if(distance_cm<=70) target_tick = distance_cm * 59.3;
  else if(distance_cm<=80) target_tick = distance_cm * 59.3;
  else if(distance_cm<=90) target_tick = distance_cm * 59.3;
  else if(distance_cm<=100) target_tick = distance_cm * 59.3;
  else if(distance_cm<=110) target_tick = distance_cm * 59.3;
  else if(distance_cm<=120) target_tick = distance_cm * 59.3;
  else if(distance_cm<=130) target_tick = distance_cm * 59.3;
  else if(distance_cm<=140) target_tick = distance_cm * 59.3;
  else if(distance_cm<=150) target_tick = distance_cm * 59.3;
  else if(distance_cm<=160) target_tick = distance_cm * 59.3;
  else if(distance_cm<=170) target_tick = distance_cm * 59.3;
  else target_tick = distance_cm * 59.5;
  
  Serial.print("Distance_cm: ");
  Serial.println(distance_cm);
  Serial.print("target tick: ");
  Serial.println(target_tick);
  
  while (encoder_right < 100)
  {
    compensation = tunePID();
    md.setSpeeds(100 + compensation, 100 - compensation);
  }
  
  while (encoder_right < 180 )
  {
    compensation = tunePID(); 
    md.setSpeeds(200 + compensation, 200 - compensation);
  }

  while (encoder_right < 260 )
  {
    compensation = tunePID();
    md.setSpeeds(300 + compensation, 300 - compensation);
  }

  while (encoder_right < target_tick - 200)
  {
    compensation = tunePID();
    md.setSpeeds(left_straight_speed + compensation, right_straight_speed - compensation);
  }
  
  while (encoder_right < target_tick)
  {
    compensation = tunePID();
    md.setSpeeds(200 + compensation, 200 - compensation);
  }
  Serial.print("encoder_left: ");
  Serial.println(encoder_left);
  Serial.print("encoder_right: ");
  Serial.println(encoder_right);
  //Okay at HPL
  //md.setBrakes(375, 400);
  // at HWL2
  md.setBrakes(left_break_speed, right_break_speed);
  delay(100);
  //md.setBrakes(0, 0);
} 

// Rotation

void rotate_right(int angle) {
  encoder_right = 0;
  encoder_left = 0;
  double compensation = 0;
  error = 0;
  integralError = 0;
  if (angle <= 5) target_tick = angle * 5.2;
  else if (angle <= 10) target_tick = angle * 6.3;
  else if (angle <= 15) target_tick = angle * 6.4;
  else if (angle <= 30) target_tick = angle * 7.7; //7.72
  else if (angle <= 45) target_tick = angle * 8.01; //8.635
  else if (angle <= 60) target_tick = angle * 8.3;
  else if (angle <= 90) target_tick = angle * 8.47; //8.643
  else if (angle <= 180) target_tick = angle * 9.75;    //tune 180
  else if (angle <= 360) target_tick = angle * 9.37;
  else if (angle <= 720) target_tick = angle * 9.15;
  else if (angle <= 900) target_tick = angle * 9.16;
  else if (angle <= 1080) target_tick = angle * 9.06;
  else target_tick = angle * 9.0;

  while (encoder_right < target_tick*0.2 )
  {
    compensation = tunePID();
    md.setSpeeds(150 + compensation, -(150 - compensation));
  }

  while (encoder_right < target_tick*0.7) 
  {
    compensation = tunePID();
    md.setSpeeds(left_rotate_speed + compensation, -(right_rotate_speed - compensation));
  }
  
  while (encoder_right < target_tick) 
  {
    compensation = tunePID();
    md.setSpeeds(150 + compensation, -(150 - compensation));
  }
  
  md.setBrakes(right_break_speed,left_break_speed); 
  delay(80);
  md.setBrakes(0, 0);
}

void rotate_left(int angle) {
  encoder_right = 0;
  encoder_left = 0;
  double compensation = 0;
  error = 0;
  integralError = 0;
  if (angle <= 5) target_tick = angle * 5.2;
  else if (angle <= 10) target_tick = angle * 6.3;
  else if (angle <= 15) target_tick = angle * 6.4;
  else if (angle <= 30) target_tick = angle * 7.7; //7.72
  else if (angle <= 45) target_tick = angle * 8.01; //8.635
  else if (angle <= 60) target_tick = angle * 8.3;
  else if (angle <= 90) target_tick = angle * 8.47; //8.643
  else if (angle <= 180) target_tick = angle * 9.75;    //tune 180
  else if (angle <= 360) target_tick = angle * 9.37;
  else if (angle <= 720) target_tick = angle * 9.15;
  else if (angle <= 900) target_tick = angle * 9.16;
  else if (angle <= 1080) target_tick = angle * 9.06;
  else target_tick = angle * 9.0;

// ramping
  while (encoder_right < target_tick* 0.2 )
  {
    compensation = tunePID();
    md.setSpeeds(- (150 + compensation), 150 - compensation);
  }

  while (encoder_right < target_tick*0.7) 
  {
    compensation = tunePID();
    md.setSpeeds(-(left_rotate_speed + compensation), (right_rotate_speed - compensation));
  }
// achieveing max speed  
  while (encoder_right < target_tick) 
  {
    compensation = tunePID();
    md.setSpeeds(-(150 + compensation), (150 - compensation));
  }
  
  md.setBrakes(right_break_speed,left_break_speed); 
  delay(80);
  md.setBrakes(0, 0);
}




/////////////////////////////////////////////////////////////////////////
///////////////////////////////PID TUNING////////////////////////////////
/////////////////////////////////////////////////////////////////////////


// rising left and right will capture the increase in the encoder values
// that will be used to tune with PID then translate into the distance
void left_encoder_rising () {
  encoder_left++;
}

void right_encoder_rising () {
  encoder_right++;
}

double tunePID () {
  double compensation, pervious_encoder_right;
  //double Kp, Ki, Kd, p, i, d;
   double Kp, Ki, Kd, p, i;
  // Okay at HPL
//  Kp = 50;
//  Ki = 0.1;
//  Kd = 0.01;
  // Okay at HWL2
  Kp = 49;
  Ki = 0.1;
  //Kd = 0.01;
  error = encoder_right - encoder_left;
  integralError += error;
  p = error * Kp;
  i = integralError * Ki;
  //d = (pervious_encoder_right - encoder_right) * Kd;
  // compensation = p + i + d;
  compensation = p + i;
  pervious_encoder_right = encoder_right;  
  return compensation;
}
