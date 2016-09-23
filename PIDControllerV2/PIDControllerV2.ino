#include <PinChangeInt.h>
#include "DualVNH5019MotorShield.h"
#define LEFT_MOTOR_PIN 3
#define RIGHT_MOTOR_PIN 5

DualVNH5019MotorShield md;

volatile float encoder_left = 0;
volatile float encoder_right = 0;
double error = 0.0, integral = 0.0, target_tick = 0.0;
float left_straight_speed, right_straight_speed;

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
}

void loop() {
  move_forward_ramp_up(150);
  while(1){
    
  }
}

void move_forward_ramp_up (int distance_cm) {
  encoder_left = 0;
  encoder_right = 0;
  double compensation = 0;
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
  md.setBrakes(375, 400);
  delay(100);
  //md.setBrakes(0, 0);
} 

// rising left and right will capture the increase in the encoder values
// that will be used to tune with PID then translate into the distance
void left_encoder_rising () {
  encoder_left++;
}

void right_encoder_rising () {
  encoder_right++;
}

double tunePID () {
  double compensation, last_tick;
  double Kp, Ki, Kd, p, i, d;
  // Okay at HPL
//  Kp = 50;
//  Ki = 0.1;
//  Kd = 0.01;
  // at HWL2
  Kp = 49;
  Ki = 0.1;
  Kd = 0.01;
  error = encoder_right - encoder_left;
  integral += error;
  p = error * Kp;
  i = integral * Ki;
  d = (last_tick - encoder_right) * Kd;
  compensation = p + i + d;
  last_tick = encoder_right;  
  return compensation;
}

