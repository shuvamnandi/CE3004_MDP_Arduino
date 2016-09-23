#include <PinChangeInt.h>
#include "DualVNH5019MotorShield.h"
#define LEFT_MOTOR_PIN 3
#define RIGHT_MOTOR_PIN 5

DualVNH5019MotorShield md;

volatile float encoder_left = 0;
volatile float encoder_right = 0;
double error = 0.0, integralError = 0.0, target_tick = 0.0;
float left_straight_speed, right_straight_speed;
float left_rotate_speed, right_rotate_speed;
float left_brake_speed, right_brake_speed;
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
  left_brake_speed = 385;    //250
  right_brake_speed = 400;
}

void loop() {
  move_forward_ramp_up(90);
  delay(1000);
  move_backward_ramp_up(90);
  delay(1000);
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
  if (distance_cm <= 10) target_tick = distance_cm * 58.3;
  else if(distance_cm<=20) target_tick = distance_cm * 58.5;
  else if(distance_cm<=30) target_tick = distance_cm * 58.5;
  else if(distance_cm<=40) target_tick = distance_cm * 59.0;
  else if(distance_cm<=50) target_tick = distance_cm * 59.3;
  else if(distance_cm<=60) target_tick = distance_cm * 59.3;
  else if(distance_cm<=70) target_tick = distance_cm * 59.3;
  else if(distance_cm<=80) target_tick = distance_cm * 59.3;
  else if(distance_cm<=90) target_tick = distance_cm * 59.6; //calibration done
  else if(distance_cm<=100) target_tick = distance_cm * 59.8; //calibration done
  else if(distance_cm<=110) target_tick = distance_cm * 59.8; 
  else if(distance_cm<=120) target_tick = distance_cm * 60.3; 
  else if(distance_cm<=130) target_tick = distance_cm * 60.3; 
  else if(distance_cm<=140) target_tick = distance_cm * 60.5; 
  else if(distance_cm<=150) target_tick = distance_cm * 60.5; 
  else if(distance_cm<=160) target_tick = distance_cm * 60.5; 
  else if(distance_cm<=170) target_tick = distance_cm * 60.5; 
  else target_tick = distance_cm * 60.5;
  
  Serial.print("Distance_cm: ");
  Serial.println(distance_cm);
  Serial.print("target tick: ");
  Serial.println(target_tick);
  
  while (encoder_right < 100)
  {
    compensation = tunePID();
    md.setSpeeds(100 + compensation, 100 - compensation);
  }
  
  while (encoder_right < 200)
  {
    compensation = tunePID(); 
    md.setSpeeds(200 + compensation, 200 - compensation);
  }

  while (encoder_right < 250)
  {
    compensation = tunePID();
    md.setSpeeds(300 + compensation, 300 - compensation);
  }

  while (encoder_right < target_tick - 200)
  {
    compensation = tunePID();
    md.setSpeeds(left_straight_speed + compensation, right_straight_speed - compensation);
  }
  
  while (encoder_right < target_tick - 100)
  {
    compensation = tunePID();
    md.setSpeeds(200 + compensation, 200 - compensation);
  }

  while (encoder_right < target_tick)
  {
    compensation = tunePID();
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

void move_backward_ramp_up (int distance_cm) {
  encoder_left = 0;
  encoder_right = 0;
  double compensation = 0;
  error = 0.0;
  integralError = 0.0;
  //target_tick = distance_cm * 58.5;
  if (distance_cm<= 10) target_tick = distance_cm * 58.3;
  else if(distance_cm<=20) target_tick = distance_cm * 58.3;
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
  else if(distance_cm<=150) target_tick = distance_cm * 72.5;
  else if(distance_cm<=160) target_tick = distance_cm * 72.5;
  else if(distance_cm<=170) target_tick = distance_cm * 72.5;
  else target_tick = distance_cm * 72.5;
  
  Serial.print("Distance_cm: ");
  Serial.println(distance_cm);
  Serial.print("target tick: ");
  Serial.println(target_tick);
  
  while (encoder_right < 100)
  {
    compensation = tunePID();
    md.setSpeeds(-(100 + compensation), -(100 - compensation));
  }
  
  while (encoder_right < 200)
  {
    compensation = tunePID(); 
    md.setSpeeds(-(200 + compensation), -(200 - compensation));
  }

  while (encoder_right < 250)
  {
    compensation = tunePID();
    md.setSpeeds(-(300 + compensation), -(300 - compensation));
  }

  while (encoder_right < target_tick - 200)
  {
    compensation = tunePID();
    md.setSpeeds(-(left_straight_speed + compensation), -(right_straight_speed - compensation));
  }
  
  while (encoder_right < target_tick - 100)
  {
    compensation = tunePID();
    md.setSpeeds(-(200 + compensation), -(200 - compensation));
  }

  while (encoder_right < target_tick)
  {
    compensation = tunePID();
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

void rotate_left(int angle) {
  encoder_left = 0;
  encoder_right = 0;
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
  
  md.setBrakes(left_brake_speed, right_brake_speed); 
  delay(80);
  md.setBrakes(0, 0);
}

void rotate_right(int angle) {
  encoder_left = 0;
  encoder_right = 0;
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
  
  md.setBrakes(left_brake_speed, right_brake_speed); 
  delay(80);
  md.setBrakes(0, 0);
}


// rising left and right will capture the increase in the encoder values
// that will be used to tune with PID then translate into the distance
void left_encoder_rising () {
  encoder_left++;
}

void right_encoder_rising () {
  encoder_right++;
}

/////////////////////////////////////////////////////////////////////////
///////////////////////////////PID TUNING////////////////////////////////
/////////////////////////////////////////////////////////////////////////

double tunePID () {
  double compensation, pervious_encoder_right;
  double Kp, Ki, Kd, p, i, d;
  // Okay at HPL
//  Kp = 50;
//  Ki = 0.1;
//  Kd = 0.01;
  // Okay at HWL2
  Kp = 53; // increase in case it is going left, decrease in case it is going right
  Ki = 0.1;
  //Kd = 0.01;
  error = encoder_right - encoder_left;
  integralError += error;
  p = error * Kp;
  i = integralError * Ki;
  //d = (pervious_encoder_right - encoder_right) * Kd;
  //compensation = p + i + d;
  compensation = p + i;
  pervious_encoder_right = encoder_right;  
  return compensation;
}
