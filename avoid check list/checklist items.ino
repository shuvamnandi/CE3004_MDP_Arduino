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
