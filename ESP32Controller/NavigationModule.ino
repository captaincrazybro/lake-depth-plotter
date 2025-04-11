int MAX_SPEED = 230;

// In this function, remember that x is longitude and y is lattitude!!
Motor_Data Calculate_Motor_Data(double x0, double y0, double x1, double y1, double dir) {
  double xc = x1 - x0;
  double yc = y1 - y0;
  double c_angle = atan2(yc, xc);
  double ang_diff = Parse_Angle(c_angle - dir);
  int motor_diff = MAX_SPEED/2 * abs(ang_diff)/PI;
  
  Motor_Data data;
  // Calculates motor values for appropriate tragectory
  if (ang_diff > 0) {
    // Turn left
    data.left = MAX_SPEED - motor_diff;
    data.right = MAX_SPEED;
  } else {
    // Turn right
    data.right = MAX_SPEED - motor_diff;
    data.left = MAX_SPEED;
  }

  return data;
} 

double Parse_Angle(double ang) {
  if (ang > PI) {
    return ang-2*PI;
  } else if (ang < -PI) {
    return 2*PI + ang;
  } else {
    return ang;
  }
}
