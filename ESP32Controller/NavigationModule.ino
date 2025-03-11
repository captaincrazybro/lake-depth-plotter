int MAX_SPEED = 230;

Motor_Data Calculate_Motor_Data(double x0, double y0, double x1, double y1, double dir) {
  double xc = x1 - x0;
  double yc = y1 - y0;
  double c_angle = atan2(yc, xc)*180.0f/PI;
  double ang_diff = Parse_Angle(c_angle - dir);
  int motor_diff = MAX_SPEED/2 * abs(ang_diff)/180.0f;
  
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
  if (ang > 180.0f) {
    return ang-360.0f;
  } else if (ang < -180.0f) {
    return 360.0f + ang;
  } else {
    return ang;
  }
}
