#define VOLTAGE_READER_PIN 4
// Battery level reading that corresponds to a low battery reading
#define LOW_BATTERY_THRESHOLD 2250

int MAX_FSPEED = 220;
int MAX_RSPEED = 162;

void Navigation_Init() {
  pinMode(VOLTAGE_READER_PIN, INPUT);
  analogReadResolution(12);
}

// In this function, remember that x is longitude and y is lattitude!!
Motor_Data Calculate_Motor_Data(double x0, double y0, double x1, double y1, double dir) {
  double xc = x1 - x0;
  double yc = y1 - y0;
  double c_angle = atan2(yc, xc);
  double ang_diff = Parse_Angle(c_angle - dir);
  int motor_diff = (MAX_FSPEED - MAX_RSPEED) * abs(ang_diff)/PI;
  
  Motor_Data data;
  // Calculates motor values for appropriate tragectory
  if (ang_diff > 0) {
    // Turn left
    data.left = MAX_FSPEED - motor_diff;
    data.right = MAX_FSPEED;
  } else {
    // Turn right
    data.right = MAX_FSPEED - motor_diff;
    data.left = MAX_FSPEED;
  }

  return data;
} 

bool Is_Battery_Low() {
  int batteryLevel = analogRead(VOLTAGE_READER_PIN);
  return batteryLevel <= LOW_BATTERY_THRESHOLD;
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
