#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <QMC5883LCompass.h>

// Compass instance
QMC5883LCompass compass;

static const int GPS_RXPin = 7, GPS_TXPin = 8;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial gps_ss(GPS_RXPin, GPS_TXPin);

void GPS_Init() {
  gps_ss.begin(GPSBaud);
  Serial.println("Initialized GPS Module!");
  compass.init();
  // TODO: Add calibration code once done
}

void GPS_Update() {
  while (gps_ss.available() > 0){
    gps.encode(gps_ss.read());
    if (gps.location.isUpdated()) {
      Serial.print("Successful GPS Reading: ");
      Serial.print(gps.location.lat());
      Serial.print(", ");
      Serial.println(gps.location.lng());
    }
  }
}

double Get_Current_Tragectory() {
  // Collects four sets of points
  double past_points[4][2];
  while (!gps.location.isUpdated()) { GPS_Update(); };
  past_points[0][0] = gps.location.lat();
  past_points[0][1] = gps.location.lng();
  delay(300);
  while (!gps.location.isUpdated()) { GPS_Update(); };
  past_points[1][0] = gps.location.lat();
  past_points[1][1] = gps.location.lng();
  delay(300);
  while (!gps.location.isUpdated()) { GPS_Update(); };
  past_points[2][0] = gps.location.lat();
  past_points[2][1] = gps.location.lng();
  delay(300);
  while (!gps.location.isUpdated()) { GPS_Update(); };
  past_points[3][0] = gps.location.lat();
  past_points[3][1] = gps.location.lng();
  
  // Use Least Squares Method to Calcualte angle
  double lat_avg = (past_points[0][0] + past_points[1][0] + past_points[2][0] + past_points[3][0])/4.0;
  double lng_avg = (past_points[0][1] + past_points[1][1] + past_points[2][1] + past_points[3][1])/4.0;
  double m_num = (past_points[0][0] - lat_avg)*(past_points[0][1] - lng_avg) + (past_points[1][0] - lat_avg)*(past_points[1][1] - lng_avg) + (past_points[2][0] - lat_avg)*(past_points[2][1] - lng_avg) + (past_points[3][0] - lat_avg)*(past_points[3][1] - lng_avg);;
  double m_den = (past_points[0][0] - lat_avg)*(past_points[0][0] - lat_avg) + (past_points[1][0] - lat_avg)*(past_points[1][0] - lat_avg) + (past_points[2][0] - lat_avg)*(past_points[2][0] - lat_avg) + (past_points[3][0] - lat_avg)*(past_points[3][0] - lat_avg);;
  double m = m_num/m_den;
  double ang = atan(m);
  // Corrects for atan error
  double lat_diff = past_points[3][0] - past_points[0][0];
  double lng_diff = past_points[3][1] - past_points[0][1];
  if (lat_diff < 0 && lng_diff > 0) {
    ang += PI;
  } else if (lat_diff < 0 && lng_diff < 0) {
    ang -= PI;
  }
  // Other angle algorithm
  return ang;
}

float Compass_Get_Trajectory() {
  return compass.getAzimuth();
}

TinyGPSPlus Get_GPS() {
  return gps;
}
