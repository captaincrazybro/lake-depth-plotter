#define RIGHT_TRIG_PIN 32
#define RIGHT_ECHO_PIN 34
#define LEFT_TRIG_PIN 33
#define LEFT_ECHO_PIN 35
// Defines the minimum object detection range
#define MIN_DET_RANGE 150

int leftCount = 0;
int rightCount = 0;

void Object_Detection_Init() {
  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);
  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(LEFT_ECHO_PIN, INPUT);
}

int Get_Right_Distance() {
  // Clears the right trig pin
  digitalWrite(RIGHT_TRIG_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(RIGHT_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(RIGHT_TRIG_PIN, LOW);
  // Reads the echo pin, returns the sound wave travel time in microseconds
  long duration = pulseIn(RIGHT_ECHO_PIN, HIGH);
  // Calculating the distance
  return duration * 0.034 / 2;
}

int Get_Left_Distance() {
  // Clears the right trig pin
  digitalWrite(LEFT_TRIG_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(LEFT_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(LEFT_TRIG_PIN, LOW);
  // Reads the echo pin, returns the sound wave travel time in microseconds
  long duration = pulseIn(LEFT_ECHO_PIN, HIGH);
  // Calculating the distance
  return duration * 0.034 / 2;
}

bool Right_Obstacle_Detected() {
  int distance = Get_Right_Distance();
  if (distance <= MIN_DET_RANGE) {
    rightCount++;
    if (rightCount >= 3) return true;
  } else {
    rightCount = 0;
  }
  return false;
}

bool Left_Obstacle_Detected() {
  int distance = Get_Left_Distance();
  if (distance <= MIN_DET_RANGE) {
    leftCount++;
    if (leftCount >= 3) return true;
  } else {
    leftCount = 0;
  }
  return false;
}