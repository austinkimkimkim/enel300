#include <Arduino.h>

const int TRIG_PIN = 9;
const int ECHO_PIN = 10;

const int WINDOW_SIZE = 10;
float readings[WINDOW_SIZE];
int readIndex = 0;
float total = 0;

// Initialize pins and moving average array
void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  for (int i = 0; i < WINDOW_SIZE; i++) readings[i] = 0;
}

//Get distance of sensor. Distance in cm
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  return (duration * 0.0343) / 2.0;
}

//Compute moving average value
//Add new distance to array and remove oldest reading
//Compute the average of array
float movingAverage(float newReading) {
  total -= readings[readIndex];
  readings[readIndex] = newReading;
  total += newReading;
  readIndex = (readIndex + 1) % WINDOW_SIZE;
  return total / WINDOW_SIZE;
}

void loop() {
  float dist = getDistance();
  float smoothed = movingAverage(dist);
  Serial.println(smoothed);
  delay(100);
}