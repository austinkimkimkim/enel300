#include <LiquidCrystal.h>   // Built in Arduino library for the lcd

// inputs for the lcd on some type shit
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

// ultrasonic pins 
const int TRIG = 2;   // this the Output, sends a short pulse to trigger a measurement
const int ECHO = 3;   // this the Input, sensor outputs a pulse whose width = the time it took/ time of flight (what it do flight crew FTC)

// read the distance and return it as cms
// Returns -1 if no echo was received 
float readDistanceCm() {
  // Ensure a clean trigger pulse:
  // Pull TRIG low so if the last loop has high it wont bug out, short delay as well same thing for the second time we run it
  // Pulse TRIG high for 10 microseconds which is essentially the actual signal
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  // Measure how long ECHO stays HIGH this is in microseconds
  // The sensor sets ECHO HIGH while the ultrasound travels there and back
  // timeout 30000 us prevents the program from blocking too long
  long us = pulseIn(ECHO, HIGH, 30000);

  // If pulseIn timed out it returns 0 basically “no valid reading”
  if (us == 0) return -1;

  // Convert the echo from (us) to distance (cm)
  // speed of sound = 0.0343 cm/us kakaw kakaw kakaw (echo btw)
  // Divide by 2 because the pulse time includes the trip to the object and back 
  return us * 0.0343f / 2.0f;
}

void setup() {
  // sensor pins
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  //the LCD as a 16x2 display
  lcd.begin(16, 2);
  lcd.clear();

  // Print a message on the first row
  lcd.setCursor(0,0);
  lcd.print("Distance:");
}

void loop() {
  // Smoothing to reduce jittering 
  // takes 5 samples ignores invalid ones then average the rest.
  float sum = 0;
  int good = 0;

  for (int i = 0; i < 5; i++) {
    float d = readDistanceCm();

    // take only readings below 200 cm and above 0
    if (d > 0 && d < 200) {
      sum += d;
      good++;
    }

    // small delay between samples so we not spamming the sensor
    delay(20);
  }

  // update the second message or row of the LCD
  lcd.setCursor(0,1);

  if (good == 0) {
    // if no valid readings we show an error message
    lcd.print("No echo        ");
  } else {
    //average distance from valid samples
    float d = sum / good;

    // we print the "message" on the lcd which is the distance and "cm"
    lcd.print(d, 1);
    lcd.print(" cm            ");
  }

  // delay so it dont explode 
  delay(150);
}