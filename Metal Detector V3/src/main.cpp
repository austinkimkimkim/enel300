#include <Arduino.h>

#define CYCLES_PER_SIGNAL 5000
#define LED_THRESHOLD 700
#define SENSITIVITY_POT_APIN A1
#define LED_PIN 9
#define SERIAL_PRINT_INTERVAL 200

volatile unsigned long lastSignalTime = 0;
volatile unsigned long signalTimeDelta = 0;
volatile boolean firstSignal = true;
volatile unsigned long storedTimeDelta = 0;
unsigned long lastPrintTime = 0;

float mapFloat(int input, int inMin, int inMax, float outMin, float outMax);

//Every pulse cycle
ISR(TIMER1_COMPA_vect)
{
  unsigned long currentTime = micros();
  signalTimeDelta = currentTime - lastSignalTime;
  lastSignalTime = currentTime;

  if (firstSignal)
  {
    firstSignal = false;
  }
  else if (storedTimeDelta == 0)
  {
    //Baseline time difference
    storedTimeDelta = signalTimeDelta;
  }

  OCR1A += CYCLES_PER_SIGNAL;
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Calibrating");

  // Timer setup
  TCCR1A = 0b00000000; 
  TCCR1B = 0b00000111; 
  TIMSK1 |= (1 << OCIE1A);
  OCR1A = 1;

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void loop()
{
  float sensitivity = mapFloat(analogRead(SENSITIVITY_POT_APIN), 0, 1023, 0.5, 10.0);
  int storedTimeDeltaDifference = (storedTimeDelta - signalTimeDelta) * sensitivity;

  digitalWrite(LED_PIN, storedTimeDeltaDifference > LED_THRESHOLD ? HIGH : LOW);

  unsigned long now = millis();
  if (now - lastPrintTime >= SERIAL_PRINT_INTERVAL)
  {
    lastPrintTime = now;
    if (storedTimeDelta == 0)
    {
      Serial.println("Calibrating");
    }
    else
    {
      Serial.print("Baseline: ");   Serial.print(storedTimeDelta);
      Serial.print("  |  Current: ");  Serial.print(signalTimeDelta);
      Serial.print("  |  Diff: ");     Serial.print(storedTimeDeltaDifference);
      Serial.print("  |  Sensitivity: "); Serial.print(sensitivity, 1);
      Serial.print("  |  Metal: ");
      Serial.println(storedTimeDeltaDifference > LED_THRESHOLD ? "YES" : "no");
    }
  }
}

float mapFloat(int input, int inMin, int inMax, float outMin, float outMax)
{
  float scale = (float)(input - inMin) / (inMax - inMin);
  return ((outMax - outMin) * scale) + outMin;
}