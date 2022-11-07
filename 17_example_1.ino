#include <Servo.h>

// Arduino pin assignment

#define PIN_POTENTIOMETER 0 // Potentiometer at Pin A3
#define PIN_LED 9
// Add IR Sensor Definition Here !!!
#define PIN_SERVO 10

#define _DUTY_MIN 553  // servo full clock-wise position (0 degree)
#define _DUTY_NEU 1476 // servo neutral position (90 degree)
#define _DUTY_MAX 2399 // servo full counter-clockwise position (180 degree)

#define LOOP_INTERVAL 50   // Loop Interval (unit: msec)
#define _DIST_MIN 100
#define _DIST_MAX 250

#define _EMA_ALPHA 0.45

Servo myservo;
unsigned long last_loop_time;   // unit: msec
float dist_prev = _DIST_MAX;
float dist_ema;

void setup()
{
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);
  pinMode(PIN_LED, OUTPUT);
  
  Serial.begin(1000000);
}

void loop()
{
  unsigned long time_curr = millis();
  int a_value, duty;
  float dist_raw;
  float dist;

  // wait until next event time
  if (time_curr < (last_loop_time + LOOP_INTERVAL))
    return;
  last_loop_time += LOOP_INTERVAL;

  // Remove Next line !!!
  // a_value = analogRead(PIN_POTENTIOMETER);
  // Read IR Sensor value !!!
  a_value = analogRead(PIN_POTENTIOMETER);
  // Convert IR sensor value into distance !!!
  dist_raw = (6762.0/(a_value-9)-4.0)*10.0-100.0;
  dist = dist_raw;
  // we need distance range filter here !!!
if (dist_raw < _DIST_MIN) {
    dist_raw = dist_prev;    // Set Lower Value
    digitalWrite(PIN_LED, 1);       // LED OFF
  } else if (dist_raw > _DIST_MAX) {
    dist_raw = dist_prev;    // Set Higher Value
    digitalWrite(PIN_LED, 1);       // LED OFF
  } else {
    dist_prev = dist_raw;
    digitalWrite(PIN_LED, 0);       // LED ON      
  }
  // we need ema filter here !!!
  dist_ema = _EMA_ALPHA * dist_raw + (1 - _EMA_ALPHA) * dist_ema;
  dist_prev = dist;

  // map distance into duty
  // duty = map(a_value, 0, 1023, _DUTY_MIN, _DUTY_MAX);
   duty = ((dist_ema - 100.0)/(250.0-100.0))*(_DUTY_MAX - _DUTY_MIN)+_DIST_MIN;
  myservo.writeMicroseconds(duty);

  // print IR sensor value, distnace, duty !!!
  Serial.print("ADC Read: "); Serial.print(a_value);
  Serial.print(",IR: "); Serial.print(dist);
  Serial.print(",dist: "); Serial.print(dist_raw);
  Serial.print(",ema: "); Serial.print(dist_ema);
  Serial.print(",duty: "); Serial.print(duty);
  Serial.print(" = ");
  Serial.print((a_value / 1024.0) * 5.0);
  Serial.print(" Volt => Duty : ");
  Serial.print(duty);
  Serial.println("usec");
}
