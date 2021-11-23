// Arduino pin assignment
#include <Servo.h>
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10
Servo myServo;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 0);
  myServo.attach(PIN_SERVO);
  myServo.write(90);
  delay(2000);
// initialize serial port
  Serial.begin(57600);

}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  Serial.print("min:0,max:500,dist:");
  Serial.println(raw_dist);
  delay(20);
  
  if (raw_dist < 255) {
    myServo.write(140);
    delay(800);
  }
  else if (raw_dist > 255) {
    myServo.write(50);
    delay(800);
  }
  
}
