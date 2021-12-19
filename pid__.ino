#include <Servo.h>

#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

#define _DIST_TARGET 255 //pd 205 255
#define _DIST_MIN 100
#define _DIST_MAX 410
#define _DIST_ALPHA 0.1

#define _DUTY_MIN 1000
#define _DUTY_NEU 1490
#define _DUTY_MAX 2000

#define _SERVO_ANGLE 30
#define _SERVO_SPEED 100 //100

#define _INTERVAL_DIST 20 
#define _INTERVAL_SERVO 20 
#define _INTERVAL_SERIAL 100 

#define _KP 2.6 //pd 1.2
#define _KD 100//pd 25
#define _KI 0.02
#define N_SAMPLES 100
#define _ITERM_MAX 10 //10
Servo myservo;

float dist_target; 
float dist_raw, dist_ema;

unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

int duty_chg_per_interval; 
int duty_target, duty_curr;
float error_curr, error_prev, control, pterm, dterm, iterm;

void setup() {
  Serial.begin(57600);
  myservo.attach(PIN_SERVO);
  pinMode(PIN_LED, OUTPUT);

  error_curr = error_prev = 0.0;
  dist_target = _DIST_TARGET ;
  myservo.writeMicroseconds(_DUTY_NEU);
  delay(300);

  duty_target = duty_curr = _DUTY_NEU;
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / _SERVO_ANGLE) * (_INTERVAL_SERVO / 1000.0);
  

}

void loop() {

  // Event generator //

  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
      last_sampling_time_dist += _INTERVAL_DIST;
      event_dist = true;
  }
  
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO){
      last_sampling_time_servo += _INTERVAL_SERVO;
      event_servo = true;
  }
  
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL){
      last_sampling_time_serial += _INTERVAL_SERIAL;
      event_serial = true;
  }

  // Event handlers //

  if(event_dist) {
    event_dist = false;
    dist_raw = ir_distance_filtered();

    error_curr = _DIST_TARGET - dist_raw;
    pterm = _KP * error_curr;
    dterm = _KD * (error_curr - error_prev);
    iterm += _KI * error_curr;
    control = pterm + dterm + iterm;
    
    duty_target = _DUTY_NEU + control;
    
    if(iterm > _ITERM_MAX) iterm = _ITERM_MAX;
    if(iterm < - _ITERM_MAX) iterm = - _ITERM_MAX;
    
    
    if(duty_target < _DUTY_MIN) duty_target = _DUTY_MIN;
    if(duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;
    
    error_prev = error_curr;
  }
  
  if(event_servo){
    event_servo = false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_curr > duty_target){
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    } else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }

    // update servo position
    myservo.writeMicroseconds(duty_curr);
  }
  
  if(event_serial) {
    event_serial = false;
    /*
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",dterm:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
    */
    
    Serial.print("IR:");
    Serial.print(dist_raw);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
    
    }
}

float ir_distance(){ // return value unit: mm
  float value, volt = float(analogRead(PIN_IR));
  value = ((6762.0 / (volt - 9.0)) - 4.0) * 10.0;
  return value;
}

float ir_distance2(void){ // return value unit: mm
  float ir_val[N_SAMPLES];
  float tmp;

  // take N_SAMPLES and sort them in an ascending order.
  ir_val[0] = float(analogRead(PIN_IR));
  for (int i=1; i<N_SAMPLES; i++) { 
    delayMicroseconds(10);
    ir_val[i] = float(analogRead(PIN_IR));
    for(int j = i-1; j >= 0; j--) { // i-1 to 0
      if(ir_val[j] > ir_val[j+1]) {
        tmp = ir_val[j];
        ir_val[j] = ir_val[j+1];
        ir_val[j+1] = tmp;
      }
    }
  }

  int cnt = N_SAMPLES/3;
  tmp = 0.0;
  for(int i = 0; i < cnt; i++) {
      tmp += ir_val[i];
  }
  tmp = tmp / (float) cnt;
  tmp = ((6762.0/(tmp-9.0))-4.0) * 10.0;

  return tmp;
}
float ir_distance_filtered(){
  return dist_ema = _DIST_ALPHA * ir_distance() + (1 - _DIST_ALPHA) * dist_ema;
}
