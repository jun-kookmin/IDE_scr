#define PIN7 7
int i = 0;
void setup() {
  pinMode(PIN7, OUTPUT);

  digitalWrite(PIN7, LOW);
    delay(1000);
  
}

void loop() {
while(1) {
    digitalWrite(PIN7, HIGH);
    delay(200);
    digitalWrite(PIN7, LOW);
    delay(200);
    i += 1;
    if (i == 5)
      break;
  }
digitalWrite(PIN7, HIGH);
exit(0);
    
  
}
