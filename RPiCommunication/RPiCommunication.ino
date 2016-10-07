char command[32];
int n;
void setup() {
  // initialize digital pin 13 as an output.
  Serial.begin(115200);
  pinMode(12, OUTPUT);
}
void loop(){
//if (Serial.available()) {
//  light(Serial.read()-'0');
//}
//delay(500);
Serial.println("Hello");
//light(3);
}

void light(int n){
  for (int i = 0; i < n; i++) {
    digitalWrite(12, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);              // wait for a second
    digitalWrite(12, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);              // wait for a second
  }
}

int getRPiMessage(){
  //memset(n, 0, sizeof(n));
  n = Serial.read()-'0';
  return n;
}

void setRPiMessage(int left, int right, int center_bot, int center_left, int center_right){
  Serial.print("A2PC|");
  Serial.print(left);
  Serial.print(":");
  Serial.print(right);
  Serial.print(":");
  Serial.print(center_bot);
  Serial.print(":");
  Serial.print(center_left);
  Serial.print(":");
  Serial.print(center_right);
  Serial.print("\0");
  Serial.print("\n");
  Serial.flush();
}

