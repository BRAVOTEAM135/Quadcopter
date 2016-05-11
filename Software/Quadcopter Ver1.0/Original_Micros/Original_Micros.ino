void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
unsigned long a = micros();
delay(500);
unsigned long b = micros();

 Serial.print(b);
 Serial.print(" - ");
 Serial.print(a);
 Serial.print(" = ");
  Serial.println(b - a);

}
