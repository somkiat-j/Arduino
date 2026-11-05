int count;

void setup() {
  Serial.begin(9600);
  Serial.println("Mr.Somkiat Jaidee");
  Serial.println("-----------------");
  delay(1000);
}

void loop() {
  count+=1;
  Serial.println("Hello World" + count);
  delay(1000);
}
