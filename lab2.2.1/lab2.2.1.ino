int age = 25;
float temperature = 28.5;
char grade = 'A';
bool ledStatus = true;
int counter = 0;

void setup() {
  Serial.begin(115200);

  Serial.println("=== Variable Example ===");

  Serial.print("Age = ");
  Serial.println(age);

  Serial.print("Temperature = ");
  Serial.println(temperature);

  Serial.print("Grade = ");
  Serial.println(grade);

  Serial.print("LED Status = ");
  Serial.println(ledStatus);
}

void loop() {
    //counter = counter + 1;
    counter+=1;
    //counter++;
    //counter--;


    Serial.print("Counter = ");
    Serial.println(counter);

    delay(1000);
}
