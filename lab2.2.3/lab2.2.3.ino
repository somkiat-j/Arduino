int age = 0;

void setup() {
    Serial.begin(115200);

    Serial.println("Please enter your age:");
}

void loop() {
    if (Serial.available() > 0) {
        age = Serial.parseInt();

        Serial.print("Your age is: ");
        Serial.println(age);

        Serial.println("Please enter your age again:");
    }
}
