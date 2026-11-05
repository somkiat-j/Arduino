int age = 0;
const int BT = 17;
void setup() {
    pinMode(BT, OUTPUT);
    Serial.begin(115200);
    Serial.println("Please enter your age:");
    digitalWrite(BT,HIGH);
}

void loop() {
    if (Serial.available() > 0) {
        digitalWrite(BT, LOW);
        age = Serial.parseInt();

        Serial.print("Your age is: ");
        Serial.println(age);

        Serial.println("Please enter your age again:");
    }
}
