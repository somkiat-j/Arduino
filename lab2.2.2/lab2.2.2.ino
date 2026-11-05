const int MAX_SCORE = 100;
const float PI_VALUE = 3.14159;

void setup() {
    Serial.begin(115200);

    Serial.print("Maximum Score = ");
    Serial.println(MAX_SCORE);

    Serial.print("PI = ");
    Serial.println(PI_VALUE, 2);
}

void loop() {
}