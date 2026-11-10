int BT = 17;
int BZ = 13;
void setup() {
    Serial.begin(115200);
    pinMode(BT, OUTPUT);
    pinMode(BZ, OUTPUT);
    int count = 1;
    while (count <= 5) {
        digitalWrite(BT, LOW);  //ติด
        analogWrite(BZ, 255);  //ติด   0-255
        delay(500);
        digitalWrite(BT, HIGH);  //ดับ
        analogWrite(BZ, 0);  //ดับ 
        delay(500);
        Serial.println(count);
        count++;
    }
}

void loop() {
}
