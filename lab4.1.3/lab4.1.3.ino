void setup() {
    Serial.begin(115200);
    for (int count = 1; count <= 10; count++) {
        if(count == 5){
            break;
        }
        Serial.println(count);
    }
    Serial.println("-----------------");

    for (int count = 10; count >= 0; count--) {
        if(count == 7){
            continue;
        }
        Serial.println(count);
    }
}

void loop() {
}