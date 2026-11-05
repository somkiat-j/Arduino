#define LED_PIN  17
#define SW1 16

void setup() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(SW1, INPUT_PULLUP);
}

void loop() {
    if (digitalRead(SW1) == LOW) {
        digitalWrite(LED_PIN, LOW);
    } else {
        digitalWrite(LED_PIN, HIGH);
    }
}