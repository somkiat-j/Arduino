const int BT = 17;
const int WIFI = 2;
const int NTP = 15;
const int IOT = 12;

void setup() {
  Serial.begin(115200);
  pinMode(BT, OUTPUT);
  pinMode(WIFI, OUTPUT);

  digitalWrite(BT, HIGH); //ดับ
  digitalWrite(WIFI, HIGH); //ดับ
}

void loop() {
   if(Serial.read() == 'a'){
      digitalWrite(BT, LOW);
   }

  if(Serial.read() == 's'){
      digitalWrite(WIFI, LOW);
   }

}
