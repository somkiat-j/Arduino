const int BT = 17;
const int WIFI = 2;

void setup() {
  Serial.begin(115200);
  pinMode(BT, OUTPUT);    //กำหนดค่าให้เป็น OUTPUT
  pinMode(WIFI, OUTPUT);
   
  digitalWrite(BT, HIGH); //ให้ LED BT ดับก่อน
  digitalWrite(WIFI, HIGH);
}

void loop() {
  char ch = Serial.read();
  if(ch == 'a'){
     digitalWrite(BT, LOW);
  }

  if(ch == 'w'){
     digitalWrite(WIFI, LOW);
  }

}
