int epin = 6;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial.println("listening");

  pinMode(epin,OUTPUT);
}

void loop() {

  if(Serial1.available() > 0 ){
    while(Serial1.available() >0 ){
      Serial.write(Serial1.read());
      //Serial.println();
    }
  }
  digitalWrite(epin,HIGH);
  delay(100);
  Serial1.write("///SN=?\r\n");
  delay(10);
  digitalWrite(epin,LOW);
  delay(500);
}
