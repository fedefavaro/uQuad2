
uint8_t buff[25];

void setup() {
  Serial.begin(115200);
  Serial1.begin(10000);
  for(int i=0;i<25;i++) {
    buff[i]=i*0x0F+0xAA;
  }
}

void loop() {
  
  
  Serial1.write(buff,25);
  for(int i=0;i<25;i++) {
    Serial.print(buff[i],HEX);
    Serial.print(" ");
  }
  Serial.println();
  delay(10);
  
  
}
