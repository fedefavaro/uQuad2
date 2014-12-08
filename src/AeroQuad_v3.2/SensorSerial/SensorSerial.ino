
char incomingByte;   // for incoming serial data
int cont = 0;
void setup() {
        Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
        Serial1.begin(9600); 
}

void loop() {

        // send data only when you receive data:
        if (Serial1.available() > 0) {
                // read the incoming byte:
                incomingByte = Serial1.read();
                if(incomingByte ==
                'R'){
                    cont = 0;
                }
                Serial.print(incomingByte);
                cont++;
                if(cont==4){
                    Serial.println();
                }
        }
        
}
