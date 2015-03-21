#include <FutabaSBUS.h>
#include <Arduino.h>
#include <DueTimer.h>
#include <stdbool.h>

FutabaSBUS sbus;
int i;
unsigned long timeOut;
unsigned long aux;
char in;

typedef enum {STOPPED,RUNNING} state_t;
state_t actual_state = STOPPED;
int vel = 0;
uint8_t buffIndex = 0;
char strBuff[10];

void setup(){
  
  sbus.begin();  
  sbus.setPassthrough(false);

  Serial.begin(115200);
  Serial.println("Presione R para comenzar y S para detener");
  Serial.println("Presione V para ver la velocidad seteada");
  Serial.println("Para variar la velocidad ingrese el numero directamente seguido del caracter !");

  rxSBUS.attachInterrupt(myHandler);
  rxSBUS.start(timerCount);
  
  for(i=1; i<17; i++) 
  {
    sbus.servo(i,0);
  }
  
}

void myHandler(void) 
{
  sbus.ticker_500us();
}

void loop(){
  
     if(Serial.available()) {
          buffIndex = 0;
          in = Serial.read();
          strBuff[buffIndex] = in;
          strBuff[buffIndex+1] = '\0';
          if (in == 'R') {
                Serial.println("RUNNING"); //user feedback
                actual_state = RUNNING;
          } else if (in == 'S') {
                Serial.println("STOPPED"); //user feedback
                actual_state = STOPPED;
                vel = 0;
          } else if (in == 'V') {
                Serial.print("Velocidad: ");
                Serial.println(vel);      
          } else if ((atoi(strBuff) > 0) && (atoi(strBuff) < 10)) {
                timeOut = micros();
                while(strBuff[buffIndex] != '!') {
                   if (Serial.available() > 0) 
                      strBuff[++buffIndex] = Serial.read();
                   if (buffIndex > 4) {
                      Serial.println("el numero debe ser menor a cuatro cifras...");
                      goto usage;
                   }
                   if (micros() - timeOut > 800) {
                      Serial.println("timeout...");
                      goto usage; //descarto valor de velocidad
                   }
                }
                strBuff[++buffIndex] = '\0';
                vel = atoi(strBuff);
                Serial.print("Velocidad: ");
                Serial.println(vel);
          } else {
             usage:
             Serial.println("Presione R para comenzar y S para detener");
             Serial.println("Presione V para ver la velocidad seteada");
             Serial.println("Para variar la velocidad ingrese un numero de hasta 4 cifras seguido del caracter !");
          }
       }
    
    
    if (actual_state == RUNNING) {
       sbus.servo(0,vel);
       sbus.servo(1,vel);
       sbus.servo(2,vel);
       sbus.servo(3,vel);
       sbus.servo(4,vel);
       sbus.servo(5,vel);
/*     for(i=0; i<100; i++) 
       {

          sbus.servo(1,i*20);
          delay(20); //miliseconds

       }

       for(i=100; i>0; i--)
       {

          sbus.servo(1,i*20);
          delay(20);

       }
*/     
    }
}



