
/*
Configurar:
stty -F /dev/ttyUSB0 cs8 115200 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts

Escribir:
echo "Hello Arduino!" >> /dev/ttyUSB0

Leer(otro terminal):
tail -f /dev/ttyUSB0
*/



#define TIMEOUT         3000
#define LENGTH          64 


long  previousMillis = 0;
int   bread          = 0;
char  c;
bool  msgReceived    = false;
bool  timeout        = false;
char buffer[LENGTH+1];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {

  
  if(Serial.available()>0) 
  {
    bread = 0;
    previousMillis = micros();
    while(!timeout)
    {
        if(Serial.available()>0)
        {
            c = Serial.read();
            if(c == '!')
            {
                msgReceived = true;
                buffer[bread]='\r\n';
                bread++;
                //Serial.println("Message complete"); //debug
                break;
            }
            if(bread>LENGTH-1)
            {
                Serial.println("String received is too long");
                break;
            }
            else
            {
                buffer[bread]=c;
            }
            bread++;
        }
        timeout=((micros()-previousMillis)>TIMEOUT);
    }
    if(timeout)
    {
         Serial.println("Se debe finalizar el mensaje con el caracter '!'");
         timeout = false;
    }
    
  }

  if(msgReceived == true)
  {
     Serial.write(buffer, bread);
     msgReceived = false;
  }
    
  delay(100);
}
