
String inputString = "";        
boolean stringComplete = false; 

void setup() {
  
  Serial1.begin(115200);
 
  inputString.reserve(60); //segun mis calculos necesito 53 bytes
}

void loop() {

  if (stringComplete) {
  
    GetString();
	//coso
	
	// free 
    inputString = "";
    stringComplete = false;
  }
}


void GetString() {
	char inChar;
	bool start = false;
    while ((Serial1.available()) && (stringComplete == false;)) {
	
	
	
    // get the new byte:
    inChar = (char)Serial1.read();
	
	//new string
	if (inChar == 'A')
		start = true;
		
    // add it to the inputString:
	if (strat){
	
		inputString += inChar;
		// end of string
		if (inChar == 'Z') 
			stringComplete = true;
    
	}
  }
  return;
}

