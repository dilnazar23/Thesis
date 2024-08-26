// command:
// S,750,750,750,750,\n
void setup()
{ 
  // MYSERIAL.print is used to allow compatability between both the Mega (Serial.print) 
  // and the Zero Native Port (SerialUSB.print), and is defined in FingerLib.h
  Serial.begin(9600);
  Serial.println("Started");

  // configure finger pins
  //finger.attach(1, 2, A0);
  Serial.println("Pins configured");
}

void loop()
{       
    if (Serial.available() > 0) {
        char start_byte = Serial.read();
        if (start_byte == 'S'){
            Serial.println(F("Receive: "));            
            Serial.read(); // remove the comma
            String income_data = "";
            int data[4];
            int i = 0;
            while(Serial.available() > 0){
                char income_byte = Serial.read();
                if (income_byte == '\n'){
                    break;
                }
                else if(income_byte == ','){
                    i = i % 4;
                    data[i] = income_data.toInt();
                    i++;
                    income_data = "";
                    continue;
                }
                else{
                    income_data += income_byte;
                }
                Serial.print(income_byte);
                Serial.print(" ");                
            }
            
            Serial.println("This is the data array:");
            for (int i = 0; i < 4; i++){
                Serial.print(data[i]);
                Serial.print(" ");
            }
            //moveFingers(incomeBytes);
        }
    }


}
