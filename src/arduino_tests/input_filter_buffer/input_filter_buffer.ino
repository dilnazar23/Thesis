#include "CircleBuffer.h"

void setup(){           
    Serial.begin(9600);
}
void loop(){
    CIRCLE_BUFFER<int> cb1;
    cb1.begin(16);
    int prev = 0;
    while(1){
        int data1 = analogRead(A0);
        cb1.write(data1);
        // if (abs(cb1.readMean()-prev)>10){
        //     prev = cb1.readMean();
        //     Serial.println(F("Mean1: "));
        //     Serial.println(cb1.readMean());
        // }
        Serial.print(F("Mean1: "));
        Serial.println(cb1.readMean());

    }
}
