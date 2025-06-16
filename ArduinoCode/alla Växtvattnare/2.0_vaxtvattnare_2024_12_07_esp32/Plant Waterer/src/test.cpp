#include <Arduino.h>
#define moisturePin 4 //jordfuktighet
void setup() {
    pinMode(moisturePin, INPUT);
    Serial.begin(115200);
}
void loop()
{
  Serial.println(constrain(map(analogRead(moisturePin), 980, 2560, 100, 0),0,100));
}