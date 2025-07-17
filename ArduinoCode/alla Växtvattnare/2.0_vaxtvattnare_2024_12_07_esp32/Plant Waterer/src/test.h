#include <Arduino.h>
#define moisturePin 36 //jordfuktighet

int raw = 0;
float filtered = 0;
int mapped = 0;
const int filterWeight = 90;

void setup() {
    pinMode(moisturePin, INPUT);
    Serial.begin(115200);
}
void loop()
{
  raw = analogRead(moisturePin);
  filtered = Filter(raw, filtered); 
  mapped = constrain(map((int)filtered, 430, 1400, 100, 0), 0, 100);

  Serial.print("raw: ");
  Serial.print(raw);
  Serial.print(", filtered: ");
  Serial.print((int)filtered);
  Serial.print(", mapped: ");
  Serial.println(mapped);

  delay(200);
}

float Filter(float New, float Current) //Moisture sensor
{
  return (1.0 - filterWeight/100.0) * New + filterWeight/100.0 * Current;
}
