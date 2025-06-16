#include <Arduino.h>
#include "MegunoLink.h"
#include "Filter.h"
#include <Wire.h> //temperatur
#include <Adafruit_GFX.h> //oled
#include <Adafruit_SSD1306.h> //oled
#include <OneWire.h> //temperatur
#include <DallasTemperature.h> //temperatur
#include <LiquidCrystal_I2C.h> //LCD

#include <WiFi.h> //Homeassistant
#include <PubSubClient.h> //Homeassistant
#include "Privates.h" //Homeassistant

Privates privates; //Homeassistant

WiFiClient espClient; //Homeassistant
PubSubClient client(espClient); //Homeassistant
char messages[50]; //Homeassistant

#define moisturePin 32 //jordfuktighet
#define oneWireBus 4 //temperatur
#define pumpPin 26
#define x_up_Pin 17 //x_up
#define x_ner_Pin 18 //x_ner
#define y_up_Pin 19 //y_up
#define y_ner_Pin 23 //y_ner
#define ledOnPin 14

void setInputFlags();
void resolveInputFlags();
void inputAction(int input);
void parameterChange(int key);
void printScreen();
void setupWiFi(); //Homeassistant
void reconnect(); //Homeassistant
void publishMessage(); //Homeassistant

//Input & Button Logic
const int numOfInputs = 4;
const int inputPins[numOfInputs] = {x_up_Pin, x_ner_Pin, y_up_Pin, y_ner_Pin};
int inputState[numOfInputs];
int lastInputState[numOfInputs] = {LOW, LOW, LOW, LOW};
bool inputFlags[numOfInputs] = {LOW, LOW, LOW, LOW};
long lastDebounceTime[numOfInputs] = {0, 0, 0, 0};
double debounceDelay = 0;

//LCD Menu Logic
const int numOfScreens = 9;
int currentScreen = 0;
String screens[numOfScreens][2] = {{"Earth humidty", "%"}, {"Tempeture", "C"}, {"Threshold", "%"}, {"Overshoot", ""}, {"Filter weight", ""}, {"Wet raw", ""}, {"Dry raw", ""}, {"Raw humidity", ""}, {"ON", ""}};
int parameters[numOfScreens]  = {0, 0, 40, 3, 50, 1100, 3200, 0, 1};

float current_moisture;
float current_temp_DS18B20;
float previous_moisture;
float previous_temp_DS18B20;
int raw;
bool pump_status;
int threshold;
int overshoot;
int filterWeight;
bool button;
long  previousTime = 0;
long currentTime = 0;
long timeLastPressed = 0;
const long timeBeforeDisable = 10000;

int dry = 3200;
int wet = 1100;
bool vattna = false;
bool buttonState = LOW;


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Create a new exponential filter with a weight of 100 and initial value of 0.
ExponentialFilter<long> ADCFilter(50, 50);

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
LiquidCrystal_I2C lcd(0x27, 16, 2); // Default address of most PCF8574 modules, change according

OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);


void setup() {
  Serial.begin(115200);
  sensors.begin();
  setupWiFi(); //Homeassistant
  client.setServer(privates.broker, 1883); //Homeassistant
  

  pinMode(moisturePin, INPUT);
  pinMode(oneWireBus, INPUT_PULLUP);
  pinMode(pumpPin, OUTPUT);
  pinMode(ledOnPin, OUTPUT);
  pinMode(x_up_Pin, INPUT_PULLDOWN);
  pinMode(x_ner_Pin, INPUT_PULLDOWN);
  pinMode(y_up_Pin, INPUT_PULLDOWN);
  pinMode(y_ner_Pin, INPUT_PULLDOWN);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  lcd.begin();
  lcd.display();
}

void loop() {

  currentTime = millis(); 
  
  setInputFlags();
  resolveInputFlags();
  if(millis() - timeLastPressed > timeBeforeDisable)
  {
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    lcd.noBacklight();
  }
  else
  {
    display.ssd1306_command(SSD1306_DISPLAYON);
    lcd.backlight();
  }
    

  threshold = parameters[2];
  overshoot = parameters[3];
  filterWeight = parameters[4];
  wet = parameters[5];
  dry = parameters[6];
  parameters[0] = current_moisture;
  parameters[1] = current_temp_DS18B20;
  parameters[7] = raw;


  if (currentScreen == 0 || currentScreen == 1 || currentScreen == 7) //Earth Humidity and Temperature and raw
    if (current_moisture != previous_moisture || current_temp_DS18B20 != previous_temp_DS18B20)
    {
      printScreen();
    }
  /*
    Serial.print(digitalRead(x_up_Pin));
    Serial.print(digitalRead(x_ner_Pin));
    Serial.print(digitalRead(y_up_Pin));
    Serial.println(digitalRead(y_ner_Pin));
  */
  if (parameters[8] == 1)
  {
    digitalWrite(ledOnPin, HIGH);

    if (current_moisture < threshold - overshoot) { //
      vattna = true;
      pump_status = true;
      //Serial.print("Pump water ON:  ");
      digitalWrite(pumpPin, HIGH);
      delay(1);
      //digitalWrite(pumpPin, LOW);
    }
    else if (current_moisture > threshold + overshoot) {
      vattna = false;
      pump_status = false;
      //Serial.print("Pump water OFF:  ");
      digitalWrite(pumpPin, LOW);
    }
    else {
      pump_status = vattna;
      digitalWrite(pumpPin, vattna);
      //Serial.print("Pump water vattna:  ");
    }

  }
  else
  {
    //Serial.print("Pump water OFF:  ");
    digitalWrite(pumpPin, LOW);
    digitalWrite(ledOnPin, LOW);
  }
  //Serial.println(currentTime - previousTime);
  //Serial.print(current_temp_DS18B20);
  //Serial.println(" degreed C");//shows degrees character
  sensors.requestTemperatures();
  previous_temp_DS18B20 = current_temp_DS18B20;
  current_temp_DS18B20 = sensors.getTempCByIndex(0);

  previous_moisture = current_moisture;
  ADCFilter.SetWeight(filterWeight);
  raw = analogRead(moisturePin);
  ADCFilter.Filter(raw);
  current_moisture = ADCFilter.Current(); //felet är här kan inte använda 25 och 27

  current_moisture = map(current_moisture, wet, dry, 100, 0);
  current_moisture = constrain(current_moisture, 0, 100);
  /*
    Serial.print(", Earth Humidity: ");
    Serial.print(current_moisture);
    Serial.print(" %");//shows degrees character

    Serial.print(", threshold: ");
    Serial.print(threshold);
    Serial.print("%");//shows degrees character

    Serial.print(", overshoot: ");
    Serial.print(overshoot);

    Serial.print(", Filter weight: ");
    Serial.print(filterWeight);

    Serial.print(", Temperature: ");
    Serial.print(current_temp_DS18B20);

    Serial.print(", Raw: ");
    Serial.println(raw);
  */
  /*
  Serial.print(digitalRead(inputPins[0]));
  Serial.print(digitalRead(inputPins[1]));
  Serial.print(digitalRead(inputPins[2]));
  Serial.println(digitalRead(inputPins[3]));
  */
  
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  // Display static text

  display.print(current_moisture);
  display.println("%");
  display.print(current_temp_DS18B20);
  display.println(" C");
  display.display();

  if(!client.connected()){reconnect();} //Homeassistant
  client.loop(); //Homeassistant
  if(currentTime - previousTime >= 1000)
  {
    //Serial.print("Sending messages: ");
    //Serial.println(messages);
    publishMessage();
    previousTime = millis();
  }
}

void setInputFlags() {
  for (int i = 0; i < numOfInputs; i++) {
    int reading = digitalRead(inputPins[i]);

    if (reading != inputState[i]) {
      inputState[i] = reading;
      if (inputState[i] == HIGH) {
        inputFlags[i] = HIGH;
      }

    }
    lastInputState[i] = reading;
  }
}

void resolveInputFlags() {
  for (int i = 0; i < numOfInputs; i++) {
    if (inputFlags[i] == HIGH) {
      timeLastPressed = millis();
      inputAction(i);
      inputFlags[i] = LOW;
      printScreen();
    }
  }
}

void inputAction(int input) {
  if (input == 0) {
    if (currentScreen == 0) {
      currentScreen = numOfScreens - 1;
    } else {
      currentScreen--;
    }
  } else if (input == 1) {
    if (currentScreen == numOfScreens - 1) {
      currentScreen = 0;
    } else {
      currentScreen++;
    }
  } else if (input == 2) {
    parameterChange(0);
  } else if (input == 3) {
    parameterChange(1);
  }
}

void parameterChange(int key) {

  if (currentScreen == 8)                                          //On
  {
    if (key == 0 && parameters[currentScreen] < 1) {
      parameters[currentScreen]++;
    } else if (key == 1 && parameters[currentScreen] > 0) {
      parameters[currentScreen]--;
    }
  }
  else if (currentScreen == 5 || currentScreen == 6)            //dry raw, wet raw
  {
    if (key == 0) {
      parameters[currentScreen] += 20;
    } else if (key == 1) {
      if (parameters[currentScreen] > 0)
        parameters[currentScreen] -= 20;
    }
  }
  else if (currentScreen == 3)                                   //Overshoot
  {
    if (key == 0) {
      parameters[currentScreen] += 1;
    } else if (key == 1) {
      if (parameters[currentScreen] > 0)
        parameters[currentScreen] -= 1;
    }
  }
  else if (currentScreen >= 2 && currentScreen != 7)            //threshold, Filter weight, dry raw, wet raw
  {
    if (key == 0) {
      parameters[currentScreen] += 2;
    } else if (key == 1) {
      if (parameters[currentScreen] > 0)
        parameters[currentScreen] -= 2;
    }
  }
  // Earth humidity, Temperature and raw cant be changed
}

void printScreen() {

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(screens[currentScreen][0]);
  lcd.setCursor(0, 1);
  lcd.print(parameters[currentScreen]);
  lcd.print(" ");
  lcd.print(screens[currentScreen][1]);
}

void setupWiFi() //Homeassistant
{
  WiFi.begin(privates.ssid, privates.pass);
  // Serial.print("\nConnecting to ");
  // Serial.print(privates.ssid);
  // while(WiFi.status() != WL_CONNECTED)
  // {
  //   delay(100);
  //   Serial.print(".");
  // }
  // Serial.print("\nConnected to ");
  // Serial.println(privates.ssid);
}
void reconnect() //Homeassistant
{
  // Serial.print("\nConnecting to ");
  // Serial.println(privates.broker);
  if(client.connect("boll", privates.brokerUser, privates.brokerPass))
  {
    // Serial.print("\nConnected to ");
    // Serial.println(privates.broker);
  }
  else
  {
    // Serial.println("\nTrying connect again");
  }
}

void publishMessage() //Homeassistant
{
    if(!client.connected()){reconnect();}
    client.loop();
    snprintf(messages, 10, "%ld", (int)current_moisture); //Homeassistant
    client.publish(privates.topicSoilHumidity, messages); //Homeassistant
    snprintf(messages, 10, "%ld", (int)current_temp_DS18B20); //Homeassistant
    client.publish(privates.topicAirTemp, messages); //Homeassistant
}