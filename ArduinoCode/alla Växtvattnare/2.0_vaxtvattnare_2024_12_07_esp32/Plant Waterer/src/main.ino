#include <Arduino.h>
#include <MegunoLink.h> //MegunoLink
#include <Filter.h> //Filter
#include <Wire.h> //temperatur
#include <Adafruit_GFX.h> //oled
#include <Adafruit_SSD1306.h> //oled
#include <U8g2lib.h> //oled
#include <Adafruit_I2CDevice.h>
#include <PressButton.h> //Interface
#include <RotaryEncoder.h> //Interface
#include <EEPROM.h> //Save Settings
#include <WiFi.h> //Home assistant
#include <PubSubClient.h> //Home assistant
#include "Privates.h" //Homeassistant 
#include <math.h>
#include <OneWire.h> //temperatur
#include <DallasTemperature.h> //temperatur
//-----------------------------------------------------------------------
// OLED display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
//-----------------------------------------------------------------------
//IO-pins
#define confirmBtnPin 32 //input digitalRead()
#define ledOnPin 33 //output digitalWrite()
#define moisturePin1 39 //input analogRead(), fel använd 36
#define moisturePin2 34 //input analogRead(), fel använd 39
#define moisturePin3 36 //input analogRead(), fel använd 34
#define moisturePin4 35 //input analogRead(), fel använd 35
#define oneWireBus 17 //input digitalRead(), digitalWrite(), oneWire bus
#define pumpPin1 4 //output didigitalWrite()
#define pumpPin2 14 //output didigitalWrite()
#define pumpPin3 18 //output didigitalWrite()
#define pumpPin4 19 //output didigitalWrite()
//rotary encoder
#define outputA 13 //input digitalRead()
#define outputB 16 //input digitalRead()
//-----------------------------------------------------------------------
//Settings
#define DISP_ITEM_ROWS 4
#define DISP_CHAR_WIDTH 16
#define PACING_MC 30 //25
#define FLASH_RST_CNT 3 //30
#define SETTINGS_CHKVAL 3647 
#define CHAR_X SCREEN_WIDTH/DISP_CHAR_WIDTH // 240/16
#define CHAR_Y SCREEN_HEIGHT/DISP_ITEM_ROWS // 240/8
#define SHIFT_UP 0 //240/8
//-----------------------------------------------------------------------
//Varibles

//Homeassistant
//-----------------------------------------------------------------------
Privates privates; 
WiFiClient espClient; 
PubSubClient client(espClient); 
char messages[50]; 
volatile  bool TopicArrived = false;
const     int mqttpayloadSize = 10;
char mqttpayload [mqttpayloadSize] = {'\0'};
String mqtttopic;
unsigned long lastMqttReconnectAttempt = 0;
const unsigned long mqttReconnectInterval = 5000; // 5 seconds
//-----------------------------------------------------------------------
//rotary encoder
RotaryEncoder encoder(outputA,outputB,5,1,60000);
//-----------------------------------------------------------------------
//Buttons
PressButton btnOk(confirmBtnPin);
//-----------------------------------------------------------------------
//Menu structure
enum pageType{
  MENU_ROOT,
  MENU_TARGET_MOISTURE,
  MENU_MISC,
};
enum pageType currPage = MENU_ROOT;
void page_MenuRoot();
void page_MENU_TARGET_MOISTURE();
void page_MENU_MISC();
//-----------------------------------------------------------------------
//Menu internals
boolean updateAllItems = true;
boolean updateItemValue;
uint8_t itemCnt;
uint8_t pntrPos;
uint8_t dispOffset;
uint8_t root_pntrPos = 1;
uint8_t root_dispOffset = 0;
uint8_t flashCntr;
boolean flashIsOn;
void initMenuPage(String title, uint8_t itemCount);
void captureButtonDownStates();
void incrementDecrementFloat(float *v, float amount, float min, float max);
void incrementDecrementDouble(double *v, double amount, double min, double max);
void doPointerNavigation();
bool menuItemPrintable(uint8_t xPos, uint8_t yPos);
//-----------------------------------------------------------------------
//Print tools
void printPointer();
void printOnOff(bool val);
void printUint32_tAtWidth(uint32_t value, uint8_t width, char c);
void printDoubleAtWidth(double value, uint8_t width, char c);
//-----------------------------------------------------------------------
//Settings
#pragma pack(1) //memory alignment
struct Mysettings{

  int targetMoisture = 70;
  boolean power = true;
  int dry = 1400;
  int wet = 300;
  int overshoot = 3;
  int filterWeight = 85;
  uint16_t settingsCheckValue = SETTINGS_CHKVAL;
};

Mysettings settings;
void sets_SetDeafault();
void sets_Load();
void sets_Save();
//-----------------------------------------------------------------------
//Time
unsigned long previousTime = 0; 
unsigned long currentTime;
unsigned long previousSensorRead = 0;
unsigned long loopTime;
int readInterval = 1000;
//-----------------------------------------------------------------------
//rotary encoder
int counter = 1; 
int aState = 0;
int bState = 0;
int aLastState = 0;  
// Oled - Adafruit_SSD1306
//-----------------------------------------------------------------------
U8G2_SH1106_128X64_NONAME_F_HW_I2C display1(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SH1106_128X64_NONAME_F_HW_I2C display2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
long timeLastPressed = 0;
const long timeBeforeDisable = 10000;
//-----------------------------------------------------------------------
double passedTime,previousPassedTime1,previousPassedTime2 = 0;
bool initPage = true;
bool changeValue = false;
bool changeValues [10];
//-----------------------------------------------------------------------
//Moisture sensor
ExponentialFilter<long> ADCFilter(50, 50); // Create a new exponential filter with a weight of 100 and initial value of 0.
int current_moisture1;
int previous_moisture1;
int current_moisture2;
int previous_moisture2;
int raw1;
int raw2;
float filtered1 = 0;
float filtered2 = 0;
int mapped = 0;
bool pump_status1;
bool vattna1 = false; //true = vattnar, false = inte vattnar
bool pump_status2;
bool vattna2 = false; //true = vattnar, false = inte vattnar
//-----------------------------------------------------------------------
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
int current_airTemp;
int previous_airTemp;
//-----------------------------------------------------------------------
void setup() {//=================================================SETUP=======================================================
  Serial.begin(115200);

  // WiFi.disconnect(true);
  // WiFi.mode(WIFI_OFF);

  sensors.begin();

  analogSetAttenuation(ADC_11db); // Match input range to expected voltage
  analogSetWidth(12);             // Ensure full resolution

  setupWiFi(); //Home assistant
  client.setServer(privates.broker, 1883); //Home assistant
  client.setCallback(callback);

  currentTime = millis();
  loopTime = currentTime;

  pinMode(confirmBtnPin, INPUT_PULLDOWN);
  pinMode(ledOnPin, OUTPUT);
  pinMode(moisturePin1, INPUT);
  pinMode(moisturePin2, INPUT);
  pinMode(moisturePin3, INPUT);
  pinMode(moisturePin4, INPUT);
  pinMode(oneWireBus, INPUT_PULLUP);
  pinMode(pumpPin1, OUTPUT);
  // pinMode(pumpPin2, OUTPUT);
  // pinMode(pumpPin3, OUTPUT);
  // pinMode(pumpPin4, OUTPUT);
  pinMode(outputA,INPUT_PULLUP);
  pinMode(outputB,INPUT_PULLUP);

  Wire.begin();
  Wire.setClock(3400000 );      //3.4 MHz
  display1.setBusClock(3400000 );  //3.4 MHz
  display2.setBusClock(3400000 ); //3.4 MHz

  display1.setI2CAddress(0x78); 
  display1.begin();  
  display1.setFont(u8g2_font_6x10_mf);	// choose a suitable font
  display1.clearBuffer();					// clear the internal memory
  display1.setCursor(0, 0);

  display2.setI2CAddress(0x7A);
  display2.begin();  
  display2.setFont(u8g2_font_6x10_mf);	// choose a suitable font  
  display2.clearBuffer();					// clear the internal memory
  display2.setCursor(0, 0);

  EEPROM.begin(sizeof(settings));
  sets_Load();
}
void loop() { //=================================================LOOP=======================================================

  const float UPDATE_INTERVAL1 = 0.25;
  const float UPDATE_INTERVAL2 = 0.01;
  passedTime = millis() / 1000.0;
  bool shouldUpdate1 = (passedTime - previousPassedTime1 >= UPDATE_INTERVAL1);
  bool shouldUpdate2 = (passedTime - previousPassedTime2 >= UPDATE_INTERVAL2);
  
  if(shouldUpdate1)
  {
    updateSettings();
    updateSensorValues(); 
    updateDisp2();
    previousPassedTime1 = passedTime;
  }
    
  //display1.clearBuffer();                             //nytt
  switch (currPage)
  {
    case MENU_ROOT: page_MenuRoot(); break;
    case MENU_TARGET_MOISTURE: page_MENU_TARGET_MOISTURE(); break;
    case MENU_MISC: page_MENU_MISC(); break;
  }

  printPointer();              
  if ((updateAllItems || updateItemValue) && shouldUpdate2)
  {
    display1.sendBuffer();                        //välldigt långsam
    previousPassedTime2 = passedTime;
  } 

  updateAllItems = false;
  updateItemValue = false;
  
  captureButtonDownStates();

  //Homeassistant
  if(passedTime - previousTime > 1.0)
    {
      publishMessage();
      subscribeMessage();
      previousTime = passedTime;
    }
}

void page_MenuRoot(){//=================================================ROOT_MENU============================================
  if(initPage)
  {
    pntrPos = root_pntrPos;
    dispOffset = root_dispOffset;
    
    initMenuPage(F("MAIN MENU"), 2);
    initPage = false;
  }
  doPointerNavigation();
  
  if(menuItemPrintable(1,1)){display1.print(F("TARGET MOISTURE     "));}
  if(menuItemPrintable(1,2)){display1.print(F("MISC         "));} 

  if(btnOk.PressReleased())
  {
    FlashPointer();
    root_pntrPos = pntrPos;
    root_dispOffset = dispOffset;
    initPage = true;
    switch (pntrPos)
    {
    case 1: currPage = MENU_TARGET_MOISTURE; return;
    case 2: currPage = MENU_MISC; return;
    }
  }
}
void page_MENU_TARGET_MOISTURE(){//=================================================TARGET_MOISTURE============================================
  if(initPage)
  {
    pntrPos = 1;
    dispOffset = 0;
    initMenuPage(F("TARGET_MOISTURE"), 2);
    changeValue = false;
    initPage = false;
  }

  if(changeValue)
    incrementDecrementInt(&settings.targetMoisture, 1.0, 0.0, 100.0);
  else 
    doPointerNavigation(); 

  if(menuItemPrintable(1,1)){display1.print(F("Target Moist =     "));}
  if(menuItemPrintable(1,2)){display1.print(F("Back              "));}

  if(menuItemPrintable(12,1)){printUint32_tAtWidth(settings.targetMoisture, 3, '%');}

  if(btnOk.PressReleased())
  {
    FlashPointer();
    
    switch (pntrPos)
    {
      case 1: changeValue = !changeValue; break;
      case 2: currPage = MENU_ROOT; sets_Save(); initPage = true; return;
    }
  }
}
void page_MENU_MISC(){//=================================================MISC==========================================================
  if(initPage)
  {
    pntrPos = 1;
    dispOffset = 0;
    initMenuPage(F("MISC"), 6);
    changeValues [10];
    initPage = false;
  }

  if(menuItemPrintable(1,1)){display1.print(F("POWER        =     "));}
  if(menuItemPrintable(1,2)){display1.print(F("DryCallib    =     "));}
  if(menuItemPrintable(1,3)){display1.print(F("WetCallib    =     "));}
  if(menuItemPrintable(1,4)){display1.print(F("FilterWeight =     "));}
  if(menuItemPrintable(1,5)){display1.print(F("Overshoot    =     "));}
  if(menuItemPrintable(1,6)){display1.print(F("Back               "));}

  if(menuItemPrintable(12,1)){printOnOff(settings.power);}
  if(menuItemPrintable(12,2)){printUint32_tAtWidth(settings.dry, 3, ' ');}
  if(menuItemPrintable(12,3)){printUint32_tAtWidth(settings.wet, 3, ' ');}
  if(menuItemPrintable(12,4)){printUint32_tAtWidth(settings.filterWeight, 3, ' ');}
  if(menuItemPrintable(12,5)){printUint32_tAtWidth(settings.overshoot, 3, ' ');}
      
  if(btnOk.PressReleased())
  {
    FlashPointer();
    
    switch (pntrPos)
    {
      case 1: changeValues [0] = !changeValues [0]; break; 
      case 2: changeValues [1] = !changeValues [1]; break; 
      case 3: changeValues [2] = !changeValues [2]; break;
      case 4: changeValues [3] = !changeValues [3]; break; 
      case 5: changeValues [4] = !changeValues [4]; break;
      case 6: currPage = MENU_ROOT; sets_Save(); initPage = true; return;
    }
  }
  double amountConstant = 0.01;
  if(changeValues[0])
  {
    *&settings.power = !*&settings.power;
    changeValues [0] = false;
    updateItemValue = true; 
  }
  else if(changeValues[1])incrementDecrementInt(&settings.dry, 1, 3000, 3500);
  else if(changeValues[2])incrementDecrementInt(&settings.wet, 1, 800, 1300);
  else if(changeValues[3])incrementDecrementInt(&settings.filterWeight, 1, 1, 100);
  else if(changeValues[4])incrementDecrementInt(&settings.overshoot, 1, 1, 5);
  else 
    doPointerNavigation(); 
}
//======================================================TOOLS - menu Internals==================================================
void initMenuPage(String title, uint8_t itemCount){
  display1.clearBuffer();
  printPointer();
  uint8_t fillCnt = (DISP_CHAR_WIDTH - title.length()) / 2;

  btnOk.ClearWasDown();
 
  itemCnt = itemCount;
  flashCntr = 0;
  flashIsOn = false;
  updateAllItems = true;
}
void captureButtonDownStates(){
  btnOk.CaptureDownState();
}

void doPointerNavigation(){
  currentTime = millis();
  if (currentTime >= (loopTime + 1) ) {
    aState = digitalRead(outputA); 
   bState = digitalRead(outputB);
  if (aState > aLastState)
  {
    if (bState != aState){  
        if(pntrPos > 1)
        {
          flashIsOn = false; flashCntr = 0; 
          if(pntrPos - dispOffset == 1){updateAllItems = true; dispOffset--;}
          pntrPos--;
          printPointer();
        }
        counter ++;
      } 
    else{
        if(pntrPos < itemCnt)
        {
          flashIsOn = false; flashCntr = 0; 
          if(pntrPos - dispOffset == DISP_ITEM_ROWS){updateAllItems = true; dispOffset++;}
          pntrPos++;
          printPointer();
        }
        counter --;
    }
   }
   aLastState = aState; 
   loopTime = currentTime;
  }
}
void incrementDecrementInt(int *v, int amount, int min, int max)
{
  int enc = encoder.readEncoder();
  if(enc != 0) {
    *v += (enc*amount);
    *v = constrain(*v,min,max);
    //Serial.println(enc*amount);
    //updateItemValue = true;                                  //nytt
  } 
  delayMicroseconds(5);
}
void incrementDecrementFloat(float *v, float amount, float min, float max)
{
  int enc = encoder.readEncoder();
  if(enc != 0) {
    *v += (enc*amount);
    *v = constrain(*v,min,max);
    //Serial.println(enc*amount);
    updateItemValue = true;
  } 
  delayMicroseconds(5);
}
void incrementDecrementDouble(double *v, double amount, double min, double max)
{
  int enc = encoder.readEncoder();
  if(enc != 0) {
    *v += (enc*amount);
    *v = constrain(*v,min,max);
    //Serial.println(enc*amount);
    updateItemValue = true;
  } 
  delayMicroseconds(5);
}

bool isFlashChanged(){
  if(flashCntr == 0){
    flashIsOn = !flashIsOn;
    flashCntr = FLASH_RST_CNT;
    return true;
  }
  else{flashCntr--; return false;}
}

bool menuItemPrintable(uint8_t xPos, uint8_t yPos){
  if(!(updateAllItems || (updateItemValue && pntrPos == yPos))){return false;}
  uint8_t yMaxOffset = 0;
  if(yPos > DISP_ITEM_ROWS) {yMaxOffset = yPos - DISP_ITEM_ROWS;}
  if(dispOffset <= (yPos) && dispOffset >= yMaxOffset){display1.setCursor(CHAR_X*xPos, CHAR_Y*(yPos - dispOffset)); return true;}
  return false;
}

bool menuItemPrintableDisp2(uint8_t xPos, uint8_t yPos){ 
  if(!(updateAllItems || (updateItemValue && pntrPos == yPos))){return false;}
  uint8_t yMaxOffset = 0;
  if(yPos > DISP_ITEM_ROWS) {yMaxOffset = yPos - DISP_ITEM_ROWS;}
  if(0 <= (yPos) && 0 >= yMaxOffset){display2.setCursor(CHAR_X*xPos, CHAR_Y*(yPos)); return true;}
  return false;
}

//======================================================TOOLS_display========================================================
void printPointer(){
  //Serial.println("printPointer");
  display1.drawStr(0, 1*CHAR_Y, " ");
  display1.drawStr(0, 2*CHAR_Y, " ");
  display1.drawStr(0, 3*CHAR_Y, " ");
  display1.drawStr(0, 4*CHAR_Y, " ");
  display1.drawStr(0, (pntrPos - dispOffset)*CHAR_Y, "*");
  //display1.sendBuffer();
}
void FlashPointer(){
  display1.drawStr(0, 1*CHAR_Y, " ");
  display1.drawStr(0, 2*CHAR_Y, " ");
  display1.drawStr(0, 3*CHAR_Y, " ");
  display1.drawStr(0, 4*CHAR_Y, " ");
  display1.sendBuffer();                  //nytt

  delay(100);
  //Serial.println("FlashPointer");
  display1.drawStr(0, (pntrPos - dispOffset)*CHAR_Y, "*");
  display1.sendBuffer();                  //nytt
}

void printOnOff(bool val){
  if(val){display1.print(F("ON    "));}
  else   {display1.print(F("OFF   "));}
}
void printChars(uint8_t cnt, char c){
  if(cnt > 0){
    char cc[] = " "; cc[0] = c;
    for(u_int8_t i = 1; i < cnt; i++){display1.print(cc);}
  }
}
uint8_t getUint32_tCharCnt(uint32_t value)
{
  if(value == 0){return 1;}
  uint32_t tensCalc = 10; int8_t cnt = 1;
  while (tensCalc <= value && cnt < 20){tensCalc *= 10; cnt += 1;}
  return cnt;
}
uint8_t getDoubleCharCnt(double value)
{
  if(value == 0){return 1;}
  uint32_t tensCalc = 10; int8_t cnt = 1;
  while (tensCalc <= value && cnt < 20){tensCalc *= 10; cnt += 1;}
  return cnt;
}
void printUint32_tAtWidth(uint32_t value, uint8_t width, char c){
  display1.print(value);
  display1.print(c);
  printChars(width-getUint32_tCharCnt(value), ' ');
}
void printDoubleAtWidth(double value, uint8_t width, char c){
  char buf[10];
  dtostrf(value, width-getDoubleCharCnt(value), 1, buf); // 1 decimal
  display1.print(buf);
  display1.print(c);
}
//======================================================DISPLAY_2======================================================
void printCharsDisplay2(uint8_t cnt, char c){
  if(cnt > 0){
    char cc[] = " "; cc[0] = c;
    for(u_int8_t i = 1; i < cnt; i++){display2.print(cc);}
  }
}
void printUint32_tAtWidthDisplay2(uint32_t value, uint8_t width, char c){
  display2.print(value);
  display2.print(c);
  printCharsDisplay2(width-getUint32_tCharCnt(value), ' ');
}
void printDoubleAtWidthDisplay2(double value, uint8_t width, char c){
  char buf[10];
  dtostrf(value, width-getDoubleCharCnt(value), 1, buf); // 1 decimal
  display2.print(buf);
  display2.print(c);
}

//======================================================TOOLS_settings======================================================
void sets_SetDeafault()
{
  Mysettings tempSets;
  memcpy(&settings, &tempSets, sizeof settings);
}

void sets_Load()
{
  EEPROM.get(0,settings);
  if(settings.settingsCheckValue != SETTINGS_CHKVAL){sets_SetDeafault();}
}
void sets_Save()
{
  EEPROM.put(0, settings);
  EEPROM.commit();
}

void updateSettings()
{
  if(millis() - timeLastPressed > timeBeforeDisable)
  {
    //display1.ssd1306_command(SSD1306_DISPLAYOFF);
    //display2.ssd1306_command(SSD1306_DISPLAYOFF);
  }
  else
  {
  //   display1.ssd1306_command(SSD1306_DISPLAYON);
  //   display2.ssd1306_command(SSD1306_DISPLAYON);
  }

  if(settings.power)
  {
    digitalWrite(ledOnPin, HIGH);
    //Pump1--------------------------------------------------------------------
    if (current_moisture1 < settings.targetMoisture - settings.overshoot) 
    { //
      vattna1 = true;
      pump_status1 = true;
      //Serial.print("Pump water ON:  ");
      digitalWrite(pumpPin1, HIGH);
      delay(1);
      //digitalWrite(pumpPin, LOW);
    }
    else if (current_moisture1 > settings.targetMoisture + settings.overshoot) {
      vattna1 = false;
      pump_status1 = false;
      //Serial.print("Pump water OFF:  ");
      digitalWrite(pumpPin1, LOW);
    }
    else {
      pump_status1 = vattna1;
      digitalWrite(pumpPin1, vattna1);
      //Serial.print("Pump water vattna:  ");
    }
    //Pump2--------------------------------------------------------------------
    if (current_moisture2 < settings.targetMoisture - settings.overshoot) 
    { //
      vattna2 = true;
      pump_status2 = true;
      //Serial.print("Pump water ON:  ");
      digitalWrite(pumpPin2, HIGH);
      delay(1);
      //digitalWrite(pumpPin, LOW);
    }
    else if (current_moisture2 > settings.targetMoisture + settings.overshoot) {
      vattna2 = false;
      pump_status2 = false;
      //Serial.print("Pump water OFF:  ");
      digitalWrite(pumpPin2, LOW);
    }
    else {
      pump_status2 = vattna2;
      digitalWrite(pumpPin2, vattna2);
      //Serial.print("Pump water vattna:  ");
    }
    //------------------------------------------------------------------------
    updateAllItems = true;

    client.loop();
    if(TopicArrived)
    {
      // Serial.print("Message arrived: ");
      // Serial.println(mqttpayload);
      //settings.targetTemp = atof(mqttpayload);
      TopicArrived = false;
    }
  }
  else
  {
    digitalWrite(ledOnPin, LOW);
  }
}

void updateSensorValues() {

  unsigned long currentTime = millis();
  if (currentTime - previousSensorRead >= readInterval)
  {
    previousSensorRead = currentTime;

    sensors.requestTemperatures(); // Request temperature readings from the sensor
    previous_airTemp = current_airTemp;
    current_airTemp = sensors.getTempCByIndex(0);

    previous_moisture1 = current_moisture1;
    
    raw1 = analogRead(moisturePin1);
    raw2 = analogRead(moisturePin2);
    if(raw1 > 0)
    {
      filtered1 = Filter(raw1, filtered1); 
      current_moisture1 = constrain(map((int)filtered1, settings.wet, settings.dry, 100, 0), 0, 100);
    }
    if(raw2 > 0)
    {
      filtered2 = Filter(raw2, filtered2); 
      current_moisture2 = constrain(map((int)filtered2, settings.wet, settings.dry, 100, 0), 0, 100);
    }

    Serial.print("Current Moist: ");
    Serial.print(current_moisture1);
    Serial.print(" %");

    Serial.print(", Raw1: ");
    Serial.print(raw1);

    Serial.print(", Raw2: ");
    Serial.print(raw2);

    Serial.print(", Previous Moist: ");
    Serial.print(previous_moisture1);
    Serial.print(" %");

    Serial.print(", Target Moist: ");
    Serial.print(settings.targetMoisture);
    Serial.print("%");

    Serial.print(", Is Watering1: ");
    Serial.print(pump_status1 ? "Yes" : "No");

    Serial.print(", Is Watering2: ");
    Serial.print(pump_status2 ? "Yes" : "No");

    Serial.print(", Overshoot: ");
    Serial.print(settings.overshoot);

    Serial.print(", Filter w: ");
    Serial.print(settings.filterWeight);

    Serial.print(", Air Temp: ");
    Serial.println(current_airTemp);

  }
}

void updateDisp2()
{
  display2.clearBuffer();
  if(menuItemPrintableDisp2(1,1)){display2.print(F("TargetMoist   =     "));}
  if(menuItemPrintableDisp2(1,2)){display2.print(F("CurrentMoist1 =     "));}
  if(menuItemPrintableDisp2(1,3)){display2.print(F("CurrentMoist2 =     "));}
  if(menuItemPrintableDisp2(1,4)){display2.print(F("AirTemp       =     "));}

  if(menuItemPrintableDisp2(13,1)){printUint32_tAtWidthDisplay2(settings.targetMoisture, 3, '%');}
  if(menuItemPrintableDisp2(13,2)){printUint32_tAtWidthDisplay2(current_moisture1, 3, '%');}
  if(menuItemPrintableDisp2(13,3)){printUint32_tAtWidthDisplay2(current_moisture2, 3, '%');}
  if(menuItemPrintableDisp2(13,4)){printDoubleAtWidthDisplay2(current_airTemp, 3, 'C');}
  display2.sendBuffer();
  display2.clearBuffer();                             //nytt
}

void setupWiFi() //Homeassistant
{
  WiFi.begin(privates.ssid, privates.pass);
  Serial.print("\nConnecting to ");
  Serial.print(privates.ssid);
  while(WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.print(".");
  }
  Serial.print("\nConnected to ");
  Serial.println(privates.ssid);
}

void reconnect() //Homeassistant
{
  // Only try to reconnect if enough time has passed
  unsigned long now = millis();
  if (now - lastMqttReconnectAttempt > mqttReconnectInterval) {
    lastMqttReconnectAttempt = now;
    if (!client.connected()) {
      client.connect("boll", privates.brokerUser, privates.brokerPass);
    }
  }
}

void publishMessage() //Homeassistant
{
  if(!client.connected()){reconnect();}
  client.loop();
  //int "%d", float "%f", bool "%s", obs vikitgt!
  snprintf(messages, sizeof(messages), "%d", current_moisture1);
  client.publish(privates.topicSoilMoisture1, messages);
  snprintf(messages, sizeof(messages), "%d", current_moisture2);
  client.publish(privates.topicSoilMoisture2, messages);
  snprintf(messages, sizeof(messages), "%d", settings.targetMoisture); 
  client.publish(privates.topicTargetMoisture, messages);
  snprintf(messages, sizeof(messages), "%s", pump_status1 ? "true" : "false");
  client.publish(privates.topicIsWatering, messages);
  snprintf(messages, sizeof(messages), "%d", current_airTemp);
  client.publish(privates.topicAirTemp, messages);
}

void subscribeMessage() //Homeassistant
{
    // if(!client.connected()){reconnect();}
    // client.loop();
    // client.subscribe(privates.topicTargetTemp); 
}

void callback(char* topic, byte* payload, unsigned int length) //Homeassistant
{
  if ( !TopicArrived )
  {
    memset( mqttpayload, '\0', mqttpayloadSize ); // clear payload char buffer
    mqtttopic = ""; //clear topic string buffer
    mqtttopic = topic; //store new topic
    memcpy( mqttpayload, payload, length );
    TopicArrived = true;
  }
}

float Filter(float New, float Current) //Moisture sensor
{
  return (1.0 - settings.filterWeight/100.0) * New + settings.filterWeight/100.0 * Current;
}
