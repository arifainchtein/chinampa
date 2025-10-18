#include "Arduino.h"
#include <LittleFS.h>
#include <NewPing.h>
#include <ChinampaWifiManager.h>
#include <Timer.h>
#include <PCF8563TimeManager.h>
#include <Esp32SecretManager.h>
#include <Arduino_JSON.h>
#include <FastLED.h>
#include <TM1637Display.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <ChinampaData.h>
#include <sha1.h>
#include <totp.h>
#include <LoRa.h>

#include <Wire.h>

#define UI_CLK 23
#define UI1_DAT 26
#define UI2_DAT 25
#define LED_PIN 19
#define NUM_LEDS 8
#define OP_MODE 34
#define RTC_BATT_VOLT 36
#define FISH_TANK_OUTFLOW_FLOW_METER 39
#define TANK_LEVEL_TRIGGER 5
#define TANK_LEVEL_ECHO 5
#define TANK_LEVEL_MAX_DISTANCE 60  // in cm
NewPing fish_tank_height_sensor(TANK_LEVEL_TRIGGER, TANK_LEVEL_ECHO, TANK_LEVEL_MAX_DISTANCE); // NewPing setup of pins and maximum distance.



#define FISH_OUTPUT_SOLENOID_RELAY 18


#define LED_PIN 19
#define RELAY_PIN 32
#define RTC_CLK_OUT 4

#define SCK 14
#define MOSI 13
#define MISO 12
#define LoRa_SS 15
#define LORA_RESET 16
#define LORA_DI0 17
int badPacketCount = 0;
byte msgCount = 0;         // count of outgoing messages
byte localAddress = 0xFF;  // address of this device
byte destination = 0xAA;
bool initiatedWifi=false;
const float R1 = 1000000.0; // Resistance of R1 in ohms (1 MΩ)
const float R2 = 2000000.0; // Resistance of R2 in ohms (2 MΩ)
const float Vref = 3.3;
bool cleareddisplay1=false;
int delayTime = 10;
bool loraActive = false;
bool opmode = false;
String serialNumber;

uint8_t secondsSinceLastDataSampling = 0;
PCF8563TimeManager timeManager(Serial);
GeneralFunctions generalFunctions;
Esp32SecretManager secretManager(timeManager);
ChinampaCommandData chinampaCommandData;
ChinampaConfigData chinampaConfigData;
bool isHost = true;
ChinampaData chinampaData;
Timer dsUploadTimer(30);
bool uploadToDigitalStables = false;
bool internetAvailable;
#define uS_TO_S_FACTOR 60000000 /* Conversion factor for micro seconds to minutes */

uint8_t currentFunctionValue = 10;
ChinampaWifiManager wifiManager(Serial,LittleFS, timeManager, secretManager, chinampaData,chinampaConfigData);

float operatingStatus = 3;
bool wifiActive = false;
bool apActive = false;
long requestTempTime = 0;
JSONVar jsonData;
TM1637Display display1(UI_CLK, UI1_DAT);
TM1637Display display2(UI_CLK, UI2_DAT);
CRGB leds[NUM_LEDS];
long lastTimeUpdateMillis = 0;
RTCInfoRecord currentTimerRecord;
#define TIME_RECORD_REFRESH_SECONDS 3
volatile bool clockTicked = false;
#define UNIQUE_ID_SIZE 8

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

//String display1TempURL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Indoor%20Temperature:Indoor%20Temperature%20Data";
String display1TempURL = "http://192.168.1.117/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Indoor%20Temperature:Indoor%20Temperature%20Data";

/********************************************************************/

#define TEMPERATURE 27

#define MIN_HUMIDITY 60
#define MAX_HUMIDITY 70
/********************************************************************/
// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)


volatile int flowMeterPulseCount = 0;
const float calibrationFactor = 450.0; //for YF-G1


OneWire oneWire2(TEMPERATURE);
DallasTemperature microTempSensor(&oneWire2);



struct DisplayData {
  int value;
  int dp;
} displayData;

//
// interrupt functions
//



void IRAM_ATTR clockTick() {
  portENTER_CRITICAL_ISR(&mux);
  clockTicked = true;
  portEXIT_CRITICAL_ISR(&mux);
}


 void IRAM_ATTR  fishTankOutflowPulseCounter()
{
  flowMeterPulseCount++;
}



//
// end of interrupt functions
//

//
// Lora Functions
//
void onReceive(int packetSize) {
  if (packetSize == 0) return;  // if there's no packet, return
  if (packetSize == sizeof(PanchoCommandData)) {
    LoRa.readBytes((uint8_t *)&chinampaCommandData, sizeof(ChinampaCommandData));
    long commandcode = chinampaCommandData.commandcode;
    bool validCode = secretManager.checkCode(commandcode);
    if (validCode) {

      //secretManager.saveSleepPingMinutes(rosieConfigData.sleepPingMinutes);
      //secretManager.saveConfigData(rosieConfigData.fieldId,  stationName );

      int rssi = LoRa.packetRssi();
      float Snr = LoRa.packetSnr();
      Serial.println(" Receive chinampaCommandData: ");
      Serial.print(" Field Id: ");
      Serial.print(chinampaCommandData.fieldId);
      Serial.print(" commandcode: ");
      Serial.print(chinampaCommandData.commandcode);

    } else {
      Serial.print(" Receive chinampaCommandData but invalid code: ");
      Serial.println(commandcode);
      Serial.print(chinampaCommandData.fieldId);
    }
  } else {
    badPacketCount++;
    Serial.print("Received  invalid data chinampaCommandData data, expected: ");
    Serial.print(sizeof(ChinampaCommandData));
    Serial.print("  received");
    Serial.println(packetSize);
  }
}

void sendMessage() {
  LoRa.beginPacket();  // start packet
  LoRa.write((uint8_t *)&chinampaData, sizeof(ChinampaData));
  LoRa.endPacket();  // finish packet and send it
  msgCount++;        // increment message ID
}

//
// End of Lora Functions
//

int processDisplayValue1(String displayURL, struct DisplayData *displayData) {
  int value = 0;
  bool debug = false;
  // Serial.print("getting data for ");
  //  Serial.println(displayURL);

  String displayValue = wifiManager.getTeleonomeData(displayURL, debug);
  //   Serial.print("received ");
  //  Serial.print(displayValue);
  if (displayValue.indexOf("Error") > 0) {
    value = 9999;
  } else {
    jsonData = JSON.parse(displayValue);
    if (jsonData["Value Type"] == JSONVar("int")) {
      auto val = (const char *)jsonData["Value"];
      if (val == NULL) {
        value = (int)jsonData["Value"];
      } else {
        String s((const char *)jsonData["Value"]);
        value = s.toInt();
      }
      displayData->dp = -1;
      Serial.print("int value= ");
      Serial.println(value);
    } else if (jsonData["Value Type"] == JSONVar("double")) {
      auto val = (const char *)jsonData["Value"];
      if (val == NULL) {
        double valueF = (double)jsonData["Value"];
        if (valueF == (int)valueF) {
          value = (int)valueF;
          displayData->dp = -1;
        } else {
          value = (int)(100 * valueF);
          displayData->dp = 1;
        }
      } else {
        String s((const char *)jsonData["Value"]);
        float valueF = s.toFloat();
        if (valueF == (int)valueF) {
          value = (int)valueF;
          displayData->dp = -1;
        } else {
          value = (int)(100 * valueF);
          displayData->dp = 1;
        }
      }
    } else {
      value = 9997;
      displayData->dp = -1;
    }
  }
  displayData->value = value;

  return value;
}


int processDisplayValue(double valueF, struct DisplayData *displayData) {
  int value = 0;

  if (valueF == (int)valueF) {
    value = (int)valueF;
    displayData->dp = -1;
  } else {
    value = (int)(100 * valueF);
    displayData->dp = 1;
  }
  displayData->value = value;

  return value;
}


void readSensorData(){

  //
  // read the fish tank water height
  //
   float distance = fish_tank_height_sensor.ping_cm();
   chinampaData.measuredHeight = distance;
   chinampaData.fishTankAvailablePercentage = distance * 100 / TANK_LEVEL_MAX_DISTANCE;

  //
  // read the fish tank outflow flow
  //
   unsigned long currentTime = millis();
  unsigned long timeElapsed = currentTime - lastFlowReadTime;
  
  // Disable interrupt while reading
  detachInterrupt(digitalPinToInterrupt(FISH_TANK_OUTFLOW_FLOW_METER));
  
  // Store pulse count and reset
  int currentPulseCount = flowMeterPulseCount;
  flowMeterPulseCount = 0;
  
  // Re-enable interrupt
  attachInterrupt(digitalPinToInterrupt(FISH_TANK_OUTFLOW_FLOW_METER), fishTankOutflowPulseCounter, RISING);
  
  // Calculate flow rate in L/min
  // (pulses / calibration factor) = liters
  // (liters / seconds) * 60 = L/min
  float litersFlowed = currentPulseCount / FLOW_CALIBRATION_FACTOR;
  chinampaData.fishtankoutflowflowRate = (litersFlowed / (timeElapsed / 1000.0)) * 60.0;
  
  // Add to total volume
  fishTankTotalOutflow += litersFlowed;
  
  // Update last read time
  lastFlowReadTime = currentTime;

 
  
  microTempSensor.requestTemperatures();  // Send the command to get temperatures
  chinampaData.microtemperature = microTempSensor.getTempCByIndex(0);
  Serial.println(" Outdoor T:" + String(chinampaData.temperature) );

    //
    // RTC_BATT_VOLT Voltage
    //

    float total = 0;
    uint8_t samples = 10;
    for (int x = 0; x < samples; x++)
    {                                            // multiple analogue readings for averaging
      total = total + analogRead(RTC_BATT_VOLT); // add each value to a total
      delay(1);
    }
    float average = total / samples;
    float voltage = (average / 4095.0) * Vref;
    // Calculate the actual voltage using the voltage divider formula
   // float rtcBatVoltage = (voltage * (R1 + R2)) / R2;
    chinampaData.rtcBatVolt = (voltage * (R1 + R2)) / R2;

    chinampaData.rssi = 0;
    chinampaData.snr = 0;
    //wifiManager.setSensorString(sensorData);

    
      cleareddisplay1=true;
    
}

void restartWifi()
{
  //FastLED.setBrightness(50);
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB(0, 0, 0);
  }
  leds[1] = CRGB(255, 0, 255);
  leds[2] = CRGB(255, 0, 255);
  leds[3] = CRGB(255, 0, 255);
  leds[5] = CRGB(255, 0, 255);
  leds[9] = CRGB(255, 0, 255);
  leds[11] = CRGB(255, 0, 255);
  leds[12] = CRGB(255, 0, 255);
  leds[13] = CRGB(255, 0, 255);
  FastLED.show();
  if (!initiatedWifi)
  {

    leds[7] = CRGB(255, 0, 255);
    FastLED.show();
   // Serial.print(F("Before Starting Wifi cap="));
   // Serial.println(digitalStablesData.capacitorVoltage);
    wifiManager.start();
    initiatedWifi = true;
  }
  Serial.println("Starting wifi");

  wifiManager.restartWifi();
 
  bool stationmode = wifiManager.getStationMode();
  chinampaData.internetAvailable = wifiManager.getInternetAvailable();
  //     digitalWrite(WATCHDOG_WDI, HIGH);
  //    delay(2);
  //    digitalWrite(WATCHDOG_WDI, LOW);
  Serial.print("Starting wifi stationmode=");
 // Serial.println(stationmode);
 // Serial.print("digitalStablesData.internetAvailable=");
 // Serial.println(digitalStablesData.internetAvailable);

  //  serialNumber = wifiManager.getMacAddress();
  wifiManager.setSerialNumber(serialNumber);
  wifiManager.setLora(loraActive);
  String ssid = wifiManager.getSSID();
  String ipAddress = "";
  uint8_t ipi;
  if (stationmode)
  {
    ipAddress = wifiManager.getIpAddress();
 //   Serial.print("ipaddress=");
  //  Serial.println(ipAddress);

    if (ipAddress == "" || ipAddress == "0.0.0.0")
    {

      setApMode();
    }
    else
    {
      setStationMode(ipAddress);
    }
  }
  else
  {
    setApMode();
  }
  //    digitalWrite(WATCHDOG_WDI, HIGH);
  //    delay(2);
  //    digitalWrite(WATCHDOG_WDI, LOW);

  chinampaData.loraActive = loraActive;
  uint8_t ipl = ipAddress.length() + 1;
  char ipa[ipl];
  ipAddress.toCharArray(ipa, ipl);
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
 // Serial.println("in ino Done starting wifi");
}




void setup() {
  Serial.begin(115200);
  Wire.begin();
 
  
   pinMode(FISH_TANK_OUTFLOW_FLOW_METER, INPUT_PULLUP);
   flowMeterPulseCount = 0;
   flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  flowMeterPreviousMillis = 0;
  xxx attachInterrupt(digitalPinToInterrupt(FISH_TANK_OUTFLOW_FLOW_METER), fishTankOutflowPulseCounter, FALLING);

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(255, 255, 0);
  }
  FastLED.show();
  // put your setup code here, to run once:
  display1.setBrightness(0x0f);
  display2.setBrightness(0x0f);
  display1.clear();
  display2.clear();
  pinMode(RTC_CLK_OUT, INPUT_PULLUP);  // set up interrupt%20Pin
  digitalWrite(RTC_CLK_OUT, HIGH);     // turn on pullup resistors
  // attach interrupt%20To set_tick_tock callback on rising edge of INT0
  attachInterrupt(digitalPinToInterrupt(RTC_CLK_OUT), clockTick, RISING);
  timeManager.start();
  timeManager.PCF8563osc1Hz();
  currentTimerRecord = timeManager.now();
  chinampaData.secondsTime = timeManager.getCurrentTimeInSeconds(currentTimerRecord);
  String deviceshortname="SEED";
  deviceshortname.toCharArray(chinampaData.deviceshortname, deviceshortname.length() + 1);

  String devicename="Seedling Monitor";
  devicename.toCharArray(chinampaData.devicename, devicename.length() + 1);
  
  microTempSensor.begin();
  uint8_t address[8];
  microTempSensor.getAddress(address, 0);
  for (uint8_t i = 0; i < 8; i++) {
    //if (address[i] < 16) Serial.print("0");
    serialNumber += String(address[i], HEX);
  }

  Serial.print("serial number:");
  Serial.println(serialNumber);


  microTempSensor.begin();


  SPI.begin(SCK, MISO, MOSI);
  pinMode(LoRa_SS, OUTPUT);
  pinMode(LORA_RESET, OUTPUT);
  pinMode(LORA_DI0, INPUT);
  digitalWrite(LoRa_SS, HIGH);
  LoRa.setPins(LoRa_SS, LORA_RESET, LORA_DI0);
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  leds[0] = CRGB(255, 255, 0);
  leds[1] = CRGB(255, 255, 0);
  leds[2] = CRGB(255, 255, 0);
  FastLED.show();
  const uint8_t lora[] = {
    TSEG_F | TSEG_E | TSEG_D,                         // L
    TSEG_E | TSEG_G | TSEG_C | TSEG_D,                 // o
    TSEG_E | TSEG_G,                                 // r
    TSEG_A | TSEG_B | TSEG_C | TSEG_E | TSEG_F | TSEG_G  // A
  };

  const uint8_t on[] = {
    TSEG_E | TSEG_G | TSEG_C | TSEG_D,  // o
    TSEG_C | TSEG_E | TSEG_G           // n
  };

  const uint8_t off[] = {
    TSEG_E | TSEG_G | TSEG_C | TSEG_D,  // o
    TSEG_A | TSEG_G | TSEG_E | TSEG_F,  // F
    TSEG_A | TSEG_G | TSEG_E | TSEG_F   // F
  };

  display1.setSegments(lora, 4, 0);

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
    leds[1] = CRGB(255, 0, 0);
    display2.setSegments(off, 3, 0);
  } else {
    Serial.println("Starting LoRa worked!");
    leds[1] = CRGB(0, 0, 255);
    display2.setSegments(on, 2, 0);
    loraActive = true;
  }
  FastLED.show();
  delay(1000);



  display1.clear();
  display2.clear();

  chinampaData.currentFunctionValue = currentFunctionValue;

  //tankAndFlowSensorController.begin(currentFunctionValue);

  operatingStatus = secretManager.getOperatingStatus();
  String grp = "9slwJcM9";//secretManager.getGroupIdentifier();
  char gprid[16];
  grp.toCharArray(gprid, 16);
  strcpy(chinampaData.groupidentifier, gprid);

  String identifier = "Chinampa";
  char ty[25];
  identifier.toCharArray(ty, 25);
  strcpy(chinampaData.deviceTypeId, ty);

  chinampaConfigData.fieldId = secretManager.getFieldId();

  if (!initiatedWifi){
   // Serial.print(F("Before Starting Wifi cap="));
   // Serial.println(digitalStablesData.capacitorVoltage);
    wifiManager.start();
    initiatedWifi = true;
  }
  Serial.println("Starting wifi");

  
  bool stationmode = wifiManager.getStationMode();
   chinampaData.internetAvailable = wifiManager.getInternetAvailable();
 
  Serial.print("Starting wifi stationmode=");
  Serial.print(stationmode);

   Serial.print("  internetAvailable=");
  Serial.println(chinampaData.internetAvailable);
  

  //  serialNumber = wifiManager.getMacAddress();
  wifiManager.setSerialNumber(serialNumber);
  wifiManager.setLora(loraActive);
  String ssid = wifiManager.getSSID();
  String ipAddress = "";
  uint8_t ipi;
  if (stationmode) {
    ipAddress = wifiManager.getIpAddress();
    Serial.print("line 430 ipaddress=");
    Serial.println(ipAddress);

    if (ipAddress == "" || ipAddress == "0.0.0.0") {
      setApMode();
    } else {
      setStationMode(ipAddress);
    }
  } else {
    setApMode();
  }



  internetAvailable = wifiManager.getInternetAvailable();

  pinMode(RTC_BATT_VOLT, INPUT);
  pinMode(OP_MODE, INPUT_PULLUP);
 // pinMode(SENSOR_INPUT_3, INPUT);
  pinMode(RELAY_PIN, OUTPUT);


  opmode = digitalRead(OP_MODE);
  if (loraActive) {
    leds[1] = CRGB(0, 0, 255);
  } else {
    leds[1] = CRGB(0, 0, 0);
  }
  leds[2] = CRGB(255, 255, 0);
  FastLED.show();

  display1.showNumberDec(0, false);
  display2.showNumberDec(0, false);
  requestTempTime = millis();
  readSensorData();
     
  dsUploadTimer.start();
  Serial.println("Ok-Ready");
}


void loop() {
  // put your main code here, to run repeatedly:
  if (clockTicked) {
    portENTER_CRITICAL(&mux);
    clockTicked = false;
    portEXIT_CRITICAL(&mux);
    secondsSinceLastDataSampling++;
    currentTimerRecord = timeManager.now();
    wifiManager.setCurrentTimerRecord(currentTimerRecord);
    chinampaData.secondsTime = timeManager.getCurrentTimeInSeconds(currentTimerRecord);
    //Serial.println("secondsSinceLastDataSampling=" +  String(secondsSinceLastDataSampling));
    
    dsUploadTimer.tick();

    if (currentTimerRecord.second == 0) {
      //Serial.println(F("new minute"));

      if (currentTimerRecord.minute == 0) {
        //		Serial.println(F("New Hour"));
        if (currentTimerRecord.hour == 0) {
          //	Serial.println(F("New Day"));
        }
      }
    }
  }

  if (dsUploadTimer.status() && internetAvailable) {
    //char secret[27];
    String secret = "J5KFCNCPIRCTGT2UJUZFSMQK";
    leds[2] = CRGB(0, 255, 0);


    TOTP totp = TOTP(secret.c_str());
    char totpCode[7];  //get 6 char code

    long timeVal = timeManager.getCurrentTimeInSeconds(currentTimerRecord);
    long code = totp.gen_code(timeVal);
    Serial.print("timeVal=");
    Serial.print(timeVal);

    Serial.print("totp=");
    Serial.print(code);
    chinampaData.dsLastUpload = timeVal;

    wifiManager.setCurrentToTpCode(code);
    bool uploadok = wifiManager.uploadDataToDigitalStables();
    if (uploadok) {
      leds[2] = CRGB(0, 0, 255);
    } else {
      leds[2] = CRGB(255, 0, 0);
    }
    FastLED.show();

    dsUploadTimer.reset();
  }

   

  if (secondsSinceLastDataSampling >= chinampaData.dataSamplingSec) {
    if (loraActive) {
      leds[1] = CRGB(0, 255, 0);
    }

    
    FastLED.show();
    readSensorData();
    secondsSinceLastDataSampling = 0;
   

   
  }

  if(chinampaData.humidifierstatus){
    leds[5] = CRGB(0, 0, 255);
    leds[6] = CRGB(0, 0, 255);
    leds[7] = CRGB(0, 0, 255);
  }else{
    leds[5] = CRGB(0, 0, 0);
    leds[6] = CRGB(0, 0, 0);
    leds[7] = CRGB(0, 0, 0);    
  }
    FastLED.show();

  
  if (currentTimerRecord.second == 0 || currentTimerRecord.second == 30) {
   // leds[5] = CRGB(0, 255, 0);
    FastLED.show();

    const uint8_t hu[] = {
      TSEG_B | TSEG_C | TSEG_E | TSEG_F | TSEG_G,
      TSEG_B | TSEG_C | TSEG_E | TSEG_F | TSEG_D,
      0,                                             
      0   
    };
    if(cleareddisplay1){
      cleareddisplay1=false;
      display1.clear();

      if (loraActive) {
        sendMessage();
        leds[1] = CRGB(0, 0, 255);
        FastLED.show();
      }
    }
    display1.setSegments(hu, 2, 0);
    int hi = (int)(chinampaData.greenhouseHum * 100);
    display2.showNumberDecEx(hi, (0x80 >> 1), false);

    

  } else if (currentTimerRecord.second == 10 || currentTimerRecord.second == 40) {
   // leds[4] = CRGB(0, 255, 0);
    FastLED.show();
    const uint8_t t[] = {
      TSEG_F | TSEG_E | TSEG_D | TSEG_G,  // t
      TSEG_A | TSEG_D | TSEG_E | TSEG_F | TSEG_G,   //E
      0,                                             
      0  
    };
    if(cleareddisplay1){
      cleareddisplay1=false;
      display1.clear();
    }
    display1.setSegments(t, 2, 0);
    int tempi = (int)(chinampaData.greenhouseTemp * 100);
    display2.showNumberDecEx(tempi, (0x80 >> 1), false);
    
  } else if (currentTimerRecord.second == 20  || currentTimerRecord.second == 50 ) {
    
    const uint8_t t2[] = {
      TSEG_F | TSEG_E | TSEG_D | TSEG_A| TSEG_B| TSEG_C,  // O
      0x00,
      TSEG_F | TSEG_E | TSEG_D | TSEG_G,  // t
      TSEG_A | TSEG_D | TSEG_E | TSEG_F| TSEG_G   //E
    };
    Serial.print("Sensor1=");
    Serial.println(chinampaData.temperature);
    if(cleareddisplay1){
      cleareddisplay1=false;
      display1.clear();
    }
    display1.setSegments(t2, 4, 0);
    int value1 = processDisplayValue(chinampaData.temperature, &displayData);
    if (displayData.dp > 0) {
      display2.showNumberDecEx(value1, (0x80 >> displayData.dp), false);
    } else {
      display2.showNumberDec(value1, false);
    }
    delay(100);
  }

  if (Serial.available() != 0) {
    String command = Serial.readString();
    Serial.print(F("command="));
    Serial.println(command);
    if (command.startsWith("Ping")) {
      Serial.println(F("Ok-Ping"));
      
    } else if (command.startsWith("SetGroupId")) {
      String grpId = generalFunctions.getValue(command, '#', 1);
      secretManager.setGroupIdentifier(grpId);
      Serial.print(F("set group id to "));
      Serial.println(grpId);

      Serial.println(F("Ok-SetGroupId"));
    } else if (command.startsWith("GetWifiStatus")) {


      uint8_t status = wifiManager.getWifiStatus();
      Serial.print("WifiStatus=");
      Serial.println(status);


      Serial.println("Ok-GetWifiStatus");

    } else if (command.startsWith("ConfigWifiSTA")) {
      //ConfigWifiSTA#ssid#password
      //ConfigWifiSTA#MainRouter24##VisualizerTestHome#
      String ssid = generalFunctions.getValue(command, '#', 1);
      String password = generalFunctions.getValue(command, '#', 2);
      String hostname = generalFunctions.getValue(command, '#', 3);
      bool staok = wifiManager.configWifiSTA(ssid, password, hostname);
      if (staok) {
        leds[0] = CRGB(0, 0, 255);
      } else {
        leds[0] = CRGB(255, 0, 0);
      }
      FastLED.show();
      Serial.println("Ok-ConfigWifiSTA");

    } else if (command.startsWith("ConfigWifiAP")) {
      //ConfigWifiAP#soft_ap_ssid#soft_ap_password#hostaname
      //ConfigWifiAP#pancho5##pancho5

      String soft_ap_ssid = generalFunctions.getValue(command, '#', 1);
      String soft_ap_password = generalFunctions.getValue(command, '#', 2);
      String hostname = generalFunctions.getValue(command, '#', 3);

      bool stat = wifiManager.configWifiAP(soft_ap_ssid, soft_ap_password, hostname);
      if (stat) {
        leds[0] = CRGB(0, 255, 0);
      } else {
        leds[0] = CRGB(255, 0, 0);
      }
      FastLED.show();
      Serial.println("Ok-ConfigWifiAP");

    } else if (command.startsWith("GetOperationMode")) {
      uint8_t switchState = digitalRead(OP_MODE);
      if (switchState == LOW) {
        Serial.println(F("PGM"));
      } else {
        Serial.println(F("RUN"));
      }
    } else if (command.startsWith("SetTime")) {
      //SetTime#24#10#19#4#17#32#00
      uint8_t switchState = digitalRead(OP_MODE);
      if (switchState == LOW) {
        timeManager.setTime(command);
        Serial.println("Ok-SetTime");
      } else {
        Serial.println("Failure-SetTime");
      }

    } else if (command.startsWith("SetFieldId")) {
      // fieldId= GeneralFunctions::getValue(command, '#', 1).toInt();
    } else if (command.startsWith("GetTime")) {
      timeManager.printTimeToSerial(currentTimerRecord);
      Serial.flush();
      Serial.println("Ok-GetTime");
      Serial.flush();
    } else if (command.startsWith("GetCommandCode")) {
      long code = 123456;  //secretManager.generateCode();
      //
      // patch a bug in the totp library
      // if the first digit is a zero, it
      // returns a 5 digit number
      if (code < 100000) {
        Serial.print("0");
        Serial.println(code);
      } else {
        Serial.println(code);
      }

      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("VerifyUserCode")) {
      String codeInString = generalFunctions.getValue(command, '#', 1);
      long userCode = codeInString.toInt();
      boolean validCode = true;  //secretManager.checkCode( userCode);
      String result = "Failure-Invalid Code";
      if (validCode) result = "Ok-Valid Code";
      Serial.println(result);
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("GetSecret")) {
      uint8_t switchState = digitalRead(OP_MODE);
      if (switchState == LOW) {
        //  char secretCode[SHARED_SECRET_LENGTH];
        String secretCode = secretManager.readSecret();
        Serial.println(secretCode);
        Serial.println("Ok-GetSecret");
      } else {
        Serial.println("Failure-GetSecret");
      }
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("SetSecret")) {
      uint8_t switchState = digitalRead(OP_MODE);
      if (switchState == LOW) {
        //SetSecret#IZQWS3TDNB2GK2LO#6#30
        String secret = generalFunctions.getValue(command, '#', 1);
        int numberDigits = generalFunctions.getValue(command, '#', 2).toInt();
        int periodSeconds = generalFunctions.getValue(command, '#', 3).toInt();
        secretManager.saveSecret(secret, numberDigits, periodSeconds);
        Serial.println("Ok-SetSecret");
        Serial.flush();
        delay(delayTime);
      } else {
        Serial.println("Failure-SetSecret");
      }


    } else if (command == "Flush") {
      while (Serial.read() >= 0)
        ;
      Serial.println("Ok-Flush");
      Serial.flush();
    } else if (command.startsWith("PulseStart")) {
      //inPulse=true;
      Serial.println("Ok-PulseStart");
      Serial.flush();
      delay(delayTime);

    } else if (command.startsWith("PulseFinished")) {
      //	inPulse=false;
      Serial.println("Ok-PulseFinished");
      Serial.flush();
      delay(delayTime);

    } else if (command.startsWith("IPAddr")) {
      //	currentIpAddress = generalFunctions.getValue(command, '#', 1);
      Serial.println("Ok-IPAddr");
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("SSID")) {
      String currentSSID = generalFunctions.getValue(command, '#', 1);
      wifiManager.setCurrentSSID(currentSSID.c_str());
      Serial.println("Ok-currentSSID");
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("GetIpAddress")) {
      Serial.println(wifiManager.getIpAddress());
      Serial.println("Ok-GetIpAddress");
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("RestartWifi")) {
      wifiManager.restartWifi();
      Serial.println("Ok-restartWifi");
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("HostMode")) {
      Serial.println("Ok-HostMode");
      Serial.flush();
      delay(delayTime);
      isHost = true;
    } else if (command.startsWith("NetworkMode")) {
      Serial.println("Ok-NetworkMode");
      Serial.flush();
      delay(delayTime);
      isHost = false;
    } else if (command.startsWith("GetSensorData")) {


      //	Serial.print(wiFiManager.getSensorData());
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("AsyncData")) {
      Serial.print("AsyncCycleUpdate#");
      Serial.println("#");
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("GetLifeCycleData")) {
      Serial.println("Ok-GetLifeCycleData");
      Serial.flush();
    } else if (command.startsWith("GetWPSSensorData")) {
      Serial.println("Ok-GetWPSSensorData");
      Serial.flush();
    } else {
      //
      // call read to flush the incoming
      //
      Serial.println("Failure-Command Not Found-" + command);
      Serial.flush();
      delay(delayTime);
    }
  }
}


void setStationMode(String ipAddress) {
  Serial.println("settting Station mode, address ");
  Serial.println(ipAddress);
  leds[0] = CRGB(0, 0, 255);
  FastLED.show();
  const uint8_t ip[] = {
    TSEG_F | TSEG_E,                         // I
    TSEG_F | TSEG_G | TSEG_A | TSEG_B | TSEG_E  // P
  };
  uint8_t ipi;
  for (int i = 0; i < 4; i++) {
    ipi = GeneralFunctions::getValue(ipAddress, '.', i).toInt();
    display1.showNumberDec(ipi, false);
    delay(1000);
  }
}

void setApMode() {

  leds[0] = CRGB(0, 0, 255);
  FastLED.show();
  Serial.println("settting AP mode");
  //
  // set ap mode
  //
  //  wifiManager.configWifiAP("PanchoTankFlowV1", "", "PanchoTankFlowV1");
  String apAddress = wifiManager.getApAddress();
  Serial.println("settting AP mode, address ");
  Serial.println(apAddress);
  const uint8_t ap[] = {
    TSEG_F | TSEG_G | TSEG_A | TSEG_B | TSEG_C | TSEG_E,  // A
    TSEG_F | TSEG_G | TSEG_A | TSEG_B | TSEG_E           // P
  };
  display1.setSegments(ap, 2, 0);
  delay(1000);
  uint8_t ipi;

  for (int i = 0; i < 4; i++) {
    ipi = GeneralFunctions::getValue(apAddress, '.', i).toInt();
    display1.showNumberDec(ipi, false);
    delay(1000);
  }
  for (int i = 2; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  leds[0] = CRGB(0, 255, 0);
  if (loraActive) {
    leds[1] = CRGB(0, 0, 255);
  } else {
    leds[1] = CRGB(255, 0, 0);
  }

  FastLED.show();
}
