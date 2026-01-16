#include "Arduino.h"
#include <LittleFS.h>
#include <NewPing.h>
//#include <Chinampa//wifiManager.h>
#include <Timer.h>
#include <PCF8563TimeManager.h>
#include <Esp32SecretManager.h>
#include <FastLED.h>
#include <TM1637Display.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DataManager.h>
#include <ChinampaData.h>
#include <DigitalStablesData.h>
#include <sha1.h>
#include <totp.h>
#include <LoRa.h>
#include <ErrorDefinitions.h>
#include <ErrorManager.h>
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
#define TANK_LEVEL_ECHO 35
#define TANK_LEVEL_MAX_DISTANCE 60                                                              // in cm
NewPing fish_tank_height_sensor(TANK_LEVEL_TRIGGER, TANK_LEVEL_ECHO, TANK_LEVEL_MAX_DISTANCE);  // NewPing setup of pins and maximum distance.
#define LED_PIN 19
#define PUMP_RELAY_PIN 32
#define FISH_OUTPUT_SOLENOID_RELAY 18
#define RTC_CLK_OUT 4
#define SCK 14
#define MOSI 13
#define MISO 12
#define LoRa_SS 15
#define LORA_RESET 16
#define LORA_DI0 17

bool turningPumpOn = false;
bool turningPumpOff = false;

bool sendMessageNow = true;
uint8_t displayStatus = 0;
uint8_t loraLastResult = -99;
LoRaError cadResult;

ErrorManager errorManager;
float avgRssi = 0;
#define REG_OP_MODE 0x01
#define REG_IRQ_FLAGS 0x12
#define REG_RSSI_VALUE 0x1B
#define MODE_CAD 0x87
#define IRQ_CAD_DONE_MASK 0x04
#define IRQ_CAD_DETECTED_MASK 0x02

#define CAD_TIMEOUT 5000    // CAD timeout in milliseconds
#define MAX_RETRIES 5       // Maximum transmission retries
#define MIN_BACKOFF 500     // Minimum backoff time in milliseconds
#define MAX_BACKOFF 1500    // Maximum backoff time in milliseconds
#define RSSI_THRESHOLD -60  // RSSI threshold in dBm

int badPacketCount = 0;
byte msgCount = 0;         // count of outgoing messages
byte localAddress = 0xFF;  // address of this device
byte destination = 0xAA;
bool initiatedWifi = false;
const float R1 = 1000000.0;  // Resistance of R1 in ohms (1 MΩ)
const float R2 = 2000000.0;  // Resistance of R2 in ohms (2 MΩ)
const float Vref = 3.3;
bool cleareddisplay1 = false;
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
DataManager dataManager(Serial, LittleFS);

DigitalStablesData fishTankDSD, sumpTroughDSD;
uint8_t currentFunctionValue = 10;

//ChinampaWifiManager wifiManager(Serial, LittleFS, timeManager, secretManager, chinampaData, chinampaConfigData);

float operatingStatus = 3;
bool wifiActive = false;
bool apActive = false;
long requestTempTime = 0;

TM1637Display display1(UI_CLK, UI1_DAT);
TM1637Display display2(UI_CLK, UI2_DAT);
CRGB leds[NUM_LEDS];
long lastTimeUpdateMillis = 0;
RTCInfoRecord currentTimerRecord, lastReceptionRTCInfoRecord;
#define TIME_RECORD_REFRESH_SECONDS 3
volatile bool clockTicked = false;
#define UNIQUE_ID_SIZE 8
unsigned long lastFlowReadTime = 0;
const float FLOW_CALIBRATION_FACTOR = 63.0;
float fishTankTotalOutflow = 0.0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

//String display1TempURL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Indoor%20Temperature:Indoor%20Temperature%20Data";
String display1TempURL = "http://192.168.1.117/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Indoor%20Temperature:Indoor%20Temperature%20Data";
String timezone;
/********************************************************************/

#define TEMPERATURE 27

#define MIN_HUMIDITY 60
#define MAX_HUMIDITY 70
/********************************************************************/
// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)


volatile int flowMeterPulseCount = 0;
const float calibrationFactor = 63.0;  //for YF-G1
volatile bool loraReceived = false;
volatile int loraPacketSize = 0;

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


void IRAM_ATTR fishTankOutflowPulseCounter() {
  flowMeterPulseCount++;
}


//
// end of interrupt functions
//

//
// Lora Functions
//



void LoRa_rxMode() {
  LoRa.disableInvertIQ();  // normal mode
  LoRa.receive();          // set receive mode
}

void LoRa_txMode() {
  LoRa.idle();             // set standby mode
  LoRa.disableInvertIQ();  // normal mode
}

void IRAM_ATTR onReceive(int packetSize) {
  loraReceived = true;
  loraPacketSize = packetSize;
}

void processLora(int packetSize) {
  if (packetSize == 0) return;  // if there's no packet, return
  Serial.println("Lora received " + String(packetSize));

  // Serial.println(" DigitalStablesData " + String(sizeof(DigitalStablesData)));
  //  Serial.println(" SeedlingMonitorData " + String(sizeof(SeedlingMonitorData)));

  if (packetSize == sizeof(DigitalStablesData)) {
    boolean validData = false;
    DigitalStablesData tempData;
    memset(&tempData, 0, sizeof(DigitalStablesData));
     LoRa.readBytes((uint8_t *)&tempData, sizeof(DigitalStablesData));
     
    memcpy(tempData.sentbyarray, "Chinamp", 8);  
    sendDSDMessage(tempData);
   
    tempData.devicename[sizeof(tempData.devicename) - 1] = '\0';
    Serial.println("received from " + String(tempData.devicename));
    //  dataManager.printDigitalStablesData(tempData);

    Serial.print("Device name bytes: ");
    for (int i = 0; i < sizeof(tempData.devicename); i++) {
      Serial.print((int)tempData.devicename[i]);
      Serial.print(" ");
    }
    Serial.println();

    Serial.print("Device name length: ");
    Serial.println(strlen(tempData.devicename));

    if (strncmp(tempData.devicename, "FISHTANK", 8) == 0) {
      chinampaData.previousFishTankMeasuredHeight = fishTankDSD.measuredHeight;
      memcpy(&fishTankDSD, &tempData, sizeof(DigitalStablesData));

      float difference = abs(chinampaData.fishTankMeasuredHeight - chinampaData.previousFishTankMeasuredHeight);
      float tenPercentThreshold = chinampaData.previousFishTankMeasuredHeight * 0.10;
      if (difference > tenPercentThreshold) {
        chinampaData.sensorstatus[1] = true;
      } else {
        chinampaData.sensorstatus[1] = false;
      }
      validData = true;
      fishTankDSD.rssi = LoRa.packetRssi();
      fishTankDSD.snr = LoRa.packetSnr();

      dataManager.storeDigitalStablesData(fishTankDSD);
      chinampaData.secondsSinceLastFishTankData = 0;
      chinampaData.minimumFishTankLevel = fishTankDSD.troughlevelminimumcm;
      chinampaData.maximumFishTankLevel = fishTankDSD.troughlevelmaximumcm;
      chinampaData.fishTankMeasuredHeight = fishTankDSD.measuredHeight;
      chinampaData.fishTankHeight = fishTankDSD.maximumScepticHeight;

      Serial.println("Data received from FishTank fishTankMeasuredHeight=" + String(chinampaData.fishTankMeasuredHeight));
      leds[3] = CRGB(0, 255, 0);
      FastLED.show();
    } else if (strcmp(tempData.devicename, "SumpTrough") == 0) {
      chinampaData.previousSumpTroughMeasuredHeight = sumpTroughDSD.measuredHeight;
      memcpy(&sumpTroughDSD, &tempData, sizeof(DigitalStablesData));
      // dataManager.printDigitalStablesData(tempData);
      float difference = abs(chinampaData.sumpTroughMeasuredHeight - chinampaData.previousSumpTroughMeasuredHeight);
      float tenPercentThreshold = chinampaData.previousSumpTroughMeasuredHeight * 0.10;
      if (difference > tenPercentThreshold) {
        chinampaData.sensorstatus[2] = true;
      } else {
        chinampaData.sensorstatus[2] = false;
      }
      sumpTroughDSD.rssi = LoRa.packetRssi();
      sumpTroughDSD.snr = LoRa.packetSnr();
      validData = true;
      chinampaData.secondsSinceLastSumpTroughData = 0;
      chinampaData.minimumSumpTroughLevel = sumpTroughDSD.troughlevelminimumcm;
      chinampaData.maximumSumpTroughLevel = sumpTroughDSD.troughlevelmaximumcm;
      chinampaData.sumpTroughMeasuredHeight = sumpTroughDSD.measuredHeight;
      chinampaData.sumpTroughHeight = sumpTroughDSD.maximumScepticHeight;
      chinampaData.outdoortemperature = sumpTroughDSD.outdoortemperature;
      chinampaData.outdoorhumidity = sumpTroughDSD.outdoorhumidity;

      Serial.println("Data received from SumpTrough sumpTroughMeasuredHeight=" + String(chinampaData.sumpTroughMeasuredHeight));
      leds[4] = CRGB(0, 255, 0);
      FastLED.show();
    }
    if (validData) {
      long messageReceivedTime = timeManager.getCurrentTimeInSeconds(currentTimerRecord);
      lastReceptionRTCInfoRecord.year = currentTimerRecord.year;
      lastReceptionRTCInfoRecord.month = currentTimerRecord.month;
      lastReceptionRTCInfoRecord.date = currentTimerRecord.date;
      lastReceptionRTCInfoRecord.hour = currentTimerRecord.hour;
      lastReceptionRTCInfoRecord.minute = currentTimerRecord.minute;
      lastReceptionRTCInfoRecord.second = currentTimerRecord.second;
      FastLED.show();
    } else {
    }
  }
}
LoRaError performCAD() {
  if (!loraActive) {
    errorManager.setLoRaError(LORA_INIT_FAILED);
    return LORA_INIT_FAILED;
  }

  // Multiple RSSI checks with averaging
  const int SAMPLES = 3;
  const int CHECK_DELAY = 2;  // ms between samples
  float rssiSum = 0;
  // First set of samples
  LoRa.idle();
  LoRa.receive();

  for (int i = 0; i < SAMPLES; i++) {
    rssiSum += LoRa.rssi();
    delay(CHECK_DELAY);
  }

  avgRssi = rssiSum / SAMPLES;

  // If average RSSI is above threshold, channel is busy
  if (avgRssi > RSSI_THRESHOLD) {
    LoRa.idle();
    errorManager.setLoRaError(LORA_CHANNEL_BUSY);
    return LORA_CHANNEL_BUSY;
  }

  // Double-check with a second set of samples
  rssiSum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    rssiSum += LoRa.rssi();
    delay(CHECK_DELAY);
  }

  avgRssi = rssiSum / SAMPLES;
  LoRa.idle();

  Serial.print("checkcad, avgRssi=");
  Serial.print(avgRssi);
  if (avgRssi > RSSI_THRESHOLD) {
    // errorMgr.setLoRaError(LORA_CHANNEL_BUSY, avgRssi);
    return LORA_CHANNEL_BUSY;
  }

  errorManager.clearLoRaError(LORA_CHANNEL_BUSY);  // Clear any previous channel busy error
  return LORA_OK;
}

void sendDSDMessage(DigitalStablesData digitalStablesData) {
  LoRa.beginPacket();  // start packet
  LoRa.write((uint8_t *)&digitalStablesData, sizeof(DigitalStablesData));
  LoRa.endPacket();  // finish packet and send it
  msgCount++;        // increment message ID
  LoRa_txMode();
}

void sendMessage() {
  LoRa.beginPacket();  // start packet
  LoRa.write((uint8_t *)&chinampaData, sizeof(ChinampaData));
  LoRa.endPacket();  // finish packet and send it
  msgCount++;        // increment message ID



  LoRa_txMode();
  uint8_t result = 99;
  int retries = 0;
  boolean keepGoing = true;
  long startsendingtime = millis();
  while (keepGoing) {
    cadResult = performCAD();
    if (cadResult == LORA_OK) {
      // Channel is clear, attempt transmission
      LoRa.beginPacket();
      // Send the provided data object
      LoRa.write((uint8_t *)&chinampaData, sizeof(ChinampaData));
      if (!LoRa.endPacket(true)) {
        result = LORA_TX_FAILED;
      } else {
        result = LORA_OK;
      }
      Serial.print("took ");
      Serial.print(millis() - startsendingtime);
      keepGoing = false;
    } else if (cadResult != LORA_CHANNEL_BUSY) {
      // If error is not due to busy channel, return the error
      result = cadResult;
      int backoff = random(MIN_BACKOFF * (1 << retries), MAX_BACKOFF * (1 << retries));

      Serial.print("Channel busy, retry ");
      Serial.print(retries + 1);
      Serial.print(" of ");
      Serial.print(MAX_RETRIES);
      Serial.print(". Waiting ");
      Serial.print(backoff);
      Serial.println("ms");
      // Channel is busy, implement exponential backoff
      delay(backoff);
      retries++;
      keepGoing = retries < MAX_RETRIES;
    }
  }

  if (result == 99) {
    result = LORA_MAX_RETRIES_REACHED;
  }

  Serial.print(" ,Lora returns ");
  Serial.println(result);
  delay(500);
  LoRa_rxMode();
}

//
// End of Lora Functions
//

//int processDisplayValue1(String displayURL, struct DisplayData *displayData) {
//  int value = 0;
//  bool debug = false;
//  // Serial.print("getting data for ");
//  //  Serial.println(displayURL);
//
//  String displayValue = "";//wifiManager.getTeleonomeData(displayURL, debug);
//  //   Serial.print("received ");
//  //  Serial.print(displayValue);
//  if (displayValue.indexOf("Error") > 0) {
//    value = 9999;
//  } else {
//    jsonData = JSON.parse(displayValue);
//    if (jsonData["Value Type"] == JSONVar("int")) {
//      auto val = (const char *)jsonData["Value"];
//      if (val == NULL) {
//        value = (int)jsonData["Value"];
//      } else {
//        String s((const char *)jsonData["Value"]);
//        value = s.toInt();
//      }
//      displayData->dp = -1;
//      Serial.print("int value= ");
//      Serial.println(value);
//    } else if (jsonData["Value Type"] == JSONVar("double")) {
//      auto val = (const char *)jsonData["Value"];
//      if (val == NULL) {
//        double valueF = (double)jsonData["Value"];
//        if (valueF == (int)valueF) {
//          value = (int)valueF;
//          displayData->dp = -1;
//        } else {
//          value = (int)(100 * valueF);
//          displayData->dp = 1;
//        }
//      } else {
//        String s((const char *)jsonData["Value"]);
//        float valueF = s.toFloat();
//        if (valueF == (int)valueF) {
//          value = (int)valueF;
//          displayData->dp = -1;
//        } else {
//          value = (int)(100 * valueF);
//          displayData->dp = 1;
//        }
//      }
//    } else {
//      value = 9997;
//      displayData->dp = -1;
//    }
//  }
//  displayData->value = value;
//
//  return value;
//}


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


void readSensorData() {


  // Serial.println("fishTankAvailablePercentage=" + String(chinampaData.fishTankAvailablePercentage));
  // Serial.println("minimumFishTankHeight=" + String(chinampaData.minimumFishTankHeight));
  boolean keepgoing = true;
  chinampaData.alertstatus = false;
  chinampaData.alertcode = 0;

  if (chinampaData.secondsSinceLastSumpTroughData <= chinampaData.sumpTroughStaleDataSeconds && chinampaData.secondsSinceLastFishTankData <= chinampaData.fishTankStaleDataSeconds) {
    leds[5] = CRGB(0, 0, 0);
    chinampaData.alertstatus = false;
    chinampaData.alertcode = 99;
    FastLED.show();
  }

  if (chinampaData.secondsSinceLastFishTankData > chinampaData.fishTankStaleDataSeconds) {
    digitalWrite(PUMP_RELAY_PIN, LOW);
    digitalWrite(FISH_OUTPUT_SOLENOID_RELAY, LOW);
    chinampaData.fishtankoutflowsolenoidrelaystatus = false;
    leds[3] = CRGB(255, 0, 0);
    leds[5] = CRGB(255, 0, 0);
    leds[6] = CRGB(255, 0, 0);
    leds[7] = CRGB(255, 0, 0);
    Serial.println("Going red because fish data is stale,chinampaData.secondsSinceLastFishTankData=" + String(chinampaData.secondsSinceLastFishTankData));
    keepgoing = false;
    FastLED.show();
    chinampaData.alertstatus = true;
    chinampaData.alertcode = 1;
  }

  if (chinampaData.secondsSinceLastSumpTroughData > chinampaData.sumpTroughStaleDataSeconds) {
    digitalWrite(PUMP_RELAY_PIN, LOW);
    digitalWrite(FISH_OUTPUT_SOLENOID_RELAY, LOW);
    chinampaData.fishtankoutflowsolenoidrelaystatus = false;
    leds[4] = CRGB(255, 0, 0);
    leds[5] = CRGB(255, 0, 0);
    leds[6] = CRGB(255, 0, 0);
    leds[7] = CRGB(255, 0, 0);
    Serial.println("Going red because chinampaData.secondsSinceLastSumpTroughData data is stale,chinampaData.secondsSinceLastSumpTroughData=" + String(chinampaData.secondsSinceLastSumpTroughData));
    keepgoing = false;
    FastLED.show();
    chinampaData.alertstatus = true;
    chinampaData.alertcode = 2;
  }

  if (chinampaData.secondsSinceLastFishTankData > chinampaData.fishTankStaleDataSeconds && chinampaData.secondsSinceLastSumpTroughData > chinampaData.sumpTroughStaleDataSeconds) {
    chinampaData.alertstatus = true;
    chinampaData.alertcode = 3;
  }
  if (keepgoing) {
    leds[3] = CRGB(0, 255, 0);
    //  leds[5] = CRGB(0, 255, 0);
    if (chinampaData.fishTankMeasuredHeight >= (chinampaData.fishTankHeight - chinampaData.minimumFishTankLevel)) {
      //
      // close the fish tank solenoid
      //

      digitalWrite(FISH_OUTPUT_SOLENOID_RELAY, LOW);
      chinampaData.fishtankoutflowsolenoidrelaystatus = false;
      leds[7] = CRGB(0, 0, 0);
      //
      // now check the pump
      //
      if (chinampaData.secondsSinceLastSumpTroughData > chinampaData.sumpTroughStaleDataSeconds) {
        digitalWrite(PUMP_RELAY_PIN, LOW);
        leds[6] = CRGB(255, 0, 0);
        if (chinampaData.pumprelaystatus) turningPumpOff = true;
        chinampaData.pumprelaystatus = false;
      } else {
        if (chinampaData.sumpTroughMeasuredHeight >= (chinampaData.sumpTroughHeight - chinampaData.minimumSumpTroughLevel)) {
          digitalWrite(PUMP_RELAY_PIN, LOW);
          leds[6] = CRGB(255, 0, 255);
          chinampaData.alertstatus = true;
          chinampaData.alertcode = 5;
          if (chinampaData.pumprelaystatus) turningPumpOff = true;
          chinampaData.pumprelaystatus = false;

        } else {
          if (!chinampaData.pumprelaystatus) turningPumpOn = true;
          digitalWrite(PUMP_RELAY_PIN, HIGH);
          chinampaData.pumprelaystatus = true;
          leds[6] = CRGB(0, 255, 0);
        }

        Serial.println("line 328 1");
        keepgoing = false;
      }
      FastLED.show();
    }
    if (turningPumpOn) {
      sendMessage();
      turningPumpOn = false;
    }
    if (turningPumpOff) {
      sendMessage();
      turningPumpOff = false;
    }
  }

  if (keepgoing) {
    if (chinampaData.fishTankMeasuredHeight < (chinampaData.fishTankHeight - chinampaData.minimumFishTankLevel) && chinampaData.fishTankMeasuredHeight >= (chinampaData.fishTankHeight - chinampaData.maximumFishTankLevel)) {

      //
      // everything active
      //

      digitalWrite(FISH_OUTPUT_SOLENOID_RELAY, HIGH);
      chinampaData.fishtankoutflowsolenoidrelaystatus = true;
      leds[7] = CRGB(0, 255, 0);
      Serial.println("line 502  fisdh tank is green");
      //
      // now check the pump
      //
      if (chinampaData.secondsSinceLastSumpTroughData > chinampaData.sumpTroughStaleDataSeconds) {
        if (chinampaData.pumprelaystatus) turningPumpOff = true;
        digitalWrite(PUMP_RELAY_PIN, LOW);
        chinampaData.pumprelaystatus = false;
        Serial.println("line 508  sump stale");
        leds[6] = CRGB(255, 0, 0);
      } else {
        if (chinampaData.sumpTroughMeasuredHeight >= (chinampaData.sumpTroughHeight - chinampaData.minimumSumpTroughLevel)) {
          if (chinampaData.pumprelaystatus) turningPumpOff = true;
          digitalWrite(PUMP_RELAY_PIN, LOW);
          chinampaData.pumprelaystatus = false;
          Serial.println("line 513  sump too low pump off");
          leds[6] = CRGB(0, 0, 0);
          leds[6] = CRGB(255, 0, 255);
          chinampaData.alertstatus = true;
          chinampaData.alertcode = 5;
        } else {
          if (!chinampaData.pumprelaystatus) turningPumpOn = true;
          digitalWrite(PUMP_RELAY_PIN, HIGH);
          chinampaData.pumprelaystatus = true;
          Serial.println("line 513  sump above min pump on");
          leds[6] = CRGB(0, 255, 0);
        }
      }
      if (turningPumpOn) {
        sendMessage();
        turningPumpOn = false;
      }

      if (turningPumpOff) {
        sendMessage();
        turningPumpOff = false;
      }

      keepgoing = false;
      FastLED.show();
    }
  }

  if (keepgoing) {
    if (chinampaData.fishTankMeasuredHeight < (chinampaData.fishTankHeight - chinampaData.maximumFishTankLevel)) {
      //
      // open the fish tankflow
      //
      Serial.println("line 529, fish tank too high");
      digitalWrite(FISH_OUTPUT_SOLENOID_RELAY, HIGH);
      chinampaData.fishtankoutflowsolenoidrelaystatus = true;
      leds[7] = CRGB(0, 0, 255);

      // the fish tank is too high, turn the pump off
      //
      digitalWrite(PUMP_RELAY_PIN, LOW);
      chinampaData.pumprelaystatus = false;
      leds[6] = CRGB(255, 255, 0);
      FastLED.show();
      Serial.println("line 468");
      keepgoing = false;
    }
  }



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
  Serial.println("line 596,litersFlowed=" + String(litersFlowed) + "  currentPulseCount=" + String(currentPulseCount) + " timeElapsed=" + String(timeElapsed));

  // Add to total volume
  fishTankTotalOutflow += litersFlowed;

  // Update last read time
  lastFlowReadTime = currentTime;

  chinampaData.fishtankoutPulsePerMinute = 60 * (currentPulseCount / (timeElapsed / 1000.0));


  if (digitalRead(FISH_OUTPUT_SOLENOID_RELAY) && chinampaData.fishtankoutflowflowRate < 2) {
    //digitalWrite(PUMP_RELAY_PIN, LOW);
    //  digitalWrite(FISH_OUTPUT_SOLENOID_RELAY, LOW);
    leds[3] = CRGB(255, 0, 0);
    leds[5] = CRGB(255, 0, 0);
    leds[6] = CRGB(255, 0, 0);
    leds[7] = CRGB(255, 0, 0);
    Serial.println("Going red because fish solenouid is open and the fish opuitflow flow is less less than 2, flow=" + String(chinampaData.fishtankoutflowflowRate));
    FastLED.show();
    chinampaData.alertstatus = true;
    chinampaData.alertcode = 4;
  }

  microTempSensor.requestTemperatures();  // Send the command to get temperatures
  chinampaData.microtemperature = microTempSensor.getTempCByIndex(0);
  //Serial.println(" Micro T:" + String(chinampaData.microtemperature) );
  if (chinampaData.microtemperature > chinampaData.microtemperatureMaximum) {
    chinampaData.sensorstatus[0] = true;
  } else {
    chinampaData.sensorstatus[0] = false;
  }

  //
  // RTC_BATT_VOLT Voltage
  //

  float total = 0;
  uint8_t samples = 10;
  for (int x = 0; x < samples; x++) {           // multiple analogue readings for averaging
    total = total + analogRead(RTC_BATT_VOLT);  // add each value to a total
    delay(1);
  }
  float average = total / samples;
  float voltage = (average / 4095.0) * Vref;
  // Calculate the actual voltage using the voltage divider formula
  // float rtcBatVoltage = (voltage * (R1 + R2)) / R2;
  chinampaData.rtcBatVolt = (voltage * (R1 + R2)) / R2;

  chinampaData.rssi = 0;
  chinampaData.snr = 0;
  ////wifiManager.setSensorString(sensorData);


  cleareddisplay1 = true;
}

void restartWifi() {
  //FastLED.setBrightness(50);
  for (int i = 0; i < NUM_LEDS; i++) {
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
  if (!initiatedWifi) {

    leds[7] = CRGB(255, 0, 255);
    FastLED.show();
    // Serial.print(F("Before Starting Wifi cap="));
    // Serial.println(digitalStablesData.capacitorVoltage);
    //wifiManager.start();
    initiatedWifi = true;
  }
  Serial.println("Starting wifi");

  //wifiManager.restartWifi();

  bool stationmode = //wifiManager.getStationMode();
  chinampaData.internetAvailable = //wifiManager.getInternetAvailable();
  //     digitalWrite(WATCHDOG_WDI, HIGH);
  //    delay(2);
  //    digitalWrite(WATCHDOG_WDI, LOW);
  Serial.print("Starting wifi stationmode=");
  // Serial.println(stationmode);
  // Serial.print("digitalStablesData.internetAvailable=");
  // Serial.println(digitalStablesData.internetAvailable);

  //  serialNumber = //wifiManager.getMacAddress();
  //wifiManager.setSerialNumber(serialNumber);
  //wifiManager.setLora(loraActive);
  String ssid = "";//wifiManager.getSSID();
  String ipAddress = "";
  uint8_t ipi;
  if (stationmode) {
    ipAddress = "";//wifiManager.getIpAddress();
    //   Serial.print("ipaddress=");
    //  Serial.println(ipAddress);

    if (ipAddress == "" || ipAddress == "0.0.0.0") {

      setApMode();
    } else {
      setStationMode(ipAddress);
    }
  } else {
    setApMode();
  }
  //    digitalWrite(WATCHDOG_WDI, HIGH);
  //    delay(2);
  //    digitalWrite(WATCHDOG_WDI, LOW);

  chinampaData.loraActive = loraActive;
  uint8_t ipl = ipAddress.length() + 1;
  char ipa[ipl];
  ipAddress.toCharArray(ipa, ipl);
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
  // Serial.println("in ino Done starting wifi");
}




void setup() {
  Serial.begin(115200);
  Wire.begin();

  //
  // data from cofiguration
  //
  double latitude = -37.13305556;
  double longitude = 144.47472222;

  secretManager.getDeviceConfig(chinampaData.devicename, chinampaData.deviceshortname, timezone, latitude, longitude);
  double fishq = 63;
  secretManager.getChinampaParameters(fishq);
  chinampaData.fishtankoutQFactor = fishq;
  pinMode(FISH_TANK_OUTFLOW_FLOW_METER, INPUT_PULLUP);
  flowMeterPulseCount = 0;
  chinampaData.fishtankoutflowflowRate = 0.0;
  // flowMilliLitres = 0;
  //  totalMilliLitres = 0;
  // flowMeterPreviousMillis = 0;
  attachInterrupt(digitalPinToInterrupt(FISH_TANK_OUTFLOW_FLOW_METER), fishTankOutflowPulseCounter, FALLING);

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

  pinMode(TANK_LEVEL_TRIGGER, OUTPUT);
  pinMode(TANK_LEVEL_ECHO, INPUT);
  pinMode(PUMP_RELAY_PIN, OUTPUT);  // set up interrupt%20Pin
  pinMode(FISH_OUTPUT_SOLENOID_RELAY, OUTPUT);

  pinMode(RTC_CLK_OUT, INPUT_PULLUP);  // set up interrupt%20Pin
  digitalWrite(RTC_CLK_OUT, HIGH);     // turn on pullup resistors
  // attach interrupt%20To set_tick_tock callback on rising edge of INT0
  attachInterrupt(digitalPinToInterrupt(RTC_CLK_OUT), clockTick, RISING);
  timeManager.start();
  timeManager.PCF8563osc1Hz();
  currentTimerRecord = timeManager.now();
  chinampaData.secondsTime = timeManager.getCurrentTimeInSeconds(currentTimerRecord);
  String deviceshortname = "CHIN";
  deviceshortname.toCharArray(chinampaData.deviceshortname, deviceshortname.length() + 1);

  String devicename = "Chinampa";
  devicename.toCharArray(chinampaData.devicename, devicename.length() + 1);

  microTempSensor.begin();
  microTempSensor.setWaitForConversion(false);  // Don't block during conversion
  microTempSensor.setResolution(9);
  microTempSensor.getAddress(chinampaData.serialnumberarray, 0);
  for (uint8_t i = 0; i < 8; i++) {
    //if (address[i] < 16) Serial.print("0");
    serialNumber += String(chinampaData.serialnumberarray[i], HEX);
  }

  Serial.print("serial number:");
  Serial.println(serialNumber);


  for (uint8_t i = 0; i < 12; i++) {
    chinampaData.sensorstatus[i] = 0;
  }

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
    TSEG_F | TSEG_E | TSEG_D,                            // L
    TSEG_E | TSEG_G | TSEG_C | TSEG_D,                   // o
    TSEG_E | TSEG_G,                                     // r
    TSEG_A | TSEG_B | TSEG_C | TSEG_E | TSEG_F | TSEG_G  // A
  };

  const uint8_t on[] = {
    TSEG_E | TSEG_G | TSEG_C | TSEG_D,  // o
    TSEG_C | TSEG_E | TSEG_G            // n
  };

  const uint8_t off[] = {
    TSEG_E | TSEG_G | TSEG_C | TSEG_D,  // o
    TSEG_A | TSEG_G | TSEG_E | TSEG_F,  // F
    TSEG_A | TSEG_G | TSEG_E | TSEG_F   // F
  };

  display1.setSegments(lora, 4, 0);
  Serial.println("about to start LoRa");
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
    chinampaData.loraActive = loraActive;
  }
  FastLED.show();
  // delay(1000);



  display1.clear();
  display2.clear();

  //tankAndFlowSensorController.begin(currentFunctionValue);

  operatingStatus = secretManager.getOperatingStatus();
  String grp = "9slwJcM9";  //secretManager.getGroupIdentifier();
  char gprid[16];
  grp.toCharArray(gprid, 16);
  strcpy(chinampaData.groupidentifier, gprid);

  String identifier = "Chinampa";
  char ty[25];
  identifier.toCharArray(ty, 25);
  strcpy(chinampaData.deviceTypeId, ty);

  chinampaConfigData.fieldId = secretManager.getFieldId();

  if (!initiatedWifi) {
    // Serial.print(F("Before Starting Wifi cap="));
    // Serial.println(digitalStablesData.capacitorVoltage);
    //wifiManager.start();
    initiatedWifi = true;
  }
  Serial.println("Starting wifi");


  bool stationmode = "";///wifiManager.getStationMode();
  chinampaData.internetAvailable =false; //wifiManager.getInternetAvailable();

  Serial.print("Starting wifi stationmode=");
  Serial.print(stationmode);

  Serial.print("  internetAvailable=");
  Serial.println(chinampaData.internetAvailable);


  //  serialNumber = //wifiManager.getMacAddress();
  //wifiManager.setSerialNumber(serialNumber);
  //wifiManager.setLora(loraActive);
  String ssid = "";///wifiManager.getSSID();
  String ipAddress = "";
  uint8_t ipi;
  if (stationmode) {
    ipAddress = "";//wifiManager.getIpAddress();
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



  internetAvailable = false;//wifiManager.getInternetAvailable();

  pinMode(RTC_BATT_VOLT, INPUT);
  pinMode(OP_MODE, INPUT_PULLUP);

  if (loraActive) {
    // LoRa_rxMode();
    // LoRa.setSyncWord(0xF3);
    LoRa.onReceive(onReceive);
    // put the radio into receive mode
    LoRa.receive();
  }

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

  dsUploadTimer.start();

  digitalWrite(PUMP_RELAY_PIN, LOW);
  digitalWrite(FISH_OUTPUT_SOLENOID_RELAY, LOW);
  leds[3] = CRGB(255, 0, 0);
  leds[4] = CRGB(255, 0, 0);
  leds[5] = CRGB(255, 0, 0);
  leds[6] = CRGB(255, 0, 0);
  leds[7] = CRGB(255, 0, 0);
  Serial.println("Going red because fish data is stale,chinampaData.secondsSinceLastFishTankData=" + String(chinampaData.secondsSinceLastFishTankData));
  FastLED.show();
  chinampaData.alertstatus = true;
  chinampaData.alertcode = 1;
  chinampaData.secondsSinceLastSumpTroughData = 99;
  chinampaData.secondsSinceLastFishTankData = 99;

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
    //wifiManager.setCurrentTimerRecord(currentTimerRecord);
    chinampaData.secondsTime = timeManager.getCurrentTimeInSeconds(currentTimerRecord);
    //  Serial.println("secondsSinceLastFishTankData=" +  String(secondsSinceLastFishTankData));
    //  Serial.println("chinampaData.secondsSinceLastSumpTroughData=" +  String(chinampaData.secondsSinceLastSumpTroughData));
    chinampaData.secondsSinceLastFishTankData++;
    chinampaData.secondsSinceLastSumpTroughData++;
    dsUploadTimer.tick();

    if (currentTimerRecord.second == 0) {
      //Serial.println(F("new minute"));

      if (currentTimerRecord.minute == 0) {
        //    Serial.println(F("New Hour"));
        if (currentTimerRecord.hour == 0) {
          //  Serial.println(F("New Day"));
        }
      }
    }
  }

  if (loraReceived) {
    //Serial.printf("lora recive Free Heap: %d \n", xPortGetFreeHeapSize());
    //    Serial.printf("lora recive loraPacketSize: %d \n", loraPacketSize);
    //    Serial.println("");
    processLora(loraPacketSize);
    //
    // check to see if the sensor malfunction
    //


    loraReceived = false;
    //    bool show = false;
    //    currentPalette = RainbowStripeColors_p;
    //    currentBlending = NOBLEND;
    //    if (!inSerial){
    //   //   performLedShow(250);
    //   ledShowDuration = 250;  // Set desired duration in milliseconds
    //    runLedShow = true;    // Set flag to trigger the show
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

    //wifiManager.setCurrentToTpCode(code);
    bool uploadok =false; //wifiManager.uploadDataToDigitalStables();
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


  FastLED.show();
  if (currentTimerRecord.second == 0 || currentTimerRecord.second == 30) {
    // leds[5] = CRGB(0, 255, 0);
    sendMessageNow = true;

    FastLED.show();
    const uint8_t fish[] = {
      TSEG_A | TSEG_E | TSEG_F | TSEG_G,                   // F
      TSEG_B | TSEG_C | TSEG_E | TSEG_F | TSEG_G,          // H
      TSEG_E | TSEG_F,                                     // I
      TSEG_A | TSEG_C | TSEG_D | TSEG_E | TSEG_F | TSEG_G  // G

    };
    if (cleareddisplay1) {
      cleareddisplay1 = false;
      display1.clear();
    }

    const uint8_t good[] = {
      TSEG_A | TSEG_C | TSEG_D | TSEG_E | TSEG_F | TSEG_G,  //G
      TSEG_C | TSEG_D | TSEG_E | TSEG_G,                    // o
      TSEG_C | TSEG_D | TSEG_E | TSEG_G,                    // o
      TSEG_B | TSEG_C | TSEG_D | TSEG_E | TSEG_G            // d

    };

    const uint8_t high[] = {
      TSEG_B | TSEG_C | TSEG_E | TSEG_F | TSEG_G,           // H
      TSEG_E | TSEG_F,                                      // I
      TSEG_A | TSEG_C | TSEG_D | TSEG_E | TSEG_F | TSEG_G,  // G
      TSEG_B | TSEG_C | TSEG_E | TSEG_F | TSEG_G            // H

    };

    const uint8_t low[] = {
      TSEG_D | TSEG_E | TSEG_F,           //L
      TSEG_C | TSEG_D | TSEG_E | TSEG_G,  // o
      TSEG_C | TSEG_D | TSEG_E,           // u
      0x00

    };
    const uint8_t staL[] = {
      TSEG_A | TSEG_C | TSEG_D | TSEG_F | TSEG_G,           // S
      TSEG_F | TSEG_D | TSEG_E | TSEG_G,                    // t
      TSEG_A | TSEG_C | TSEG_D | TSEG_E | TSEG_B | TSEG_G,  //a
      TSEG_D | TSEG_E | TSEG_F                              //L
    };

    display1.setSegments(fish, 4, 0);
    int fishtanklevel = (int)(chinampaData.fishTankMeasuredHeight * 100);
    //display2.showNumberDecEx(fishtanklevel, (0x80 >> 1), false);
    if (chinampaData.secondsSinceLastFishTankData > chinampaData.fishTankStaleDataSeconds) {
      display2.setSegments(staL, 4, 0);
    } else if (chinampaData.fishTankMeasuredHeight >= (chinampaData.fishTankHeight - chinampaData.minimumFishTankLevel)) {
      display2.setSegments(low, 4, 0);
    } else if (chinampaData.fishTankMeasuredHeight < (chinampaData.fishTankHeight - chinampaData.minimumFishTankLevel) && chinampaData.fishTankMeasuredHeight >= (chinampaData.fishTankHeight - chinampaData.maximumFishTankLevel)) {
      display2.setSegments(good, 4, 0);
    } else if (chinampaData.fishTankMeasuredHeight < (chinampaData.fishTankHeight - chinampaData.maximumFishTankLevel)) {
      display2.setSegments(high, 4, 0);
    }

  } else if (currentTimerRecord.second == 5 || currentTimerRecord.second == 25 || currentTimerRecord.second == 45) {

    if (loraActive && sendMessageNow) {
      leds[1] = CRGB(0, 0, 255);
      FastLED.show();
      sendMessage();
      sendMessageNow = false;
      leds[1] = CRGB(0, 255, 0);
      FastLED.show();
    }
  } else if (currentTimerRecord.second == 10 || currentTimerRecord.second == 40) {
    sendMessageNow = true;
    // leds[4] = CRGB(0, 255, 0);
    FastLED.show();
    const uint8_t fflo[] = {
      TSEG_A | TSEG_F | TSEG_E | TSEG_G,  // F
      TSEG_A | TSEG_F | TSEG_E | TSEG_G,  // F
      TSEG_D | TSEG_E | TSEG_F,           //L
      TSEG_C | TSEG_D | TSEG_E | TSEG_G   // o
    };
    if (cleareddisplay1) {
      cleareddisplay1 = false;
      display1.clear();
    }
    display1.setSegments(fflo, 4, 0);
    int ftfr = (int)(chinampaData.fishtankoutflowflowRate * 100);
    display2.showNumberDecEx(ftfr, (0x80 >> 1), false);

  } else if (currentTimerRecord.second == 20 || currentTimerRecord.second == 50) {

    sendMessageNow = true;

    const uint8_t t2[] = {
      TSEG_B | TSEG_C | TSEG_D | TSEG_E | TSEG_F,  // U
      0x00,
      TSEG_F | TSEG_E | TSEG_D | TSEG_G,          // t
      TSEG_A | TSEG_D | TSEG_E | TSEG_F | TSEG_G  //E
    };

    if (cleareddisplay1) {
      cleareddisplay1 = false;
      display1.clear();
    }
    display1.setSegments(t2, 4, 0);
    int value1 = processDisplayValue(chinampaData.microtemperature, &displayData);
    if (displayData.dp > 0) {
      display2.showNumberDecEx(value1, (0x80 >> displayData.dp), false);
    } else {
      display2.showNumberDec(value1, false);
    }
    delay(100);
  } else if (chinampaData.alertstatus && (currentTimerRecord.second == 25 || currentTimerRecord.second == 55)) {

    const uint8_t alrt[] = {
      TSEG_A | TSEG_B | TSEG_C | TSEG_D | TSEG_E | TSEG_G,  // a
      TSEG_D | TSEG_E | TSEG_F,                             // L
      TSEG_G | TSEG_E,                                      //r
      TSEG_F | TSEG_E | TSEG_D | TSEG_G                     // t

    };

    if (cleareddisplay1) {
      cleareddisplay1 = false;
      display1.clear();
    }
    display1.setSegments(alrt, 4, 0);
    display2.showNumberDec(chinampaData.alertcode, false);
    delay(100);
  }

  if (Serial.available() != 0) {
    String command = Serial.readString();
    Serial.print(F("command="));
    Serial.println(command);
    if (command.startsWith("Ping")) {
      Serial.println(F("Ok-Ping"));

    } else if (command.startsWith("printLastFishTankData")) {

      dataManager.printDigitalStablesData(fishTankDSD);
      Serial.println("Ok-printLastFishTankData");
      Serial.flush();
    } else if (command.startsWith("printLastSumpTroughData")) {

      dataManager.printDigitalStablesData(sumpTroughDSD);
      Serial.println("Ok-printLastFishTankData");
      Serial.flush();
    } else if (command.startsWith("printCurrentChinampaData")) {

      dataManager.printChinampaData(chinampaData);
      Serial.println("Ok-printCurrentDSDData");
      Serial.flush();
    } else if (command.startsWith("GetDeviceConfig")) {
      // double latitude = 0.0;
      // double longitude = 0.0;
      String timezoneStr = "AEST-10AEDT,M10.1.0,M4.1.0/3";
      double latitude = -37.13305556;
      double longitude = 144.47472222;
      secretManager.getDeviceConfig(chinampaData.devicename, chinampaData.deviceshortname, timezoneStr, latitude, longitude);
      Serial.print(chinampaData.devicename);
      Serial.print("#");
      Serial.print(chinampaData.deviceshortname);
      Serial.print("#");
      Serial.print(timezoneStr);
      Serial.print("#");
      Serial.print(chinampaData.latitude);
      Serial.print("#");
      Serial.print(chinampaData.longitude);
      Serial.print("#");
      Serial.println(F("Ok-GetDeviceSensorConfig"));
    } else if (command.startsWith("SetDeviceConfig")) {
      // SetDeviceConfig#Chinampa #CHIN #AEST-10AEDT,M10.1.0,M4.1.0/3#-37.13305556#144.47472222#
      String devicename = generalFunctions.getValue(command, '#', 1);
      String deviceshortname = generalFunctions.getValue(command, '#', 2);
      String timezone = generalFunctions.getValue(command, '#', 3);
      Serial.print("deviceshortname=");
      Serial.println(deviceshortname);
      double latitude = generalFunctions.stringToDouble(generalFunctions.getValue(command, '#', 4));
      double longitude = generalFunctions.stringToDouble(generalFunctions.getValue(command, '#', 5));

      uint8_t devicenamelength = devicename.length() + 1;
      devicename.toCharArray(chinampaData.devicename, devicenamelength);
      deviceshortname.toCharArray(chinampaData.deviceshortname, deviceshortname.length() + 1);

      secretManager.saveDeviceConfig(devicename, deviceshortname, timezone, latitude, longitude);

      Serial.println(F("Ok-SetDeviceSensorConfig"));
    } else if (command.startsWith("SetDeviceName")) {
      String devicename = generalFunctions.getValue(command, '#', 1);
      uint8_t devicenamelength = devicename.length() + 1;
      devicename.toCharArray(chinampaData.devicename, devicenamelength);
      Serial.println(F("Ok-SetDeviceName"));
    } else if (command.startsWith("SetDeviceShortName")) {
      String deviceshortname = generalFunctions.getValue(command, '#', 1);
      uint8_t deviceshortnamelength = deviceshortname.length() + 1;
      deviceshortname.toCharArray(chinampaData.deviceshortname, deviceshortnamelength);
      Serial.print(F("digitalStablesData.deviceshortname="));
      Serial.println(chinampaData.deviceshortname);
      Serial.println(F("Ok-SetDeviceShortName"));
    } else if (command.startsWith("SetGroupId")) {
      String grpId = generalFunctions.getValue(command, '#', 1);
      secretManager.setGroupIdentifier(grpId);
      Serial.print(F("set group id to "));
      Serial.println(grpId);

      Serial.println(F("Ok-SetGroupId"));
    } else if (command.startsWith("GetWifiStatus")) {


      uint8_t status = 0;//wifiManager.getWifiStatus();
      Serial.print("WifiStatus=");
      Serial.println(status);


      Serial.println("Ok-GetWifiStatus");

    } else if (command.startsWith("ConfigWifiSTA")) {
      //ConfigWifiSTA#ssid#password
      //ConfigWifiSTA#MainRouter24##VisualizerTestHome#
      String ssid = generalFunctions.getValue(command, '#', 1);
      String password = generalFunctions.getValue(command, '#', 2);
      String hostname = generalFunctions.getValue(command, '#', 3);
      bool staok = false;//wifiManager.configWifiSTA(ssid, password, hostname);
      if (staok) {
        leds[0] = CRGB(0, 0, 255);
      } else {
        leds[0] = CRGB(255, 0, 0);
      }
      FastLED.show();
      Serial.println("Ok-ConfigWifiSTA");

    } else if (command.startsWith("ConfigWifiAP")) {
      //ConfigWifiAP#soft_ap_ssid#soft_ap_password#hostaname
      //ConfigWifiAP#Chinampa##Chinampa

      String soft_ap_ssid = generalFunctions.getValue(command, '#', 1);
      String soft_ap_password = generalFunctions.getValue(command, '#', 2);
      String hostname = generalFunctions.getValue(command, '#', 3);

      bool stat =false; //wifiManager.configWifiAP(soft_ap_ssid, soft_ap_password, hostname);
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
      timeManager.setTime(command);
      Serial.println("Ok-SetTime");

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
      //  inPulse=false;
      Serial.println("Ok-PulseFinished");
      Serial.flush();
      delay(delayTime);

    } else if (command.startsWith("IPAddr")) {
      //  currentIpAddress = generalFunctions.getValue(command, '#', 1);
      Serial.println("Ok-IPAddr");
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("SSID")) {
      String currentSSID = generalFunctions.getValue(command, '#', 1);
      //wifiManager.setCurrentSSID(currentSSID.c_str());
      Serial.println("Ok-currentSSID");
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("GetIpAddress")) {
     // Serial.println(wifiManager.getIpAddress());
      Serial.println("Ok-GetIpAddress");
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("RestartWifi")) {
      //wifiManager.restartWifi();
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


      //  Serial.print(//wifiManager.getSensorData());
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
    LoRa_rxMode();
  }
}


void setStationMode(String ipAddress) {
  Serial.println("settting Station mode, address ");
  Serial.println(ipAddress);
  leds[0] = CRGB(0, 0, 255);
  FastLED.show();
  const uint8_t ip[] = {
    TSEG_F | TSEG_E,                            // I
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
  //  //wifiManager.configWifiAP("PanchoTankFlowV1", "", "PanchoTankFlowV1");
  String apAddress ="";// //wifiManager.getApAddress();
  Serial.println("settting AP mode, address ");
  Serial.println(apAddress);
  const uint8_t ap[] = {
    TSEG_F | TSEG_G | TSEG_A | TSEG_B | TSEG_C | TSEG_E,  // A
    TSEG_F | TSEG_G | TSEG_A | TSEG_B | TSEG_E            // P
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
