#include "Arduino.h"
#include <LittleFS.h>
#include <NewPing.h>
#include <ChinampaWifiManager.h>
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
#include <VitalSignsTracker.h>
#include <esp_arduino_version.h>
#if ESP_ARDUINO_VERSION_MAJOR >= 3
#include <esp_core_dump.h>  // crash capture needs the IDF 5 core-dump API (core 3.x); chinampa builds on core 2.0.17 too
#endif

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

//bool turningPumpOn = false;
//bool turningPumpOff = false;

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
#define RSSI_THRESHOLD -85  // RSSI threshold in dBm

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

// Vital signs - wall powered and never sleeps, so only the reset, uptime (awakeSecondsTotal),
// TX duration and TX-failure parts apply. Sent after the data pulse every
// VITAL_SIGNS_INTERVAL_MS, and right after every boot. See VitalSignsTracker.h and
// Projects/Annabelle/VitalSigns_Design.pdf.
VitalSignsTracker vitalSigns;
const uint32_t FIRMWARE_BUILD = VitalSignsTracker::buildStamp(__DATE__, __TIME__);
#define VITAL_SIGNS_GAP_MS 1000                          // after the data packet, so Annabelle has read it
#define VITAL_SIGNS_INTERVAL_MS (10UL * 60UL * 1000UL)
unsigned long lastVitalSignsMs = 0;

// Event log (LittleFS, survives resets and power loss): boots with their reset reason, crashes
// (from the core dump), WiFi drops with the ESP32's disconnect reason, reconnects, and Sump pull
// failures. Read it with the serial command GetEventLog.
//
// Why: chinampa pulls the Sump data over the Sump's own WiFi (SSID SumpTrough, 192.168.4.1).
// Nothing used to reconnect after a drop, and if the Sump was unreachable at boot setApMode()
// switched to access-point mode for good - both leave LED 4 green/red (no WiFi data) until a
// manual reset. checkWifi() now retries.
#define EVENT_LOG_FILE "/eventlog.txt"
#define EVENT_LOG_OLD_FILE "/eventlog.old"
#define EVENT_LOG_MAX_BYTES 16384        // then the log moves to EVENT_LOG_OLD_FILE and starts again
#define LAST_CRASH_FILE "/lastcrash.txt" // newest crash summary line + its fingerprint
#define WIFI_RETRY_SECONDS 120           // no Sump WiFi this long -> start a new connection attempt
#define SUMP_PULL_FAIL_LOG 3             // log when this many WiFi pulls in a row fail
bool fsMounted = false;
String lastCrash = "";
volatile bool wifiDisconnectEvent = false;  // set by the WiFi event task, handled in checkWifi()
volatile uint8_t wifiLastDisconnectReason = 0;
bool wifiWasConnected = false;
uint32_t wifiDownSeconds = 0;               // consecutive seconds without the Sump WiFi
uint16_t wifiDropCount = 0;                 // since boot
uint16_t wifiReconnectCount = 0;            // since boot
bool wifiDroppedSinceVitals = false;
uint16_t sumpPullFailStreak = 0;
// LoRa receptions per source, logged every hour - shows whether the Sump's LoRa packets get
// through now that the radio goes back to receive right after repeating a packet.
uint16_t loraRxFish = 0, loraRxSump = 0, loraRxOther = 0;
uint8_t loraRxHour = 255;  // 255 = first hour after boot, partial, not logged
int wifiLastRssi = 0;                       // while connected; logged with a drop (RSSI reads 0 once down)
volatile bool loraDio0Fired = false;        // set by onLoraDio0(), handled at the top of loop()
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

ChinampaWifiManager wifiManager(Serial, LittleFS, timeManager, secretManager, chinampaData, chinampaConfigData);

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

// DIO0 interrupt: only set a flag. arduino-LoRa's own LoRa.onReceive() handler reads the radio
// over SPI inside the interrupt, and SPI on ESP32 core 3.x takes a mutex, which must never be
// waited on in an ISR (Annabelle's xQueueSemaphoreTake assert PANIC, 2026-10-05). The packet is
// read in loop() with LoRa.parsePacket().
void IRAM_ATTR onLoraDio0() {
  loraDio0Fired = true;
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

    if (strncmp(tempData.devicename, "FISHTANK", 8) == 0) loraRxFish++;
    else if (strcmp(tempData.devicename, "SumpTrough") == 0) loraRxSump++;
    else loraRxOther++;

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
      chinampaData.lux = 0;  // lux removed from DigitalStablesData 2026-09-01; field kept so ChinampaData stays 248 bytes
      
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
    return LORA_INIT_FAILED;
  }

  // 1. Prepare for a clean reading
  LoRa.idle();   
  LoRa.receive(); 
  
  // 2. Faster Sampling
  // We reduce the delay and sample count to minimize "blind time"
  const int SAMPLES = 4;
  float rssiSum = 0;

  for (int i = 0; i < SAMPLES; i++) {
    rssiSum += LoRa.rssi();
    delayMicroseconds(500); // Very fast check
  }

  avgRssi = rssiSum / SAMPLES;

  // 3. Forgiving Threshold
  // Using -85 allows the system to ignore background greenhouse noise
  // but still detect another LoRa unit nearby.
  if (avgRssi > -85) { 
    
    LoRa.idle();
    return LORA_CHANNEL_BUSY;
  }

  // Clear for transmission

  errorManager.clearLoRaError(LORA_CHANNEL_BUSY);
  return LORA_OK;
}

LoRaError performCAD1() {
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
  uint8_t result = 99;
  int retries = 0;
  boolean keepGoing = true;
  long startsendingtime = millis();

  // 1. SWITCH TO TX MODE
  LoRa_txMode();

  while (keepGoing) {
    // Check if the channel is clear
    cadResult = performCAD();
    
    if (cadResult == LORA_OK) {
      // 2. TRANSMIT ONCE (The original code sent twice, causing "deafness")
      LoRa.beginPacket();
      // Send the rebroadcast data
      LoRa.write((uint8_t *)&chinampaData, sizeof(ChinampaData));
      
      // Use false (blocking) to ensure transmission finishes before we switch back to RX
      vitalSigns.beginTx(nullptr);  // no current sensor: duration + failure count only
      bool txOk = LoRa.endPacket(false);
      vitalSigns.endTx(txOk);
      if (!txOk) {
        result = LORA_TX_FAILED;
      } else {
        result = LORA_OK;
        msgCount++; // Only increment if actually sent
      }
      
     
      keepGoing = false;
    } 
    else if (cadResult == LORA_CHANNEL_BUSY) {
      // 3. CHANNEL BUSY HANDLING
      // If the Fish Tank or Sump is currently talking, we wait a moment
      retries++;
      if (retries < MAX_RETRIES) {
        int backoff = random(MIN_BACKOFF, MAX_BACKOFF); // Keep backoff short for responsiveness
       // if(debug) Serial.println("Channel busy, retrying in " + String(backoff) + "ms");
        delay(backoff);
      } else {
        result = LORA_MAX_RETRIES_REACHED;
        keepGoing = false;
      }
    } else {
      // Hardware error
      result = cadResult;
      keepGoing = false;
    }
  }

  // 4. RETURN TO RECEIVE MODE IMMEDIATELY
  // This is vital so we don't miss the next incoming sensor packet
  LoRa_rxMode();
}

void sendMessage11() {
  // REMOVE the first LoRa.beginPacket/endPacket block entirely.
  
  uint8_t result = 99;
  int retries = 0;
  boolean keepGoing = true;
  LoRa_txMode();
  while (keepGoing) {
    cadResult = performCAD();
    if (cadResult == LORA_OK) {
      LoRa.beginPacket();
      LoRa.write((uint8_t *)&chinampaData, sizeof(ChinampaData));
      LoRa.endPacket(true); // true = async
      result = LORA_OK;
      keepGoing = false;
    } else {
      // If busy, don't use long delays. 
      // A 600s gap in your data suggests this loop is getting stuck.
      retries++;
      if(retries >= MAX_RETRIES) keepGoing = false;
      delay(random(50, 200)); // Keep backoff short for a 3m distance
    }
  }

  // CRITICAL: Immediately return to listening
  LoRa_rxMode(); 
}
void sendMessage1() {
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

  bool gotData = wifiManager.pullSumpDataViaWifi();
  if(gotData){
    leds[4] = CRGB(0, 255, 255);
    FastLED.show();
    if (sumpPullFailStreak >= SUMP_PULL_FAIL_LOG) eventLog("sump pull ok again after " + String(sumpPullFailStreak) + " failures");
    sumpPullFailStreak = 0;
  } else if (WiFi.status() == WL_CONNECTED) {
    // WiFi is up but the Sump did not answer (HTTP error or bad JSON) - a Sump-side problem
    sumpPullFailStreak++;
    if (sumpPullFailStreak == SUMP_PULL_FAIL_LOG) eventLog("sump pull failing with WiFi connected (" + String(SUMP_PULL_FAIL_LOG) + " in a row)");
  }
   Serial.println("gotData=" + String(gotData));
  // Serial.println("minimumFishTankHeight=" + String(chinampaData.minimumFishTankHeight));
  boolean keepgoing = true;
  chinampaData.alertstatus = false;
  chinampaData.alertcode = 0;
  boolean sendAMessage=false;
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
      if (chinampaData.fishtankoutflowsolenoidrelaystatus){
        sendAMessage=true;
      }
      chinampaData.fishtankoutflowsolenoidrelaystatus = false;
      leds[7] = CRGB(0, 0, 0);
       
      //
      // now check the pump
      //
      if (chinampaData.secondsSinceLastSumpTroughData > chinampaData.sumpTroughStaleDataSeconds) {
        digitalWrite(PUMP_RELAY_PIN, LOW);
        leds[6] = CRGB(255, 0, 0);
        if (chinampaData.pumprelaystatus){
          // because the pump is on but we
          // are about to turn it off sendMessage
          sendAMessage=true;
        }
        chinampaData.pumprelaystatus = false;
      } else {
        if (chinampaData.sumpTroughMeasuredHeight >= (chinampaData.sumpTroughHeight - chinampaData.minimumSumpTroughLevel)) {
          digitalWrite(PUMP_RELAY_PIN, LOW);
          leds[6] = CRGB(255, 0, 255);
          chinampaData.alertstatus = true;
          chinampaData.alertcode = 5;
          if (chinampaData.pumprelaystatus){
            // because the pump is on but we
            // are about to turn it off sendMessage
            sendAMessage=true;
          }
          chinampaData.pumprelaystatus = false;

        } else {
          if (!chinampaData.pumprelaystatus){
            // because the pump is off but we
            // are about to turn it on sendMessage
            sendAMessage=true;
          }
          digitalWrite(PUMP_RELAY_PIN, HIGH);
          chinampaData.pumprelaystatus = true;
          leds[6] = CRGB(0, 255, 0);
        }

        Serial.println("line 328 1");
        keepgoing = false;
      }
      FastLED.show();
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
        if (chinampaData.pumprelaystatus){
            // because the pump is on but we
            // are about to turn it off sendMessage
            sendAMessage=true;
        }
        digitalWrite(PUMP_RELAY_PIN, LOW);
        chinampaData.pumprelaystatus = false;
        Serial.println("line 508  sump stale");
        leds[6] = CRGB(255, 0, 0);
      } else {
        if (chinampaData.sumpTroughMeasuredHeight >= (chinampaData.sumpTroughHeight - chinampaData.minimumSumpTroughLevel)) {
          if (chinampaData.pumprelaystatus){
            // because the pump is on but we
            // are about to turn it off sendMessage
            sendAMessage=true;
          }
          digitalWrite(PUMP_RELAY_PIN, LOW);
          chinampaData.pumprelaystatus = false;
          Serial.println("line 513  sump too low pump off");
          leds[6] = CRGB(0, 0, 0);
          leds[6] = CRGB(255, 0, 255);
          chinampaData.alertstatus = true;
          chinampaData.alertcode = 5;
        } else {
          if (!chinampaData.pumprelaystatus){
            // because the pump is off but we
            // are about to turn it on sendMessage
            sendAMessage=true;
          }
          digitalWrite(PUMP_RELAY_PIN, HIGH);
          chinampaData.pumprelaystatus = true;
          Serial.println("line 513  sump above min pump on");
          leds[6] = CRGB(0, 255, 0);
        }
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
      if (!chinampaData.fishtankoutflowsolenoidrelaystatus){
        // because the solenoid is off but we
        // are about to turn it on sendMessage
        sendAMessage=true;
      }
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


if (chinampaData.secondsSinceLastFishTankData < chinampaData.fishTankStaleDataSeconds && 
    chinampaData.secondsSinceLastSumpTroughData < chinampaData.sumpTroughStaleDataSeconds &&
    chinampaData.sumpTroughMeasuredHeight >= (chinampaData.sumpTroughHeight - chinampaData.minimumSumpTroughLevel) &&
    chinampaData.fishTankMeasuredHeight >= (chinampaData.fishTankHeight - chinampaData.minimumFishTankLevel)
  ) {
    if( chinampaData.alertcode != 6){
      // because the system just detected
      // that there is not enough water
      // are about to turn it on sendMessage
        sendAMessage=true;
    }
      chinampaData.alertstatus = true;
      chinampaData.alertcode = 6;
      leds[5] = CRGB(255, 0, 255);
      leds[6] = CRGB(255, 0, 255);
      leds[7] = CRGB(255, 0, 255);

}

if(chinampaData.fishTankMeasuredHeight==0){
   if( chinampaData.alertcode != 7){
        sendAMessage=true;
    }
   chinampaData.alertstatus = true;
    chinampaData.alertcode = 7;
    leds[5] = CRGB(255, 0, 255);
    leds[6] = CRGB(255, 0, 255);
    leds[7] = CRGB(255, 0, 255);
    digitalWrite(PUMP_RELAY_PIN, LOW);
    digitalWrite(FISH_OUTPUT_SOLENOID_RELAY, LOW);
    chinampaData.pumprelaystatus = false;
    chinampaData.fishtankoutflowsolenoidrelaystatus = false;
}

if(chinampaData.sumpTroughMeasuredHeight==0){
   if( chinampaData.alertcode != 8){
        sendAMessage=true;
    }
   chinampaData.alertstatus = true;
      chinampaData.alertcode = 8;
      leds[5] = CRGB(255, 0, 255);
      leds[6] = CRGB(255, 0, 255);
      leds[7] = CRGB(255, 0, 255);
    digitalWrite(PUMP_RELAY_PIN, LOW);
    digitalWrite(FISH_OUTPUT_SOLENOID_RELAY, LOW);
    chinampaData.pumprelaystatus = false;
    chinampaData.fishtankoutflowsolenoidrelaystatus = false;


}

if(chinampaData.fishTankMeasuredHeight==0 && chinampaData.sumpTroughMeasuredHeight==0){
    if( chinampaData.alertcode != 9){
        sendAMessage=true;
    }
   chinampaData.alertstatus = true;
   
      chinampaData.alertcode = 9;
      leds[5] = CRGB(255, 0, 255);
      leds[6] = CRGB(255, 0, 255);
      leds[7] = CRGB(255, 0, 255);
    digitalWrite(PUMP_RELAY_PIN, LOW);
    digitalWrite(FISH_OUTPUT_SOLENOID_RELAY, LOW);
    chinampaData.pumprelaystatus = false;
    chinampaData.fishtankoutflowsolenoidrelaystatus = false;


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

 if (sendAMessage) {
    sendMessage();
 }
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

  //  serialNumber = //wifiManager.getMacAddress();
  wifiManager.setSerialNumber(serialNumber);
  wifiManager.setLora(loraActive);
  String ssid = wifiManager.getSSID();
  String ipAddress = "";
  uint8_t ipi;
  if (stationmode) {
    ipAddress = wifiManager.getIpAddress();
      Serial.print("ipaddress=");
     Serial.println(ipAddress);

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




// XOR over every byte except the checksum itself - same scheme as the other LoRa records.
uint8_t vitalSignsChecksum(const VitalSignsRecord &r) {
  const uint8_t *p = (const uint8_t *)&r;
  size_t offset = offsetof(VitalSignsRecord, checksum);
  uint8_t checksum = 0;
  for (size_t i = 0; i < sizeof(VitalSignsRecord); i++) {
    if (i != offset) checksum ^= p[i];
  }
  return checksum;
}

// Sends the VitalSignsRecord VITAL_SIGNS_GAP_MS after the data packet (Annabelle has one LoRa FIFO).
void sendVitalSigns() {
  VitalSignsRecord record = vitalSigns.buildRecord(chinampaData.serialnumberarray, FIRMWARE_BUILD, wifiStatusMask());
  wifiDroppedSinceVitals = false;
  record.totpcode = secretManager.generateCode();
  record.checksum = vitalSignsChecksum(record);
  delay(VITAL_SIGNS_GAP_MS);
  LoRa_txMode();
  bool sent = false;
  for (int retries = 0; retries < MAX_RETRIES && !sent; retries++) {
    cadResult = performCAD();
    if (cadResult == LORA_OK) {
      LoRa.beginPacket();
      LoRa.write((uint8_t *)&record, sizeof(record));
      sent = LoRa.endPacket(false);
      break;
    } else if (cadResult == LORA_CHANNEL_BUSY) {
      delay(random(MIN_BACKOFF, MAX_BACKOFF));
    } else {
      break;
    }
  }
  LoRa_rxMode();
  if (sent) vitalSigns.markSent();
  lastVitalSignsMs = millis();
}

void eventLog(const String &msg) {
  char ts[24];
  snprintf(ts, sizeof(ts), "%04d-%02d-%02d %02d:%02d:%02d ", currentTimerRecord.year, currentTimerRecord.month,
           currentTimerRecord.date, currentTimerRecord.hour, currentTimerRecord.minute, currentTimerRecord.second);
  Serial.print("EventLog ");
  Serial.print(ts);
  Serial.println(msg);
  if (!fsMounted) return;
  File f = LittleFS.open(EVENT_LOG_FILE, "a");
  if (!f) return;
  if (f.size() > EVENT_LOG_MAX_BYTES) {
    f.close();
    LittleFS.remove(EVENT_LOG_OLD_FILE);
    LittleFS.rename(EVENT_LOG_FILE, EVENT_LOG_OLD_FILE);
    f = LittleFS.open(EVENT_LOG_FILE, "a");
    if (!f) return;
  }
  f.print(ts);
  f.println(msg);
  f.close();
}

void printEventLog() {
  Serial.print("LastCrash=");
  Serial.println(lastCrash.length() ? lastCrash : "none");
  Serial.println("WifiDrops=" + String(wifiDropCount) + " WifiReconnects=" + String(wifiReconnectCount) +
                 " WifiDownSeconds=" + String(wifiDownSeconds) + " WifiMask=" + String(wifiStatusMask()));
  const char *files[] = { EVENT_LOG_OLD_FILE, EVENT_LOG_FILE };
  for (const char *name : files) {
    File in = fsMounted ? LittleFS.open(name, "r") : File();
    if (!in) continue;
    while (in.available()) Serial.write(in.read());
    in.close();
  }
}

const char *resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXT";
    case ESP_RST_SW: return "SW";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    default: return "OTHER";
  }
}

const char *wifiReasonName(uint8_t reason) {
  switch (reason) {
    case 2: return "AUTH_EXPIRE";
    case 4: return "ASSOC_EXPIRE";
    case 8: return "ASSOC_LEAVE";
    case 15: return "4WAY_HANDSHAKE_TIMEOUT";
    case 200: return "BEACON_TIMEOUT";
    case 201: return "NO_AP_FOUND";
    case 202: return "AUTH_FAIL";
    case 203: return "ASSOC_FAIL";
    case 204: return "HANDSHAKE_TIMEOUT";
    case 205: return "CONNECTION_FAIL";
    default: return "";
  }
}

// Runs in the WiFi event task, not loop(): only record, checkWifi() does the logging.
void onWifiEvent(arduino_event_id_t event, arduino_event_info_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    wifiLastDisconnectReason = info.wifi_sta_disconnected.reason;
    wifiDisconnectEvent = true;
  }
}

// Reported in the VitalSigns i2cDeviceMask field (device-specific bits):
// bit 0 = connected to the Sump WiFi now, bit 1 = in access-point mode (not even trying the Sump),
// bit 2 = the WiFi dropped at least once since the previous VitalSigns record,
// bit 3 = WiFi up but the Sump is not answering (SUMP_PULL_FAIL_LOG pulls in a row failed).
uint8_t wifiStatusMask() {
  uint8_t mask = 0;
  if (WiFi.status() == WL_CONNECTED) mask |= 0x01;
  if (!(WiFi.getMode() & WIFI_MODE_STA)) mask |= 0x02;
  if (wifiDroppedSinceVitals) mask |= 0x04;
  if (sumpPullFailStreak >= SUMP_PULL_FAIL_LOG) mask |= 0x08;
  return mask;
}

// Starts a connection to the Sump network without waiting (connectSTA() blocks for up to 21 s,
// too long to stop the pump/solenoid logic every couple of minutes). Also leaves the AP fallback.
void startWifiReconnect() {
  WiFi.disconnect(false);
  if (!(WiFi.getMode() & WIFI_MODE_STA)) WiFi.mode(WIFI_STA);
  WiFi.begin(secretManager.getSSID().c_str(), secretManager.getWifiPassword().c_str());
}

// Once a second from loop().
void checkWifi() {
  bool connected = WiFi.status() == WL_CONNECTED;
  if (wifiDisconnectEvent) {
    wifiDisconnectEvent = false;
    if (wifiWasConnected) {  // retries while already down also raise this event - log only the drop
      uint8_t reason = wifiLastDisconnectReason;
      eventLog("wifi dropped reason=" + String(reason) + " " + wifiReasonName(reason) + " lastRssi=" + String(wifiLastRssi));
    }
  }
  if (connected) {
    if (!wifiWasConnected) {
      if (wifiDownSeconds > 0) {
        wifiReconnectCount++;
        eventLog("wifi connected ip=" + WiFi.localIP().toString() + " after " + String(wifiDownSeconds) + " s down");
      } else {
        eventLog("wifi connected ip=" + WiFi.localIP().toString());
      }
    }
    wifiWasConnected = true;
    wifiDownSeconds = 0;
    wifiLastRssi = WiFi.RSSI();
    return;
  }
  if (wifiWasConnected) {
    wifiWasConnected = false;
    wifiDropCount++;
    wifiDroppedSinceVitals = true;
  }
  wifiDownSeconds++;
  if (wifiDownSeconds % WIFI_RETRY_SECONDS == 0) {
    bool apMode = !(WiFi.getMode() & WIFI_MODE_STA);
    eventLog(String("wifi down ") + wifiDownSeconds + " s" + (apMode ? " (AP mode)" : "") + ", reconnecting");
    startWifiReconnect();
  }
}

// Same capture as Annabelle: summary of the core dump of the last panic, one line, kept in
// LAST_CRASH_FILE and the event log. Decode with Projects/Annabelle/claude/decode_crash.sh (needs
// the .elf of the build that crashed - save it after every flash).
void recordCoreDump() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  String knownFingerprint = "";
  File f = LittleFS.open(LAST_CRASH_FILE, "r");
  if (f) {
    lastCrash = f.readStringUntil('\n');
    knownFingerprint = f.readStringUntil('\n');
    f.close();
    lastCrash.trim();
    knownFingerprint.trim();
  }
  if (esp_core_dump_image_check() != ESP_OK) return;  // no (valid) dump in flash

  esp_core_dump_summary_t *summary = (esp_core_dump_summary_t *)malloc(sizeof(esp_core_dump_summary_t));
  if (summary == nullptr) return;
  if (esp_core_dump_get_summary(summary) != ESP_OK) {
    free(summary);
    return;
  }
  size_t dumpAddr = 0, dumpSize = 0;
  esp_core_dump_image_get(&dumpAddr, &dumpSize);
  uint32_t fp = (uint32_t)dumpSize ^ summary->exc_pc ^ summary->exc_tcb;
  for (uint32_t i = 0; i < summary->exc_bt_info.depth && i < 16; i++) fp = fp * 31 + summary->exc_bt_info.bt[i];
  char fingerprint[12];
  snprintf(fingerprint, sizeof(fingerprint), "%08lx", (unsigned long)fp);
  if (knownFingerprint == fingerprint) {  // already recorded
    free(summary);
    return;
  }

  char reason[200] = "";
  if (esp_core_dump_get_panic_reason(reason, sizeof(reason)) != ESP_OK) strcpy(reason, "?");
  char buf[64];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d;", currentTimerRecord.year, currentTimerRecord.month,
           currentTimerRecord.date, currentTimerRecord.hour, currentTimerRecord.minute, currentTimerRecord.second);
  String line = String(buf) + reason + ";" + String(summary->exc_task) + ";";
  snprintf(buf, sizeof(buf), "0x%08lx;", (unsigned long)summary->exc_pc);
  line += buf;
  for (uint32_t i = 0; i < summary->exc_bt_info.depth && i < 16; i++) {
    snprintf(buf, sizeof(buf), "%s0x%08lx", i > 0 ? " " : "", (unsigned long)summary->exc_bt_info.bt[i]);
    line += buf;
  }
  if (summary->exc_bt_info.corrupted) line += " (corrupted)";
  line += ";";
  for (int i = 0; i < 8 && summary->app_elf_sha256[i]; i++) line += (char)summary->app_elf_sha256[i];
  const esp_core_dump_summary_extra_info_t &ex = summary->ex_info;
  snprintf(buf, sizeof(buf), ";cause=%lu vaddr=0x%08lx a0=0x%08lx", (unsigned long)ex.exc_cause,
           (unsigned long)ex.exc_vaddr, (unsigned long)((ex.exc_a[0] & 0x3FFFFFFFUL) | 0x40000000UL));
  line += buf;
  for (int i = 0; i < EPCx_REGISTER_COUNT; i++) {
    if (!(ex.epcx_reg_bits & (1 << i))) continue;
    snprintf(buf, sizeof(buf), " epc%d=0x%08lx", i + 1, (unsigned long)ex.epcx[i]);
    line += buf;
  }
  free(summary);
  line.replace("\n", " ");
  line.replace("\r", " ");
  lastCrash = line;

  File out = LittleFS.open(LAST_CRASH_FILE, "w");
  if (out) {
    out.println(lastCrash);
    out.println(fingerprint);
    out.close();
  }
  eventLog("crash " + lastCrash);
#endif
}

void setup() {
  vitalSigns.captureBoot();  // reset reason only, no I/O
  Serial.begin(115200);
  Wire.begin();

  //
  // data from cofiguration
  //
  double latitude = -37.13305556;
  double longitude = 144.47472222;

  secretManager.getDeviceConfig(chinampaData.devicename, chinampaData.deviceshortname, timezone, latitude, longitude);
  secretManager.saveWifiParameters("SumpTrough", "", "Chinampa", "",  "Chinampa", true);
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
  vitalSigns.recordBoot(chinampaData.secondsTime);
  fsMounted = LittleFS.begin(false);  // no format: the web UI files live there
  eventLog(String("boot reason=") + resetReasonName(esp_reset_reason()) + (fsMounted ? "" : " (LittleFS not mounted, log not saved)"));
  if (fsMounted) recordCoreDump();
  WiFi.onEvent(onWifiEvent);
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
    chinampaData.sensorstatus[i] = false;
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

    LoRa.setTxPower(17);
    LoRa.setSpreadingFactor(9);
    LoRa.enableCrc();
    LoRa.setSignalBandwidth(125E3);
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
    Serial.print(F("Before Starting Wifi"));
    wifiManager.start();
    initiatedWifi = true;
  }
  Serial.println("Started wifi");


  bool stationmode = wifiManager.getStationMode();
  chinampaData.internetAvailable =wifiManager.getInternetAvailable();

  Serial.print("Starting wifi stationmode=");
  Serial.print(stationmode);

  Serial.print("  internetAvailable=");
  Serial.println(chinampaData.internetAvailable);


    serialNumber =wifiManager.getMacAddress();
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



  internetAvailable = false;//wifiManager.getInternetAvailable();

  pinMode(RTC_BATT_VOLT, INPUT);
  //pinMode(OP_MODE, INPUT_PULLUP);

  if (loraActive) {
    // LoRa_rxMode();
    // LoRa.setSyncWord(0xF3);
    attachInterrupt(digitalPinToInterrupt(LORA_DI0), onLoraDio0, RISING);
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
  if (loraDio0Fired) {
    loraDio0Fired = false;
    int packetSize = LoRa.parsePacket();
    if (packetSize > 0) {
      loraPacketSize = packetSize;
      loraReceived = true;
    } else {
      LoRa_rxMode();  // DIO0 without a good packet (CRC error, TxDone): back to continuous receive
    }
  }
  if (clockTicked) {
    portENTER_CRITICAL(&mux);
    clockTicked = false;
    portEXIT_CRITICAL(&mux);
    secondsSinceLastDataSampling++;
    currentTimerRecord = timeManager.now();
    wifiManager.setCurrentTimerRecord(currentTimerRecord);
    chinampaData.secondsTime = timeManager.getCurrentTimeInSeconds(currentTimerRecord);
    //  Serial.println("chinampaData.secondsSinceLastSumpTroughData=" +  String(chinampaData.secondsSinceLastSumpTroughData));
    chinampaData.secondsSinceLastFishTankData++;
    chinampaData.secondsSinceLastSumpTroughData++;
    dsUploadTimer.tick();
    checkWifi();
    if (currentTimerRecord.hour != loraRxHour) {  // hour changed (robust to a missed :00 tick)
      if (loraRxHour != 255) {
        eventLog("lora rx last hour: fish=" + String(loraRxFish) + " sump=" + String(loraRxSump) + " other=" + String(loraRxOther) +
                 " | wifi drops since boot=" + String(wifiDropCount));
      }
      loraRxHour = currentTimerRecord.hour;
      loraRxFish = loraRxSump = loraRxOther = 0;
    }

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
    LoRa_rxMode();  // parsePacket() leaves the radio idle after a packet
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
      if (vitalSigns.resetReportPending() || millis() - lastVitalSignsMs > VITAL_SIGNS_INTERVAL_MS) {
        sendVitalSigns();
      }
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
    if (command.startsWith("GetEventLog")) {
      printEventLog();
      Serial.println("Ok-GetEventLog");
    } else if (command.startsWith("ClearEventLog")) {
      if (fsMounted) {
        LittleFS.remove(EVENT_LOG_FILE);
        LittleFS.remove(EVENT_LOG_OLD_FILE);
      }
      Serial.println("Ok-ClearEventLog");
    } else if (command.startsWith("Ping")) {
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


      uint8_t status = wifiManager.getWifiStatus();
      Serial.print("WifiStatus=");
      Serial.println(status);


      Serial.println("Ok-GetWifiStatus");

    } else if (command.startsWith("ConfigWifiSTA")) {
      //ConfigWifiSTA#ssid#password
      //ConfigWifiSTA#MainRouter24##VisualizerTestHome#
     // ConfigWifiSTA#SumpTrough##Chinampa#
      String ssid = generalFunctions.getValue(command, '#', 1);
      String password = generalFunctions.getValue(command, '#', 2);
      String hostname = generalFunctions.getValue(command, '#', 3);
      bool staok = wifiManager.configWifiSTA(ssid, password, hostname);
      secretManager.saveWifiParameters("SumpTrough", "", "Chinampa", "",  "Chinampa", true);
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

      bool stat =wifiManager.configWifiAP(soft_ap_ssid, soft_ap_password, hostname);
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


    //    Serial.print(wifiManager.getSensorData());
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
  wifiManager.configWifiAP("Chinampa", "", "Chinampa");
  String apAddress =wifiManager.getApAddress();
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




