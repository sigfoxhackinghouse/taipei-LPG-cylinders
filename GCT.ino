/*
 GCT project by Guillaume Pigeard
 under MIT license

 Release Notes:

 v1.0 [05/06/2019] - Initial release
 ------------------------------------
*/


#define DEBUG
#define USE_SIGFOX

#ifdef DEBUG
 #define DEBUG_PRINT(x)  Serial.print(x)
 #define DEBUG_PRINTLN(x)  Serial.println(x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTLN(x)
#endif

#if defined(USE_SIGFOX) && !defined(DEBUG)
 #define SENDMESSAGE(x) transceiver.sendMessage(x)
#elif defined(USE_SIGFOX) && defined(DEBUG)
 #define SENDMESSAGE(x) transceiver.sendMessage(x);Serial.print("SF Send.Message: ");Serial.println(x)
#else
 #define SENDMESSAGE(x) Serial.print("SF Send.Message: ");Serial.println(x)
#endif

#define HEADER_HEARTBEAT 0b1000
#define HEADER_TAMPERED  0b1001
#define HEADER_UPSIDE    0b1010
#define HEADER_MAC       0b1011
#define HEADER_ACTIVATED 0b1100

#define SHAKE_MMS_LIMIT   1.2
#define UPSIDE_MMS_LIMIT  7

#define DELAY_LOOP       250
#define DELAY_UPSIDE    5000
#define TRIGGER_UPSIDE  3000
#define DELAY_SHAKING   5000
#define TRIGGER_SHAKING 2000

#define TAMPERED_PIN 4

#include <WiFi.h>
#include <SIGFOX.h>
#include <MPU9250.h>
#include <esp_system.h>
#include <Preferences.h>

Preferences preferences;

static const uint8_t Wisol_TX = 23;
static const uint8_t Wisol_RX = 19;
static const String device = "g88pi";  //  Set this to your device name if you're using UnaBiz Emulator.
static const bool useEmulator = false;  //  Set to true if using UnaBiz Emulator.
static const bool echo = true;  //  Set to true if the SIGFOX library should display the executed commands.
static const Country country = COUNTRY_TW;  //  Set this to your country to configure the SIGFOX transmission frequencies.
static Wisol transceiver(country, useEmulator, device, echo, Wisol_RX, Wisol_TX);

const int wdtTimeout = 15000;  //time in ms to trigger the watchdog
hw_timer_t *timer = NULL;

MPU9250 IMU(Wire,0x68);

bool bActivated = false;
byte dataArray[12];
char outputBuffer[25];

typedef struct infoEvent {
  unsigned long start = 0;
  unsigned long duration = 0;
  unsigned long last = 0;
  bool active = false;
};

infoEvent instantShaking;
infoEvent instantUpsideDown;
infoEvent tampering;

void IRAM_ATTR resetModule() {
  ets_printf("reboot\n");
  esp_restart();
}

void initMessage() {
  for (int i = 0; i < sizeof(dataArray); i++) {
    dataArray[i]=0;
  }
  preferences.putBytes("dataArray", dataArray, 12);
  outputBuffer[0]='\0';
}

void sendData(int length = 12) {
  for (int i = 0; i < sizeof(dataArray); i++) {
    sprintf(outputBuffer + i * 2, "%02x", dataArray[i]);
  }
  //outputBuffer[length * 2 + 1] = '\0';
  DEBUG_PRINT("Sending : ");
  DEBUG_PRINTLN(outputBuffer);
  preferences.putBytes("dataArray", dataArray, 12); 
  SENDMESSAGE(outputBuffer);
  initMessage();
}

int sendMacMessage(int header = HEADER_MAC){
  initMessage();
  int nbNetworks = WiFi.scanNetworks();
  int macNb = 0;
  for (int i = 0; i < nbNetworks; ++i)
  {
    uint8_t* mac = WiFi.BSSID(i); 
    if((mac[0] & 0b00000010) == 0 && (mac[0] & 0b00000001) == 0) {
      if(macNb == 0) {
        dataArray[0] = header<<4 | mac[0]>>4;
        dataArray[1] = (mac[0]<<4 & 0b11000000) | mac[1]>>2;
        dataArray[2] = mac[1]<<6 | mac[2]>>2;
        dataArray[3] = mac[2]<<6 | mac[3]>>2;
        dataArray[4] = mac[3]<<6 | mac[4]>>2;
        dataArray[5] = mac[4]<<6 | mac[5]>>2;
        dataArray[6] = mac[5]<<6;  
        macNb++;
      }
      else if (macNb == 1) {
        dataArray[6] |= mac[0]>>2;
        dataArray[7]  = mac[1];
        dataArray[8]  = mac[2];
        dataArray[9]  = mac[3];
        dataArray[10] = mac[4];
        dataArray[11] = mac[5];
        dataArray[12] = mac[6];
        macNb++;
        break;
      }
    }
  }
  if(macNb < 2) {
    return -1;
  } else {
    sendData();
  }
}

int sendActivatedMessage(){
  return sendMacMessage(HEADER_ACTIVATED);
}

int sendTamperedMessage(){
  initMessage();
  dataArray[0] = HEADER_TAMPERED<<4;
  sendData(1);
}


int sendUpsideDownMessage(){
  unsigned long totalTime = (millis() - instantUpsideDown.start) / 1000;
  unsigned long duration = instantUpsideDown.duration / 1000;
  initMessage();
  dataArray[0] = HEADER_UPSIDE<<4 | (duration & 0x3C000) >> 14; //4b
  dataArray[1] = (duration & 0x03FC0) >> 6;
  dataArray[2] = (duration & 0x0003F) << 2 | (totalTime & 0x30000) >> 16;
  dataArray[3] = (totalTime & 0x0FF00) >> 8;
  dataArray[4] = (totalTime & 0x000FF);
  sendData(5);
}

void setup()   {
  #if  defined (DEBUG)  || !defined (USE_SIGFOX)
    Serial.begin(115200);
  #endif
  pinMode(TAMPERED_PIN,INPUT_PULLUP);
  if (!transceiver.begin()) stop(F("Unable to init SIGFOX module"));
  if (!IMU.begin()) stop(F("Unable to init accelerometer")); 
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  preferences.begin("wdt", false);
  if (digitalRead(TAMPERED_PIN) == HIGH) {
    bActivated = false;
    preferences.putBool("bActivated", false);
  } else {
    bActivated = preferences.getBool("bActivated", false);
  }
  preferences.getBytes("dataArray", dataArray, 12);
  timer = timerBegin(0, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(timer, &resetModule, true);  //attach callback
  timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
  timerAlarmEnable(timer);                          //enable interrupt
  delay(100);
  DEBUG_PRINTLN("Setup done");
  if(dataArray[0] != 0){
    sendData();
  }
}

void updateAccel() {
  IMU.readSensor();
}

bool isTampered(){
  if(digitalRead(TAMPERED_PIN) == HIGH && bActivated == true && tampering.active == false) {
    tampering.active = true;
    tampering.start = millis();
    DEBUG_PRINTLN("Tampered");
    sendTamperedMessage();
  }
  return tampering.active;
}


bool isActivated(){
  if(digitalRead(TAMPERED_PIN) == LOW && bActivated == false) {
    DEBUG_PRINTLN("Activated");
    bActivated = true;
    preferences.putBool("bActivated", bActivated);
    sendActivatedMessage();
    return true;
  } else if (bActivated) {
    return true;
  }
  return false;
}

bool isUpsideDown(){
  if(IMU.getAccelZ_mss() > UPSIDE_MMS_LIMIT ) {
    DEBUG_PRINTLN("UpsideDown");
    return true;
  }
  return false;
}

bool isShaking(){
    if(IMU.getGyroX_rads() <= -SHAKE_MMS_LIMIT ||  IMU.getGyroX_rads() >= SHAKE_MMS_LIMIT || IMU.getGyroY_rads() <= -SHAKE_MMS_LIMIT || IMU.getGyroY_rads() >= SHAKE_MMS_LIMIT) { 
    DEBUG_PRINTLN("Shaking");
    return true;
  }
  return false;
}


void loop() {
  timerWrite(timer, 0); //reset timer (feed watchdog)
  if(isActivated()) {
    if(isTampered()){
      /*if((millis()-tampering.start) >= 10*60*1000){
        sendTamperedMessage();
        tampering.start = millis();
      }*/
    }
    updateAccel();
    if(isShaking()){
      //DEBUG_PRINTLN("isShaking");
      if(millis() - instantShaking.last > DELAY_SHAKING + DELAY_LOOP || instantShaking.start == 0) {
        DEBUG_PRINTLN("Set startShaking");
        instantShaking.start = millis();
        instantShaking.last = millis();
        instantShaking.duration = 0;
      }
      if(instantShaking.active) {
        if(instantShaking.last > 0) {
          instantShaking.duration += millis() - instantShaking.last;
        } 
      } else {
        instantShaking.active = true;
        //DEBUG_PRINTLN("bShaking");
      }
      instantShaking.last = millis();
      if(instantShaking.duration >= TRIGGER_SHAKING) {
        DEBUG_PRINT("Shaking : ");
        DEBUG_PRINT(instantShaking.duration);
        DEBUG_PRINT("ms / ");
        DEBUG_PRINT(millis() - instantShaking.start);
        DEBUG_PRINTLN("ms");
        instantShaking.start = 0;
        instantShaking.duration = 0;
        instantShaking.last = 0;
        sendMacMessage();
      } else if (millis() - instantShaking.start >= DELAY_SHAKING && instantShaking.duration < TRIGGER_SHAKING) {
        DEBUG_PRINT("Shaking dismissed : ");
        DEBUG_PRINT(instantShaking.duration);
        DEBUG_PRINT("ms / ");
        DEBUG_PRINT(millis() - instantShaking.start);
        DEBUG_PRINTLN("ms");
        instantShaking.start = millis();
        instantShaking.duration = 0;
        instantShaking.last = millis();
      }
    } else {
      if (instantShaking.start > 0 &&  millis() - instantShaking.start >= DELAY_SHAKING && instantShaking.duration < TRIGGER_SHAKING) {
        DEBUG_PRINT("Shaking dismissed : ");
        DEBUG_PRINT(instantShaking.duration);
        DEBUG_PRINT("ms / ");
        DEBUG_PRINT(millis() - instantShaking.start);
        DEBUG_PRINTLN("ms");
        instantShaking.start = 0;
        instantShaking.duration = 0;
        instantShaking.last = 0;
      }
      instantShaking.active = false;
      //DEBUG_PRINTLN("!bShaking");
    }
  
    if(isUpsideDown()){
      DEBUG_PRINTLN("isUpsideDown");
      if(millis() - instantUpsideDown.last > DELAY_UPSIDE + DELAY_LOOP || instantUpsideDown.start == 0) {
        DEBUG_PRINTLN("Set startUpsideDown");
        instantUpsideDown.start = millis();
        instantUpsideDown.last = millis();
        instantUpsideDown.duration = 0;
      }
      if(instantUpsideDown.active) {
        if(instantUpsideDown.last > 0) {
          instantUpsideDown.duration += millis() - instantUpsideDown.last;
          DEBUG_PRINTLN("Add durationUpsideDown");        }
      } else {
        instantUpsideDown.active = true;
      }
      instantUpsideDown.last = millis(); 
      if(instantUpsideDown.duration >= TRIGGER_UPSIDE) {
        DEBUG_PRINT("UpsideDown : ");
        DEBUG_PRINT(instantUpsideDown.duration);
        DEBUG_PRINT("ms / ");
        DEBUG_PRINT(millis() - instantUpsideDown.start);
        DEBUG_PRINTLN("ms");
        sendUpsideDownMessage();
        instantUpsideDown.start = 0;
        instantUpsideDown.duration = 0;
        instantUpsideDown.last = 0;
      } else if(millis() - instantUpsideDown.start >= DELAY_UPSIDE && instantUpsideDown.duration < TRIGGER_UPSIDE) {
        DEBUG_PRINT("UpsideDown dismissed : ");
        DEBUG_PRINT(instantUpsideDown.duration);
        DEBUG_PRINT("ms / ");
        DEBUG_PRINT(millis() - instantUpsideDown.start);
        DEBUG_PRINTLN("ms");
        instantUpsideDown.start = millis();
        instantUpsideDown.last = millis();
        instantUpsideDown.duration = 0;
      }
    } else {
      if(instantUpsideDown.start > 0 && millis() - instantUpsideDown.start >= DELAY_UPSIDE && instantUpsideDown.duration < TRIGGER_UPSIDE) {
        DEBUG_PRINT("UpsideDown dismissed : ");
        DEBUG_PRINT(instantUpsideDown.duration);
        DEBUG_PRINT("ms / ");
        DEBUG_PRINT(millis() - instantUpsideDown.start);
        DEBUG_PRINTLN("ms");
        instantUpsideDown.start = 0;
        instantUpsideDown.last = 0;
        instantUpsideDown.duration = 0;
      }
      instantUpsideDown.active = false;
    }
  }
  delay(DELAY_LOOP);
  
}
