/*
Auris ble demo
*/

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BleCtrl.h>
#include <Wire.h>
#include <RTClib.h>
#include "Freenove_WS2812_Lib_for_ESP32.h"




#include "esp32notifications.h"


#define HARDWARE_STANDARD

#ifdef HARDWARE_STANDARD
    #define BUTTON_A    25 // left button - use this GPIO pin
    #define BUTTON_B    26 // center button - use this GPIO pin
    #define BUTTON_C    27 // right button - use this GPIO pin
#else
    #error Hardware buttons not supported!
#endif





// the pin that is connected to SQW
#define CLOCK_INTERRUPT_PIN 2
// #define VIBRATION_PIN 15
RTC_DS3231 rtc;

#define NUM_OF_TASK 3
#define TIME_SCALE 2400//240 000 000 Hz
static volatile unsigned int Task_Delay[NUM_OF_TASK] = {0};

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define DEVICE_NAME "Auris-放欣睡"

#define LEDS_COUNT  1
#define LEDS_PIN  13
#define CHANNEL   0
Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL, TYPE_GRB); //宣告WS2812
// Pinout of Microcontroller
#define VIBRATION_PIN   12   // Digital Output

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
BLEAdvertising *pAdvertising;
uint8_t txValue = 0;
uint8_t ble_state = BLE_IDLE;
char  manufacturerData[8] = {0x97,0x03,0x00,0x80,0x06,0x00,0x61,0x00};
// char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
int countDownInterval = 5;
char buffer[256] = {};

SoftwareSerial BioSerial(16,17);//RXD, TXD
// int LED_PIN = 2;
void BLEHandler();
void TaskHandler1();
void TaskHandler2();

void BleTask(std::string rxValue);
void onAlarm();
void BleInit();
// static uint8_t read_i2c_register(uint8_t addr, uint8_t reg);

class BleServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      ble_state = BLE_CONNECTED;
    };

    void onDisconnect(BLEServer* pServer) {
      ble_state = BLE_DISCONNECTED;
    }
};

class BleCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        BleTask(rxValue);
      }
    }
};

void BleInit(){
  BLEDevice::init(DEVICE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new BleServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(
                      TX_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
											RX_UUID,
											BLECharacteristic::PROPERTY_WRITE
										);
  pRxCharacteristic->setCallbacks(new BleCallbacks());
  pService->start();

  BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
  oAdvertisementData.setFlags(0x04); // BR_EDR_NOT_SUPPORTED 0x04
  oAdvertisementData.setName(DEVICE_NAME);
  oAdvertisementData.setPartialServices(BLEUUID(SERVICE_UUID));
  // oAdvertisementData.setManufacturerData(manufacturerData);

  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->setAdvertisementData(oAdvertisementData);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();

  ble_state = BLE_ADV;
}

void setup() {
  Serial.begin(115200);
  // pinMode(LED_PIN, OUTPUT);
  pinMode(VIBRATION_PIN, OUTPUT);
  pinMode(LEDS_PIN,OUTPUT);

  BleInit();
  strip.begin();
  strip.setBrightness(128);
  strip.setLedColorData(0, 255, 255, 255);
  strip.show();

  // if(!rtc.begin()){
  //   Serial.println("RTC not found!!");
  //   Serial.flush();
  //   abort();
  // }
  // if(rtc.lostPower()) {
  //   // this will adjust to the date and time at compilation
  //   rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // }
  // //we don't need the 32K Pin, so disable it
  // rtc.disable32K();
  // // Making it so, that the alarm will trigger an interrupt
  // pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), onAlarm, FALLING);

  // rtc.clearAlarm(1);
  // rtc.clearAlarm(2);
  // // stop oscillating signals at SQW Pin
  // // otherwise setAlarm1 will fail
  // rtc.writeSqwPinMode(DS3231_OFF);
  // // turn off alarm 2 (in case it isn't off already)
  // // again, this isn't done at reboot, so a previously set alarm could easily go overlooked
  // rtc.disableAlarm(2);

  // // schedule an alarm 10 seconds in the future
  // if(!rtc.setAlarm1(
  //           rtc.now() + TimeSpan(20),
  //           DS3231_A1_Second // this mode triggers the alarm when the seconds match. See Doxygen for other options
  // )) {
  //       Serial.println("Error, alarm wasn't set!");
  // }else {
  //       sprintf((char *)buffer, "Alarm will happen in %d seconds!", countDownInterval);
  //       Serial.println(buffer);
  // }

  Serial.println("Start advertising...");
}

void loop() {
  BLEHandler();
  TaskHandler1();
  TaskHandler2();
  unsigned char i;
  for (i = 0; i < NUM_OF_TASK; i++){
    if (Task_Delay[i]){
      Task_Delay[i]--;
    }
  }
}

void BLEHandler(){
  if (Task_Delay[0] == 0){
    switch (ble_state)
    {
    case BLE_IDLE:
        Task_Delay[0] = 1;
      break;
    case BLE_ADV:
        ble_state = BLE_IDLE;
      break;
    case BLE_CONNECTED:
        Serial.println("Connected !!!");
        ble_state = BLE_IDLE;
        Task_Delay[0] = 1;
      break;
    case BLE_NOTIFY:
        pTxCharacteristic->setValue(&txValue, 1);
        pTxCharacteristic->notify();
        txValue++;
        Task_Delay[0] = 10 * TIME_SCALE;
      break;
    case BLE_DISCONNECTED:
        BLEDevice::startAdvertising();
        Serial.println("Start advertising again ...");
        ble_state = BLE_ADV;
        Task_Delay[0] = 500 * TIME_SCALE;
    default:
        Task_Delay[0] = 1;
      break;
    }
  }
}

char date[20] = "YYYY/MM/DD hh:mm:ss";
void TaskHandler1(){
  if (Task_Delay[1] == 0){
    //  // print current time
    // // the value at SQW-Pin (because of pullup 1 means no alarm)
    // Serial.print(" SQW: ");
    // Serial.print(digitalRead(CLOCK_INTERRUPT_PIN));
    // // whether a alarm happened happened
    // Serial.print(" Alarm1: ");
    // Serial.println(rtc.alarmFired(1));
    // // Serial.print(" Alarm2: ");
    // // Serial.println(rtc.alarmFired(2));
    // // control register values (see https://datasheets.maximintegrated.com/en/ds/DS3231.pdf page 13)
    // // Serial.print(" Control: 0b");
    // // Serial.println(read_i2c_register(DS3231_ADDRESS, DS3231_CONTROL), BIN);

    // // resetting SQW and alarm 1 flag
    // // using setAlarm1, the next alarm could now be configurated
    // if(rtc.alarmFired(1)) {
    //   rtc.clearAlarm(1);
    //   rtc.now().toString(date);
    //   Serial.println(date);
    //   Serial.println(" Alarm cleared");
    // }
    Task_Delay[1] = 2000 * TIME_SCALE;
  }
}

void TaskHandler2(){
  if (Task_Delay[2] == 0){
    // DateTime now = rtc.now();

    // Serial.print(now.year(), DEC);
    // Serial.print('/');
    // Serial.print(now.month(), DEC);
    // Serial.print('/');
    // Serial.print(now.day(), DEC);
    // Serial.print(" (");
    // Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    // Serial.print(") ");
    // Serial.print(now.hour(), DEC);
    // Serial.print(':');
    // Serial.print(now.minute(), DEC);
    // Serial.print(':');
    // Serial.print(now.second(), DEC);
    // Serial.println();

    // Serial.print(" since midnight 1/1/1970 = ");
    // Serial.print(now.unixtime());
    // Serial.print("s = ");
    // Serial.print(now.unixtime() / 86400L);
    // Serial.println("d");

    // calculate a date which is 7 days, 12 hours, 30 minutes, 6 seconds into the future
    // DateTime future (now + TimeSpan(7,12,30,6));

    // Serial.print(" now + 7d + 12h + 30m + 6s: ");
    // Serial.print(future.year(), DEC);
    // Serial.print('/');
    // Serial.print(future.month(), DEC);
    // Serial.print('/');
    // Serial.print(future.day(), DEC);
    // Serial.print(' ');
    // Serial.print(future.hour(), DEC);
    // Serial.print(':');
    // Serial.print(future.minute(), DEC);
    // Serial.print(':');
    // Serial.print(future.second(), DEC);
    // Serial.println();

    // Serial.print("Temperature: ");
    // Serial.print(rtc.getTemperature());
    // Serial.println(" C");

    // Serial.println();
    Task_Delay[2] = 1000 * TIME_SCALE;
  }
}

void BleTask(std::string rxValue) {
  Serial.print("Received Value: ");
  for (int i = 0; i < rxValue.length(); i++)
    Serial.print(rxValue[i]);

  Serial.println();
  Serial.println("*********");

     // Serial.print(rxValue); //查看藍芽輸入的值
        switch(rxValue[0]){
          case '1':
            digitalWrite(VIBRATION_PIN,HIGH);
            strip.setLedColorData(0, 255, 0, 0);  // 亮綠燈
            strip.show();
            break;
          case '2':
            digitalWrite(VIBRATION_PIN,LOW);
            strip.setLedColorData(0, 0, 0, 0);  // 關燈
            strip.show();
            break;
          case '3':
            digitalWrite(VIBRATION_PIN,HIGH);
            strip.setLedColorData(0, 255, 255, 0);  // 亮黃燈
            strip.show();
            break;
          case '4':
            digitalWrite(VIBRATION_PIN,LOW);
            strip.setLedColorData(0, 0, 0, 0); // 關燈
            strip.show();
            break;
          case '5':
            digitalWrite(VIBRATION_PIN,HIGH);
            strip.setLedColorData(0, 0, 255, 0);  // 亮紅燈
            strip.show();
            break;
          case '6':
            digitalWrite(VIBRATION_PIN,LOW);
            strip.setLedColorData(0, 0, 0, 0); // 關燈
            strip.show();
            break;
          default:
            break;
        }

  // if(rxValue[0] == 'C' && rxValue[1] == 'D'){
  //   int time = 0;
  //   sscanf(rxValue.c_str(), "CD:%d", &time);
  //   countDownInterval = time;
  //   sprintf((char *)buffer, "Alarm will happen in %d seconds!", countDownInterval);
  //   Serial.println(buffer);
  // }

  // if(rxValue[0] == 'D' && rxValue[1] == 'T'){
  //   int secSinceEpoch = 0;
  //   sscanf(rxValue.c_str(), "DT:%d", &secSinceEpoch);

  //   DateTime future (secSinceEpoch);
  //   Serial.print(future.year(), DEC);
  //   Serial.print('/');
  //   Serial.print(future.month(), DEC);
  //   Serial.print('/');
  //   Serial.print(future.day(), DEC);
  //   Serial.print(' ');
  //   Serial.print(future.hour(), DEC);
  //   Serial.print(':');
  //   Serial.print(future.minute(), DEC);
  //   Serial.print(':');
  //   Serial.print(future.second(), DEC);
  //   Serial.println();

  //   sprintf((char *)buffer, "alarm set to %d", secSinceEpoch);
  //   Serial.println(buffer);
  //   rtc.clearAlarm(1);
  //   rtc.writeSqwPinMode(DS3231_OFF);
  //   if(!rtc.setAlarm1(
  //           future,
  //           DS3231_A1_Minute// this mode triggers the alarm when the seconds match. See Doxygen for other options
  //   )) {
  //     sprintf((char *)buffer, "Error, alarm wasn't set! %s!", future.toString(date));
  //   }else {
  //     sprintf((char *)buffer, "# Alarm will happen at %s!", future.toString(date));
  //   }
  //   Serial.println(buffer);
  // }

  // if(rxValue[0] == 'V' && rxValue[1] == 'B'){
  //   int vibrate = 0;
  //   sscanf(rxValue.c_str(), "VBR:%d", &vibrate);
  //   sprintf((char *)buffer, "Vibrate pin is set to %d ", vibrate);
  //   Serial.println(buffer);
  //   digitalWrite(VIBRATION_PIN, vibrate);
  // }
}

// void onAlarm() {
//     // rtc.clearAlarm(1);
//     // DateTime now = rtc.now();
//     // now.toString(date);
//     Serial.print("Alarm occured>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ");
//     // Serial.println(date);
// }

// static uint8_t read_i2c_register(uint8_t addr, uint8_t reg) {
//     Wire.beginTransmission(addr);
//     Wire.write((byte)reg);
//     Wire.endTransmission();
//     Wire.requestFrom(addr, (byte)1);
//     return Wire.read();
// }