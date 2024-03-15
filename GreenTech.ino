//BLYNK cloud set up 
#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL2xZ9ZVErH"
#define BLYNK_TEMPLATE_NAME "GreenTech"
#define BLYNK_AUTH_TOKEN "S7FB8kq6HDndjjwHXiH14i4CWE8dXpwh"
//WIFI set up
#include <SPI.h>
#include <WiFiS3.h>
#include <BlynkSimpleWifi.h>
char ssid[] = "iPhone TJ";
char pass[] = "01234567";
//Libraries
#include "DEV_Config.h"
#include "TSL2591.h"
#include <Servo.h>
#include<DHT.h>
#define DHTPIN 16
#define DHTTYPE DHT11
#include "BraccioRobot.h"
#define INPUT_BUFFER_SIZE 50

static char inputBuffer[INPUT_BUFFER_SIZE];
Position armPosition;

DHT dht(DHTPIN,DHTTYPE);

//Variable definition
int Lux;

//Soil Moisture Sensor//
int soilSensor = 15;
int soilData;
int soilMoisture;

//Temperature and humidity Sensor//
int thSensor = 16;
float temperature_C;
float humidity;

//OLED display//


//Relay
int relayPin = 17;
int relaySwitch;

//Light
int light = 19;
int lightStatus;


BlynkTimer timer; 



void setup() {
pinMode(soilSensor,INPUT);
pinMode(thSensor,INPUT);
pinMode(relayPin,OUTPUT);
pinMode(light,OUTPUT);

Serial.begin(115200);

BraccioRobot.init();

dht.begin();

TSL2591_Init();


Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
timer.setInterval(100L,lightIntensitySensor);
timer.setInterval(100L,soilMoistureSensor);
timer.setInterval(100L,temperatureHumiditySensor);
}

void lightIntensitySensor() {
  Lux = TSL2591_Read_Lux();
  Blynk.virtualWrite(V0, Lux);
}

void soilMoistureSensor() {
  int soilData = analogRead(soilSensor); 
  int soilMoisture = map(soilData, 0, 1023, 100, 0);
  Blynk.virtualWrite(V1,soilMoisture);
}

void temperatureHumiditySensor() {
  float temperature_C = dht.readTemperature();
  float humidity = dht.readHumidity();
  Blynk.virtualWrite(V2, temperature_C);
  Blynk.virtualWrite(V3, humidity);
}
BLYNK_WRITE(V4){
  relaySwitch = param.asInt();
  digitalWrite(relayPin, relaySwitch);
}


BLYNK_WRITE(V11){
  int dataEnter = param.asInt();
  int data =0;
  if(dataEnter == 1 && data == 0){
    BraccioRobot.moveToPosition(armPosition.set(0, 90, 90, 90,90,  10), 100); 
    delay(1000);
    BraccioRobot.moveToPosition(armPosition.set(0, 90, 7, 0,90,  10), 100); 
    BraccioRobot.moveToPosition(armPosition.set(0, 90, 7, 0,90,  60), 100);
    BraccioRobot.moveToPosition(armPosition.set(0, 90, 50, 0,90,  60), 100);
    BraccioRobot.moveToPosition(armPosition.set(180, 90, 50, 0,90,  60), 100);
    BraccioRobot.moveToPosition(armPosition.set(180, 90, 8, 0,90,  60), 100);
    BraccioRobot.moveToPosition(armPosition.set(180, 90, 8, 0,90,  10), 100);
  }
}

BLYNK_WRITE(V12){
  lightStatus = param.asInt();
  digitalWrite(light, lightStatus);
}

void loop() {
  Blynk.run();
  timer.run();
  handleInput();
}
void handleInput() {
  if (Serial.available() > 0) {
    byte result = Serial.readBytesUntil('\n', inputBuffer, INPUT_BUFFER_SIZE);
    inputBuffer[result] = 0;
    interpretCommand(inputBuffer, result);
  }
}

void interpretCommand(char* inputBuffer, byte commandLength) {
  if (inputBuffer[0] == 'P') {
    positionArm(&inputBuffer[0]);
  } else if (inputBuffer[0] == 'H') {
    homePositionArm();
  } else if (inputBuffer[0] == '0') {
    BraccioRobot.powerOff();
    Serial.println("OK");
  }  else if (inputBuffer[0] == '1') {
    BraccioRobot.powerOn();
    Serial.println("OK");
  } else {
    Serial.println("E0");
  }
  Serial.flush();
}

void
positionArm(char *in) {
  int speed = armPosition.setFromString(in);
  if (speed > 0) {
    BraccioRobot.moveToPosition(armPosition, speed);
    Serial.println("OK");
  } else {
    Serial.println("E1");
  }
}

void
homePositionArm() {
  BraccioRobot.moveToPosition(armPosition.set(90, 90, 90, 90, 90, 73), 100);
  Serial.println("OK");
}

