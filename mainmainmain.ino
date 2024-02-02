/****************************************
 * Include Libraries
 ****************************************/
#include "UbidotsEsp32Mqtt.h"
#include <Ticker.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <LiquidCrystal_I2C.h>

/****************************************
 * Define Constants
 ****************************************/
const char* UBIDOTS_TOKEN = "BBUS-ZnVXe9kdq6Teh9dNqIwY6dtPg9iYsn";  // Put here your Ubidots TOKEN
const char* WIFI_SSID = "Pepepopo";      // Put here your Wi-Fi SSID
const char* WIFI_PASS = "i84ikpjd";      // Put here your Wi-Fi password

/* MQTT Topics and Variable Names for Ubidots */
const char *DEVICE = "08B61F708D28";     // Put Device Label from Ubidots
const char *tempLabel = "temperature";   // Put Variable Name from Ubidots Device to Publish to
const char *humidLabel = "humidity";  // Put Variable Name from Ubidots Device to Publish to
const char *lightLabel = "light";
const char *pressureLabel = "pressure";
const char *SUBSCRIBE_VARIABLE_LABEL = "temperature"; // Replace with the variable label to subscribe to
const char *SUBSCRIBE_VARIABLE_LABEL2 = "updateflag"; // Replace with the variable label to subscribe to
const char *SUBSCRIBE_VARIABLE_LABEL3 = "test";
const char *SUBSCRIBE_VARIABLE_LABEL4 = "timerr";
const char *MQTT_TOPIC = "/v2.0/devices/08b61f708d28/updateflag/lv"; //May need to swap when version changes, Needs to swap when device and/or variables change
const char *MQTT_TOPIC2 = "/v2.0/devices/08b61f708d28/test/lv";
const char *MQTT_TOPIC3 = "/v2.0/devices/08b61f708d28/timerr/lv";


/* Timer Settings */
const int PUBLISH_FREQUENCY = 5000; // Update rate in milliseconds
const int WATER_FREQUENCY = 10000;
int motorFlag = 0; // placeholder variable for motor control
int ledFlag = 0;
unsigned long delayStartTime = 0; // Variable to control delay when if statement conditions are met when variable is used
unsigned long delayStartTime2 = 0;
unsigned long delayStartTime3 = 0;
const unsigned long delayDuration = 5000;  // 5 seconds delay
int trueHelper = 0;
int doubleTimer= 0;
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;
unsigned long timer;
unsigned long timer2;
unsigned long indiTimer;
int lcdFlag=0;

/* Hardware Pin Settings */
uint8_t analogPin = 34; // Pin used to read data from GPIO34 ADC_CH6.
int motorPin = 17; //USE THE ONE NUMBERED
int ledPin = 12;
#define SEALEVELPRESSURE_HPA (1013.25)
#define TEMT6000 32
#define RELAY_PIN 16
Adafruit_BME280 bme; // I2C
int lcdColumns = 16;
int lcdRows = 1;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  
const float ADC_Resolution = 4095.0; // ADC resolution (12-bit ADC on ESP32)
const float SysV = 3.3; // System voltage
int AnalogValue = analogRead(analogPin); // Read the analog value from the MCP9701A
float voltage = (AnalogValue / ADC_Resolution ) * SysV; // Calculate voltage
float temp = (voltage - 0.4 ) / 0.02;// Calculate temperature in Celsius using voltage
const int LED = 2;
float lux = 0;

Ubidots ubidots(UBIDOTS_TOKEN);

/****************************************
 * Auxiliar Functions
 ****************************************/




float LightConversion(){
  float volts =  analogRead(TEMT6000) * 5 / 1024.0; // Convert reading to VOLTS
  float VoltPercent = analogRead(TEMT6000) / 1024.0 * 100; //Reading to Percent of Voltage
   
  //Conversions from reading to LUX
  float amps = volts / 10000.0;  // em 10,000 Ohms
  float microamps = amps * 1000000; // Convert to Microamps
  float lux = microamps * 2.0; // Convert to Lux */
  return lux;
}

 void publishToUbidots() {
  if (ubidots.connected()) {
    float tempValue = bme.readTemperature();
    ubidots.add(tempLabel, tempValue);
    ubidots.publish(DEVICE);

    float humidValue = bme.readHumidity();
    ubidots.add(humidLabel, humidValue);
    ubidots.publish(DEVICE);

    float lightValue = LightConversion();
    ubidots.add(lightLabel, lightValue);
    ubidots.publish(DEVICE);

    float pressureValue = bme.readPressure();
    ubidots.add(pressureLabel, pressureValue);
    ubidots.publish(DEVICE);

    Serial.println("Published");
  } else {
    Serial.println("Could not connect");
    ubidots.disconnect();
  }
}

void handleSignal() {
  // Handle the signal received
  Serial.println("Moving Forward");
  digitalWrite(motorPin, HIGH);
  motorFlag = 1;
  ubidots.add(SUBSCRIBE_VARIABLE_LABEL2,0);
  ubidots.publish(DEVICE);
  delayStartTime = millis();
}

void lcdDefault() {
  lcd.setCursor(0, 0);
     // print message
    lcd.print("H:");
    lcd.print(bme.readHumidity(), 0);
    lcd.print("% ");
    // set cursor to first column, second row
    lcd.print("T:");
    lcd.print(bme.readTemperature());
    lcd.print(" C");
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  payload[length] = '\0'; // Null-terminate the string
  String valueString = String((char *)payload);
  float receivedValue = valueString.toFloat();
  if (strcmp(topic, MQTT_TOPIC) == 0) {
    // Handle actions for MQTT_TOPIC
    if (receivedValue == 1){
      handleSignal();
      }
  }
  else if (strcmp(topic, MQTT_TOPIC2)==0){ //For LED
    unsigned long doubleTimer=(receivedValue*1000)-5000;
    unsigned long rv = receivedValue;
    if (rv!=0 && millis() - delayStartTime2 >= doubleTimer){
    Serial.println("LED on");
    delayStartTime2=millis();
    trueHelper++;
  }
    else if (rv==0){
      digitalWrite(ledPin,LOW);
      lcdDefault();
      lcdFlag=0;
    }
  }
  // else if (strcmp(topic, MQTT_TOPIC3)==0){
  //   unsigned long doubleTimer=receivedValue*1000;
  //   unsigned long rv = receivedValue;
  //   //under real scenario ubidots should be set to 86400 for once per day
  //   if (rv!=0 && millis() - delayStartTime >= doubleTimer){
  //   delayStartTime=millis();
  //   digitalWrite(motorPin,HIGH);
  // }
}


/****************************************
 * Main Functions
 ****************************************/

void setup()
{
  pinMode(motorPin, OUTPUT);
  pinMode(ledPin,OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  bool status;
  // // default settings
  // // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x76);  
  // if (!status) {
  //   Serial.println("Could not find a valid BME280 sensor, check wiring!");
  //   while (1);
  // }
  lcd.init();
  lcd.backlight();
  pinMode(RELAY_PIN, OUTPUT);
  //ubidots.setDebug(true);  // uncomment this to make debug messages available
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  ubidots.subscribeLastValue(DEVICE,SUBSCRIBE_VARIABLE_LABEL);
}

void loop()
{
  String pv;
  String uf;
  String by;
  String pl;
  //int motorFlag;
  // float funvalue = random(0,9) * 10;
  // put your main code here, to run repeatedly:

  ReadLight();


  if (!ubidots.connected())
  {
    ubidots.reconnect();
    ubidots.subscribeLastValue(DEVICE, SUBSCRIBE_VARIABLE_LABEL2);
    ubidots.subscribeLastValue(DEVICE,SUBSCRIBE_VARIABLE_LABEL3);
  }
  
  if (std::abs(static_cast<long>(millis() - timer)) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
  { // triggers the routine every PUBLISH_FREQUENCY milliseconds
    printValues();
    publishToUbidots();
    pv=ubidots.subscribeLastValue(DEVICE, SUBSCRIBE_VARIABLE_LABEL); // Insert the device and variable's Labels, respectively
    uf=ubidots.subscribeLastValue(DEVICE, SUBSCRIBE_VARIABLE_LABEL2);
    by=ubidots.subscribeLastValue(DEVICE,SUBSCRIBE_VARIABLE_LABEL3);
    pl=ubidots.subscribeLastValue(DEVICE, SUBSCRIBE_VARIABLE_LABEL4);
    if (lcdFlag==0){
    lcdDefault();
    }
    timer = millis();
}
  if (std::abs(static_cast<long>(millis() - timer2)) > WATER_FREQUENCY){
    // Serial.println("EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
    handleSignal();
    timer2 = millis();
  }

  if (motorFlag == 1 && millis() - delayStartTime >= delayDuration) {
    Serial.println("Turning Off");
    digitalWrite(motorPin, LOW);
    motorFlag = 0;
}

  if (trueHelper >= 2 && millis() - doubleTimer >= 0) {
    Serial.println("EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Harvest!!!");

    lcdFlag=1;
    trueHelper = 1;
}

  
ubidots.loop();
}

void ReadLight() {
 
  analogReadResolution(10);
 
  float volts =  analogRead(TEMT6000) * 5 / 1024.0; // Convert reading to VOLTS
  float VoltPercent = analogRead(TEMT6000) / 1024.0 * 100; //Reading to Percent of Voltage
   
  //Conversions from reading to LUX
  float amps = volts / 10000.0;  // em 10,000 Ohms
  float microamps = amps * 1000000; // Convert to Microamps
  float lux = microamps * 2.0; // Convert to Lux */
 
 
 
 
}
 
void printValues() {
  Serial.print("Analog Value: ");
  Serial.println(AnalogValue);
  Serial.print("Voltage (V): ");
  Serial.println(voltage, 4); // Display voltage with 4 decimal places
  Serial.print("Temperature (°C): ");
  Serial.println(temp, 2); // Display temperature with 2 decimal places
 
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");
 
  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");
 
  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
 
  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");
 
  analogReadResolution(10);
   
  float LightConversion();
 
 
  // Output Serial
 
  // Output Serial
  Serial.print("LUX - ");
  Serial.print(lux);
  Serial.print("\n");
}