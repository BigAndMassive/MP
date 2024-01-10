/****************************************
 * Include Libraries
 ****************************************/
#include "UbidotsEsp32Mqtt.h"
#include <Ticker.h>

/****************************************
 * Define Constants
 ****************************************/
const char* UBIDOTS_TOKEN = "BBUS-ZnVXe9kdq6Teh9dNqIwY6dtPg9iYsn";  // Put here your Ubidots TOKEN
const char* WIFI_SSID = "Pepepopo";      // Put here your Wi-Fi SSID
const char* WIFI_PASS = "i84ikpjd";      // Put here your Wi-Fi password
const char *DEVICE = "08B61F708D28";     // Put here your Device label to which data  will be published
const char *PUBLISH_VARIABLE_LABEL = "test3";   // Put here your Variable label to which data  will be published
const char *PUBLISH_VARIABLE_LABEL2 = "test2";  // Replace with the device label to subscribe to
const char *SUBSCRIBE_VARIABLE_LABEL = "test3"; // Replace with the variable label to subscribe to
const char *SUBSCRIBE_VARIABLE_LABEL2 = "updateflag"; // Replace with the variable label to subscribe to
const char *MQTT_TOPIC1 = "/v2.0/devices/08b61f708d28/test3/lv";
const char *MQTT_TOPIC2 = "/v2.0/devices/08b61f708d28/updateflag/lv";
int motor1Pin2 = 26; //USE THE ONE NUMBERED 26

const unsigned long waitTimeAfterSignal = 10000; //delay time
const int PUBLISH_FREQUENCY = 5000; // Update rate in milliseconds
unsigned long lastPublishTime = 0;  // Last time data was published
unsigned long lastSignalTime = 0;
int motorFlag = 0; // placeholder variable for motor control
unsigned long delayStartTime = 0;
const unsigned long delayDuration = 5000;  // 5 seconds delay

const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;

unsigned long timer;
uint8_t analogPin = 34; // Pin used to read data from GPIO34 ADC_CH6.

Ubidots ubidots(UBIDOTS_TOKEN);

/****************************************
 * Auxiliar Functions
 ****************************************/




 void publishToUbidots() {
  if (ubidots.connected()) {
    float value = analogRead(34);
    ubidots.add(PUBLISH_VARIABLE_LABEL, value);
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
  digitalWrite(motor1Pin2, HIGH);
  motorFlag = 1;
  ubidots.add(SUBSCRIBE_VARIABLE_LABEL2,0);
  ubidots.publish(DEVICE);
  delayStartTime = millis();
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
  if (strcmp(topic, MQTT_TOPIC2) == 0) {
    // Handle actions for MQTT_TOPIC2
    if (receivedValue == 1){
      handleSignal();
      }
  }
}

/****************************************
 * Main Functions
 ****************************************/

void setup()
{
  pinMode(motor1Pin2, OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  
  // put your setup code here, to run once:
  Serial.begin(115200);
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
  //int motorFlag;
  // float funvalue = random(0,9) * 10;
  // put your main code here, to run repeatedly:
  if (!ubidots.connected())
  {
    ubidots.reconnect();
    ubidots.subscribeLastValue(DEVICE, SUBSCRIBE_VARIABLE_LABEL2);
  }
  if (std::abs(static_cast<long>(millis() - timer)) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
  { // triggers the routine every PUBLISH_FREQUENCY milliseconds
    publishToUbidots();
    pv=ubidots.subscribeLastValue(DEVICE, SUBSCRIBE_VARIABLE_LABEL); // Insert the device and variable's Labels, respectively
    uf=ubidots.subscribeLastValue(DEVICE, SUBSCRIBE_VARIABLE_LABEL2);
    timer = millis();
}
  if (motorFlag == 1 && millis() - delayStartTime >= delayDuration) {
    Serial.println("Stopping");
    digitalWrite(motor1Pin2, LOW);
    motorFlag = 0;
}
ubidots.loop();
}