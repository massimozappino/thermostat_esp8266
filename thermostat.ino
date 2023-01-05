#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <PubSubClient.h>
#include <ArduinoJson.h>

#ifdef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP8266 ONLY!)
#error Select ESP8266 board.
#endif

#include "DHT.h"

//https://lastminuteengineers.com/esp8266-i2c-lcd-tutorial/#adjusting-the-lcd-contrast
#include <LiquidCrystal_I2C.h>

#include <EEPROM.h>

LiquidCrystal_I2C lcd(0x27,16,2); // connect SDA to D2 and SCL to D1

/************************* WiFi Access Point *********************************/
const char* ota_password       = "CHANGEME SELECT A PASSWORD FOR OTA";
const char* ssid               = "CHANGEME WIFI SSID";
const char* password           = "CHANGEME WIFI PASSWORD";
const char* mqtt_server        = "CHANGEME MQTT SERVER ADDRESS";
const char* mqtt_username      = "CHANGEME MQTT USERNAME";
const char* mqtt_password      = "CHANGEME MQTT PASSWORD";
#define DEVICE_ID                "thermostat1"

float hysteresisThesholdTop    = 0.8;
float hysteresisThesholdBottom = 0.3;
float targetTemperature        = 0.0;
float currentTemperature       = 0.0;
float currentHumidity          = 0.0;
float setTemperatureStep       = 0.1;
float maxTemperature           = 25.0;
float minTemperature           = 15.0;

boolean powerMode              = false;    //after power outage reset remote to these settings.

#define MQTT_PUBLISH_INTERVAL    60000
#define LCD_BACKLIGHT_INTERVAL   15000

#define RELAY_PIN D0
#define DHT_PIN D4
#define TEMP_DOWN_PIN D5
#define TEMP_UP_PIN D6
#define LIGHT_PIN D7
#define MENU_PIN D8

#define DHTTYPE DHT22
DHT dht(DHT_PIN, DHTTYPE);

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastLCDBacklight = 0;
bool statusLCDBacklight = true;

// MQTT
unsigned long lastMsg = 0;
byte willQoS = 0;
#define OFFLINE_MESSAGE "Offline"
#define ONLINE_MESSAGE "Online"

#define GET_DATA_TOPIC  DEVICE_ID "/data"

#define LWT_TOPIC              DEVICE_ID "/LWT"

#define POWER_TOPIC            DEVICE_ID "/power/set"
#define MODE_TOPIC             DEVICE_ID "/mode/set"
#define SET_TEMPERATURE_TOPIC  DEVICE_ID "/temperature/set"

int eepromAddr = 0;
int buttonUpState = HIGH;
int buttonDownState = HIGH;
int buttonLightState = HIGH;
int buttonMenuState = HIGH;
bool targetToBeSaved = false;

void setup() {
  Serial.println("Booting");
  EEPROM.begin(512);
  
  pinMode(RELAY_PIN, OUTPUT);
  pinMode (TEMP_UP_PIN, INPUT_PULLUP);
  pinMode (TEMP_DOWN_PIN, INPUT_PULLUP);
  pinMode (LIGHT_PIN, INPUT_PULLUP);
  pinMode (MENU_PIN, INPUT_PULLUP);

  Serial.begin(9600);
  lcd.init();
  lcd.clear();         
  lcd.backlight();
  lcd.print("Booting...");

  targetTemperature = loadTargetTemperature();
  
  dht.begin();

  setup_wifi();
  
  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);

  readTemperature();
}


void loop() {
  ArduinoOTA.handle();
  
  // handle relay before MQTT connection
  unsigned long now = millis();
  if (now - lastMsg > MQTT_PUBLISH_INTERVAL) {
    lastMsg = now;
    readTemperature();
    checkTargetTemperature();
    // Send data via MQTT
    sendValues();
    if (targetToBeSaved) {
      saveTargetTemperature(targetTemperature);
    }
    printOnLCD(false);
  }

  if (statusLCDBacklight == true && now - lastLCDBacklight > LCD_BACKLIGHT_INTERVAL) {
    statusLCDBacklight = false;
    lcd.noBacklight();
  }

  if (statusLCDBacklight == true) {
    lcd.backlight();
  }


  buttonLightState = digitalRead(LIGHT_PIN);
  if (buttonLightState == LOW) {
    Serial.println("LIGHT");
    printOnLCD(true);
  }


  buttonMenuState = digitalRead(MENU_PIN);
  if (buttonMenuState == LOW) {
    // TODO
    // printOnLCD(true);
  }
  
  buttonUpState = digitalRead(TEMP_UP_PIN);
  if (buttonUpState == LOW) {
    targetTemperature += setTemperatureStep;
    if (targetTemperature > maxTemperature) {
      targetTemperature = maxTemperature;
    }
    targetToBeSaved = true;
    printOnLCD(true);  
  }

  buttonDownState = digitalRead(TEMP_DOWN_PIN);
  if (buttonDownState == LOW) {
    targetTemperature -= setTemperatureStep;
    if (targetTemperature < minTemperature) {
      targetTemperature = minTemperature;
    }
    targetToBeSaved = true;
    printOnLCD(true);  
  }

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  delay(100);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String receivedMessage;
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    receivedMessage += (char)payload[i];    
  }
  Serial.println();

 
  if (strcmp(topic, POWER_TOPIC) == 0) {
      if(receivedMessage.equals("ON")) {
        powerMode = true;
      } else {
        powerMode = false;
      }
  }

  if (strcmp(topic, SET_TEMPERATURE_TOPIC) == 0) {
    targetTemperature = receivedMessage.toFloat();
    saveTargetTemperature(targetTemperature);
  }
  lastMsg = 0;

  printOnLCD(true);
}

void saveTargetTemperature(float temperature) {
  EEPROM.put(eepromAddr, temperature);
  EEPROM.commit();
  targetToBeSaved = false;
}

float loadTargetTemperature() {
  // read a byte from the current address of the EEPROM
  float value = minTemperature;
  EEPROM.get(eepromAddr, value);

  Serial.print(eepromAddr);
  Serial.print("\t");
  Serial.print(value, DEC);
  Serial.println();

//  // advance to the next address of the EEPROM
//  eepromAddr = eepromAddr + 1;
//
//  // there are only 512 bytes of EEPROM, from 0 to 511, so if we're
//  // on address 512, wrap around to address 0
//  if (eepromAddr == 512) { eepromAddr = 0; }
  return value;
}

void printOnLCD(bool backlight) {
  lcd.clear();         

  if (backlight == true) {
    statusLCDBacklight = true;
    lastLCDBacklight = millis();
  }

  lcd.print("Temp: " + String(currentTemperature, 1) + (char)223 + "C");

  lcd.setCursor(13, 0);   
  lcd.print(powerMode == true ? "ON" : "OFF");
  
  lcd.setCursor(0,1);   //Move cursor to character 2 on line 1
  lcd.print("Target:" + String(targetTemperature, 1) + (char)223 + "C");

  Serial.println("Temp: " + String(currentTemperature, 1) + "C");
  Serial.println("Target:" + String(targetTemperature, 1) + "C");
  Serial.print("POWER: ");
  Serial.println(powerMode == true ? "ON" : "OFF");

}

void readTemperature() {
  currentHumidity = dht.readHumidity();
  currentTemperature = dht.readTemperature();

  if (isnan(currentHumidity) || isnan(currentTemperature)) {
    Serial.println(F("Failed to read from DHT sensor!"));
  }
  Serial.print(F("Humidity: "));
  Serial.print(currentHumidity);
  Serial.print(F("%  Temperature: "));
  Serial.print(currentTemperature);
  Serial.println();
}

void sendValues() {
  DynamicJsonBuffer jsonBuffer;
  JsonObject& payload = jsonBuffer.createObject();
  payload["device_id"] = DEVICE_ID;
  if (isnan(currentHumidity) || isnan(currentTemperature)) {
    payload["temperature"] = (char*)NULL;
    payload["humidity"] = (char*)NULL;
  } else {
    payload["temperature"] = currentTemperature;
    payload["humidity"] = currentHumidity;
  }
  payload["targetTemperature"] = targetTemperature;
  String sPayload;
  payload.printTo(sPayload);
  char * cPayload = &sPayload[0u];

  client.publish(GET_DATA_TOPIC, cPayload);
}

void checkTargetTemperature(void) {
  float epsilon = 0.001;
  Serial.print("real bottom target: ");
  Serial.println(targetTemperature - hysteresisThesholdBottom);

  if (powerMode == false && currentTemperature <= targetTemperature + hysteresisThesholdBottom + epsilon) {
    heatOn();
  }

  if (powerMode == true && ((currentTemperature >= targetTemperature + hysteresisThesholdTop + epsilon)
  )) {
    heatOff();
  }
}

void heatOn() {
  Serial.println(F("Heat On "));
  powerMode = true;
  digitalWrite(RELAY_PIN, HIGH);
}

void heatOff() {
  Serial.println(F("Heat Off "));
  powerMode = false;
  digitalWrite(RELAY_PIN, LOW);
}

void setup_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(DEVICE_ID);

  // No authentication by default
  ArduinoOTA.setPassword(ota_password);

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(DEVICE_ID, mqtt_username, mqtt_password, LWT_TOPIC, willQoS, true, OFFLINE_MESSAGE)) {
      Serial.println("connected");

      client.subscribe(POWER_TOPIC);
      client.subscribe(MODE_TOPIC);
      client.subscribe(SET_TEMPERATURE_TOPIC);

      client.publish(LWT_TOPIC, ONLINE_MESSAGE, true);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}
