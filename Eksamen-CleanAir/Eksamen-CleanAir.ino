#include <WiFi.h>
#include <Wire.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoJson.h>
 
const char* ssid = "";
const char* password = "";
const char* mqttServer = "";
const int mqttPort = ;
const char* mqttUser = "";
const char* mqttPassword = "";

WiFiClientSecure espClient;
PubSubClient client(espClient);

// Pins til de forskellige sensors/LED's
const int ledRed = 17;
const int ledYellow = 16;
const int ledGreen = 4;

const int BME_SDA = 21;
const int BME_SCL = 22;
Adafruit_BME280 bme;

const int MQ_ANALOG_PIN = 34;
const int MQ_DIGITAL_PIN = 13;

// Variabler til at styre thresholds for LED'erne
float GoodMax = 1200;
float WarnMax = 2500;

// Intervaller til hvornår der skal sendes data til vores broker - Default 1 minut
unsigned long measurementInterval = 60000;

// Kontrol af Interval
unsigned long lastMeasurementTime = 0;

// boolean til at tage nye målinger når der trykkes på en knap fra frontend
boolean newMeasurement = false;

// Id'et på den givne esp
String deviceId;

void receivedMessages(char* topic, byte* payload, unsigned int length) {
  
  String message;

  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("Payload: ");
  Serial.println(message);

  if(String(topic) == "cleanair/intervals/set") {

    message.trim();
    message.replace("{", "");
    message.replace("}", "");

    char payloadBuffer[32];
    message.toCharArray(payloadBuffer, sizeof(payloadBuffer));
    unsigned long newInterval = strtoul(payloadBuffer, NULL, 10);

    if (newInterval >= 5000 && newInterval <= 3600000) {
      measurementInterval = newInterval;
      Serial.print("New measurement interval set to: ");
      Serial.println(measurementInterval);
    }
  }

  if(String(topic) == "cleanair/measurement/now") {

    newMeasurement = true;
    Serial.println("Request for a new measurement received");
    
  }

  if(String(topic) == "cleanair/thresholds/update") {
    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, message);

    if(!error) {
      if (doc.containsKey("goodMax")) {
        GoodMax = doc["goodMax"];
        Serial.print("Updated goodMaxThreshold to: ");
        Serial.println(GoodMax);
      }
      if (doc.containsKey("warnMax")) {
        WarnMax = doc["warnMax"];
        Serial.print("Updated warnMaxThreshold to: ");
        Serial.println(WarnMax);
      }
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Setup Leds + Testing Leds
  pinMode(ledRed, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  testAllLeds(5);

  // Setup BME + MQ
  Wire.begin(BME_SDA, BME_SCL);
  bme.begin(0x76);

  pinMode(MQ_DIGITAL_PIN, INPUT);
  
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to the WiFi network");


  deviceId = "CleanAir-" + WiFi.macAddress();
  deviceId.replace(":", "");
  Serial.print("Device ID: ");
  Serial.println(deviceId);


  espClient.setInsecure();
  client.setServer(mqttServer, mqttPort);
  client.setCallback(receivedMessages);

  while(!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if(client.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.print("Failed to connect, state ");
      Serial.println(client.state());
      delay(2000);
    }
  }

  // Subscriber til interval indstilling, nye anmodning om nye målinger og update af thresholds
  client.subscribe("cleanair/intervals/set");
  client.subscribe("cleanair/measurement/now");
  client.subscribe("cleanair/thresholds/update");

  // Første publish så vi ved der er forbindelse
  client.publish("cleanair/data", "Connected and Ready");

}

void loop() {
  client.loop();

  float temp = bme.readTemperature();
  float hum = bme.readHumidity();
  float pressure = bme.readPressure() / 100.0F;

  int analogVal = analogRead(MQ_ANALOG_PIN);

  unsigned long currentMillis = millis();

  if(currentMillis - lastMeasurementTime >= measurementInterval || newMeasurement) {
    lastMeasurementTime = currentMillis;

    Serial.print("[BME280] Temp: ");
    Serial.print(temp);
    Serial.print(" °C | Hum: ");
    Serial.print(hum);
    Serial.print(" % | Pressure: ");
    Serial.print(pressure);
    Serial.println(" hPa");
    Serial.print("[MQ Sensor] AirQuality: ");
    Serial.println(analogVal);
    Serial.print("Interval: ");
    Serial.println(measurementInterval);

    publishDataToBrooker(temp, hum, pressure, analogVal, measurementInterval);

    newMeasurement = false;

  }

  /*
  Tjekker om hvordan luftkvaliteten er og lyser de forskellige led'er alt efter værdien
  Grøn = God luftkvalitet
  Gul = Stadigvæk okay
  Rød = Ikke nødvendigvis dårlig, dog åben et vindue
  */ 
  if(analogVal < GoodMax) {
    digitalWrite(ledRed, LOW);
    digitalWrite(ledYellow, LOW);
    digitalWrite(ledGreen, HIGH);
  } else if (analogVal > GoodMax && analogVal < WarnMax) {
    digitalWrite(ledRed, LOW);
    digitalWrite(ledYellow, HIGH);
    digitalWrite(ledGreen, LOW);
  } else {
    digitalWrite(ledRed, HIGH);
    digitalWrite(ledYellow, LOW);
    digitalWrite(ledGreen, LOW);
  }
}

void publishDataToBrooker(float temp, float hum, float pressure, int analogVal, long measurementInterval) {
  char tempString[10];
  char humString[10];
  char pressureString[10];
  char analogString[10];

  // Convert float til String
  dtostrf(temp, 1, 2, tempString);
  dtostrf(hum, 1, 2, humString);
  dtostrf(pressure, 1, 2, pressureString);

  // Convert int til String
  itoa(analogVal, analogString, 10);

  char data[300];

  snprintf(data, sizeof(data),
    "{\"DeviceId\":\"%s\",\"Temperature\":%.2f,\"Humidity\":%.2f,\"Pressure\":%.2f,\"AirQuality\":%d,\"Interval\":%lu}",
    deviceId.c_str(), temp, hum, pressure, analogVal, measurementInterval
  );

  client.publish("cleanair/data", data);
}

void testAllLeds(int times){
  for (int i = 0; i < times; i++){
    digitalWrite(ledRed, HIGH);
    digitalWrite(ledYellow, HIGH);
    digitalWrite(ledGreen, HIGH);
  
    delay(250);

    digitalWrite(ledRed, LOW);
    digitalWrite(ledYellow, LOW);
    digitalWrite(ledGreen, LOW);

    delay(250);
  }
}
