#include <WiFi.h>
#include <PubSubClient.h>
#include <RTClib.h>
#include <ESP32Servo.h>

RTC_DS3231 rtc;
Servo myservo;

const int PIN_TO_SENSOR = 23;
const int SERVO_PIN = 18;
int pinStateCurrent = LOW;
int pinStatePrevious = LOW;

#define LIGHT_SENSOR_PIN 34
const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqttServer = "broker.hivemq.com";
int port = 1883;
String stMac;
char mac[50];
char clientId[50];
String etat = "auto" ;

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TO_SENSOR, INPUT);

  myservo.attach(SERVO_PIN, 500, 2400);

  if (!rtc.begin()) {
    Serial.println("RTC module is NOT found");
    Serial.flush();
    while (1);
  }

  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  randomSeed(analogRead(0));

  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  wifiConnect();
  

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.macAddress());
  stMac = WiFi.macAddress();
  stMac.replace(":", "_");
  Serial.println(stMac);

  client.setServer(mqttServer, port);
  client.setCallback(callback);
}

void wifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
}

void mqttReconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    long r = random(1000);
    sprintf(clientId, "clientId-%ld", r);
    if (client.connect(clientId)) {
      Serial.print(clientId);
      Serial.println(" connected");
      client.subscribe("cmd");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* message, unsigned int length) {
  String stMessage;
  for (int i = 0; i < length; i++) {
    stMessage += (char)message[i];
  }
   

  if (String(topic) == "cmd") {
    if (etat == "manuel")
  {
      if (stMessage == "servo_open") {
        openServo();
      } else if (stMessage == "servo_close") {
        closeServo();
      }}
  
    if (stMessage == "auto") {
      etat = "auto";
    } else if (stMessage == "manuel") {
      etat = "manuel";
     
    }
    publishLightAndMotionStatus();
  }
}

void loop() {
  if (etat == "auto")
  {
    int lightValue = analogRead(LIGHT_SENSOR_PIN);
    int servoPosition = map(lightValue, 0, 4095, 0, 90);
    myservo.write(servoPosition);
  }

  pinStatePrevious = pinStateCurrent;
  pinStateCurrent = digitalRead(PIN_TO_SENSOR);

  int analogValue = analogRead(LIGHT_SENSOR_PIN);

  if (analogValue < 40) {
    client.publish("light", String(analogValue).c_str());
  } else if (analogValue < 800) {
    client.publish("light", String(analogValue).c_str());
  } else if (analogValue < 2000) {
    client.publish("light", String(analogValue).c_str());
  } else if (analogValue < 3200) {
    client.publish("light", String(analogValue).c_str());
  } else {
    client.publish("light", String(analogValue).c_str());
  }

  if (pinStateCurrent == HIGH) {
    client.publish("motion", "Motion detected!");
  } else {
    client.publish("motion", "Motion stopped!");
  }

  delay(1000);
  if (!client.connected()) {
    mqttReconnect();
  }
  client.loop();
}

void publishLightAndMotionStatus() {
  DateTime now = rtc.now();
  String lightPayload = String(analogRead(LIGHT_SENSOR_PIN));
  String motionPayload = (pinStateCurrent == HIGH) ? "Motion detected!" : "Motion stopped!";

  client.publish("cmd", (lightPayload + " | " + motionPayload + " | " + now.timestamp()).c_str());
}

void openServo() {
  myservo.write(90);
  delay(1000);
}

void closeServo() {
  myservo.write(0);
  delay(1000);
}
