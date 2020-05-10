#include <ArduinoMqttClient.h>
#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
  #include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
  #include <WiFi101.h>
#elif defined(ARDUINO_ESP8266_ESP12)
  #include <ESP8266WiFi.h>
#endif

#define SECRET_SSID ""
#define SECRET_PASS ""

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "";
int        port     = 1883;
const char topic[]  = "doorbell/frontdoor";

unsigned long previousMillis = 0;

int readDelay = 50;           // 50 is the max, higher makes it unreliable 
int buttonPressDelay = 3000;  // 3000 = 3 seconds 
int inputPin = 4;            // D3 or 0 (for GPIO0)
//int intputValue = 0;

void loopHandler() {
  // Read the input value on the input pin
  int inputValue = digitalRead(inputPin);
  
  // Turn on the internal led, internal led is active low
  digitalWrite(BUILTIN_LED, inputValue);

  if (inputValue == 0) {
    Serial.println("Doorbell pressed ...");
    
    mqttClient.beginMessage(topic);
    mqttClient.print("true");
    mqttClient.endMessage();
    delay(buttonPressDelay);
  }
}

void setup() {
  // Pull-up to prevent "floating" input
  pinMode(inputPin, INPUT_PULLUP);
  pinMode(BUILTIN_LED, OUTPUT);
  
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  WiFi.hostname("Doorbell");
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }

  Serial.println("You're connected to the network");
  Serial.println();

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();
}

void loop() {
  // call poll() regularly to allow the library to send MQTT keep alives which
  // avoids being disconnected by the broker
  mqttClient.poll();
  
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= readDelay) {
    // save the last time a message was sent
    previousMillis = currentMillis;

    loopHandler();
  }
}
