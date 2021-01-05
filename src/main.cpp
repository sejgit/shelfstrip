/*
  Shelfstrip

  Mega controller of three RGBW strips
  Using a Node CU as pass through WiFi
  MQTT status and control of the strip program to be run
  Not referenced here: ISY controller sends MQTT messages
  ELClient runs on the Node CU
 */


/*
  Includes
*/
#include <stdlib.h>
#include <Arduino.h>
#include <ELClient.h>
#include <ELClientCmd.h>
#include <ELClientMqtt.h>
#include <avr/wdt.h>
#include <ArduinoJson.h>


/*
  Pin definition
*/
#define S1_WHITEPIN 2
#define S1_BLUEPIN 3
#define S1_REDPIN 4
#define S1_GREENPIN 5

#define S2_WHITEPIN 6
#define S2_BLUEPIN 7
#define S2_REDPIN 8
#define S2_GREENPIN 9

#define S3_WHITEPIN 10
#define S3_BLUEPIN 11
#define S3_REDPIN 12
#define S3_GREENPIN 13

#define FADESPEED 4000     // make this higher to slow down

void pinsetup() {
  pinMode(S1_REDPIN, OUTPUT);
  pinMode(S1_GREENPIN, OUTPUT);
  pinMode(S1_BLUEPIN, OUTPUT);
  pinMode(S1_WHITEPIN, OUTPUT);

  pinMode(S2_REDPIN, OUTPUT);
  pinMode(S2_GREENPIN, OUTPUT);
  pinMode(S2_BLUEPIN, OUTPUT);
  pinMode(S2_WHITEPIN, OUTPUT);

  pinMode(S3_REDPIN, OUTPUT);
  pinMode(S3_GREENPIN, OUTPUT);
  pinMode(S3_BLUEPIN, OUTPUT);
  pinMode(S3_WHITEPIN, OUTPUT);
}


/*
  MQTT
*/
#define toptopic "sej" // main topic
#define clientId "shelfstrip" // client ID for this unit
#define concat(first, second, third, fourth) first second third fourth //macro for concatenation

const char* topic_control = concat(toptopic, "/", clientId, "/control"); // reset position
const char* topic_status= concat(toptopic, "/", clientId, "/status"); // program status


/*
  Global vars
*/
boolean state = false; // start with off
int brightness = 255; // start with full 100% bright
int red = 0; // red led
int green = 0; // green led
int blue = 0; // blue led
int white = 0; // white led
int program = 0; // start with off
char buf[100]; // spare char buffer
boolean change = false;


/*
  Initialize ELClient & MQTT
*/
// Initialize a connection to esp-link using the normal hardware serial port both for
// SLIP and for debug messages.
ELClient esp(&Serial, &Serial);

// Initialize CMD client (for GetTime)
ELClientCmd cmd(&esp);

// Initialize the MQTT client
ELClientMqtt mqtt(&esp);

// Callback made from esp-link to notify of wifi status changes
// Here we just print something out for grins
void wifiCb(void* response) {
  ELClientResponse *res = (ELClientResponse*)response;
  if (res->argc() == 1) {
    uint8_t status;
    res->popArg(&status, 1);

    if(status == STATION_GOT_IP) {
      Serial.println("WIFI CONNECTED");
    } else {
      Serial.print("WIFI NOT READY: ");
      Serial.println(status);
    }
  }
}

bool connected;

// Callback when MQTT is connected
void mqttConnected(void* response) {
  Serial.println("MQTT connected!");
  mqtt.publish(toptopic, concat("connected ", toptopic, clientId, ".") , true );
  mqtt.subscribe(topic_control);
  connected = true;
}

// Callback when MQTT is disconnected
void mqttDisconnected(void* response) {
  Serial.println("MQTT disconnected");
  connected = false;
}


// Callback when an MQTT message arrives for one of our subscriptions
void mqttData(void* response) {
  ELClientResponse *res = (ELClientResponse *)response;

  Serial.print("Received: topic=");
  String topic = res->popString();
  Serial.println(topic);

  char data[50];
  Serial.print("data=");
  res->popChar(data);
  Serial.println(data);

   // topic control
  if(strcmp(topic.c_str(), topic_control)==0){
      DynamicJsonDocument doc(200);
      DeserializationError error = deserializeJson(doc, data);
      if(error) {
          Serial.print(("deserializeJson() failed: "));
          Serial.println(error.f_str());
      } else {
          state = (doc["state"] == "ON");
          brightness = doc["brightness"];
          red = doc["state"]["r"];
          green = doc["state"]["g"];
          blue = doc["state"]["b"];
          white = doc["state"]["w"];
          program = doc["program"];
          change = true;
          Serial.println(state);
          Serial.println(brightness);
          Serial.println("LED Color");
          Serial.println(red);
          Serial.println(green);
          Serial.println(blue);
          Serial.println(white);
          Serial.println(program);
      }
  }
}


void mqttPublished(void* response) {
  Serial.println("MQTT published");
}


/*
  Set-up
*/
void setup() {
  Serial.begin(115200);
  Serial.println("shelfstrip EL-Client starting!");

  // Sync-up with esp-link, this is required at the start of any sketch and initializes the
  // callbacks to the wifi status change callback. The callback gets called with the initial
  // status right after Sync() below completes.
  esp.wifiCb.attach(wifiCb); // wifi status change callback, optional (delete if not desired)
  bool ok;
  do {
    ok = esp.Sync();      // sync up with esp-link, blocks for up to 2 seconds
    if (!ok) Serial.println("EL-Client sync failed!");
  } while(!ok);
  Serial.println("EL-Client synced!");

  // Set-up callbacks for events and initialize with es-link.
  mqtt.connectedCb.attach(mqttConnected);
  mqtt.disconnectedCb.attach(mqttDisconnected);
  mqtt.publishedCb.attach(mqttPublished);
  mqtt.dataCb.attach(mqttData);
  mqtt.setup();

  //Serial.println("ARDUINO: setup mqtt lwt");
  //mqtt.lwt("/lwt", "offline", 0, 0); //or mqtt.lwt("/lwt", "offline");

  Serial.println("EL-MQTT ready");
}

int count;
static uint32_t last;

void loop() {
    int r, g, b, w;
    esp.Process();

    if (connected && (millis()-last) > FADESPEED) {
        if(change == true){
            Serial.println("change");
            if (program == 0) {
                analogWrite(S1_WHITEPIN, 0);
                analogWrite(S1_BLUEPIN, 0);
                analogWrite(S1_REDPIN, 0);
                analogWrite(S1_GREENPIN, 0);
                analogWrite(S2_WHITEPIN, 0);
                analogWrite(S2_BLUEPIN, 0);
                analogWrite(S2_REDPIN, 0);
                analogWrite(S2_GREENPIN, 0);
                analogWrite(S3_WHITEPIN, 0);
                analogWrite(S3_BLUEPIN, 0);
                analogWrite(S3_REDPIN, 0);
                analogWrite(S3_GREENPIN, 0);
            } else if (program == 1) {
                analogWrite(S1_WHITEPIN, brightness);
                analogWrite(S1_BLUEPIN, 0);
                analogWrite(S1_REDPIN, 0);
                analogWrite(S1_GREENPIN, 0);
                analogWrite(S2_WHITEPIN, brightness);
                analogWrite(S2_BLUEPIN, 0);
                analogWrite(S2_REDPIN, 0);
                analogWrite(S2_GREENPIN, 0);
                analogWrite(S3_WHITEPIN, brightness);
                analogWrite(S3_BLUEPIN, 0);
                analogWrite(S3_REDPIN, 0);
                analogWrite(S3_GREENPIN, 0);
            } else if (program == 2) {
                analogWrite(S1_WHITEPIN, 0);
                analogWrite(S1_BLUEPIN, 0);
                analogWrite(S1_REDPIN, 0);
                analogWrite(S1_GREENPIN, brightness);
                analogWrite(S2_WHITEPIN, 0);
                analogWrite(S2_BLUEPIN, 0);
                analogWrite(S2_REDPIN, brightness);
                analogWrite(S2_GREENPIN, 0);
                analogWrite(S3_WHITEPIN, 0);
                analogWrite(S3_BLUEPIN, 0);
                analogWrite(S3_REDPIN, 0);
                analogWrite(S3_GREENPIN, brightness);
            } else if(program == 3){
                r = 0;
                g = 0;
                b = 30;
                w = 0;
                count = count + 1;
                analogWrite(S1_WHITEPIN, w);
                analogWrite(S1_BLUEPIN, b);
                analogWrite(S1_REDPIN, r);
                analogWrite(S1_GREENPIN, g);
                analogWrite(S2_WHITEPIN, w);
                analogWrite(S2_BLUEPIN, b);
                analogWrite(S2_REDPIN, r);
                analogWrite(S2_GREENPIN, g);
                analogWrite(S3_WHITEPIN, w);
                analogWrite(S3_BLUEPIN, b);
                analogWrite(S3_REDPIN, r);
                analogWrite(S3_GREENPIN, g);
            }

            // TODO build "test" into buid JSON
            mqtt.publish(topic_status, "test" , true);
            change = false;
        }
        last = millis();
    }
}
