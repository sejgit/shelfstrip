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
#define concat(first, second, third) first second third //macro for concatenation
#define topic_clientId concat(toptopic, "/", clientId)

const char* topic_control_reset= concat(topic_clientId , "/control", "/reset"); // reset position
const char* message_control_reset[] = {"--", "RESET", "Resetting"};

const char* topic_control_program= concat(topic_clientId, "/control", "/program"); // run program
const char* message_control_program[] = {"OFF", "WHITE", "ACCENT"};

const char* topic_control_accent= concat(topic_clientId, "/control", "/accent"); // accent #
const char* topic_control_bright= concat(topic_clientId, "/control", "/bright"); // bright

const char* topic_status= concat(topic_clientId, "/status", "/status"); // program status
const char* message_status[] = {"OK", "NOK", "CHANGING"};


/*
  Global vars
*/
int program = 0; // start with off
int accent = 0; // start with no accent
int bright = 255; // start with full 100% bright
char buf[25]; // spare char buffer
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
  mqtt.publish(toptopic, concat("connected ", clientId, ".") , true );
  mqtt.publish(topic_status, message_status[0] , true );
  mqtt.subscribe(topic_control_reset);
  mqtt.publish(topic_control_reset, message_control_reset[0] , true );
  mqtt.subscribe(topic_control_program);
  mqtt.publish(topic_control_program, message_control_program[program] , true );
  mqtt.subscribe(topic_control_accent);
  mqtt.publish(topic_control_accent, itoa(accent, buf, 10) , true );
  mqtt.subscribe(topic_control_bright);
  mqtt.publish(topic_control_bright, itoa(bright, buf, 10) , true );
  connected = true;
}

// Callback when MQTT is disconnected
void mqttDisconnected(void* response) {
  Serial.println("MQTT disconnected");
  connected = false;
}

/*
  Handle reset through MQTT or wifi fail
 */
void resetDevice(void)
{
    Serial.println("Rebooting...");
    mqtt.publish(topic_control_reset, message_control_reset[2], true);
    wdt_enable( WDTO_4S);
    delay(2000);
    mqtt.publish(topic_control_reset, message_control_reset[0], true);
    while(1) {}
}


// Callback when an MQTT message arrives for one of our subscriptions
void mqttData(void* response) {
  ELClientResponse *res = (ELClientResponse *)response;

  Serial.print("Received: topic=");
  String topic = res->popString();
  Serial.println(topic);

  Serial.print("data=");
  String data = res->popString();
  Serial.println(data);

  // Reset the camera if RESET received
  if(strcmp(topic.c_str(), topic_control_reset)==0){
      if (strcmp(data.c_str(), message_control_reset[1]) == 0) {
      resetDevice();
    }
  }

   // Set strip program
  if(strcmp(topic.c_str(), topic_control_program)==0){
      if (strcmp(data.c_str(), message_control_program[0]) == 0) {
          program = 0;
          change = true;
          mqtt.publish(topic_status, message_status[2], true);
      } if (strcmp(data.c_str(), message_control_program[1]) == 0) {
          program = 1;
          change = true;
          mqtt.publish(topic_status, message_status[2], true);
      } if (strcmp(data.c_str(), message_control_program[2]) == 0) {
          program = 2;
          change = true;
          mqtt.publish(topic_status, message_status[2], true);
      } else {
          mqtt.publish(topic_status, message_status[1], true);
      }
  }

   // Set accent
  if(strcmp(topic.c_str(), topic_control_accent)==0){
      if (atoi(data.c_str()) >= 0 && atoi(data.c_str()) <= 2) {
          accent = atoi(data.c_str());
          change = true;
          mqtt.publish(topic_status, message_status[2], true);
      } else {
          mqtt.publish(topic_status, message_status[1], true);
      }
  }

   // Set bright
  if(strcmp(topic.c_str(), topic_control_bright)==0){
      if (atoi(data.c_str()) >= 0 && atoi(data.c_str()) <= 255) {
          bright = atoi(data.c_str());
          change = true;
          mqtt.publish(topic_status, message_status[2], true);
      } else {
          mqtt.publish(topic_status, message_status[1], true);
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

static int count;
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
            }
            if (program == 1) {
                analogWrite(S1_WHITEPIN, bright);
                analogWrite(S1_BLUEPIN, 0);
                analogWrite(S1_REDPIN, 0);
                analogWrite(S1_GREENPIN, 0);
                analogWrite(S2_WHITEPIN, bright);
                analogWrite(S2_BLUEPIN, 0);
                analogWrite(S2_REDPIN, 0);
                analogWrite(S2_GREENPIN, 0);
                analogWrite(S3_WHITEPIN, bright);
                analogWrite(S3_BLUEPIN, 0);
                analogWrite(S3_REDPIN, 0);
                analogWrite(S3_GREENPIN, 0);
          }
          if (program == 2) {
              if(accent ==0){
                  analogWrite(S1_WHITEPIN, 0);
                  analogWrite(S1_BLUEPIN, 0);
                  analogWrite(S1_REDPIN, 0);
                  analogWrite(S1_GREENPIN, bright);
                  analogWrite(S2_WHITEPIN, 0);
                  analogWrite(S2_BLUEPIN, 0);
                  analogWrite(S2_REDPIN, bright);
                  analogWrite(S2_GREENPIN, 0);
                  analogWrite(S3_WHITEPIN, 0);
                  analogWrite(S3_BLUEPIN, 0);
                  analogWrite(S3_REDPIN, 0);
                  analogWrite(S3_GREENPIN, bright);
              }
              if(accent ==1){
                  r = 0;
                  g = 0;
                  b = 30;
                  w = 0;
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
          }
          mqtt.publish(topic_status, message_status[0], true);
          change = false;
        }
        last = millis();
    }
}
