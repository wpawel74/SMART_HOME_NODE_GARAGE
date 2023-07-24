/*
  NODE_GARAGE.ino
  The ESP8266 used as IoT node to garage
*/
#include <ESP8266WiFi.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include "EspMQTTClient.h"

//#define LOG_ENABLED
#ifdef LOG_ENABLED
#define LOG(...)         Serial.print(__VA_ARGS__)
#else
#define LOG(...)         do {} while(0)
#endif

// The client will handle wifi connection (connecting, retrying, etc) and MQTT connection
EspMQTTClient client(
  "HOME",               // WIFI SSID
  "",             // WIFI password
  "smart.home",            // MQTT server ip address
  "ESP8266_GARAGE",       // MQTT user name, can be omitted if not needed
  "homeland",             // MQTT password, can be omitted if not needed
  "ESP8266_Node_Garage",  // MQTT unique client name
  1883                    // MQTT port
);

#define ACRELAY_PIN           15 // D8
#define SWITCH1_PIN           16 // D0 ... HV4
#define SWITCH2_PIN           3  // RX ... HV3
#define BUTTON1_PIN           12 // D6 ... HV2 ... RF_1 NICE.SMILO
#define BUTTON2_PIN           14 // D5 ... HV1 ... RF_2
#define ENABLE1_PIN           13 // D7
#define DOOR_RATE             3000   // 3 sec
#define ACRELAY_RATE          1000   // 1 sec

unsigned long ac_tick = 0, ac_timer = 60;
int button1_ls = 0, button2_ls = 0, switch1_ls = 0, switch2_ls = 0;

#define BMP180_SUPPORT
#ifdef BMP180_SUPPORT

#define ALTITUDE              1655.0 // Altitude of SparkFun's HQ in Boulder, CO. in meters
#define BMP180_RATE           30000   // 3 sec

SFE_BMP180 bmp180;
unsigned long bmp180_rate = BMP180_RATE;

void bmp180Handler(){
  double T,P,p0,a;
  char status;
  String out = "";

  status = bmp180.startTemperature();
  if (status != 0){
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = bmp180.getTemperature(T);
    if (status != 0){
      // Print out the measurement:
      LOG("BPM180: temperature ");
      LOG(T,2);
      LOG(" deg C, ");
      LOG((9.0/5.0)*T+32.0,2);
      LOG(" deg F\n");
      if( client.isConnected() ){
        out = T;
        client.publish("HOME/GARAGE/BMP180/TEMPERATURE", out);
      }
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = bmp180.startPressure(3);
      if (status != 0){
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = bmp180.getPressure(P,T);
        if (status != 0){
          // Print out the measurement:
          LOG("BMP180: absolute pressure ");
          LOG(P,2);
          LOG(" mb, ");
          LOG(P*0.0295333727,2);
          LOG(" inHg\n");
          if( client.isConnected() ){
            out = P;
            client.publish("HOME/GARAGE/BMP180/PRESSURE", out);
          }
        }
      }
    }
  }

  client.executeDelayed( bmp180_rate, bmp180Handler );
}
#endif // BMP180_SUPPORT

ICACHE_RAM_ATTR void button1_irq() {
  ac_tick = 1;
  ac_timer = 60;
  digitalWrite(ACRELAY_PIN, HIGH);
  if( button1_ls == 0 && client.isConnected() )
    client.publish("HOME/GARAGE/DOOR/BUTTON1/STATE", "1" );
  button1_ls = 1;
  LOG("1\n");
}

ICACHE_RAM_ATTR void button2_irq() {
  if( button2_ls == 0 && client.isConnected() )
    client.publish("HOME/GARAGE/DOOR/BUTTON2/STATE", "1" );
  button2_ls = 1;
  LOG("2\n");
}

static void AcRelayHandler(){
  if( ac_tick ){
    if( ac_timer < ++ac_tick ){
      digitalWrite(ACRELAY_PIN, 0);    
      ac_tick = 0;
    }
  }
  if( client.isConnected() ){
    client.publish("HOME/GARAGE/DOOR/ACRELAY/STATUS", digitalRead( ACRELAY_PIN ) ? "1": "0" );
    if( button1_ls == 1 && digitalRead( BUTTON1_PIN ) == LOW ) client.publish("HOME/GARAGE/DOOR/BUTTON1/STATE", "0" );
    if( button2_ls == 1 && digitalRead( BUTTON2_PIN ) == LOW ) client.publish("HOME/GARAGE/DOOR/BUTTON2/STATE", "0" );
  }
  if( button1_ls == 1 && digitalRead( BUTTON1_PIN ) == LOW ) button1_ls = 0;
  if( button2_ls == 1 && digitalRead( BUTTON2_PIN ) == LOW ) button2_ls = 0;
  client.executeDelayed( ACRELAY_RATE, AcRelayHandler );
  //LOG("ACRELAY HANDLER");
}

static void garageDoorSensorHandler(){
  if( client.isConnected() ){
    if( switch1_ls != digitalRead( SWITCH1_PIN ) )
      client.publish("HOME/GARAGE/DOOR/SWITCH1/STATUS", digitalRead( SWITCH1_PIN ) ? "1": "0" );
    if( switch2_ls != digitalRead( SWITCH2_PIN ) )
      client.publish("HOME/GARAGE/DOOR/SWITCH2/STATUS", digitalRead( SWITCH2_PIN ) ? "1": "0" );
  }
  switch1_ls = digitalRead( SWITCH1_PIN );
  switch2_ls = digitalRead( SWITCH2_PIN );

  LOG("ESP8266: SWITCH1(");
  LOG(digitalRead( SWITCH1_PIN ));
  LOG(") SWITCH2(");
  LOG(digitalRead( SWITCH2_PIN ));
  LOG(")\n");
  client.executeDelayed( DOOR_RATE, garageDoorSensorHandler );
}

void setup()
{
  Serial.begin(9600);
  pinMode(ACRELAY_PIN, OUTPUT);
  pinMode(SWITCH1_PIN, INPUT);
  pinMode(SWITCH2_PIN, FUNCTION_3);
  pinMode(SWITCH2_PIN, INPUT);
  pinMode(BUTTON1_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);
  pinMode(ENABLE1_PIN, OUTPUT);
  digitalWrite(ENABLE1_PIN, LOW);
  LOG("ESP8266: GPIO ... configured\n");

  attachInterrupt(digitalPinToInterrupt(BUTTON1_PIN), button1_irq, RISING);
  attachInterrupt(digitalPinToInterrupt(BUTTON2_PIN), button2_irq, RISING);
  LOG("ESP8266: IRQ  ... initialized\n");

  client.executeDelayed( ACRELAY_RATE, AcRelayHandler );
  client.executeDelayed( DOOR_RATE, garageDoorSensorHandler );

#ifdef BMP180_SUPPORT
  if (bmp180.begin()){
    LOG("BMP180:       ... detected\n");
    client.executeDelayed( bmp180_rate, bmp180Handler );
  }
#endif // BMP180_SUPPORT
}

// For client
void onConnectionEstablished()
{
#ifdef BMP180_SUPPORT
  client.subscribe("HOME/GARAGE/BMP180/RATE", [](const String & payload) {
    LOG("MQTT: BMP180 rate ");
    LOG(payload);
    LOG(" ms\n");
    if( payload.toInt() > 0 ){
      bmp180_rate = payload.toInt();
    }
  });
#endif // BMP180_SUPPORT
  client.subscribe("HOME/GARAGE/DOOR/BUTTON1/PIN", [](const String & payload) {
    LOG("MQTT: BUTTON1 ");
    LOG(payload);
    LOG("\n");
    digitalWrite(ENABLE1_PIN, payload.toInt());
  });
  client.publish("HOME/GARAGE/DOOR/SWITCH1/STATUS", digitalRead( SWITCH1_PIN ) ? "1": "0" );
  client.publish("HOME/GARAGE/DOOR/SWITCH2/STATUS", digitalRead( SWITCH2_PIN ) ? "1": "0" );
  client.publish("HOME/GARAGE/DOOR/ACRELAY/STATUS", digitalRead( ACRELAY_PIN ) ? "1": "0" );
  client.publish("HOME/GARAGE/DOOR/BUTTON1/STATE", digitalRead( BUTTON1_PIN ) ? "1": "0" );
  client.publish("HOME/GARAGE/DOOR/BUTTON2/STATE", digitalRead( BUTTON2_PIN ) ? "1": "0" );
  switch1_ls = digitalRead( SWITCH1_PIN );
  switch2_ls = digitalRead( SWITCH2_PIN );

  LOG("MQTT: ... CONNECTED!\n");
}

void loop()
{
  client.loop();
}
