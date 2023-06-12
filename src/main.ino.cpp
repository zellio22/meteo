# 1 "C:\\Users\\zellio\\AppData\\Local\\Temp\\tmpgmjkgyap"
#include <Arduino.h>
# 1 "C:/Users/zellio/Documents/PlatformIO/Projects/meteo/src/main.ino"

#include <Arduino.h>

#include <SPI.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include <Adafruit_BMP085.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
Adafruit_BMP085 bmp180;
Adafruit_BMP280 bmp280;
#define DHTPIN 2
#define DHTTYPE DHT11
#include <wifi_pass.h>
DHT dht(DHTPIN, DHTTYPE);


const char* mqtt_server = "192.168.1.14";
const int mqttPort = 8500;

unsigned long last_reading = 0;
unsigned long ms_between_reads = 5000;

struct MyStruct_send {

  String temperature_dht;
  String humidity_dht;
  String temperature_bmp180;
  String pression_bmp180;
  String temperature_bmp280;
  String pression_bmp280;
  String ram;
  int wifi;
};

struct MyStruct_res {
  int var1;
  int var2;
  int var3;
  int var4;
  int loop_time = 2000;
};
MyStruct_res retour_mqtt;
MyStruct_send capteur={"0","0","0","0"};
WiFiClient espClient;
PubSubClient client(espClient);
void setup();
void setup_mqtt();
void callback(char* topic, byte *payload, unsigned int length);
void reconnect();
String structToJson(MyStruct_send myStruct_send);
void loop();
#line 51 "C:/Users/zellio/Documents/PlatformIO/Projects/meteo/src/main.ino"
void setup(){
    Serial.begin(115200);
    dht.begin();

    if (!bmp180.begin()) {
     Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
    }


    bmp280.setSampling(Adafruit_BMP280::MODE_FORCED,
                Adafruit_BMP280::SAMPLING_X2,
                Adafruit_BMP280::SAMPLING_X16,
                Adafruit_BMP280::FILTER_X16,
                Adafruit_BMP280::STANDBY_MS_63);


    while ( !Serial ) delay(100);
    Serial.println(F("BMP280 test"));
    unsigned status;
    status = bmp280.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
          delay(500);
          Serial.print(".");
        }
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("IP MAC Address: ");
    Serial.println(WiFi.macAddress());
    Serial.println("IP Sub net Mask: ");
    Serial.println(WiFi.subnetMask());
    setup_mqtt();

    setCpuFrequencyMhz(80);


}

void setup_mqtt(){
    client.setServer(mqtt_server, mqttPort);
    client.setKeepAlive(70);
    client.setCallback(callback);
    reconnect();
}

void callback(char* topic, byte *payload, unsigned int length) {
    Serial.println("-------Nouveau message du broker mqtt-----");
    StaticJsonDocument<200> jsonDocument;
    DeserializationError error = deserializeJson(jsonDocument, payload, length);
    retour_mqtt.var1 = {jsonDocument["var1"]};
    retour_mqtt.var2 = {jsonDocument["var2"]};
    retour_mqtt.var3 = {jsonDocument["var3"]};
    retour_mqtt.var4 = {jsonDocument["var4"]};
    retour_mqtt.loop_time = {jsonDocument["loop_time"]};
    Serial.print("Canal:");
    Serial.println(topic);
    Serial.print("donnee 1: ");
    Serial.println(retour_mqtt.var1);
    Serial.print("donnee 2: ");
    Serial.println(retour_mqtt.var2);
    Serial.print("donnee 3: ");
    Serial.println(retour_mqtt.var3);
    Serial.print("donnee 4: ");
    Serial.println(retour_mqtt.var4);
    Serial.print("loop_time: ");
    Serial.println(retour_mqtt.loop_time);

}

void reconnect(){
    while (!client.connected()) {
        Serial.println("Connection au serveur MQTT ...");
        if (client.connect("ESP32Client")) {
            Serial.print("MQTT connecté: ");
            Serial.println(client.connect("ESP32Client"));
            Serial.print("Client state: ");
            Serial.println(client.state());
    }
        else {
            Serial.print("echec, code erreur= ");
            Serial.println(client.state());
            Serial.println("nouvel essai dans 2s");
        delay(1000);

        }
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    client.subscribe("esp_meteo/to_esp");
    }

}


String structToJson(MyStruct_send myStruct_send) {

    StaticJsonDocument<200> jsonDocument;
    jsonDocument["humidity_dht"] = capteur.humidity_dht;
    jsonDocument["temp dht"] =capteur.temperature_dht;
    jsonDocument["pression_bmp_180"] = capteur.pression_bmp180;
    jsonDocument["temp_bmp_180"] = capteur.temperature_bmp180;
    jsonDocument["pression_bmp_280"] = capteur.pression_bmp280;
    jsonDocument["temp_bmp_280"] = capteur.temperature_bmp280;
    jsonDocument["Ram_esp"] = capteur.ram;
    jsonDocument["wifi_rssi"]=capteur.wifi;

    String jsonString;

    serializeJsonPretty(jsonDocument, jsonString);


    return jsonString;
}
boolean status =0;
void loop(){

    size_t freeHeap = ESP.getFreeHeap();
    Serial.print("Espace de RAM disponible : ");
    Serial.print(freeHeap);
    Serial.println(" octets");


    capteur.humidity_dht=dht.readHumidity();
    capteur.temperature_dht=dht.readTemperature();
    capteur.pression_bmp180=String(bmp180.readPressure());
    capteur.temperature_bmp180=String(bmp180.readTemperature());
    capteur.pression_bmp280=String(bmp280.readPressure());
    capteur.temperature_bmp280=String(bmp280.readTemperature());
    capteur.ram=String(freeHeap);
    capteur.wifi=WiFi.RSSI();
    Serial.print("Humidity dht 11: "); Serial.println(capteur.humidity_dht);
    Serial.print("Temp dht 11: "); Serial.println(capteur.temperature_dht);
    Serial.print("Pression bmp 180: "); Serial.println(capteur.pression_bmp180);
    Serial.print("Temp bmp 180: "); Serial.println(capteur.temperature_bmp180);
    Serial.print("Pression bmp 280: "); Serial.println(capteur.pression_bmp280);
    Serial.print("Temp bmp 280: "); Serial.println(capteur.temperature_bmp280);
    Serial.print("Wifi rssi: ");Serial.println(capteur.wifi);


    client.loop();
    reconnect();

    while (WiFi.status() != WL_CONNECTED) {
          delay(500);
          Serial.println("reco au wifi");
          Serial.print(".");
        }

    status=client.publish("esp_meteo/from_esp",structToJson(capteur).c_str());
    Serial.println("send mqtt");

    delay(retour_mqtt.loop_time);


}