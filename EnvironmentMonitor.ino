// Things that you probably want enabled
#define ADC_ENABLED
#define OTA_ENABLED
//#define DS2438_ENABLED
#define DHT_ENABLED

#define NUM_OF(x) (sizeof(x)/sizeof(x[0]))

#include "secrets.h" // Network keys, etc.

//#define ESP32

//#define PRINT(x) /*x*/

#define PRINT(x) Serial.println(x)


#include "Adafruit_MQTT.h" // The generic pub/sub library didn't work
#include "Adafruit_MQTT_Client.h"

#ifdef ESP32
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#else
#include "ESP8266WiFi.h"
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti;
#endif

#ifdef OTA_ENABLED

#include <ArduinoOTA.h>
#endif

#ifdef DS2438_ENABLED
#include <OneWire.h>
#include <DS2438.h>
#endif

#ifdef DHT_ENABLED
#include "DHTesp.h"
DHTesp dht;
/** Pin number for DHT11 data pin */
int dhtPin = 5;
#endif

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883


void callback(char* topic, byte* message, unsigned int length);

WiFiClient espClient;
Adafruit_MQTT_Client mqtt(&espClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish temperatureFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish humidityFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");\

// To get value at startup https://io.adafruit.com/api/docs/mqtt.html#using-the-get-topic
Adafruit_MQTT_Publish alertGetFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity_alert/get"); 


Adafruit_MQTT_Subscribe alert = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/humidity_alert", MQTT_QOS_1);
Adafruit_MQTT_Subscribe errors = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/errors");

#ifdef DS2438_ENABLED
const uint8_t ONE_WIRE_PIN = 0;
OneWire ow(ONE_WIRE_PIN);

// NOTE - there are also bugs where stack variables are referenced, ie the address! Make sure these are fixed.
DS2438 ds2438(&ow); // https://github.com/jhautbois/arduino-onewire-DS2438 - note auto-search for device address
#endif

unsigned long startTime;

float voltage = -1;
float temperature = -1;
float humidity = -1;
float humidityAlert = 70; // When humidity goes above this value, sound an alert


long lastNetworkCheckMillis = 0;  
long lastPacketReceivedMillis = 0;  
bool isConnected = false;
int currentNetwork = 0;


String deviceName;


// ------- declarations
void connectToPreferredAccessPoint();
void setupOTA(void);


void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);  
  
  uint8_t mac[6];
  WiFi.macAddress(mac);

  deviceName = "mon_" + String(mac[3], HEX) + String(mac[4], HEX) + String(mac[5], HEX);


#ifdef DS2438_ENABLED
  if (ds2438.begin(DS2438_MODE_CHB | DS2438_MODE_TEMPERATURE) == false) {
      PRINT("DS2438 not found");
  }
#endif

#ifdef DHT_ENABLED
  PRINT("Setting up DHT...");
  dht.setup(dhtPin, DHTesp::DHT22);
#endif

  for (int i=0; i<NUM_OF(networks); i++) {
    wifiMulti.addAP(networks[i][0], networks[i][1]);
  }
    
  connectToPreferredAccessPoint();

  setupOTA();
}

void readBatterySensor() {
#ifdef DS2438_ENABLED
    int err = ds2438.update(); // James modified this library for greater debugging
    if (ds2438.isError()) {
      PRINT(String("Sensor error ") + String(err));
      voltage = -1;
      temperature = -1;
    } else {

        temperature = ds2438.getTemperature();
        voltage = ds2438.getVoltage(DS2438_CHB);
    }
#endif

#ifdef DHT_ENABLED
TempAndHumidity lastValues = dht.getTempAndHumidity();
Serial.println("Temperature: " + String(lastValues.temperature));
Serial.println("Humidity: " + String(lastValues.humidity));
Serial.println(dht.getStatusString());
temperature = lastValues.temperature;
humidity = lastValues.humidity;
#endif
}




     
void connectToPreferredAccessPoint() {
  isConnected = false;

  WiFi.mode(WIFI_STA); // Don't advertise us as an AP

  int status = WL_IDLE_STATUS; 
  for (int i=0; i<100 && status != WL_CONNECTED; i++) {
      PRINT("Connecting try " + String(i));
      status = wifiMulti.run();
      if (status != WL_CONNECTED) {
        delay(500);
      }
  }

  isConnected = (status == WL_CONNECTED);

  if (isConnected == false) {
    PRINT("No AP found.");
  }
  else {    
    for (int i=0; i<NUM_OF(networks); i++) {
      if (String(networks[i][0]).equals(WiFi.SSID())) {
        currentNetwork = i;
        PRINT("Connected to " + WiFi.SSID());  
        PRINT("At " + WiFi.localIP());  

          mqtt.subscribe(&alert);
          mqtt.subscribe(&errors);
        
        return;
      }
    }
    PRINT("ERROR on " + WiFi.SSID());  
  }
}      

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");

  alertGetFeed.publish(""); // Tell Adafruit IO to send us the alert value once on startup
}

int frame=0;

void loop()
{ 
#ifdef OTA_ENABLED
  ArduinoOTA.handle();
#endif

  wl_status_t status = WiFi.status(); 
  if (status != WL_CONNECTED) {  
          connectToPreferredAccessPoint();
  }
  char buff[8];
  readBatterySensor();

  MQTT_connect();

   sprintf(buff,"%f", temperature);
        temperatureFeed.publish(buff);
        sprintf(buff,"%f", humidity);
        humidityFeed.publish(buff);
              for (int i=0; i< 3; i++) {
          digitalWrite(LED_BUILTIN, LOW); 
        delay(10);                     
        digitalWrite(LED_BUILTIN, HIGH);  
        delay(10); 
        }

    int numLoops = 30;

    if (humidity >= humidityAlert) {
      PRINT("Humidity alert!");
       digitalWrite(LED_BUILTIN, LOW); 
       numLoops = 3;
    }

   for (int i=0; i<numLoops; i++) {
    #ifdef OTA_ENABLED
  ArduinoOTA.handle();
  #endif

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(4000))) {
    if (subscription == &alert) {
      humidityAlert = atof((char *)alert.lastread);
      PRINT("Setting humidity alert to ");
      PRINT(humidityAlert);
    } else if(subscription == &errors) {
      Serial.print(F("ERROR: "));
      Serial.println((char *)errors.lastread);
    }
  }
  
  if(! mqtt.ping()) {
    mqtt.disconnect();
  }

  delay(1000);      
   } 
}


void setupOTA(void) {
    #ifdef OTA_ENABLED
  ArduinoOTA.setHostname(String(deviceName+"_sensor").c_str());
  ArduinoOTA.setPassword(password);
  ArduinoOTA.onStart([]() { 
    PRINT("Updating...");
                    });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) { PRINT(String(progress*100/total) + "%");
                        });

  ArduinoOTA.onEnd([]() { 
    PRINT("Update done");
    ESP.restart();
                        });

   ArduinoOTA.onError([](ota_error_t error) { PRINT("Error updating");
                                                ESP.restart(); });

   /* setup the OTA server */
   ArduinoOTA.begin();
   #endif
}
