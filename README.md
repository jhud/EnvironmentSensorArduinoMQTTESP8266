# EnvironmentSensorArduinoMQTTESP8266
Arduino sketch for ESP8266 to read temperature/humidity and post it to Adafruit IO MQTT.

Supports a couple of different sensors, and OTA updates.
This is a rough "just works" sketch for my own home data logger - I'll add to it as I need more sensors.

Thanks to the authors of the original libraries - most of this was lifted from the example code.

![Pic of Adafruit IO graphs](/example_pic.png)

## Instructions

- Install all the libraries

- Rename ```secrets.h.changeme.txt``` to ```secrets.h``` and fill it with your MQTT keys and wifi credentials. Make sure you do not commit this file to source control!
