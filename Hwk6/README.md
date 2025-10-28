Hwk6: MQTT v5 Project Details
This file contains the necessary details for configuring and running the MQTT v5 assignment code.

Local Broker IP Address
The IP address used for the local Mosquitto broker (running on the Linux machine connected to the "S24 Ultra Sergio" hotspot) during Part B testing was:
10.121.44.169
This IP address is configured in the app_main.c file when the USE_LOCAL_PYTHON_LOGGER switch is set to 1.

Adafruit IO Details (Part C)
Username: sergio241618
Password: aio_ZXzM52w7a5E6chR3v3MobGOVTVt0
Feeds:
Temperature: u77.temperature (Published to sergio241618/feeds/u77.temperature)
GPS: u77.gps (Published to sergio241618/feeds/u77.gps)

These details are configured in the app_main.c file when the USE_LOCAL_PYTHON_LOGGER switch is set to 0.