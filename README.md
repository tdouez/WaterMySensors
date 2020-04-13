# WaterMySensors
Watermeters MySensors

Technical Specification

Designed to fit in a 3D enclosure available on Thingiverse https://www.thingiverse.com/thing:4280207 ESP8266 board Onboard OLED for information and leak detection Powered and programmed over USB and OTA. Wifi - Web pages

Functional Specification

Measure volume and flow of your house water meter. You need to set the correct pulsefactor of your meter (pulses per m3). The sensor starts by fetching current volume reading from broker.

Reports to MQTT broker : flow - liters per minute volume - liters per day pulse - counter pulses leak - leak (0 : no leak, 1 : continuous consumption during 24h, 2 : consumption in excess of 24h threshold )

Web configuration pulse factor leak threshold mqtt configuration

Web information volume pulse flow 24h consumption leak information
