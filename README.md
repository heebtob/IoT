# Adafruit Feather M0 RFM69HCW
RFM Server which receives messages over 433 MHz and sends them to MQTT Server.
CO2 Sensor which sends messages to the RFM69 Server over 433 MHz.

# Arduino UNO
Temperature and Humidity Sensor (EmonTH) which sends data over 433 MHz to RFM69 Server.

# ESP8266
Lightswitch connected to an IO Ports to send switch commands via UDP to a server (for example Crestron).
Receive ARTNET commands over WIFI and sends them to a DMX Controller for LED Scene Control.

# MKR with ETH Shield
Receive ARTNET commands over Ethernet and sends them to a DMX Controller for LED Scene Control.
Lightswitch connected to an IO Ports to send switch commands via UDP to a server (for example Crestron).

# MKR GSM
Send Alarm SMS received over MQTT
Send Alarm SMS at mains power outage or if battery level is low.

# ESPHome
ESP32 ESP-IDF external component to receive Art-Net DMX data over Wi-Fi and output DMX512 using an RS-485 transceiver. See [ESPHome/README.md](ESPHome/README.md) for setup, wiring, and configuration.

# Feller UNI-Taster
ESPHome external component for Feller EDIZIOdue colore UNI-Taster 392x: button sensors, tri-colour LED lights with hardware blinking, and global brightness. See [ESPHome/components/feller_uni_taster/README.md](ESPHome/components/feller_uni_taster/README.md) and the [complete ESP32 example](ESPHome/feller-uni-taster.yaml) for wiring and setup. Paste this source into ESPHome Builder:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/heebtob/IoT
      ref: main
      path: ESPHome/components
    components: [feller_uni_taster]
```
