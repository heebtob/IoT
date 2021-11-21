#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "CrestronButton.h"

#ifndef STASSID
#define STASSID "ssid"
#define STAPSK  "pw"
#endif

// ----------------- UDP/ARTNET --------------------
WiFiUDP Udp;
byte crestronIP[] {192,168,0,200};
int crestronPort = 51515;
int WLanStatus = WL_IDLE_STATUS;

// button ------------------------------------------
#define btnCount 6
DebouncedButton buttons[btnCount];

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  //Serial.begin(115200);
  //WiFi.config(IPAddress(10, 11, 120, 20), IPAddress(10, 11, 120, 1), IPAddress(255, 255, 255, 0));
  WiFi.mode(WIFI_STA);
  WiFi.begin(STASSID, STAPSK);
  while (WiFi.status() != WL_CONNECTED) {
    //Serial.print('.');
    delay(500);
  }
  //Serial.print("Connected! IP address: ");
  //Serial.println(WiFi.localIP());

  buttons[0].initialize(5, 100, 0x0701);
  buttons[1].initialize(4, 100, 0x0702);
  buttons[2].initialize(0, 100, 0x0703);
  buttons[3].initialize(2, 100, 0x0704);
  buttons[4].initialize(14, 100, 0x0705);
  buttons[5].initialize(12, 100, 0x0706);

  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  /*if (WLanStatus != WL_CONNECTED && WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println(F("WiFi connected"));

    Serial.print("Connected! IP address: ");
    Serial.println(WiFi.localIP());
  }
  WLanStatus = WiFi.status();*/

  for (int i = 0; i < btnCount; i++) {
    DebouncedButton* button = &buttons[i];
    change res = button->process();

    if (res == high) {
      /*Serial.print("button ");
      Serial.print(i);
      Serial.println(" high");*/
      char buffer [3];
      Udp.beginPacket(crestronIP, crestronPort);
      Udp.write("BTN:");
      itoa(highByte(button->getCrestronId()), buffer, 10);
      Udp.write(buffer);
      itoa(lowByte(button->getCrestronId()), buffer, 10);
      Udp.write(buffer);
      Udp.write(";0\n");
      Udp.endPacket();
    }
    if (res == low) {
      /*Serial.print("button ");
      Serial.print(i);
      Serial.println(" low");*/
      char buffer [3];
      Udp.beginPacket(crestronIP, crestronPort);
      Udp.write("BTN:");
      itoa(highByte(button->getCrestronId()), buffer, 10);
      Udp.write(buffer);
      itoa(lowByte(button->getCrestronId()), buffer, 10);
      Udp.write(buffer);
      Udp.write(";1\n");
      Udp.endPacket();
    }
  }
}
