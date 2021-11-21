// rf69_reliable_datagram_server.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging server
// with the RHReliableDatagram class, using the RH_RF69 driver to control a RF69 radio.
// It is designed to work with the other example rf69_reliable_datagram_client
// Tested on Moteino with RFM69 http://lowpowerlab.com/moteino/
// Tested on miniWireless with RFM69 www.anarduino.com/miniwireless
// Tested on Teensy 3.1 with RF69 on PJRC breakout board

#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <Ethernet.h>
#include <MQTT.h>

#define debug false

// sercom, miso, sck, mosi, padtx, padrx
SPIClass mySPI (&sercom1, 12, 13, 11, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);

#define ETH_CS 10
EthernetClient net;
MQTTClient client(256);
unsigned long next;

uint8_t mac[6] = {0xaa, 0x0f, 0x9a, 0xcb, 0xac, 0xff};

IPAddress myIP(10, 11, 120, 19);
IPAddress myMASK(255, 255, 255, 0);
IPAddress myDNS(10, 11, 120, 254);
IPAddress myGW(10, 11, 120, 254);

// The sync words and the encryption key has to be the same as the one in the server
const uint8_t syncWords[] = {0x00, 0x00};
uint8_t key[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                };

//#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 99

#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define RF69_FREQ 434.0

/*
  typedef struct {                                                      // RFM RF payload datastructure
  uint16_t temp;
  uint16_t humidity;
  uint16_t battery;
  } Payload;
  Payload emonth;*/

typedef struct {                                                      // RFM RF payload datastructure
  uint16_t temp;
  uint16_t humidity;
  uint16_t battery;
  uint16_t type;
  uint16_t ext1;
  uint16_t ext2;
  uint16_t ext3;
  uint16_t ext4;
  uint16_t ext5;
} Payload;
Payload emonth;

// Singleton instance of the radio driver
RH_RF69 driver(RFM69_CS, RFM69_INT); // Adafruit Feather M0

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SERVER_ADDRESS);

bool checkEthernet();
bool connectMqtt();
void messageReceived(String &topic, String &payload);

void setup()
{
  if (debug) {
    Serial.begin(9600);
    while (!Serial)
      ;
  }

  if (debug) Serial.println("start ethernet");

  Ethernet.init(ETH_CS);
  Ethernet.begin(mac, myIP);

  delay(3000);

  if (debug) {
    Serial.print("localIP: ");
    Serial.println(Ethernet.localIP());
    Serial.print("subnetMask: ");
    Serial.println(Ethernet.subnetMask());
    Serial.print("gatewayIP: ");
    Serial.println(Ethernet.gatewayIP());
    Serial.print("dnsServerIP: ");
    Serial.println(Ethernet.dnsServerIP());
  }

  checkEthernet();
  client.begin("10.11.120.72", net);
  client.onMessage(messageReceived);
  connectMqtt();

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  if (debug) Serial.println("Feather RFM69 RX Test!");

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (debug) Serial.println("init rf");

  if (!manager.init())
    if (debug) Serial.println("init failed");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)

  if (debug) Serial.println("configure rf");

  driver.setSyncWords(syncWords, 2);
  driver.setEncryptionKey(key);

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  driver.setTxPower(20, true);

  if (debug) Serial.println("setup finished");
}

uint8_t data[] = "ok server";
// Dont put this on the stack:
//byte buf[sizeof(Payload)] = {0};
byte buf[RH_RF69_MAX_MESSAGE_LEN] = {0};


void loop()
{
  if (manager.available())
  {
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(Payload);
    uint8_t from;
    if (manager.recvfromAck(buf, &len, &from))
    {
      memcpy(&emonth, buf, sizeof(Payload));

      if (debug) {
        Serial.print("got V2 request from : 0x");
        Serial.println(from, HEX);
        Serial.print("type:");
        Serial.println(emonth.type);
        Serial.print("temp:");
        Serial.println(emonth.temp);
        Serial.print("humidity:");
        Serial.println(emonth.humidity);
        Serial.print("battery:");
        Serial.println(emonth.battery);
        Serial.print("ext1:");
        Serial.println(emonth.ext1);
        Serial.print("ext2:");
        Serial.println(emonth.ext2);
        Serial.print("ext3:");
        Serial.println(emonth.ext3);
        Serial.print("ext4:");
        Serial.println(emonth.ext4);
        Serial.print("ext5:");
        Serial.println(emonth.ext5);
      }

      // Send a reply back to the originator client
      if (!manager.sendtoWait(data, sizeof(data), from))
        if (debug) Serial.println("sendtoWait failed");

      client.loop();
      if (!client.connected()) {
        connectMqtt();
      }

      if (debug) Serial.println((String)"{\"rfm/" + from + "/temperature\":" + emonth.temp + ",\"rfm/" + from + "/humidity\":" + emonth.humidity + ",\"rfm/" + from + "/battery\":" + emonth.battery + ",\"rfm/" + from + "/ext1\":" + emonth.ext1 + ",\"rfm/" + from + "/ext2\":" + emonth.ext2 + ",\"rfm/" + from + "/ext3\":" + emonth.ext3 + ",\"rfm/" + from + "/ext4\":" + emonth.ext4 + ",\"rfm/" + from + "/ext5\":" + emonth.ext5 + ",\"rfm/" + from + "/type\":" + emonth.type + "}");

      if (!client.publish("/rfmdata", (String)"{\"rfm/" + from + "/temperature\":" + emonth.temp + ",\"rfm/" + from + "/humidity\":" + emonth.humidity + ",\"rfm/" + from + "/battery\":" + emonth.battery + ",\"rfm/" + from + "/ext1\":" + emonth.ext1 + ",\"rfm/" + from + "/ext2\":" + emonth.ext2 + ",\"rfm/" + from + "/ext3\":" + emonth.ext3 + ",\"rfm/" + from + "/ext4\":" + emonth.ext4 + ",\"rfm/" + from + "/ext5\":" + emonth.ext5 + ",\"rfm/" + from + "/type\":" + emonth.type + "}")) {
        if (debug) Serial.println("publish failed, either connection lost, or message too large");
      } else {
        if (debug) Serial.println("publish succeeded");
      }
    }
  }
}

bool checkEthernet() {
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    if (debug) Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    return false;
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    if (debug) Serial.println("Ethernet cable is not connected.");
    return false;
  }
  return true;
}

bool connectMqtt() {
  if (debug) Serial.print("connecting...");

  int i = 0;
  while (!client.connect("client", "user", "pw")) {
    if (i == 4) {
      if (debug) Serial.print("not connected after 5 times");
      return false;
    }
    if (debug) Serial.print(".");
    i++;
    delay(400);
  }

  if (debug) Serial.println("\nconnected!");

  return true;
}

void messageReceived(String & topic, String & payload) {
  if (debug)
    Serial.println("incoming: " + topic + " - " + payload);
}
