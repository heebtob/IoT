// libraries
#include <MKRGSM.h>
#include <SPI.h>
#include <Ethernet.h>
#include <MQTT.h>

#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x)  Serial.print (x)
#define DEBUG_PRINTLN(x)  Serial.println (x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

// -------------------------------------------------------------------------------
// Enter a MAC address and IP address for your controller below.
byte mac[] = {
  0xDE, 0xAF, 0xBD, 0x14, 0xFA, 0xEC
};
byte ip[] = {10, 11, 30, 33};
byte subnet[] = {255, 255, 255, 0};
EthernetClient net;
MQTTClient client;

// -------------------------------------------------------------------------------
const char * delimiter = ";";

// -------------------------------------------------------------------------------
#include "arduino_secrets.h"
// Please enter your sensitive data in the Secret tab or arduino_secrets.h
// PIN Number
const char PINNUMBER[] = SECRET_PINNUMBER;
// initialize the library instance
GSM gsmAccess; // include a 'true' parameter for debug enabled
GSM_SMS sms;

// -------------------------------------------------------------------------------
enum ConnectionState_enum { STARTUP, CONNECTION_ESTABLISHED, CONNECTION_LOST, CONNECTION_REESTABLISHING, CONNECTION_REESTABLISHED, NORMAL, POWER_OUTAGE_SHORT, POWER_OUTAGE_LONG, POWER_LOW_DETECTED, POWER_LOW, POWER_BACK};

uint8_t netConnSM = STARTUP;
uint8_t gsmConnSM = STARTUP;
uint8_t powerSM = STARTUP;

void runNetConnSM();
void runGsmConnSM();
void runPowerSM();

unsigned long netConnLastMillis = 0;
unsigned long gsmConnLastMillis = 0;
unsigned long powerLastMillis = 0;

// -------------------------------------------------------------------------------
void sendSMS(const char * remoteNumber, const char * txtMsg);
void messageReceived(String &topic, String &payload);
bool connectMQTT();
bool checkBatteryLow();
void reboot();

// -------------------------------------------------------------------------------
void setup() {
  // initialize serial communications and wait for port to open:
#ifdef DEBUG
  Serial.begin(9600);
  DEBUG_PRINTLN("SMS Messages Sender");
#endif

  // connect timeout is 1 minute
  gsmAccess.setTimeout(60000);
}

// -------------------------------------------------------------------------------
void loop() {
  client.loop();

  runNetConnSM();
  runGsmConnSM();
  runPowerSM();

  delay(1000);
}

// -------------------------------------------------------------------------------
void runNetConnSM()
{
  switch (netConnSM)
  {
    case STARTUP:
      Ethernet.begin(mac, ip, subnet);

#ifdef DEBUG
      DEBUG_PRINT("IP: ");
      DEBUG_PRINTLN(Ethernet.localIP());

      if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        DEBUG_PRINTLN("Ethernet shield was not found.");
      }
      else if (Ethernet.hardwareStatus() == EthernetW5100) {
        DEBUG_PRINTLN("W5100 Ethernet controller detected.");
      }
      else if (Ethernet.hardwareStatus() == EthernetW5200) {
        DEBUG_PRINTLN("W5200 Ethernet controller detected.");
      }
      else if (Ethernet.hardwareStatus() == EthernetW5500) {
        DEBUG_PRINTLN("W5500 Ethernet controller detected.");
      }

      if (Ethernet.linkStatus() == Unknown) {
        DEBUG_PRINTLN("Link status unknown. Link status detection is only available with W5200 and W5500.");
      }
      else if (Ethernet.linkStatus() == LinkON) {
        DEBUG_PRINTLN("Link status: On");
      }
      else if (Ethernet.linkStatus() == LinkOFF) {
        DEBUG_PRINTLN("Link status: Off");
      }
#endif

      client.begin("10.11.30.134", net);
      client.onMessage(messageReceived);

      if (connectMQTT()) {
        client.publish("\\sms\\health", "good");
        netConnSM = CONNECTION_ESTABLISHED;
      }
      break;
    case CONNECTION_ESTABLISHED:
      if (!client.connected()) {
        DEBUG_PRINTLN("Lost connection to mqtt");
        netConnSM = CONNECTION_LOST;
        break;
      }

      // publish a message roughly every 30 second.
      if (millis() - netConnLastMillis > 30000) {
        DEBUG_PRINTLN("Publish a health message");
        netConnLastMillis = millis();
        client.publish("\\sms\\network\\health", "1");
      }
      break;
    case CONNECTION_LOST:
      DEBUG_PRINTLN("Send sms to inform about connection lost");
      sendSMS("phone", "Stoerung Kirche Wildberg.\nSMS Modul hat seit 5 Minuten keine interne Netzwerkverbindung.");
      netConnSM = CONNECTION_REESTABLISHING;
      break;
    case CONNECTION_REESTABLISHING:
      if (connectMQTT()) {
        DEBUG_PRINTLN("Connection to mqtt reestablished");
        netConnSM = CONNECTION_REESTABLISHED;
      }
      break;
    case CONNECTION_REESTABLISHED:
      if (!client.connected()) {
        DEBUG_PRINTLN("still not connected to mqtt");
        netConnSM = CONNECTION_REESTABLISHING;
      } else {
        DEBUG_PRINTLN("Send sms to inform about reestablished connection");
        sendSMS("phone", "Stoerung Kirche Wildberg.\nSMS Modul hat wieder interne Netzwerkverbindung.");
        netConnSM = CONNECTION_ESTABLISHED;
      }
      break;
  }
}

// -------------------------------------------------------------------------------
void runGsmConnSM()
{
  switch (gsmConnSM)
  {
    case STARTUP:
      if (gsmAccess.begin() == GSM_READY) {
        gsmConnSM = CONNECTION_ESTABLISHED;
        DEBUG_PRINTLN("GSM initialized.");
        client.publish("\\sms\\gsm\\health", "1");
      } else {
        DEBUG_PRINTLN("Not connected");
        client.publish("\\sms\\gsm\\health", "0");
      }
      break;
    case CONNECTION_ESTABLISHED:
      if (!gsmAccess.isAccessAlive()) {
        DEBUG_PRINTLN("Lost connection to gsm");
        gsmConnSM = CONNECTION_LOST;
      } else {
        // publish a message roughly every 60 second.
        if (millis() - gsmConnLastMillis > 60000) {
          DEBUG_PRINTLN("Publish a health message");
          gsmConnLastMillis = millis();
          client.publish("\\sms\\gsm\\health", "1");
        }
      }
      break;
    case CONNECTION_LOST:
      DEBUG_PRINTLN("publish to mqtt that gsm connection not working");
      client.publish("\\sms\\gsm\\health", "0");
      gsmConnSM = CONNECTION_REESTABLISHING;
      break;
    case CONNECTION_REESTABLISHING:
      if (gsmAccess.begin() == GSM_READY) {
        gsmConnSM = CONNECTION_REESTABLISHED;
        DEBUG_PRINTLN("GSM initialized.");
        client.publish("\\sms\\gsm\\health", "1");
      } else {
        DEBUG_PRINTLN("Not connected");
        client.publish("\\sms\\gsm\\health", "0");
      }
      break;
    case CONNECTION_REESTABLISHED:
      DEBUG_PRINTLN("Send sms to inform about reestablished connection");
      sendSMS("phone", "Stoerung Kirche Wildberg.\nSMS Modul hat wieder interne Netzwerkverbindung.");
      gsmConnSM = CONNECTION_ESTABLISHED;
      break;
  }
}

// -------------------------------------------------------------------------------
void runPowerSM()
{


  switch (powerSM)
  {
    case STARTUP:
      powerSM = NORMAL;
      break;
    case NORMAL:
      if (checkPowerOutage()) {
        powerSM = POWER_OUTAGE_SHORT;
        powerLastMillis = millis();
      }
      break;
    case POWER_OUTAGE_SHORT:
      DEBUG_PRINTLN("power outage short");
      if (millis() - powerLastMillis > 300000) {
        DEBUG_PRINTLN("Send sms to inform about long power outage");
        powerLastMillis = millis();
        sendSMS("phone", "Stoerung Kirche Wildberg.\nSMS Modul laeuft seit 5 Minuten auf Akkubetrieb.");
        powerSM = POWER_OUTAGE_LONG;
      }

      if (checkBatteryLow()) {
        powerSM = POWER_LOW_DETECTED;
      }

      if (checkPowerOutage() == false) {
        powerSM = POWER_BACK;
      }
      break;
    case POWER_OUTAGE_LONG:
      DEBUG_PRINTLN("power outage long");
      if (checkBatteryLow()) {
        powerSM = POWER_LOW_DETECTED;
      }

      if (checkPowerOutage() == false) {
        powerSM = POWER_BACK;
      }
      break;
    case POWER_LOW_DETECTED:
      DEBUG_PRINTLN("power low detected - sms to inform about it");
      sendSMS("phone", "Stoerung Kirche Wildberg.\nSMS Modul Akkustand tief.");
      powerSM = POWER_LOW_DETECTED;
      break;
    case POWER_LOW:
      DEBUG_PRINTLN("power low");
      if (checkPowerOutage() == false) {
        powerSM = POWER_BACK;
      }
      break;
    case POWER_BACK:
      DEBUG_PRINTLN("power back - send sms to inform about it");
      sendSMS("phone", "Stoerung Kirche Wildberg.\nSMS Modul nicht mehr im Akkubetrieb.");
      powerSM = NORMAL;
      powerLastMillis = millis();
      break;
  }
}

// -------------------------------------------------------------------------------
void sendSMS(const char * remoteNumber, const char * txtMsg) {
#ifdef DEBUG
  DEBUG_PRINTLN("SENDING");
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("Message:");
  DEBUG_PRINTLN(txtMsg);
#endif

  sms.beginSMS(remoteNumber);
  sms.print(txtMsg);
  sms.endSMS();

  client.publish("\\sms\\sent", remoteNumber);

  DEBUG_PRINTLN("\nCOMPLETE!\n");
}

void messageReceived(String &topic, String &payload) {
  DEBUG_PRINTLN("incoming: " + topic + " - " + payload);
  int index = payload.indexOf(';');
  String phone = payload.substring(0, index);
  String text = payload.substring(index + 1);
  DEBUG_PRINTLN("sms: " + phone + " - " + text);

  if (gsmConnSM == CONNECTION_ESTABLISHED) {
    sendSMS(phone.c_str(), text.c_str());
  }
}

bool connectMQTT() {
  DEBUG_PRINT("connecting to mqtt...");

  if (client.connect("client", "user", "pw")) {
    DEBUG_PRINTLN("\nconnected!");
    client.subscribe("\\sms\\send");
    return true;
  }
  else
  {
    DEBUG_PRINTLN("\nNOT connected!");
    return false;
  }
}

bool checkBatteryLow() {
  float batteryLevel = analogRead(ADC_BATTERY) * (3.3f / 1023.0f / 1.2f * (1.2f + 0.33f));

  //client.publish("\\sms\\battery", batteryLevel);
  DEBUG_PRINT(batteryLevel);
  DEBUG_PRINTLN("V");

  if (batteryLevel < 3.4) {
    return true;
  }
  return false;
}

bool checkPowerOutage() {
  float batteryLevel = analogRead(A1) * (2 * 3.3f / 1023.0f);

  //client.publish("\\sms\\voltage", batteryLevel);
  DEBUG_PRINT(batteryLevel);
  DEBUG_PRINTLN("V");

  if (batteryLevel < 4.5) {
    return true;
  }
  return false;
}

void reboot() {
  // Set up the generic clock (GCLK2) used to clock the watchdog timer at 1.024kHz
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(4) |            // Divide the 32.768kHz clock source by divisor 32, where 2^(4 + 1): 32.768kHz/32=1.024kHz
                    GCLK_GENDIV_ID(2);              // Select Generic Clock (GCLK) 2
  while (GCLK->STATUS.bit.SYNCBUSY);                // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_DIVSEL |          // Set to divide by 2^(GCLK_GENDIV_DIV(4) + 1)
                     GCLK_GENCTRL_IDC |             // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |           // Enable GCLK2
                     GCLK_GENCTRL_SRC_OSCULP32K |   // Set the clock source to the ultra low power oscillator (OSCULP32K)
                     GCLK_GENCTRL_ID(2);            // Select GCLK2
  while (GCLK->STATUS.bit.SYNCBUSY);                // Wait for synchronization

  // Feed GCLK2 to WDT (Watchdog Timer)
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |           // Enable GCLK2 to the WDT
                     GCLK_CLKCTRL_GEN_GCLK2 |       // Select GCLK2
                     GCLK_CLKCTRL_ID_WDT;           // Feed the GCLK2 to the WDT
  while (GCLK->STATUS.bit.SYNCBUSY);                // Wait for synchronization
}
