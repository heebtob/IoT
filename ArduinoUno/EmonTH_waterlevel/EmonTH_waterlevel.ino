/*
  emonTH V2 Low Power SI7021 Humidity & Temperature, DS18B20 & Pulse counting Node Example

  Si7021 = internal temperature & Humidity

  -----------------------------------------------------------------------------------------------------------
  -ID-  -Node Type-
  0 - Special allocation in JeeLib RFM12 driver - reserved for OOK use
  1-4     - Control nodes
  5-10  - Energy monitoring nodes
  11-14 --Un-assigned --
  15-16 - Base Station & logging nodes
  17-30 - Environmental sensing nodes (temperature humidity etc.)
  31  - Special allocation in JeeLib RFM12 driver - Node31 can communicate with nodes on any network group
  -------------------------------------------------------------------------------------------------------------
*/

#include "Arduino.h"

#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <SPI.h>

#include "Adafruit_SGP30_Software.h"

//#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x)  Serial.print (x)
#define DEBUG_PRINTLN(x)  Serial.println (x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#define flash_led false                                               // Flash LED after each sample (battery drain) default=0

// *** RFM Configuration ***
#define CLIENT_ADDRESS 30
#define SERVER_ADDRESS 99

// The sync words and the encryption key has to be the same as the one in the server
const int8_t syncWords[] = {0x00, 0x00};
uint8_t key[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                };

#define RFM69_CS      10
#define RFM69_INT      2
// Singleton instance of the radio driver
RH_RF69 driver(RFM69_CS, RFM69_INT); // Adafruit Feather M0
// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

// SI7021_status SPI temperature & humidity sensor
#include <Wire.h>
#include <SI7021.h>
SI7021 SI7021_sensor;
boolean SI7021_status;

// Ultra Sonic Distance Sensor
#define distance_sensor true
long duration;

// Hardwired emonTH pin allocations
//#define DS18B20_PWR     5
#define LED             9
#define BATT_ADC        1
#define DIP_switch1     7
#define DIP_switch2     8
#define pulse_countINT  1                                     // INT 1 / Dig 3 Screw Terminal Block Number 4
//#define pulse_count_pin 3                                     // INT 1 / Dig 3 Screw Terminal Block Number 4
#define ONE_WIRE_BUS    17
#define EXT_POWER_DOWN 6                                     // enable pin on the nano power switch
#define DHT22_DATA      16                                    // Not used in emonTH V2.0, 10K resistor R1 connects DHT22 pins.
// Define Trig and Echo pin:
#define trigPin 5
#define echoPin 3

// Note: Please update emonhub configuration guide on OEM wide packet structure change:
// https://github.com/openenergymonitor/emonhub/blob/emon-pi/configuration.md
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

// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

//################################################################################################################################
//################################################################################################################################

void setup() {
  //################################################################################################################################
  pinMode(EXT_POWER_DOWN, OUTPUT);
  digitalWrite(EXT_POWER_DOWN, LOW);

  // emonth with ultra sonic distance sensor
  emonth.type = 2; // normal emonth 1 / distance sensor 2 / gas sensor 3
  emonth.ext2 = 0;
  emonth.ext3 = 0;
  emonth.ext4 = 0;
  emonth.ext5 = 0;

  pinMode(LED, INPUT_PULLUP);

  // Unused pins configure as input pull up for low power
  // http://electronics.stackexchange.com/questions/43460/how-should-unused-i-o-pins-be-configured-on-atmega328p-for-lowest-power-consumpt
  // port map: https://github.com/openenergymonitor/emonth2/blob/master/hardware/readme.md
  pinMode(DHT22_DATA, INPUT_PULLUP);                                 //DHT22 not used on emonTH V2
  pinMode(14, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //READ DIP SWITCH POSITIONS - LOW when switched on (default off - pulled up high)
  pinMode(DIP_switch1, INPUT_PULLUP);
  pinMode(DIP_switch2, INPUT_PULLUP);
  //boolean DIP1 = digitalRead(DIP_switch1);
  //boolean DIP2 = digitalRead(DIP_switch2);

#ifdef DEBUG
  //Serial.begin(115200);
  Serial.begin(9600);
  Serial.println("emonTH");
  delay(100);
#endif

  DEBUG_PRINT("Int RFM...");
  if (!manager.init())
    DEBUG_PRINTLN("init failed");

  driver.setSyncWords(syncWords, 2);
  driver.setEncryptionKey(key);

  driver.setIdleMode(RH_RF69_OPMODE_MODE_SLEEP);
  if (driver.sleep())
    DEBUG_PRINTLN("rf in sleep mode");
  else
    DEBUG_PRINTLN("rf could not go to sleep mode");

  pinMode(BATT_ADC, INPUT);

  //################################################################################################################################
  // Setup and for presence of si7021
  //################################################################################################################################
  DEBUG_PRINTLN("Int SI7021..");

  // check if the I2C lines are HIGH
  if (digitalRead(SDA) == HIGH || digitalRead(SCL) == HIGH)
  {
    SI7021_sensor.begin();
    int deviceid = SI7021_sensor.getDeviceId();
    if (deviceid != 0) {
      SI7021_status = 1;
#ifdef DEBUG
      si7021_env data = SI7021_sensor.getHumidityAndTemperature();
      DEBUG_PRINT("SI7021 Started, ID: ");
      DEBUG_PRINTLN(deviceid);
      DEBUG_PRINT("SI7021 t: "); DEBUG_PRINTLN(data.celsiusHundredths / 100.0);
      DEBUG_PRINT("SI7021 h: "); DEBUG_PRINTLN(data.humidityBasisPoints / 100.0);
#endif
    } else {
      SI7021_status = 0;
      DEBUG_PRINTLN("SI7021 Error");
    }
  } else {
    SI7021_status = 0;
    DEBUG_PRINTLN("SI7021 Error");
  }

  if (distance_sensor) {
    digitalWrite(trigPin, HIGH);
  }

#ifdef DEBUG
  Serial.flush();
#endif
} // end of setup


//################################################################################################################################
//################################################################################################################################
void loop()
//################################################################################################################################
{

  emonth.battery = int(analogRead(BATT_ADC) * 0.0322);                //read battery voltage, convert ADC to volts x10
  DEBUG_PRINT("bat:");
  DEBUG_PRINTLN(emonth.battery);

  //Enhanced battery monitoring mode. In this mode battery values
  //sent in x*1000 mode instead of x*10. This allows to have more accurate
  //values on emonCMS x.xx instead of x.x
  // NOTE if you are going to enable this mode you need to
  // 1. Disable x*10 mode. By commenting line above.
  // 2. Change multiplier in line 353 Serial.print(emonth.battery/10.0);
  // 3. Change scales factor in the emonhub node decoder entry for the emonTH
  // See more https://community.openenergymonitor.org/t/emonth-battery-measurement-accuracy/1317
  //emonth.battery=int(analogRead(BATT_ADC)*3.222);

  // Read SI7021
  // Read from SI7021 SPI temp & humidity sensor
  if (SI7021_status == 1) {
    si7021_env data = SI7021_sensor.getHumidityAndTemperature();
    emonth.temp = (data.celsiusHundredths * 0.1);
    emonth.humidity = (data.humidityBasisPoints * 0.1);
    DEBUG_PRINT("temp:");
    DEBUG_PRINTLN(emonth.temp);
    DEBUG_PRINT("humidity:");
    DEBUG_PRINTLN(emonth.humidity);
  }

  if (distance_sensor) {
    int measureCount = 0;
    int measureSum = 0;

    for (int i = 0; i < 3; i++) {
      // Clear the trigPin by setting it LOW:
      digitalWrite(trigPin, LOW);

      delayMicroseconds(5);
      // Trigger the sensor by setting the trigPin high for 10 microseconds:
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);

      // Read the echoPin. pulseIn() returns the duration (length of the pulse) in microseconds:
      duration = pulseIn(echoPin, HIGH);

      // Calculate the distance:
      int distance = (int)(duration * (double)0.0343 / 2.0);
      if (distance > 0) {
        measureSum += distance;
        measureCount++;
      }

      // Print the distance on the Serial Monitor (Ctrl+Shift+M):
      DEBUG_PRINT("measured Distance = ");
      DEBUG_PRINT(distance);
      DEBUG_PRINTLN(" cm");

      delay(50);
    }

    emonth.ext1 = measureSum / measureCount;
    // Print the distance on the Serial Monitor (Ctrl+Shift+M):
    DEBUG_PRINT("calculated Distance = ");
    DEBUG_PRINT(emonth.ext1);
    DEBUG_PRINTLN(" cm");

    digitalWrite(trigPin, HIGH);
  }

  // Send data via RF
  DEBUG_PRINTLN("send rf data");
  uint8_t len = sizeof(buf);
  memcpy (buf, &emonth, len);
  for (int i = 0; i < 3; i++) {
    if (manager.sendtoWait(buf, len, SERVER_ADDRESS))
    {
      // Now wait for a reply from the server
      uint8_t len = sizeof(buf);
      uint8_t from;
      if (manager.recvfromAckTimeout(buf, &len, 2000, &from))
      {
#ifdef DEBUG
        DEBUG_PRINT("got reply from : 0x");
        Serial.print(from, HEX);
        DEBUG_PRINT(": ");
        DEBUG_PRINTLN((char*)buf);
#endif
        break;
      }
      else
        DEBUG_PRINTLN("No reply, is rf69_reliable_datagram_server running?");
    }
    else
      DEBUG_PRINTLN("sendtoWait failed");

    // wait for next try
    delay(50);
  }

  digitalWrite(EXT_POWER_DOWN, LOW);
  digitalWrite(EXT_POWER_DOWN, HIGH);
  delay(60000);

} // end loop
