#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <Wire.h>
#include "SparkFun_SCD30_Arduino_Library.h"
#include <Adafruit_SleepyDog.h>

#define PWR_DOWN
//#define USE_LED
//#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x)  Serial.print (x)
#define DEBUG_PRINTLN(x)  Serial.println (x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

// *** RFM Configuration ***
#define CLIENT_ADDRESS 3
#define SERVER_ADDRESS 98

#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define RF69_FREQ 434.0

// The sync words and the encryption key has to be the same as the one in the server
const uint8_t syncWords[] = {0x00, 0x00};
uint8_t key[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                };

// RFM RF payload datastructure
typedef struct {
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

// Singleton instance of the radio driver
RH_RF69 driver(RFM69_CS, RFM69_INT); // Adafruit Feather M0
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

#define VBATPIN A7
SCD30 airSensor;
int sleptMS;
const int sleepMS = 300000;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
#ifdef USE_LED
  digitalWrite(LED_BUILTIN, HIGH); // Show we're awake
#else
  digitalWrite(LED_BUILTIN, LOW); // Show we're awake
#endif
  sleptMS = 0;

#ifdef DEBUG
  Serial.begin(9600);
  while (!Serial)
    ;
#endif

  emonth.type = 4;

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  DEBUG_PRINTLN("Feather RFM69 RX Test!");

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  DEBUG_PRINTLN("init rf");

  if (!manager.init())
    DEBUG_PRINTLN("init failed");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)

  DEBUG_PRINTLN("configure rf");

  driver.setSyncWords(syncWords, 2);
  driver.setEncryptionKey(key);

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  driver.setTxPower(20, true);

  driver.setIdleMode(RH_RF69_OPMODE_MODE_SLEEP);
  if (driver.sleep()) {
    DEBUG_PRINTLN("rf in sleep mode");
  } else {
    DEBUG_PRINTLN("rf could not go to sleep mode");
  }

  DEBUG_PRINTLN("setup finished");

  Wire.begin();

  if (airSensor.begin() == false)
  {
    DEBUG_PRINTLN("Air sensor not detected. Please check wiring. Freezing...");
    while (1)
      ;
  }

  airSensor.setMeasurementInterval(300); //Change number of seconds between measurements: 2 to 1800 (30 minutes)

  //My desk is ~1600m above sealevel
  airSensor.setAltitudeCompensation(650); //Set altitude of the sensor in m

  //Pressure in Boulder, CO is 24.65inHg or 834.74mBar
  airSensor.setAmbientPressure(937); //Current ambient pressure in mBar: 700 to 1200

  float offset = airSensor.getTemperatureOffset();
  DEBUG_PRINT("Current temp offset: ");
  DEBUG_PRINT(offset);
  DEBUG_PRINTLN("C");

  //airSensor.setTemperatureOffset(5); //Optionally we can set temperature offset to 5°C

#ifndef DEBUG
  delay(15000);
#endif  
}

void loop()
{
  float measuredvbat = analogRead(VBATPIN);
  /*measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 102.4; // convert to voltage*/
  measuredvbat *= 0.064453125;
  emonth.battery = (int)measuredvbat;
  DEBUG_PRINT("VBat: " );
  DEBUG_PRINTLN(measuredvbat);

  int count = 0;
  while (count<5)
  {
    if (airSensor.dataAvailable()) {
      emonth.ext1 = airSensor.getCO2();
      DEBUG_PRINT("co2(ppm):");
      DEBUG_PRINT(emonth.ext1);
      
      emonth.temp = (int)(airSensor.getTemperature() * 10);
      DEBUG_PRINT(" temp(C):");
      DEBUG_PRINT(emonth.temp);
  
      emonth.humidity = (int)(airSensor.getHumidity() * 10);
      DEBUG_PRINT(" humidity(%):");
      DEBUG_PRINT(emonth.humidity);
  
      DEBUG_PRINTLN();
  
      if (emonth.ext1 > 0)
        break;
    }
    
    count++;
    delay(1000);
  }

  DEBUG_PRINTLN("send rf data");
  uint8_t len = sizeof(buf);
  memcpy (buf, &emonth, len);
  if (manager.sendtoWait(buf, len, SERVER_ADDRESS))
  {
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAckTimeout(buf, &len, 2000, &from))
    {
#ifdef DEBUG
      Serial.print("got reply from : 0x");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);
#endif
    }
    else
    {
      DEBUG_PRINTLN("No reply, is rf69_reliable_datagram_server running?");
    }
  }
  else
  {
    DEBUG_PRINTLN("sendtoWait failed");
  }
  DEBUG_PRINTLN("rf data sent");

  if (driver.sleep()) {
    DEBUG_PRINTLN("rf in sleep mode");
  } else {
    DEBUG_PRINTLN("rf could not go to sleep mode");
  }

#ifdef DEBUG
  Serial.println("going to sleep mode");
  Serial.flush();
#endif

#ifdef USE_LED
  digitalWrite(LED_BUILTIN, LOW); // Show we're asleep
#endif

#ifdef PWR_DOWN
  sleptMS = 0;
  int leftToSleepMS = sleepMS - sleptMS;
  while (leftToSleepMS > 100) {
    sleptMS += Watchdog.sleep(leftToSleepMS);
    leftToSleepMS = sleepMS - sleptMS;
  }
#else
  delay(sleepMS);
#endif

#ifdef USE_LED
  digitalWrite(LED_BUILTIN, HIGH); // Show we're awake again
#endif

#ifdef DEBUG
  Serial.print("I'm awake now! I slept for ");
  Serial.print(sleepMS, DEC);
  Serial.println(" milliseconds.");
  Serial.println();
#endif

}
