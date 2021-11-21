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
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>

#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <SPI.h>

#define debug false                                                   // Set to 1 to few debug serial output
#define flash_led false                                               // Flash LED after each sample (battery drain) default=0

// *** RFM Configuration ***
#define CLIENT_ADDRESS 8
#define SERVER_ADDRESS 99

// The sync words and the encryption key has to be the same as the one in the server
const int8_t syncWords[] = {0x00, 0x00};
uint8_t key[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                };

#define RFM69_CS      10
#define RFM69_INT     2
// Singleton instance of the radio driver
RH_RF69 driver(RFM69_CS, RFM69_INT); // Adafruit Feather M0
// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

// SI7021_status SPI temperature & humidity sensor
#include <Wire.h>
#include <SI7021.h>
SI7021 SI7021_sensor;
boolean SI7021_status;

// Hardwired emonTH pin allocations
#define DS18B20_PWR     5
#define LED             9
#define BATT_ADC        1
#define DIP_switch1     7
#define DIP_switch2     8
#define pulse_countINT  1                                     // INT 1 / Dig 3 Screw Terminal Block Number 4
#define pulse_count_pin 3                                     // INT 1 / Dig 3 Screw Terminal Block Number 4
#define ONE_WIRE_BUS    17
#define DHT22_PWR       6                                     // Not used in emonTH V2.0, 10K resistor R1 connects DHT22 pins
#define DHT22_DATA      16                                    // Not used in emonTH V2.0, 10K resistor R1 connects DHT22 pins.

// Note: Please update emonhub configuration guide on OEM wide packet structure change:
// https://github.com/openenergymonitor/emonhub/blob/emon-pi/configuration.md
typedef struct {                                                      // RFM RF payload datastructure
  uint16_t temp;
  uint16_t humidity;
  uint16_t battery;
} Payload;
Payload emonth;

// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

// *** Sleep mode ***
#define wakeupSleepCnt 6
volatile byte sleepCnt = 0;
byte prevADCSRA;

// *** Watchdog interrupt
ISR (WDT_vect) {
  wdt_disable();
  sleepCnt++;
}

//################################################################################################################################
//################################################################################################################################

void setup() {
  //################################################################################################################################

  pinMode(LED, OUTPUT); digitalWrite(LED, HIGH);                     // Status LED on

  // Unused pins configure as input pull up for low power
  // http://electronics.stackexchange.com/questions/43460/how-should-unused-i-o-pins-be-configured-on-atmega328p-for-lowest-power-consumpt
  // port map: https://github.com/openenergymonitor/emonth2/blob/master/hardware/readme.md
  pinMode(DHT22_PWR, INPUT_PULLUP);                                  //DHT22 not used on emonTH V2.
  pinMode(DHT22_DATA, INPUT_PULLUP);                                 //DHT22 not used on emonTH V2
  pinMode(14, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);

  //READ DIP SWITCH POSITIONS - LOW when switched on (default off - pulled up high)
  pinMode(DIP_switch1, INPUT_PULLUP);
  pinMode(DIP_switch2, INPUT_PULLUP);
  //boolean DIP1 = digitalRead(DIP_switch1);
  //boolean DIP2 = digitalRead(DIP_switch2);


  if (debug)
  {
    //Serial.begin(115200);
    Serial.begin(9600);
    Serial.println("emonTH");
    delay(100);
  }

  if (debug) Serial.println("Int RFM...");
  if (!manager.init())
    Serial.println("init failed");

  driver.setSyncWords(syncWords, 2);
  driver.setEncryptionKey(key);

  driver.setIdleMode(RH_RF69_OPMODE_MODE_SLEEP);
  if (driver.sleep()) {
    if (debug) Serial.println("rf in sleep mode");
  } else {
    if (debug) Serial.println("rf could not go to sleep mode");
  }

  // rf12_sleep(RF12_SLEEP);

  pinMode(DS18B20_PWR, INPUT_PULLUP);
  pinMode(BATT_ADC, INPUT);
  pinMode(pulse_count_pin, INPUT_PULLUP);

  //################################################################################################################################
  // Setup and for presence of si7021
  //################################################################################################################################
  if (debug) Serial.println("Int SI7021..");

  // check if the I2C lines are HIGH
  if (digitalRead(SDA) == HIGH || digitalRead(SCL) == HIGH)
  {
    SI7021_sensor.begin();
    int deviceid = SI7021_sensor.getDeviceId();
    if (deviceid != 0) {
      SI7021_status = 1;
      if (debug) {
        si7021_env data = SI7021_sensor.getHumidityAndTemperature();
        Serial.print("SI7021 Started, ID: ");
        Serial.println(deviceid);
        Serial.print("SI7021 t: "); Serial.println(data.celsiusHundredths / 100.0);
        Serial.print("SI7021 h: "); Serial.println(data.humidityBasisPoints / 100.0);
      }
    }
    else {
      SI7021_status = 0;
      if (debug) Serial.println("SI7021 Error");
    }
  }
  else {
    SI7021_status = 0;
    if (debug) Serial.println("SI7021 Error");
  }


  //################################################################################################################################
  // Interrupt pulse counting setup
  //################################################################################################################################
  /*emonth.pulsecount = 0;
    pulseCount = 0;
    WDT_number=720;
    p = 0;*/
  //attachInterrupt(pulse_countINT, onPulse, RISING);

  //################################################################################################################################
  // Power Save  - turn off what we don't need - http://www.nongnu.org/avr-libc/user-manual/group__avr__power.html
  //################################################################################################################################
  ACSR |= (1 << ACD);                     // disable Analog comparator
  if (!debug) power_usart0_disable();      //disable serial UART
  power_twi_disable();                    //Two Wire Interface module:
  power_spi_disable();
  power_timer1_disable();
  //power_timer0_disable();              //don't disable necessary for the DS18B20 library
  prevADCSRA = ADCSRA;
  ADCSRA = 0;

  // Only turn off LED if both sensor and RF69CW are working
  if (SI7021_status) {
    digitalWrite(LED, LOW);                 // turn off LED to indciate end setup
  }

  Serial.flush();
} // end of setup


//################################################################################################################################
//################################################################################################################################
void loop()
//################################################################################################################################
{
  if (debug) {
    Serial.println("enter sleep mode");
    Serial.flush();
  }

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  while (sleepCnt <= wakeupSleepCnt) {
    sleep_bod_disable();
    noInterrupts();
    MCUSR = 0;
    WDTCSR = bit (WDCE) | bit (WDE);
    WDTCSR = bit (WDIE) | bit (WDP0) | bit (WDP3);
    wdt_reset();

    interrupts();

    sleep_cpu();
  }

  sleep_disable();

  sleepCnt = 0;

  if (debug) Serial.println("wake up after sleep mode");

  ADCSRA = prevADCSRA;
  emonth.battery = int(analogRead(BATT_ADC) * 0.0322);                //read battery voltage, convert ADC to volts x10
  ADCSRA = 0;

  if (debug) {
    Serial.print("bat:");
    Serial.println(emonth.battery);
  }

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
    power_twi_enable();
    si7021_env data = SI7021_sensor.getHumidityAndTemperature();
    emonth.temp = (data.celsiusHundredths * 0.1);
    emonth.humidity = (data.humidityBasisPoints * 0.1);
    power_twi_disable();

    if (debug) {
      Serial.print("temp:");
      Serial.println(emonth.temp);
      Serial.print("humidity:");
      Serial.println(emonth.humidity);
    }
  }


  // Send data via RF
  if (debug) Serial.println("send rf data");
  power_spi_enable();
  uint8_t len = sizeof(buf);
  memcpy (buf, &emonth, len);
  if (manager.sendtoWait(buf, len, SERVER_ADDRESS))
  {
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAckTimeout(buf, &len, 2000, &from))
    {
      if (debug) {
        Serial.print("got reply from : 0x");
        Serial.print(from, HEX);
        Serial.print(": ");
        Serial.println((char*)buf);
      }
    }
    else
    {
      if (debug) Serial.println("No reply, is rf69_reliable_datagram_server running?");
    }
  }
  else
  {
    if (debug) Serial.println("sendtoWait failed");
  }
  if (debug) Serial.println("rf data sent");
  if (driver.sleep()) {
    if (debug) Serial.println("rf in sleep mode");
  } else {
    if (debug) Serial.println("rf could not go to sleep mode");
  }
  power_spi_disable();

  if (flash_led) {
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
  }

} // end loop
