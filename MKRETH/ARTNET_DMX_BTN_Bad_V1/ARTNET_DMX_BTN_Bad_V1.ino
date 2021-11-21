
#include <DMXSerial.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008
#define short_get_high_byte(x) ((HIGH_BYTE & x) >> 8)
#define short_get_low_byte(x)  (LOW_BYTE & x)
#define bytes_to_short(h,l) ( ((h << 8) & 0xff00) | (l & 0x00FF) );
#define BTN_IO_COUNT 0

byte mac[] = {0x00, 0x30, 0xA3, 0x0D, 0x3C, 0x5D} ; //the mac adress in HEX of ethernet shield or uno shield board
byte ip[] = {10, 11, 120, 214}; // the IP adress of your device, that should be in same universe of the network you are using
byte subnet[] = {255, 255, 255, 0};
byte crestronIP[] {10, 11, 120, 200};

// the next two variables are set when a packet is received
byte remoteIp[4];        // holds received packet's originating IP
unsigned int remotePort; // holds received packet's originating port

//customisation: Artnet SubnetID + UniverseID
//edit this with SubnetID + UniverseID you want to receive
byte SubnetID = {0};
byte UniverseID = {0};
short select_universe = ((SubnetID * 16) + UniverseID);

//customisation: edit this if you want for example read and copy only 4 or 6 channels from channel 12 or 48 or whatever.
const int number_of_channels = 20; //512 for 512 channels
const int start_address = 21; // 0 if you want to read from channel 1

//buffers
const int MAX_BUFFER_UDP = 576;
char packetBuffer[MAX_BUFFER_UDP]; //buffer to store incoming data
// byte buffer_channel_arduino[number_of_channels]; //buffer to store filetered DMX data

// art net parameters
unsigned int localPort = 6454;      // artnet UDP port is by default 6454
const int art_net_header_size = 17;
const int max_packet_size = 576;
char ArtNetHead[8] = "Art-Net";
char OpHbyteReceive = 0;
char OpLbyteReceive = 0;
//short is_artnet_version_1=0;
//short is_artnet_version_2=0;
//short seq_artnet=0;
//short artnet_physical=0;
short incoming_universe = 0;
boolean is_opcode_is_dmx = 0;
boolean is_opcode_is_artpoll = 0;
boolean match_artnet = 1;
short Opcode = 0;
EthernetUDP Udp;

// Variables will change:
int lastButtonState[BTN_IO_COUNT];   // the previous reading from the input pin
int buttonState[BTN_IO_COUNT]; // the current reading from the input pin
short buttonMap[] = {0x0101, 0x0102, 0x0103, 0x0104, 0x0105, 0x0106, 0x0301, 0x0201, 0x0202, 0x0203, 0x0204};
byte buttonInputMap[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13};

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime[BTN_IO_COUNT]; // the last time the output pin was toggled
unsigned long debounceDelay = 100;    // the debounce time; increase if the output flickers

unsigned long prevMillis = 0;

//relais module
bool relaisModule = true;
byte relaisPins[] = {8, 12};
int relaisDmxChannels[] = {35, 36};
byte relaisCount = 2;

void setup() {

  for (int i = 0; i < BTN_IO_COUNT; i++)
  {
    lastButtonState[i] = HIGH;
    buttonState[i] = HIGH;
    lastDebounceTime[i] = 0;
    pinMode(buttonInputMap[i], INPUT_PULLUP);
  }

  if (relaisModule)
  {
    for (int i = 0; i < relaisCount; i++)
    {
      pinMode(relaisPins[i], OUTPUT);
    }
  }

  DMXSerial.init(DMXController);

  //setup ethernet and udp socket
  Ethernet.begin(mac, ip, subnet);
  Udp.begin(localPort);

  delay(1000);
  Udp.beginPacket(crestronIP, 51515);
  Udp.write("BTN:40;3\n");
  Udp.endPacket();
}

void loop() {
  int packetSize = Udp.parsePacket();

  long nowmillis = millis();
  if ((nowmillis - prevMillis) > 20000) {
    Udp.beginPacket(crestronIP, 51515);
    Udp.write("BTN:40;2\n");
    Udp.endPacket();
    prevMillis = nowmillis;
  }

  for (int i = 0; i < BTN_IO_COUNT; i++)
  {
    // read the state of the switch into a local variable:
    int reading = digitalRead(buttonInputMap[i]);

    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH), and you've waited long enough
    // since the last press to ignore any noise:

    // If the switch changed, due to noise or pressing:
    if (reading != lastButtonState[i]) {
      // reset the debouncing timer
      lastDebounceTime[i] = millis();
    }

    if ((millis() - lastDebounceTime[i]) > debounceDelay) {
      // whatever the reading is at, it's been there for longer than the debounce
      // delay, so take it as the actual current state:

      // if the button state has changed:
      if (reading != buttonState[i]) {
        buttonState[i] = reading;

        if (buttonState[i] == HIGH) {
          char buffer [3];
          Udp.beginPacket(crestronIP, 51515);
          Udp.write("BTN:");
          itoa(highByte(buttonMap[i]), buffer, 10);
          Udp.write(buffer);
          itoa(lowByte(buttonMap[i]), buffer, 10);
          Udp.write(buffer);
          Udp.write(";0\n");
          Udp.endPacket();
        }
        if (buttonState[i] == LOW) {
          char buffer [3];
          Udp.beginPacket(crestronIP, 51515);
          Udp.write("BTN:");
          itoa(highByte(buttonMap[i]), buffer, 10);
          Udp.write(buffer);
          itoa(lowByte(buttonMap[i]), buffer, 10);
          Udp.write(buffer);
          Udp.write(";1\n");
          Udp.endPacket();
        }
      }
    }

    // save the reading. Next time through the loop, it'll be the lastButtonState:
    lastButtonState[i] = reading;
  }

  //FIXME: test/debug check
  if (packetSize > art_net_header_size && packetSize <= max_packet_size) { //check size to avoid unneeded checks
    //if(packetSize) {

    IPAddress remote = Udp.remoteIP();
    remotePort = Udp.remotePort();
    Udp.read(packetBuffer, MAX_BUFFER_UDP);

    //read header
    match_artnet = 1;
    for (int i = 0; i < 7; i++) {
      //if not corresponding, this is not an artnet packet, so we stop reading
      if (char(packetBuffer[i]) != ArtNetHead[i]) {
        match_artnet = 0; break;
      }
    }

    //if its an artnet header
    if (match_artnet == 1) {
      //artnet protocole revision, not really needed
      //is_artnet_version_1=packetBuffer[10];
      //is_artnet_version_2=packetBuffer[11];*/

      //sequence of data, to avoid lost packets on routeurs
      //seq_artnet=packetBuffer[12];*/

      //physical port of  dmx N°
      //artnet_physical=packetBuffer[13];*/

      //operator code enables to know wich type of message Art-Net it is
      Opcode = bytes_to_short(packetBuffer[9], packetBuffer[8]);

      //if opcode is DMX type
      if (Opcode == 0x5000) {
        is_opcode_is_dmx = 1; is_opcode_is_artpoll = 0;
      }

      //if opcode is artpoll
      else if (Opcode == 0x2000) {
        is_opcode_is_artpoll = 1; is_opcode_is_dmx = 0;
        //( we should normally reply to it, giving ip adress of the device)
      }

      //if its DMX data we will read it now
      if (is_opcode_is_dmx = 1) {

        //read incoming universe
        incoming_universe = bytes_to_short(packetBuffer[15], packetBuffer[14])
                            //if it is selected universe DMX will be read
        if (incoming_universe == select_universe) {

          //getting data from a channel position, on a precise amount of channels, this to avoid to much operation if you need only 4 channels for example
          //channel position
          for (int i = start_address; i < start_address+number_of_channels; i++) {
            //buffer_channel_arduino[start_address+i]= byte(packetBuffer[i+art_net_header_size+1]);
            //DMXSerial.write(start_address+i, buffer_channel_arduino[start_address+i]);
            DMXSerial.write(i, byte(packetBuffer[i + art_net_header_size + 1]));
          }

          if (relaisModule)
          {
            for (int i = 0; i < relaisCount; i++)
            {
              if (byte(packetBuffer[relaisDmxChannels[i] + art_net_header_size + 1]) > 245)
              {  
                digitalWrite(relaisPins[i], HIGH);
              }
              else if (byte(packetBuffer[relaisDmxChannels[i] + art_net_header_size + 1]) < 10)
              {
                digitalWrite(relaisPins[i], LOW);
              }
            }
          }
        }
      }
    }//end of sniffing
  }
}
