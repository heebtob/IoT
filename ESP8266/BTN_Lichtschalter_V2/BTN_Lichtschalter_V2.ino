#include <ESP8266WiFi.h> 
#include <WiFiUdp.h> 
#include <ESPDMX.h> 

#define short_get_high_byte(x) ((HIGH_BYTE & x) >> 8) 
#define short_get_low_byte(x)  (LOW_BYTE & x) 
#define bytes_to_short(h,l) ( ((h << 8) & 0xff00) | (l & 0x00FF) ); 

#ifndef STASSID 
#define STASSID "ssid" 
#define STAPSK  "pw" 
#endif 

// ----------------- UDP/ARTNET -------------------- 
WiFiUDP Udp; 
byte crestronIP[] {192, 168, 0, 200}; 
int crstronPort = 51515; 
int WLanStatus = WL_IDLE_STATUS; 
unsigned int localPort = 6454;      // local port to listen on 
void processUDP(int packetSize); 
char packetBuffer[UDP_TX_PACKET_MAX_SIZE + 1]; //buffer to hold incoming packet 

byte remoteIp[4];        // holds received packet's originating IP 
unsigned int remotePort; // holds received packet's originating port 

const int art_net_header_size = 17; 
const int max_packet_size = 576; 
char ArtNetHead[8] = "Art-Net"; 
char OpHbyteReceive = 0; 
char OpLbyteReceive = 0; 
short incoming_universe = 0; 
boolean is_opcode_is_dmx = 0; 
boolean is_opcode_is_artpoll = 0; 
boolean match_artnet = 1; 
short Opcode = 0; 

// button ------------------------------------------ 

byte buttonPin1 = 0; 
int lastButtonState1; 
int buttonState1; 
unsigned long lastDebounceTime1; 
unsigned long debounceDelay1 = 100; 
short buttonMap1[] = {0x0603, 0x0603}; 
byte toggleCount1 = 0; 

byte buttonPin2 = 4; 
int lastButtonState2; 
int buttonState2; 
unsigned long lastDebounceTime2; 
unsigned long debounceDelay2 = 100; 
short buttonMap2[] = {0x0604, 0x0604}; 
byte toggleCount2 = 0; 

// --------------------- DMX ----------------------- 

byte SubnetID = {0}; 
byte UniverseID = {0}; 
short select_universe = ((SubnetID * 16) + UniverseID); 

//customisation: edit this if you want for example read and copy only 4 or 6 channels from channel 12 or 48 or whatever. 
const int number_of_channels = 45; //512 for 512 channels 
const int start_address = 0; // 0 if you want to read from channel 1 

DMXESPSerial dmx; 

void setup() { 
  pinMode(buttonPin1, INPUT_PULLUP); 
  pinMode(buttonPin2, INPUT_PULLUP); 

  pinMode(LED_BUILTIN, OUTPUT); 
  digitalWrite(LED_BUILTIN, LOW); 

  //Serial.begin(9600); 
  //WiFi.config(IPAddress(10, 11, 120, 20), IPAddress(10, 11, 120, 1), IPAddress(255, 255, 255, 0)); 
  WiFi.mode(WIFI_STA); 
  WiFi.begin(STASSID, STAPSK); 
  while (WiFi.status() != WL_CONNECTED) { 
    //Serial.print('.'); 
    delay(500); 
  } 
  //Serial.print("Connected! IP address: "); 
  //Serial.println(WiFi.localIP()); 

  dmx.init(start_address + number_of_channels); 

  lastButtonState1 = HIGH; 
  buttonState1 = HIGH; 
  lastDebounceTime1 = 0; 

  lastButtonState2 = HIGH; 
  buttonState2 = HIGH; 
  lastDebounceTime2 = 0; 

  digitalWrite(LED_BUILTIN, HIGH); 
} 

void loop() { 
  if (WLanStatus != WL_CONNECTED && WiFi.status() == WL_CONNECTED) { 
    //Serial.println(); 
    //Serial.println(F("WiFi connected")); 

    //Serial.print("Connected! IP address: "); 
    //Serial.println(WiFi.localIP()); 
    //Serial.printf("UDP server on port %d\n", localPort); 
    Udp.begin(localPort); 
  } 
  WLanStatus = WiFi.status(); 

  processArtnet(Udp.parsePacket()); 

  // ---------------------------- Button 1 ------------------------------------- 

  int reading = digitalRead(buttonPin1); 

  //Serial.print("button 1 "); 
  //Serial.println(reading); 

  if (reading != lastButtonState1) { 
    // reset the debouncing timer 
    lastDebounceTime1 = millis(); 
  } 

  if ((millis() - lastDebounceTime1) > debounceDelay1) { 
    // whatever the reading is at, it's been there for longer than the debounce 
    // delay, so take it as the actual current state: 

    // if the button state has changed: 
    if (reading != buttonState1) { 
      buttonState1 = reading; 

      if (buttonState1 == HIGH) { 
        digitalWrite(LED_BUILTIN, LOW); 
        //Serial.println("button 1 high"); 
        char buffer [3]; 
        Udp.beginPacket(crestronIP, crstronPort); 
        Udp.write("BTN:"); 
        itoa(highByte(buttonMap1[toggleCount1]), buffer, 10); 
        Udp.write(buffer); 
        itoa(lowByte(buttonMap1[toggleCount1]), buffer, 10); 
        Udp.write(buffer); 
        Udp.write(";0\n"); 
        Udp.endPacket(); 
        delay(10); 
        digitalWrite(LED_BUILTIN, HIGH); 
      } 
      if (buttonState1 == LOW) { 
        digitalWrite(LED_BUILTIN, LOW); 
        //Serial.println("button 1 low"); 
        char buffer [3]; 
        Udp.beginPacket(crestronIP, crstronPort); 
        Udp.write("BTN:"); 
        itoa(highByte(buttonMap1[toggleCount1]), buffer, 10); 
        Udp.write(buffer); 
        itoa(lowByte(buttonMap1[toggleCount1]), buffer, 10); 
        Udp.write(buffer); 
        Udp.write(";1\n"); 
        Udp.endPacket(); 

        toggleCount1 = 1 - toggleCount1; 

        delay(10); 
        digitalWrite(LED_BUILTIN, HIGH); 
      } 
    } 
  } 

  lastButtonState1 = reading; 

  // ---------------------------- Button 2 ------------------------------------- 
  reading = digitalRead(buttonPin2); 

  //Serial.print("button 2 "); 
  //Serial.println(reading); 

  if (reading != lastButtonState2) { 
    // reset the debouncing timer 
    lastDebounceTime2 = millis(); 
  } 

  if ((millis() - lastDebounceTime2) > debounceDelay2) { 
    // whatever the reading is at, it's been there for longer than the debounce 
    // delay, so take it as the actual current state: 

    // if the button state has changed: 
    if (reading != buttonState2) { 
      buttonState2 = reading; 

      if (buttonState2 == HIGH) { 
        digitalWrite(LED_BUILTIN, LOW); 
        //Serial.println("button 2 high"); 
        char buffer [3]; 
        Udp.beginPacket(crestronIP, crstronPort); 
        Udp.write("BTN:"); 
        itoa(highByte(buttonMap2[toggleCount2]), buffer, 10); 
        Udp.write(buffer); 
        itoa(lowByte(buttonMap2[toggleCount2]), buffer, 10); 
        Udp.write(buffer); 
        Udp.write(";0\n"); 
        Udp.endPacket(); 
        delay(10); 
        digitalWrite(LED_BUILTIN, HIGH); 
      } 
      if (buttonState2 == LOW) { 
        digitalWrite(LED_BUILTIN, LOW); 
        //Serial.println("button 2 low"); 
        char buffer [3]; 
        Udp.beginPacket(crestronIP, crstronPort); 
        Udp.write("BTN:"); 
        itoa(highByte(buttonMap2[toggleCount2]), buffer, 10); 
        Udp.write(buffer); 
        itoa(lowByte(buttonMap2[toggleCount2]), buffer, 10); 
        Udp.write(buffer); 
        Udp.write(";1\n"); 
        Udp.endPacket(); 

        toggleCount2 = 1 - toggleCount2; 

        delay(10); 
        digitalWrite(LED_BUILTIN, HIGH); 
      } 
    } 
  } 

  lastButtonState2 = reading; 
} 

void processArtnet(int packetSize) { 
  if (packetSize > 0) { 
    //Debug.print("processArtnet: "); 
    //Debug.println(packetSize); 
  } 

  //check size to avoid unneeded checks 
  if (packetSize > art_net_header_size && packetSize <= max_packet_size) { 
    IPAddress remote = Udp.remoteIP(); 
    remotePort = Udp.remotePort(); 
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE); 

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
      //Debug.println("ARTNET packet received"); 
      
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
          for (int i = start_address; i < start_address + number_of_channels; i++) { 
            dmx.write(i+1, byte(packetBuffer[i + art_net_header_size + 1])); 
            //Serial.print("channel:"); 
            //Serial.print(i+1); 
            //Serial.print(" value:"); 
            //Serial.println(byte(packetBuffer[i + art_net_header_size + 1])); 
          } 

          dmx.update(); 
        } 
      } 
    }//end of sniffing 
  } 
} 
