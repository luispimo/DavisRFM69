// Sample usage of the DavisRFM69 library to sniff the packets from a Davis Instruments
// wireless Integrated Sensor Suite (ISS).
//
// This is part of the DavisRFM69 library from https://github.com/dekay/DavisRFM69
// (C) DeKay 2014-2016 dekaymail@gmail.com
// Example released under the MIT License (http://opensource.org/licenses/mit-license.php)
// Get the RFM69 and SPIFlash library at: https://github.com/LowPowerLab/
//
// This program has been adapted by Luis Piñuel to work on an Adafruit Feather ESP32 V2 board 
// with an attached RFM69W Feather Wing connected (see DavisRFM69.cpp for connection details).
//


#include <DavisRFM69.h>
#include <SPI.h>

// NOTE: *** One of DAVIS_FREQS_US, DAVIS_FREQS_EU, DAVIS_FREQS_AU, or
// DAVIS_FREQS_NZ MUST be defined at the top of DavisRFM69.h ***

#define IS_RFM69HW              //uncomment only for RFM69HW!
#define SERIAL_BAUD     115200
#define PACKET_INTERVAL 2563    // measured for ID=0
#define POWER_SAVING
boolean strmon = false;         // Print the packet when received?

DavisRFM69 radio;

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(100);
  radio.initialize();
  radio.setChannel(0);  // Frequency / Channel is *not* set in the initialization. Do it right after.
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  Serial.println(F("Waiting for signal in region defined in DavisRFM69.h"));
}

unsigned long lastRxTime = 0;
byte hopCount = 0;
boolean goodCrc = false;

// Added support to filter packets from a specific station and add specific stats
int stationID = 0;
bool correctID;
long idErrors = 0;

// Stats by sensor

void loop() {
  //process any serial input
  process_serial_commands();

  // The check for a zero CRC value indicates a bigger problem that will need
  // fixing, but it needs to stay until the fix is in.
  if (radio.receiveDone()) {
    packetStats.packetsReceived++;
    unsigned int crc = radio.crc16_ccitt(radio.DATA, 6);
    if ((crc == (word(radio.DATA[6], radio.DATA[7]))) && (crc != 0)) {
      // This is a good packet
      goodCrc = true;
      packetStats.receivedStreak++;
      correctID = (stationID == int (radio.DATA[0] & 0x7));
      if (correctID) 
        hopCount = 1; // From correct station
      else
        idErrors++; // From wrong station
    } else {
      goodCrc = false; // Incorrect packet
      packetStats.crcErrors++;
      packetStats.receivedStreak = 0;
    }

    // STRMON debugging (RAW packet)
    if (strmon) printStrm();

    // Print packet info
    print_debug_packet_info();

    // Radio hop only for correct packects (i.e. good CRC and correct station)
    if (goodCrc && correctID) {
      // Hop radio & update channel and lastRxTime
      lastRxTime = millis();  
      radio.hop();
#ifdef POWER_SAVING
      esp_sleep_enable_timer_wakeup(2 * 1000000); // light sleep (2s) to save energy
      esp_light_sleep_start();
#endif
    } else {
      // Do not hop the radio for incorrect packets but activate reception. 
      // Problem: receiveBegin() is a private function of the radio.classs 
      // Workaround: use setChannel to indirectly invoke it (could be improved but it works)
      radio.setChannel(radio.CHANNEL); 
    }
  }

  // If a packet was not received at the expected time, hop the radio anyway
  // in an attempt to keep up.  Give up after 4 failed attempts.  Keep track
  // of packet stats as we go.  We consider a consecutive string of missed
  // packets to be a single resync. 
  if ((hopCount > 0) && ((millis() - lastRxTime) > (hopCount * (PACKET_INTERVAL + (1000*stationID/16)) + 50))) {
    packetStats.packetsMissed++;
    if (hopCount == 1) packetStats.numResyncs++;
    if (++hopCount > 4) hopCount = 0;
    radio.hop();
#ifdef POWER_SAVING
    esp_sleep_enable_timer_wakeup(2 * 1000000); // light sleep (2s) to save energy
    esp_light_sleep_start();
#endif
  }
}

void process_serial_commands() {
  if (Serial.available() > 0)
  {
    char input = Serial.read();
    if (input == '?') 
    {
      Serial.println(F("Help:"));
      Serial.println(F("- r: read all RFM69HW registers"));
      Serial.println(F("- t: show radio temp"));
      Serial.println(F("- s: show packet stats"));
      Serial.println(F("- h: radio channel hop"));
      Serial.println(F("- R: RFM69HW reset"));
      Serial.println(F("- m: show elapsed time in milliseconds"));
      Serial.println(F("- c: show current radio channel"));
      Serial.println(F("- i: set the current station ID"));
      Serial.println(F("- I: show the current station ID"));
    }
    if (input == 'r') //r=dump all register values
    {
      radio.readAllRegs();
      Serial.println();
    }
    if (input == 't') // read radio temp
    {
      byte temperature =  radio.readTemperature(-1); // -1 = user cal factor, adjust for correct ambient
      byte fTemp = 9 * temperature / 5 + 32; // 9/5=1.8
      Serial.print(F("Radio Temp is "));
      Serial.print(temperature);
      Serial.print(F("C, "));
      Serial.print(fTemp); //converting to F loses some resolution, obvious when C is on edge between 2 values (ie 26C=78F, 27C=80F)
      Serial.println(F("F"));
    }
    if (input == 's') // show packet stats
    {
      Serial.print(F("packetsReceived: "));
      Serial.print(packetStats.packetsReceived);
      Serial.print(F(" packetsMissed: "));
      Serial.print(packetStats.packetsMissed);
      Serial.print(F(" numResyncs: "));
      Serial.print(packetStats.numResyncs);
      Serial.print(F(" receivedStreak: "));
      Serial.print(packetStats.receivedStreak);
      Serial.print(F(" crcErrors: "));
      Serial.print(packetStats.crcErrors); 
      Serial.print(F(" idErrors: "));
      Serial.print(idErrors);
      Serial.print(F(" packets/min: "));
      unsigned long min = millis()/60000;
      unsigned long correct_packets = packetStats.packetsReceived - packetStats.crcErrors - idErrors;
      float rate = 0.0;
      if (min>0) rate = correct_packets/min;
      Serial.println(rate);
    }
    if (input == 'h') // manually hop radio channel
    {
      radio.hop();
    }
    if (input == 'R') // reset radio
    {
        radio.reset();
    }
    if (input == 'm') // show time in milliseconds
    {
        Serial.println(millis());
    }
    if (input == 'c') // show current radio channel
    {
        Serial.println(radio.CHANNEL);
    }
    if (input == 'i') // set the current station ID, ej: i 1
    {
        stationID = Serial.parseInt();
    }
    if (input == 'I') // show the current station ID
    {
        Serial.println(stationID);
    }
  }
}

void print_debug_packet_info () {
    Serial.print(millis());
    Serial.print(F(":  "));
    Serial.print(radio.CHANNEL);
    Serial.print(F(" - Data: "));
    for (byte i = 0; i < DAVIS_PACKET_LEN; i++) {
      Serial.print(radio.DATA[i], HEX);
      Serial.print(F(" "));
    }
    Serial.print(F("  RSSI: "));
    Serial.print(radio.RSSI);
    //int freqError = radio.readReg(0x21) << 8 |radio.readReg(0x22);
    //Serial.print(F("      Freq error): "));
    //Serial.println(freqError);
    Serial.print(F(" CRC: "));
    if (goodCrc)
       Serial.println(F("OK"));
    else
       Serial.println(F("ERROR"));
}

void printStrm() {
  for (byte i = 0; i < DAVIS_PACKET_LEN; i++) {
    Serial.print(i);
    Serial.print(F(" = "));
    Serial.print(radio.DATA[i], HEX);
    Serial.print(F("\n\r"));
  }
  Serial.print(F("\n\r"));
}