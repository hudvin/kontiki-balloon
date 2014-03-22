#include <Wire.h>
#include <SoftwareSerial.h>
#include "TinyGPS.h"

#define HMC6343_ADDRESS 0x19

SoftwareSerial GPS(4, 5); //RX pin 4, TX pin 5
byte success = 0 ;
TinyGPS tGPS;
// -----------------------------------------------------------------------------
void setup()
{
  
  Serial.begin(115200);
  
  //==========================================
  // HMC6434 setup
  
  pinMode(12, OUTPUT);
  
  Wire.begin();
  
  // wait 500ms after applying power before asking the HMC6434 to do anything
  delay(500);
  
  /*
   * Set the 'variation angle correction' (magnetic declination), see p8 in datasheet.
   * At the cathedral in St Andrews on 07/08/2012 this was 3 degrees 16 seconds West
   * which is -3.2667 decimal degrees, or -33 tenths of a degree, so MSB/LSB in two's
   * complement is 11111111/11011111. This is written to EEPROM so technically doesn't
   * need to be done every time if the device isn't moving to a drastically new
   * location inbetween use.
   *
   * *** Ideally this would be dynamically updated according to GPS fix. TODO
   */
  byte deviationMSB = B11111111;
  Wire.beginTransmission(HMC6343_ADDRESS);
  Wire.write(0xF1);                          // 'Write to EEPROM' command
  Wire.write(0x0D);                          // EEPROM address of deviation angle MSB
  Wire.write(deviationMSB);
  Wire.endTransmission();

  //byte deviationLSB = B11011111;
  byte deviationLSB = B11100010;             // 15/01/2013
  Wire.beginTransmission(HMC6343_ADDRESS);
  Wire.write(0xF1);
  Wire.write(0x0C);                          // EEPROM address of deviation angle LSB
  Wire.write(deviationLSB);
  Wire.endTransmission();

  /*
   * Set the measurement rate to 10Hz (0x02) from default of 5Hz (0x01).
   * Again this is EEPROM so shouldn't need re-doing unless it is explicitly reset
   * at some point.
   */
  Wire.beginTransmission(HMC6343_ADDRESS);
  Wire.write(0xF1);
  Wire.write(0x05);                        // EEPROM address of Operational Mode Register 2
  Wire.write(0x02);                        // (OM2_1 = 1 && OM2_0 = 0) == 10Hz operation
  Wire.endTransmission();

  /*
   * Set the Heading Infinite Impulse Response (IIR) filter from its default of 0
   * to something a bit more than 0. Again, this is EEPROM so shouldn't need re-doing
   * unless it is explicitly reset at some point.
   */
  Wire.beginTransmission(HMC6343_ADDRESS);
  Wire.write(0xF1);
  Wire.write(0x14);                        // EEPROM address of the Heading IIR filter LSB
  Wire.write(0x00);                      // 0 is no filtering
  //Wire.write(0x0F);                        // 15 is filtered with 15 previous readings
  Wire.endTransmission();

  /*
   * Set the HMC6343 to 'upright front' orientation. This is temporary, but can be
   * written to an EEPROM register if required.
   */
  Wire.beginTransmission(HMC6343_ADDRESS);
  Wire.write(0x74);
  Wire.endTransmission();
  
  //==========================================
  // u-blox MAX 6 setup
  
  // u-blox MAX 6 default serial baudrate is 9600
  GPS.begin(9600);
  
  // but switch it to 4800 baud for improved stability
  // (see discussion http://ava.upuaut.net/store/wiki/doku.php?id=levelconvertor)
  GPS.print("$PUBX,41,1,0007,0003,4800,0*13\r\n"); 
  
  // then switch the *software* serial to 4800 to match (*not* the 115200 Serial to the computer via USB!)
  GPS.begin(4800);
  GPS.flush();
  
  //Serial.println("Setting Dynamic Platform Model to Pedestrian: ");
  uint8_t CFG_NAV5[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF,
                    0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27,
                    0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00,
                    0x64, 0x00, 0x2C, 0x01, 0x32, 0x3C, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00};
  calculateUBXChecksum(CFG_NAV5, (sizeof(CFG_NAV5)/sizeof(uint8_t)));
  
  while (!success)
  {
    sendUBX(CFG_NAV5, (sizeof(CFG_NAV5)/sizeof(uint8_t)));
    success = getUBX_ACK(CFG_NAV5);
  }
  success = 0;
  
  //==========================================
  
  //Serial.println("Enabling SBAS using EGNOS: ");
  uint8_t CFG_SBAS[] = {0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x07,
                        0x03, 0x00, 0x51, 0x08, 0x00, 0x00, 0x00, 0x00};
  calculateUBXChecksum(CFG_SBAS, (sizeof(CFG_SBAS)/sizeof(uint8_t)));
  
  while (!success)
  {
    sendUBX(CFG_SBAS, (sizeof(CFG_SBAS)/sizeof(uint8_t)));
    success = getUBX_ACK(CFG_SBAS);
  }
  success = 0;
  
  //==========================================
  
  //Serial.println("Disabling GPGLL messages: ");
  
  uint8_t CFG_MSG_GLL[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
  calculateUBXChecksum(CFG_MSG_GLL, (sizeof(CFG_MSG_GLL)/sizeof(uint8_t)));
  
  while (!success)
  {
    sendUBX(CFG_MSG_GLL, (sizeof(CFG_MSG_GLL)/sizeof(uint8_t)));
    success = getUBX_ACK(CFG_MSG_GLL);
  }
  success = 0;
  
  //==========================================
  
  //Serial.println("Disabling GPGSA messages: ");
  
  uint8_t CFG_MSG_GSA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
  calculateUBXChecksum(CFG_MSG_GSA, (sizeof(CFG_MSG_GSA)/sizeof(uint8_t)));
  
  while (!success)
  {
    sendUBX(CFG_MSG_GSA, (sizeof(CFG_MSG_GSA)/sizeof(uint8_t)));
    success = getUBX_ACK(CFG_MSG_GSA);
  }
  success = 0;
    
  //==========================================
    
  //Serial.println("Disabling GPGSV messages: ");
  
  uint8_t CFG_MSG_GSV[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
  calculateUBXChecksum(CFG_MSG_GSV, (sizeof(CFG_MSG_GSV)/sizeof(uint8_t)));
  
  while (!success)
  {
    sendUBX(CFG_MSG_GSV, (sizeof(CFG_MSG_GSV)/sizeof(uint8_t)));
    success = getUBX_ACK(CFG_MSG_GSV);
  }
  success = 0;
  
  //==========================================
  
  //Serial.println("Disabling GPVTG messages: ");
  
  uint8_t CFG_MSG_VTG[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
  calculateUBXChecksum(CFG_MSG_VTG, (sizeof(CFG_MSG_VTG)/sizeof(uint8_t)));
  
  while (!success)
  {
    sendUBX(CFG_MSG_VTG, (sizeof(CFG_MSG_VTG)/sizeof(uint8_t)));
    success = getUBX_ACK(CFG_MSG_VTG);
  }
  success = 0;
  
  //==========================================
  
  //Serial.println("Enabling GPGGA messages: ");
  
  uint8_t CFG_MSG_GGA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00,
                           0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00};
  calculateUBXChecksum(CFG_MSG_GGA, (sizeof(CFG_MSG_GGA)/sizeof(uint8_t)));
  
  while (!success)
  {
    sendUBX(CFG_MSG_GGA, (sizeof(CFG_MSG_GGA)/sizeof(uint8_t)));
    success = getUBX_ACK(CFG_MSG_GGA);
  }
  success = 0;
  
  //==========================================
  
  //Serial.println("Enabling GPRMC messages: ");
  
  uint8_t CFG_MSG_RMC[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04,
                           0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00};
                           //0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
  calculateUBXChecksum(CFG_MSG_RMC, (sizeof(CFG_MSG_RMC)/sizeof(uint8_t)));
  
  while (!success)
  {
    sendUBX(CFG_MSG_RMC, (sizeof(CFG_MSG_RMC)/sizeof(uint8_t)));
    success = getUBX_ACK(CFG_MSG_RMC);
  }
  success = 0;

}
// -----------------------------------------------------------------------------
void loop()
{
  
  digitalWrite(12, HIGH);
  
  //==========================================
  // HMC6343
  
  /*
   * Set the HMC6343 to return the information that we want (HeadMSB, HeadLSB,
   * PitchMSB, PitchLSB, RollMSB, RollLSB).
   */
  Wire.beginTransmission(HMC6343_ADDRESS);
  Wire.write(0x50);                        // Command to return the data we want
  Wire.endTransmission();

  byte MSByte, LSByte;
  
  Wire.requestFrom(HMC6343_ADDRESS, 6);  // request 6 bytes (see command 0x50)
  while(Wire.available() < 1);           // busy wait while no bytes to receive

  MSByte = Wire.read();
  LSByte = Wire.read();
  float heading = ((MSByte << 8) + LSByte) / 10.0; // the heading in degrees

  MSByte = Wire.read();
  LSByte = Wire.read();
  float pitch = ((MSByte << 8) + LSByte) / 10.0;   // the pitch in degrees

  MSByte = Wire.read();
  LSByte = Wire.read();
  float roll = ((MSByte << 8) + LSByte) / 10.0;     // the roll in degrees
  
  //==========================================
  // u-blox MAX 6
  
  bool newData = false;
  //unsigned long chars;
  //unsigned short sentences, failed;

  char message[15];
  int messageIndex = 0;

  // Loop fast enough for smooth HMC6343 updates, TinyGPS handles detecting whether
  // there is any new GPS data so I don't have to worry about that - Yeah bitch! Abstraction!
  for (unsigned long start = millis(); millis() - start < 100;)
  {
    while (GPS.available())
    {
      char c = GPS.read();
     
      // Just do this to send the entire NMEA sentence whole over serial
      //Serial.print(c);
     
     
      // uncomment this line if you want to see the GPS data flowing
      if (tGPS.encode(c)) { // Did a new valid sentence come in?
        newData = true;
      }
    }
  }

  float flat, flon;
  unsigned long age;
  tGPS.f_get_position(&flat, &flon, &age);
  
  //==========================================
  // sending output
  
  // HMC6434
  Serial.print(heading);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.print(roll);
  Serial.print(" ");
  
  // u-blox MAX 6
  
  /* Print an invalid latitude/longitude if it hasn't received a fix yet.
   * Valid ranges are
   *      latitude:   -90 to 90
   *      longitude:  -180 to 180
   */
  if (age == TinyGPS::GPS_INVALID_AGE) {
    Serial.println("999.000000 999.000000");
  }
  
  /*
   * Otherwise print the most recent fix. This data could be old, but it's
   * probably better not to keep switching between a realistic reading &
   * an invalid one each time we don't get new data within a few seconds?
   *
   * If we only print a real value each time newData is true, at the update
   * frequency we're talking about here we will have 7-8 invalid readings
   * for each real one.
   */
  else {
    if (newData) {
      //Serial.print("newData is true ");
    }
    else if (!newData) {
      //Serial.print("newData is false ");
    }
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" ");
    Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
  }
  
  digitalWrite(12, LOW);
  
  //delay(300);

}
// -----------------------------------------------------------------------------
/*
 * Calculates the UBX checksum. The checksum is calculated over the packet, starting
 * & including the CLASS field, up until, but excluding, the Checksum Field. The
 * checksum algorithm used is the 8-bit Fletcher Algorithm, which is used in the TCP
 * standard (RFC 1145).
 */
void calculateUBXChecksum(uint8_t ubxPacket[], uint8_t ubxPacketLength)
{
  
  /*
   * Loop starts at 2, because checksum doesn't include sync chars at 0 & 1.
   * Loop stops 2 from the end, as cheksum doesn't include itself(!).
   */
  uint8_t CK_A = 0, CK_B = 0;
  
  uint8_t limit = ubxPacketLength - 2;

  for (uint8_t i = 2; i < limit; i++)
  {
    CK_A = CK_A + ubxPacket[i];
    CK_B = CK_B + CK_A;
  }
  
  ubxPacket[ubxPacketLength-2] = CK_A;
  ubxPacket[ubxPacketLength-1] = CK_B;
  
}
// -----------------------------------------------------------------------------
void printUBXPacket(uint8_t ubxPacket[], uint8_t ubxPacketLength)
{
  
  for (uint8_t i = 0; i < ubxPacketLength; ++i)
  {
    Serial.print("[");
    Serial.print(i);
    Serial.print("] ");
    Serial.println(ubxPacket[i]);
  }
  
}
// -----------------------------------------------------------------------------
// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    GPS.write(MSG[i]);
    //Serial.print(MSG[i], HEX);
  }
  GPS.println();
}
// -----------------------------------------------------------------------------
// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  //Serial.print(" * Reading ACK response: ");
 
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B
 
  calculateUBXChecksum(ackPacket, 10);
 
  while (1) {
 
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      //Serial.println(" success.");
      return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      //Serial.println(" failed.");
      return false;
    }
 
    // Make sure data is available to read
    if (GPS.available()) {
      b = GPS.read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        //Serial.print(b, HEX);
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }
 
    }
  }
}
// -----------------------------------------------------------------------------
