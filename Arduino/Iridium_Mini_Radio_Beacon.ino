// ###################################
// # Iridium 9603N Mini Radio Beacon #
// ###################################

// Stores both source and destination RockBLOCK serial numbers in flash.
// The default values are:
#define RB_destination 0 // Serial number of the destination RockBLOCK (int). Set to zero to disable RockBLOCK message forwarding
#define RB_source 0 // Serial number of this unit (int)

// Define how often messages are sent in MINUTES (max 1440)
// This is the _quickest_ messages will be sent. Could be much slower than this depending on:
// capacitor charge time; gnss fix time; Iridium timeout; etc.
// The default value will be overwritten with the one stored in Flash - if one exists
// The value can be changed via a Mobile Terminated message
int BEACON_INTERVAL = 5;

// GNSS / Iridium antenna switching is via a Skyworks AS179-92LF RF Switch
// Switching is performed by applying either +5V or 3V3SW to the AS179's V1 and V2 pins
// +5V is the power rail for the 9603N from the MPM3610 (enabled by pulling D5 high)
// 3V3SW is the 3.3V power rail for the MAX-M8Q (switched via Q1, enabled by pulling D11 low)
// Take great care to make sure +5V and 3V3SW are not enabled at the same time!
// BADS THINGS WILL PROBABLY HAPPEN IF YOU DO ENABLE BOTH SIMULTANEOUSLY!

// Uses Cristian Maglie's FlashStorage library to store the BEACON_INTERVAL setting
// https://github.com/cmaglie/FlashStorage
// BEACON_INTERVAL can be updated via an Iridium Mobile Terminated message (e.g. from RockBLOCK Operations or a Beacon Base)

// With grateful thanks to Mikal Hart:
// Based on Mikal's IridiumSBD Beacon example: https://github.com/mikalhart/IridiumSBD
// Requires Mikal's TinyGPS library: https://github.com/mikalhart/TinyGPS
// and PString: http://arduiniana.org/libraries/pstring/

// With grateful thanks to:
// Adafruit: https://learn.adafruit.com/using-atsamd21-sercom-to-add-more-spi-i2c-serial-ports/creating-a-new-serial
// MartinL: https://forum.arduino.cc/index.php?topic=341054.msg2443086#msg2443086

// The Iridium_9603_Beacon PCB is based on the Adafruit Feather M0 (Adalogger)
// https://www.adafruit.com/products/2796
// GPS data provided by u-blox MAX-M8Q
// https://www.u-blox.com/en/product/max-m8-series

// Uses RTCZero to provide sleep functionality (on the M0)
// https://github.com/arduino-libraries/RTCZero

// With grateful thanks to CaveMoa for his SimpleSleepUSB example
// https://github.com/cavemoa/Feather-M0-Adalogger
// https://github.com/cavemoa/Feather-M0-Adalogger/tree/master/SimpleSleepUSB
// Note: you will need to close and re-open your serial monitor each time the M0 wakes up

// With thanks to David A. Mellis and Tom Igoe for the smoothing tutorial
// http://www.arduino.cc/en/Tutorial/Smoothing

// This version provides support for the LPRS eRIC4/9
// eRIC Pin4 Tx (input) is connected to MOSI (Digital Pin 23, Port B Pin 10, SERCOM4 Pad 2, Serial3 Tx)
// eRIC Pin3 Rx (output) is connected to SCK (Digital Pin 24, Port B Pin 11, SERCOM4 Pad 3, Serial3 Rx)
// eRIC Pin22 (wake from low power) is connected to A0 (Digital Pin 14)
// Power for the eRIC is switched via a FET. Pull A2 (Digital Pin 16) low to enable the eRIC.
// The message to be transmitted by the radio board should be sent in an Iridium Mobile Terminated message
// (e.g. from RockBLOCK Operations or a Beacon Base) containing [RADIO=nnnnnnnn] where "nnnnnnnn" is
// the radio message (usually the serial number of the eRIC on a Pyro Cut-Down)

// Iridium 9603N is interfaced to M0 using Serial2
// D6 (Port A Pin 20) = Enable (Sleep) : Connect to 9603 ON/OFF Pin 5
// D10 (Port A Pin 18) = Serial2 TX : Connect to 9603 Pin 6
// D12 (Port A Pin 19) = Serial2 RX : Connect to 9603 Pin 7
// SCL / D21 (Port A Pin 23) = Ring Indicator : Connect to 9603 Pin 12

// Power to the 9603N is provided by an MPM3610 5V regulator
// The regulator is enabled by pulling pin D5 high

// MAX-M8Q GNSS is interfaced to M0 using Serial1
// D1 (Port A Pin 10) = Serial1 TX : Connect to GPS RX
// D0 (Port A Pin 11) = Serial1 RX : Connect to GPS TX
// D11 (Port A Pin 16) = GPS ENable : Connect to GPS EN(ABLE)

// D13 (Port A Pin 17) = Red LED
// D9 (Port A Pin 7) = AIN 7 : Bus Voltage / 2

// Red LED on D13 shows when the SAMD is in bootloader mode (LED will fade up/down)

// If you bought your 9603N from Rock7, you can have your messages delivered to another RockBLOCK automatically:
// https://www.rock7.com/shop-product-detail?productId=50
// http://www.rock7mobile.com/products-rockblock-9603
// http://www.rock7mobile.com/downloads/RockBLOCK-9603-Developers-Guide.pdf (see last page)
// Change RB_destination (above) to the serial number of the destination RockBLOCK
// Change RB_source (above) to the serial number of this RockBLOCK
// Both RBSOURCE and RBDESTINATION can be updated via a MT message.
// Note: the RockBLOCK gateway does not remove the destination RockBLOCK address from the SBD message
// so, in this code, it is included as a full CSV field

#include <IridiumSBD.h> // Requires V2: https://github.com/mikalhart/IridiumSBD
#include <TinyGPS.h> // NMEA parsing: http://arduiniana.org
#include <PString.h> // String buffer formatting: http://arduiniana.org

#include <RTCZero.h> // M0 Real Time Clock
RTCZero rtc; // Create an rtc object

// Flash Storage
#include <FlashStorage.h>
typedef struct { // Define a struct to hold the flash variable(s)
  int PREFIX; // Flash storage prefix (0xB5); used to test if flash has been written to before 
  int INTERVAL; // Message interval in minutes
  // RockBLOCK source serial number: stored as an int; i.e. the RockBLOCK serial number of the 9603N attached to this beacon
  int RBSOURCE; 
  // RockBLOCK destination serial number: stored as an int; i.e. the RockBLOCK serial number of the 9603N you would like the messages delivered _to_
  int RBDESTINATION; // Set this to zero to disable RockBLOCK gateway message forwarding
  int CSUM; // Flash storage checksum; the modulo-256 sum of PREFIX and INTERVAL; used to check flash data integrity
} FlashVarsStruct;
FlashStorage(flashVarsMem, FlashVarsStruct); // Reserve memory for the flash variables
FlashVarsStruct flashVars; // Define the global to hold the variables
int RBSOURCE = RB_source;
int RBDESTINATION = RB_destination;

// Serial2 pin and pad definitions (in Arduino files Variant.h & Variant.cpp)
#define PIN_SERIAL2_RX       (34ul)               // Pin description number for PIO_SERCOM on D12
#define PIN_SERIAL2_TX       (36ul)               // Pin description number for PIO_SERCOM on D10
#define PAD_SERIAL2_TX       (UART_TX_PAD_2)      // SERCOM pad 2 (SC1PAD2)
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_3)    // SERCOM pad 3 (SC1PAD3)
// Instantiate the Serial2 class
Uart Serial2(&sercom1, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX);
HardwareSerial &ssIridium(Serial2);

// Serial3 pin and pad definitions (in Arduino files Variant.h & Variant.cpp)
// eRIC Tx (input) is connected to MOSI (Digital Pin 23, Port B Pin 10, SERCOM4 Pad 2, Serial3 Tx)
// eRIC Rx (output) is connected to SCK (Digital Pin 24, Port B Pin 11, SERCOM4 Pad 3, Serial3 Rx)
#define PIN_SERIAL3_RX       (24ul)               // Pin description number for PIO_SERCOM on D24
#define PIN_SERIAL3_TX       (23ul)               // Pin description number for PIO_SERCOM on D23
#define PAD_SERIAL3_TX       (UART_TX_PAD_2)      // SERCOM4 Pad 2 (SC4PAD2)
#define PAD_SERIAL3_RX       (SERCOM_RX_PAD_3)    // SERCOM4 Pad 3 (SC4PAD3)
// Instantiate the Serial3 class
Uart Serial3(&sercom4, PIN_SERIAL3_RX, PIN_SERIAL3_TX, PAD_SERIAL3_RX, PAD_SERIAL3_TX);
HardwareSerial &sseRIC(Serial3);

#define ssGPS Serial1 // Use M0 Serial1 to interface to the MAX-M8Q

// Leave the "#define GALILEO" uncommented to use: GPS + Galileo + GLONASS + SBAS
// Comment the "#define GALILEO" out to use the default u-blox M8 GNSS: GPS + SBAS + QZSS + GLONASS
#define GALILEO

// Set Nav Mode to Portable
static const uint8_t setNavPortable[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Set Nav Mode to Pedestrian
static const uint8_t setNavPedestrian[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Set Nav Mode to Automotive
static const uint8_t setNavAutomotive[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x04, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Set Nav Mode to Sea
static const uint8_t setNavSea[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x05, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Set Nav Mode to Airborne <1G
static const uint8_t setNavAir[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static const int len_setNav = 42;

// Set NMEA Config
// Set trackFilt to 1 to ensure course (COG) is always output
// Set Main Talker ID to 'GP' to avoid having to modify TinyGPS
static const uint8_t setNMEA[] = {
  0xb5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x20, 0x40, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const int len_setNMEA = 26;

// Set GNSS Config to GPS + Galileo + GLONASS + SBAS (Causes the M8 to restart!)
static const uint8_t setGNSS[] = {
  0xb5, 0x62, 0x06, 0x3e, 0x3c, 0x00,
  0x00, 0x20, 0x20, 0x07,
  0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01,
  0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01,
  0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01,
  0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01,
  0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x03,
  0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x05,
  0x06, 0x08, 0x0e, 0x00, 0x01, 0x00, 0x01, 0x01 };
static const int len_setGNSS = 66;

static const int Enable_9603N = 5; // Enables the MPM3610 to provide power for the 9603N
static const int ringIndicator = 21; // 9603 Ring Indicator on pin D21
static const int IridiumSleepPin = 6; // Iridium Sleep connected to D6
IridiumSBD isbd(ssIridium, IridiumSleepPin); // This should disable the 9603
TinyGPS tinygps;
long iterationCounter = 0; // Increment each time a transmission is attempted

static const int ledPin = 13; // WB2812B + Red LED on pin D13
//#define NoLED // Uncomment this line to disable the LED

static const int GPS_EN = 11; // GNSS Enable on pin D11
#define GPS_ON LOW
#define GPS_OFF HIGH
#define VAP A7 // Bus voltage analog pin (bus voltage divided by 2)
#define VBAT_LOW 5.0 // Minimum voltage for MPM3610 less one MBR120 voltage drop

// eRIC Pin22 (wake from low power) is connected to A0 (Digital Pin 14)
static const int eRIC_WAKE = 14;
// A2 (Digital Pin 16) enables power for the eRIC
static const int eRIC_EN = 16;

// Loop Steps
#define init          0
#define start_GPS     1
#define read_GPS      2
#define start_9603    3
#define zzz           4
#define wake          5

// Variables used by Loop
int year;
byte month, day, hour, minute, second, hundredths;
unsigned long dateFix, locationFix;
float latitude, longitude;
long altitude;
float speed;
short satellites;
long course;
long hdop;
bool fixFound = false;
bool charsSeen = false;
int loop_step = init;
float vbat = 9.0;
unsigned long tnow;

// Storage for the average voltage during Iridium callbacks
const int numReadings = 25;   // number of samples
int readings[numReadings];    // the readings from the analog input
int readIndex = 0;            // the index of the current reading
long int total = 0;           // the running total
int latest_reading = 0;       // the latest reading
int average_reading = 0;      // the average reading

// IridiumSBD Callbacks
bool ISBDCallback()
{
#ifndef NoLED
  // 'Flash' the LED
  if ((millis() / 333) % 2 == 1) {
    digitalWrite(ledPin, HIGH);
  }
  else {
    digitalWrite(ledPin, LOW);
  }
#endif

  // Check the 'battery' voltage now we are drawing current for the 9603
  // If voltage is low, stop Iridium send
  get_vbat_smooth();

  if (vbat < VBAT_LOW) {
    Serial.print("***!!! LOW VOLTAGE (ISBDCallback) ");
    Serial.print(vbat,2);
    Serial.println("V !!!***");
    return false; // Returning false causes IridiumSBD to terminate
  }
  else {     
    return true;
  }

  delay(1);
}
// V2 console and diagnostic callbacks (replacing attachConsole and attachDiags)
void ISBDConsoleCallback(IridiumSBD *device, char c) { Serial.write(c); }
void ISBDDiagsCallback(IridiumSBD *device, char c) { Serial.write(c); }

// Interrupt handler for SERCOM1 (essential for Serial2 comms)
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

// Interrupt handler for SERCOM4 (essential for Serial3 comms)
void SERCOM4_Handler()
{
  Serial3.IrqHandler();
}

// RTC alarm interrupt
void alarmMatch()
{
  int rtc_mins = rtc.getMinutes(); // Read the RTC minutes
  int rtc_hours = rtc.getHours(); // Read the RTC hours
  if (BEACON_INTERVAL > 1440) BEACON_INTERVAL = 1440; // Limit BEACON_INTERVAL to one day
  rtc_mins = rtc_mins + BEACON_INTERVAL; // Add the BEACON_INTERVAL to the RTC minutes
  while (rtc_mins >= 60) { // If there has been an hour roll over
    rtc_mins = rtc_mins - 60; // Subtract 60 minutes
    rtc_hours = rtc_hours + 1; // Add an hour
  }
  rtc_hours = rtc_hours % 24; // Check for a day roll over
  rtc.setAlarmMinutes(rtc_mins); // Set next alarm time (minutes)
  rtc.setAlarmHours(rtc_hours); // Set next alarm time (hours)
}

// Read and smooth the 'battery' voltage
// Average voltage over numReadings to smooth out any short dips
void get_vbat_smooth() {
  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  latest_reading = analogRead(VAP);
  readings[readIndex] = latest_reading;
  // add the reading to the total:
  total = total + latest_reading;
  // advance to the next position in the array:
  readIndex = readIndex + 1;
  // if we're at the end of the array...wrap around to the beginning:
  if (readIndex >= numReadings) readIndex = 0;
  // calculate the average:
  average_reading = total / numReadings; // Seems to work OK with integer maths - but total does need to be long int
  vbat = float(average_reading) * (4.3 * 3.3 / 1023.0); // Calculate average battery voltage
}

// Read the instantaneous 'battery' voltage
void get_vbat() {
  vbat = analogRead(VAP) * (4.3 * 3.3 / 1023.0); // Read battery voltage from resistor divider
}

// Initialise the smoothed 'battery' voltage
void init_vbat()
{
  // Initialise voltage sample buffer with current readings
  total = 0;
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = analogRead(VAP);
    total = total + readings[thisReading];
    delay(1);
  }
  get_vbat_smooth();
}

// Send message in u-blox UBX format
// Calculates and appends the two checksum bytes
// Doesn't add the 0xb5 and 0x62 sync chars (these need to be included at the start of the message)
void sendUBX(const uint8_t *message, const int len) {
  int csum1 = 0; // Checksum bytes
  int csum2 = 0;
  for (int i=0; i<len; i++) { // For each byte in the message
    ssGPS.write(message[i]); // Write the byte
    if (i >= 2) { // Don't include the sync chars in the checksum
      csum1 = csum1 + message[i]; // Update the checksum bytes
      csum2 = csum2 + csum1;
    }
  }
  csum1 = csum1 & 0xff; // Limit checksums to 8-bits
  csum2 = csum2 & 0xff;
  ssGPS.write((uint8_t)csum1); // Send the checksum bytes
  ssGPS.write((uint8_t)csum2);
}

void setup()
{
  pinMode(Enable_9603N, OUTPUT); // 9603N enable via MPM3610
  digitalWrite(Enable_9603N, LOW); // Disable the 9603N
  
  pinMode(GPS_EN, OUTPUT); // GNSS enable
  digitalWrite(GPS_EN, GPS_OFF); // Disable the GNSS
  
  pinMode(IridiumSleepPin, OUTPUT); // The call to IridiumSBD should have done this - but just in case
  digitalWrite(IridiumSleepPin, LOW); // Disable the Iridium 9603
  pinMode(ringIndicator, INPUT_PULLUP); // Define an input for the Iridium 9603 Ring Indicator signal

  pinMode(ledPin, OUTPUT); // LED
  digitalWrite(ledPin, LOW); // Disable the LED
  
  pinMode(eRIC_EN, OUTPUT); // Initialise eRIC enable pin
  digitalWrite(eRIC_EN, LOW); // Enable eRIC
  pinMode(eRIC_WAKE, OUTPUT); // Initialise eRIC WAKE pin
  digitalWrite(eRIC_WAKE, HIGH); // Enable eRIC

  // See if global variables have already been stored in flash
  // If they have, read them. If not, initialise them.
  flashVars = flashVarsMem.read(); // Read the flash memory
  int csum = flashVars.PREFIX + flashVars.INTERVAL + flashVars.RBSOURCE + flashVars.RBDESTINATION; // Sum the prefix and data
  csum = csum & 0xff; // Limit checksum to 8-bits
  if ((flashVars.PREFIX == 0xB5) and (csum == flashVars.CSUM)) { // Check prefix and checksum match
    // Flash data is valid so update globals using the stored values
    BEACON_INTERVAL = flashVars.INTERVAL;
    RBSOURCE = flashVars.RBSOURCE;
    RBDESTINATION = flashVars.RBDESTINATION;
  }
  else {
    // Flash data is corrupt or hasn't been initialised so do that now
    flashVars.PREFIX = 0xB5; // Initialise the prefix
    flashVars.INTERVAL = BEACON_INTERVAL; // Initialise the beacon interval
    flashVars.RBSOURCE = RBSOURCE; // Initialise the source RockBLOCK serial number
    flashVars.RBDESTINATION = RBDESTINATION; // Initialise the destination RockBLOCK serial number
    csum = flashVars.PREFIX + flashVars.INTERVAL + flashVars.RBSOURCE + flashVars.RBDESTINATION; // Initialise the checksum
    csum = csum & 0xff;
    flashVars.CSUM = csum;
    flashVarsMem.write(flashVars); // Write the flash variables
  }

  rtc.begin(); // Start the RTC now that BEACON_INTERVAL has been updated
  rtc.setAlarmSeconds(rtc.getSeconds()); // Initialise RTC Alarm Seconds
  alarmMatch(); // Set next alarm time using updated BEACON_INTERVAL
  rtc.enableAlarm(rtc.MATCH_HHMMSS); // Alarm Match on hours, minutes and seconds
  rtc.attachInterrupt(alarmMatch); // Attach alarm interrupt
  
  iterationCounter = 0; // Make sure iterationCounter is set to zero (indicating a reset)
  loop_step = init; // Make sure loop_step is set to init

  // Configure the eRIC then put it into low power mode
  // Start the eRIC serial port
  sseRIC.begin(19200);
  delay(1000); // Allow time for the port to open
  
  sseRIC.print("ER_CMD#R0"); // Reset Radio
  delay(50);
  sseRIC.print("ACK"); // Acknowledge
  delay(500);
  while(sseRIC.available()){sseRIC.read();} // Clear the serial rx buffer

  sseRIC.print("ER_CMD#C5"); // Set Channel 5
  delay(50);
  sseRIC.print("ACK"); // Acknowledge
  delay(50);
  while(sseRIC.available()){sseRIC.read();} // Clear the serial rx buffer
  
  sseRIC.print("ER_CMD#B0"); // Set Over-Air Baud Rate to 1200
  delay(50);
  sseRIC.print("ACK"); // Acknowledge
  delay(50);
  while(sseRIC.available()){sseRIC.read();} // Clear the serial rx buffer
  
  sseRIC.print("ER_CMD#P0"); // Set Transmit Power to 0dBm
  delay(50);
  sseRIC.print("ACK"); // Acknowledge
  delay(50);
  while(sseRIC.available()){sseRIC.read();} // Clear the serial rx buffer
  
  sseRIC.print("ER_CMD#D2"); // Set RX Power Saving
  delay(50);
  sseRIC.print("ACK"); // Acknowledge
  delay(500);
  while(sseRIC.available()){sseRIC.read();} // Clear the serial rx buffer
  
  sseRIC.print("ER_CMD#d2"); // Set TX Power Saving
  delay(50);
  sseRIC.print("ACK"); // Acknowledge
  delay(500);
  while(sseRIC.available()){sseRIC.read();} // Clear the serial rx buffer
  
  sseRIC.print("ER_CMD#L8?"); // Get eRIC serial number
  delay(50);
  while(sseRIC.available()){sseRIC.read();} // Clear the serial rx buffer
  sseRIC.print("ACK"); // Acknowledge
  delay(500);
  // Transmit serial number
  sseRIC.print("Iridium Mini Radio Beacon ");
  while(sseRIC.available()){sseRIC.write(sseRIC.read());} // Send serial number
  sseRIC.println();
  // Wait for the data to be transmitted
  // We should really be checking the CTS/BUSY signal here
  // but a simple delay won't cause the code to hang if the radio board isn't connected
  delay(1500);

  // Put eRIC into Low Power Mode 0
  sseRIC.print("ER_CMD#A21"); // Set Low Power Mode 0
  delay(50);
  sseRIC.print("ACK"); // Acknowledge
  delay(500);
  while(sseRIC.available()){sseRIC.read();} // Clear the serial rx buffer
  
  digitalWrite(eRIC_WAKE, LOW); // Disable eRIC
}

void loop()
{
  unsigned long loopStartTime = millis();

  switch(loop_step) {

    case init:

#ifndef NoLED
      digitalWrite(ledPin, HIGH);
#endif

      // Start the serial console
      Serial.begin(115200);
      delay(5000); // Wait 5 secs - allow time for user to open serial monitor
    
      // Send welcome message
      Serial.println("Iridium 9603N Mini Radio Beacon");

      // Echo the BEACON_INTERVAL
      Serial.print("Using a BEACON_INTERVAL of ");
      Serial.print(BEACON_INTERVAL);
      Serial.println(" minutes");
      
      // Echo RBDESTINATION and RBSOURCE
      Serial.print("Using an RBDESTINATION of ");
      Serial.println(RBDESTINATION);
      Serial.print("Using an RBSOURCE of ");
      Serial.println(RBSOURCE);

      // Setup the IridiumSBD
      // (attachConsole and attachDiags methods have been replaced with ISBDConsoleCallback and ISBDDiagsCallback)
      isbd.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE); // Change power profile to default
      isbd.useMSSTMWorkaround(false); // Redundant?

      // Check battery voltage
      // If voltage is low, go to sleep
      init_vbat(); // Use init_vbat to make sure the current true smoothed voltage is used (after coming out of deep sleep)
      if (vbat < VBAT_LOW) {
        Serial.print("***!!! LOW VOLTAGE (init) ");
        Serial.print(vbat,2);
        Serial.println(" !!!***");
        loop_step = zzz;
      }
      else {
        loop_step = start_GPS;
      }
      
      break;
      
    case start_GPS:

      // Power up the GNSS
      Serial.println("Powering up the GNSS...");
      digitalWrite(GPS_EN, GPS_ON); // Enable the GNSS

      delay(2000); // Allow time for both to start
    
      // Check battery voltage now we are drawing current for the GPS
      // If voltage is low, go to sleep
      init_vbat(); // Use init_vbat to make sure the true smoothed voltage is used
      if (vbat < VBAT_LOW) {
        Serial.print("***!!! LOW VOLTAGE (start_GPS) ");
        Serial.print(vbat,2);
        Serial.println("V !!!***");
        loop_step = zzz;
      }
      else {
        loop_step = read_GPS;
      }
      
      break;

    case read_GPS:
      // Start the GPS serial port
      ssGPS.begin(9600);

      delay(1000); // Allow time for the port to open

      // Configure GPS
      Serial.println("Configuring GNSS...");

      // Disable all messages except GGA and RMC
      ssGPS.println("$PUBX,40,GLL,0,0,0,0*5C"); // Disable GLL
      delay(1100);
      ssGPS.println("$PUBX,40,ZDA,0,0,0,0*44"); // Disable ZDA
      delay(1100);
      ssGPS.println("$PUBX,40,VTG,0,0,0,0*5E"); // Disable VTG
      delay(1100);
      ssGPS.println("$PUBX,40,GSV,0,0,0,0*59"); // Disable GSV
      delay(1100);
      ssGPS.println("$PUBX,40,GSA,0,0,0,0*4E"); // Disable GSA
      delay(1100);
      
      //sendUBX(setNavPortable, len_setNav); // Set Portable Navigation Mode
      //sendUBX(setNavPedestrian, len_setNav); // Set Pedestrian Navigation Mode
      //sendUBX(setNavAutomotive, len_setNav); // Set Automotive Navigation Mode
      //sendUBX(setNavSea, len_setNav); // Set Sea Navigation Mode
      sendUBX(setNavAir, len_setNav); // Set Airborne <1G Navigation Mode
      delay(1100);

      sendUBX(setNMEA, len_setNMEA); // Set NMEA: to always output COG; and set main talker to GP (instead of GN)
      delay(1100);

#ifdef GALILEO
      sendUBX(setGNSS, len_setGNSS); // Set GNSS - causes M8 to restart!
      delay(3000); // Wait an extra time for GNSS to restart
#endif

      while(ssGPS.available()){ssGPS.read();} // Flush RX buffer so we don't confuse TinyGPS with UBX acknowledgements

      // Reset TinyGPS and begin listening to the GNSS
      Serial.println("Beginning to listen for GNSS traffic...");
      fixFound = false; // Reset fixFound
      charsSeen = false; // Reset charsSeen
      tinygps = TinyGPS();
      
      // Look for GPS signal for up to 5 minutes
      for (tnow = millis(); !fixFound && millis() - tnow < 5UL * 60UL * 1000UL;)
      {
        if (ssGPS.available())
        {
          charsSeen = true;
          if (tinygps.encode(ssGPS.read()))
          {
            tinygps.f_get_position(&latitude, &longitude, &locationFix);
            tinygps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &dateFix);
            altitude = tinygps.altitude(); // Altitude in cm (long) - checks that we have received a GGA message
            speed = tinygps.f_speed_mps(); // Get speed - checks that we have received an RMC message
            satellites = tinygps.satellites(); // Get number of satellites
            course = tinygps.course(); // Get course over ground
            hdop = tinygps.hdop(); // Get horizontal dilution of precision
            fixFound = locationFix != TinyGPS::GPS_INVALID_FIX_TIME && 
                       dateFix != TinyGPS::GPS_INVALID_FIX_TIME && 
                       altitude != TinyGPS::GPS_INVALID_ALTITUDE &&
                       speed != TinyGPS::GPS_INVALID_F_SPEED &&
                       satellites != TinyGPS::GPS_INVALID_SATELLITES &&
                       course != TinyGPS::GPS_INVALID_ANGLE &&
                       hdop != TinyGPS::GPS_INVALID_HDOP &&
                       year != 2000;
          }
        }

        // if we haven't seen any GPS data in 10 seconds, then stop waiting
        if (!charsSeen && millis() - tnow > 10000) {
          break;
        }

        // Check battery voltage now we are drawing current for the GPS
        // If voltage is low, stop looking for GNSS and go to sleep
        get_vbat_smooth();
        if (vbat < VBAT_LOW) {
          break;
        }

#ifndef NoLED
        // 'Flash' the LED
        if ((millis() / 1000) % 2 == 1) {
          digitalWrite(ledPin, HIGH);
        }
        else {
          digitalWrite(ledPin, LOW);
        }
#endif

      }

      Serial.println(charsSeen ? fixFound ? F("A GNSS fix was found!") : F("No GNSS fix was found.") : F("Wiring error: No GNSS data seen."));
      Serial.print("Latitude (degrees): "); Serial.println(latitude, 6);
      Serial.print("Longitude (degrees): "); Serial.println(longitude, 6);
      Serial.print("Altitude (m): "); Serial.println(altitude / 100); // Convert altitude from cm to m

      if (vbat < VBAT_LOW) {
        Serial.print("***!!! LOW VOLTAGE (read_GNSS) ");
        Serial.print(vbat,2);
        Serial.println("V !!!***");
        loop_step = zzz;
      }
      else if (!charsSeen) {
        Serial.println("***!!! No GNSS data received !!!***");
        loop_step = zzz;
      }
      else {
        // Power down the GNSS
        Serial.println("Powering down the GNSS...");
        digitalWrite(GPS_EN, GPS_OFF); // Disable the GPS
        delay(1000); // Let the voltage decay
        loop_step = start_9603;
      }
      
      break;

    case start_9603:

#ifndef NoLED
      digitalWrite(ledPin, HIGH);
#endif

      // Start talking to the 9603 and power it up
      Serial.println("Beginning to talk to the 9603...");

      digitalWrite(Enable_9603N, HIGH); // Enable the 9603N
      delay(2000);

      ssIridium.begin(19200);
      delay(1000);

      if (isbd.begin() == ISBD_SUCCESS) // isbd.begin powers up the 9603
      {
        char outBuffer[120]; // Always try to keep message short (maximum should be ~101 chars including RockBLOCK destination and source)
    
        if (fixFound)
        {
          if (RBDESTINATION > 0) {
            sprintf(outBuffer, "RB%07d,%d%02d%02d%02d%02d%02d,", RBDESTINATION, year, month, day, hour, minute, second);
          }
          else {
            sprintf(outBuffer, "%d%02d%02d%02d%02d%02d,", year, month, day, hour, minute, second);
          }
          int len = strlen(outBuffer);
          PString str(outBuffer + len, sizeof(outBuffer) - len);
          str.print(latitude, 6);
          str.print(",");
          str.print(longitude, 6);
          str.print(",");
          str.print(altitude / 100); // Convert altitude from cm to m
          str.print(",");
          str.print(speed, 1); // Speed in metres per second
          str.print(",");
          str.print(course / 100); // Convert from 1/100 degree to degrees
          str.print(",");
          str.print((((float)hdop) / 100),1); // Convert from 1/100 m to m
          str.print(",");
          str.print(satellites);
          str.print(",");
          str.print("0,"); // Set pressure to zero
          str.print("0.0,"); // Set temperature to zero
          str.print(vbat, 2);
          str.print(",");
          str.print(float(iterationCounter), 0);
          if (RBDESTINATION > 0) { // Append source RockBLOCK serial number (as text) to the end of the message
            char sourceBuffer[12];
            sprintf(sourceBuffer, "RB%07d", RBSOURCE);
            str.print(",");
            str.print(sourceBuffer);
          }
        }
    
        else
        {
          // No GPS fix found!
          if (RBDESTINATION > 0) {
            sprintf(outBuffer, "RB%07d,19700101000000,0.0,0.0,0,0.0,0,0.0,0,", RBDESTINATION);
          }
          else {
            sprintf(outBuffer, "19700101000000,0.0,0.0,0,0.0,0,0.0,0,");
          }
          int len = strlen(outBuffer);
          PString str(outBuffer + len, sizeof(outBuffer) - len);
          str.print("0,"); // Set pressure to zero
          str.print("0.0,"); // Set temperature to zero
          str.print(vbat, 2);
          str.print(",");
          str.print(float(iterationCounter), 0);
          if (RBDESTINATION > 0) { // Append source RockBLOCK serial number (as text) to the end of the message
            char sourceBuffer[12];
            sprintf(sourceBuffer, "RB%07d", RBSOURCE);
            str.print(",");
            str.print(sourceBuffer);
          }
        }

        Serial.print("Transmitting message '");
        Serial.print(outBuffer);
        Serial.println("'");
        uint8_t mt_buffer[100]; // Buffer to store Mobile Terminated SBD message
        size_t mtBufferSize = sizeof(mt_buffer); // Size of MT buffer
        init_vbat(); // init_vbat to make sure the current true smoothed voltage is used now that the 9603N is active

        if (isbd.sendReceiveSBDText(outBuffer, mt_buffer, mtBufferSize) == ISBD_SUCCESS) { // Send the message; download an MT message if there is one
          if (mtBufferSize > 0) { // Was an MT message received?
            // Check message content
            mt_buffer[mtBufferSize] = 0; // Make sure message is NULL terminated
            String mt_str = String((char *)mt_buffer); // Convert message into a String
            Serial.print("Received a MT message: "); Serial.println(mt_str);

            // Check if the message contains a correctly formatted BEACON_INTERVAL: "[INTERVAL=nnn]"
            int new_interval = 0;
            int starts_at = -1;
            int ends_at = -1;
            starts_at = mt_str.indexOf("[INTERVAL="); // See is message contains "[INTERVAL="
            if (starts_at >= 0) { // If it does:
              ends_at = mt_str.indexOf("]", starts_at); // Find the following "]"
              if (ends_at > starts_at) { // If the message contains both "[INTERVAL=" and "]"
                String new_interval_str = mt_str.substring((starts_at + 10),ends_at); // Extract the value after the "="
                Serial.print("Extracted an INTERVAL of: "); Serial.println(new_interval_str);
                new_interval = (int)new_interval_str.toInt(); // Convert it to int
              }
            }
            if ((new_interval > 0) and (new_interval <= 1440)) { // Check new interval is valid
              Serial.print("New BEACON_INTERVAL received. Setting BEACON_INTERVAL to ");
              Serial.print(new_interval);
              Serial.println(" minutes.");
              BEACON_INTERVAL = new_interval; // Update BEACON_INTERVAL
              // Update flash memory
              flashVars.PREFIX = 0xB5; // Reset the prefix (hopefully redundant!)
              flashVars.INTERVAL = new_interval; // Store the new beacon interval
              int csum = flashVars.PREFIX + flashVars.INTERVAL + flashVars.RBSOURCE + flashVars.RBDESTINATION; // Update the checksum
              csum = csum & 0xff;
              flashVars.CSUM = csum;
              flashVarsMem.write(flashVars); // Write the flash variables
            }

            // Check if the message contains a correctly formatted RBSOURCE: "[RBSOURCE=nnnnn]"
            int new_source = -1;
            starts_at = -1;
            ends_at = -1;
            starts_at = mt_str.indexOf("[RBSOURCE="); // See is message contains "[RBSOURCE="
            if (starts_at >= 0) { // If it does:
              ends_at = mt_str.indexOf("]", starts_at); // Find the following "]"
              if (ends_at > starts_at) { // If the message contains both "[RBSOURCE=" and "]"
                String new_source_str = mt_str.substring((starts_at + 10),ends_at); // Extract the value after the "="
                Serial.print("Extracted an RBSOURCE of: "); Serial.println(new_source_str);
                new_source = (int)new_source_str.toInt(); // Convert it to int
              }
            }
            // toInt returns zero if the conversion fails, so it is not possible to distinguish between a source of zero and an invalid value!
            // An invalid value will cause RBSOURCE to be set to zero
            if (new_source >= 0) { // If new_source was received
              Serial.print("New RBSOURCE received. Setting RBSOURCE to ");
              Serial.println(new_source);
              RBSOURCE = new_source; // Update RBSOURCE
              // Update flash memory
              flashVars.PREFIX = 0xB5; // Reset the prefix (hopefully redundant!)
              flashVars.RBSOURCE = new_source; // Store the new RockBLOCK source serial number
              int csum = flashVars.PREFIX + flashVars.INTERVAL + flashVars.RBSOURCE + flashVars.RBDESTINATION; // Update the checksum
              csum = csum & 0xff;
              flashVars.CSUM = csum;
              flashVarsMem.write(flashVars); // Write the flash variables
            }

            // Check if the message contains a correctly formatted RBDESTINATION: "[RBDESTINATION=nnnnn]"
            int new_destination = -1;
            starts_at = -1;
            ends_at = -1;
            starts_at = mt_str.indexOf("[RBDESTINATION="); // See is message contains "[RBDESTINATION="
            if (starts_at >= 0) { // If it does:
              ends_at = mt_str.indexOf("]", starts_at); // Find the following "]"
              if (ends_at > starts_at) { // If the message contains both "[RBDESTINATION=" and "]"
                String new_destination_str = mt_str.substring((starts_at + 15),ends_at); // Extract the value after the "="
                Serial.print("Extracted an RBDESTINATION of: "); Serial.println(new_destination_str);
                new_destination = (int)new_destination_str.toInt(); // Convert it to int
              }
            }
            // toInt returns zero if the conversion fails, so it is not possible to distinguish between a destination of zero and an invalid value!
            // An invalid value will cause RBDESTINATION to be set to zero
            if (new_destination >= 0) { // If new_destination was received
              Serial.print("New RBDESTINATION received. Setting RBDESTINATION to ");
              Serial.println(new_destination);
              RBDESTINATION = new_destination; // Update RBDESTINATION
              // Update flash memory
              flashVars.PREFIX = 0xB5; // Reset the prefix (hopefully redundant!)
              flashVars.RBDESTINATION = new_destination; // Store the new RockBLOCK destination serial number
              int csum = flashVars.PREFIX + flashVars.INTERVAL + flashVars.RBSOURCE + flashVars.RBDESTINATION; // Update the checksum
              csum = csum & 0xff;
              flashVars.CSUM = csum;
              flashVarsMem.write(flashVars); // Write the flash variables
            }

            // Check if the message contains a correctly formatted RADIO message: "[RADIO=nnnnnnnn]"
            String new_radio_str;
            int new_radio = -1;
            starts_at = -1;
            ends_at = -1;
            starts_at = mt_str.indexOf("[RADIO="); // See is message contains "[RADIO="
            if (starts_at >= 0) { // If it does:
              ends_at = mt_str.indexOf("]", starts_at); // Find the following "]"
              if (ends_at > starts_at) { // If the message contains both "[RADIO=" and "]"
                new_radio_str = mt_str.substring((starts_at + 7),ends_at); // Extract the value after the "="
                Serial.print("Extracted a RADIO message of: "); Serial.println(new_radio_str);
                new_radio = 1;
              }
            }
            if (new_radio == 1) { // If a new radio message was received
              Serial.print("New RADIO message received. Sending: ");
              Serial.println(new_radio_str);
              // Wake up the eRIC
              digitalWrite(eRIC_WAKE, HIGH); // Enable eRIC
              // (Re)Start the eRIC serial port
              sseRIC.begin(19200);
              delay(2000); // Allow time for the port to open and for eRIC to start
              // (Re)Configure the eRIC - this should be unnecessary - but just in case!
              // Serial data from the eRIC is ignored - so the code won't hang
              // if the radio board is not connected
              sseRIC.print("ER_CMD#R1"); // Reset Radio
              delay(50);
              sseRIC.print("ACK"); // Acknowledge
              delay(500);
              while(sseRIC.available()){sseRIC.read();} // Clear the serial rx buffer
              sseRIC.print("ER_CMD#C5"); // Set Channel 5
              delay(50);
              sseRIC.print("ACK"); // Acknowledge
              delay(50);
              while(sseRIC.available()){sseRIC.read();} // Clear the serial rx buffer
              sseRIC.print("ER_CMD#B0"); // Set Over-Air Baud Rate to 1200
              delay(50);
              sseRIC.print("ACK"); // Acknowledge
              delay(50);
              while(sseRIC.available()){sseRIC.read();} // Clear the serial rx buffer
              sseRIC.print("ER_CMD#P0"); // Set Transmit Power to 0dBm
              delay(50);
              sseRIC.print("ACK"); // Acknowledge
              delay(50);
              while(sseRIC.available()){sseRIC.read();} // Clear the serial rx buffer
              sseRIC.print("ER_CMD#D2"); // Set RX Power Saving
              delay(50);
              sseRIC.print("ACK"); // Acknowledge
              delay(500);
              while(sseRIC.available()){sseRIC.read();} // Clear the serial rx buffer
              sseRIC.print("ER_CMD#d2"); // Set TX Power Saving
              delay(50);
              sseRIC.print("ACK"); // Acknowledge
              delay(500);
              while(sseRIC.available()){sseRIC.read();} // Clear the serial rx buffer
              // Now send the radio message three times for redundancy
              sseRIC.print(new_radio_str);
              delay(1500);
              sseRIC.print(new_radio_str);
              delay(1500);
              sseRIC.print(new_radio_str);
              delay(1500);
              // Put eRIC back into Low Power Mode 0
              sseRIC.print("ER_CMD#A21"); // Set Low Power Mode 0
              delay(50);
              sseRIC.print("ACK"); // Acknowledge
              delay(500);
              while(sseRIC.available()){sseRIC.read();} // Clear the serial rx buffer
              digitalWrite(eRIC_WAKE, LOW); // Disable eRIC
            }
          }
          
#ifndef NoLED
          // Give the LED ~ten short flashes to indicate successful transmission
          for (int flashCount = 0; flashCount < 11; flashCount++)
          {
            digitalWrite(ledPin, LOW);
            delay(100);
            digitalWrite(ledPin, HIGH);
            delay(100);
          }
#endif
        }
        ++iterationCounter; // Increment iterationCounter (regardless of whether send was successful)
      }
      
      loop_step = zzz;

      break;

    case zzz:
    
      // Get ready for sleep
      Serial.println("Putting 9603N and GNSS to sleep...");
      isbd.sleep(); // Put 9603 to sleep
      delay(1000);
      ssIridium.end(); // Close GPS, Iridium and eRIC serial ports
      ssGPS.end();
      sseRIC.end();
      delay(1000); // Wait for serial ports to clear
  
      // Disable: GNSS and MPM3610
      digitalWrite(GPS_EN, GPS_OFF); // Disable the GPS
      digitalWrite(Enable_9603N, LOW); // Disable the 9603N

      // Turn LED off
      digitalWrite(ledPin, LOW);
  
      // Close and detach the serial console (as per CaveMoa's SimpleSleepUSB)
      Serial.println("Going to sleep until next alarm time...");
      delay(1000); // Wait for serial port to clear
      Serial.end(); // Close the serial console
      USBDevice.detach(); // Safely detach the USB prior to sleeping
    
      // Sleep until next alarm match
      rtc.standbyMode();
  
      // Wake up!
      loop_step = wake;
  
      break;

    case wake:
      // Attach and reopen the serial console
      USBDevice.attach(); // Re-attach the USB
      delay(1000);  // Delay added to make serial more reliable

      // Now loop back to init
      loop_step = init;

      break;
  }
}
