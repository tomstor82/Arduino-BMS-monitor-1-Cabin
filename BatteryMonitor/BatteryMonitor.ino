//  Cabin v2.1.4
//
//  Developed from my original OLED_BMS_u8g2 sketch
//  Added two additional pages Amperage and Voltage Analog needles
//  Intended for the cabin of the boat for more simplistic classic indications
//  Original pages are the exact same
//
//
//  WRITTEN BY:
//  Tom Storebø 7th of May 2019
//  GAUGE PAGE DEVELOPED FROM:
//  Rudi Imbrechts "Analog Gauge v1.02" - 21st of December 2015
//
//  Change log
//  25/05/19  5A scale amp needle, 50 degree needle movements amp and volt, scale lines voltmeter.
//  01/06/19  Temp hi and lo on bottom right text screen, whilst cycles removed and Ah taken its place.
//  03/06/19  Shifted the middle of text page 4 pixels left; Added BMS status to flags as well as change name from faults to flag. Various pos adjustments on text page.
//  11/06/19  Swapped low and high cell in bars
//  24/06/19  Edited CANBus data from Orion Jr as follows: removed BMS calculations for rawU, DCL, CCL; set Big Endian byte order on BMS status. Changed to 1 byte hex (0x01) instead of 2 byte (0x0001) in BMS status on text page.
//  27/06/19  Added delayed amperage reading for more stability in watt and time indications and changed rawI from float to int.
//  30/06/19  Added rxBuf for "ry" in gauge for displaying "sun" only when relay enabled; added bms status to new variable "fs" for "wrench" icon display; adjusted "0A" DCH & CHG limits.
//  09/07/19  Replaced counter with total pack cycles
//  27/07/19  Added counter and adjusted cycles on text page; Adjusted Ah digit position
//  30/08/19  Removed constraints on values and assigned minimum values to variables to avoid out of scale indications when power up; Changed low SOC warning to display below 20% SOC as long as charge safety relay in open.
//  05/09/19  Returned limits of bars and text to previous values. **Never change things that work**; Improved Amp bar code; Refined when sun charge icon should appear.
//  11/09/19  Changed time and watt calculations to use average current. Changed charging symbols to use average current.
//  21/09/19  Removed the absolute and average current algorithm as I detected no values from CANBUS.
//  22/09/19  Edited CANBUS data to fit Avg and Abs current as I suspect the rxId int is unable to fit a 5th Id. AvgI used for watt and clock computations; **** modified clock calculations. ****
//  23/09/19  Adjusted clock position two pixels left when hours exceed 99.
//  24/09/19  Replaced avgI with rawI for displaying lightning symbol whilst charging.
//  25/09/19  Added button press to swap between average and instant Watt reading. But had no success changing between the values. **** Deleted ****
//  29/09/19  Added button press to change contrast level. ** Replaced "if" in void loop button pressing for "else if & else" same with "hits". Saves 20 bytes.
//  01/10/19  Changed abbreviatons on BMS status messages to easier understand their meaning.
//  27/10/19  Changed constraints on "bars" to use bar variable instead of using indicated values. Added BMS CANBUS input MPO and changed MPE name in sketch. MPO as Active low to be connected to MPI and a 10kOhm pull-up resistor to BAT+.
//  23/08/22  New feature: Added stop icon if rawI above DCL. Corrected MPO activation byte position.
//  06/08/23  FIX: Reduced charge detect to 0A for display of lightening symbol. Fixed issue with stop sign appearing prematurely as rawI needed to be divided by 10 before comparing with DCL
//  06/08/23  MOD: Added screen blink when clearing BMS fault with putting display to sleep momentarily. Cleaned up code with else if on button clicks.
//  08/08/23  FIX: Replaced rawI for avgI for sun symbol.
//
//  Sketch 25360 bytes
//
//  HARDWARE:
//  Arduino Uno clone
//  SH1106 128x64 OLED
//  MCP2515 TJ1A050 CANBus Transeiver
//  Pushbutton and 10kOhm resistor (pull down)

#include <U8g2lib.h>
#include <mcp_can.h>
#include <SPI.h>    // SPI library CANBUS
#include <Wire.h>   // I2C library OLED

//  OLED library driver
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

//  CANBUS Shield pins
#define CAN0_INT 9                              // Set INT to pin 9
MCP_CAN CAN0(10);                               // Set CS to pin 10

//  Debugging
byte debug = 0;

//  MCP_CAN DATA
long unsigned int rxId;     // Stores 4 bytes 32 bits
unsigned char len = 0;      // Stores at least 1 byte
unsigned char rxBuf[8];     // Stores 8 bytes, 1 character  = 1 byte

//  MCP_CAN SEND DATA
byte mpo[8] = {0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00}; // Multi-purpose output activation signal
//byte mpe[8] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Multi-purpose enable activation signal

//  CANBUS data Identifier List
//  ID 0x03B BYT0+1:INST_VOLT BYT2+3:INST_AMP BYT4+5:ABS_AMP BYT6:SOC **** ABS_AMP from OrionJr errendous ****
//  ID 0x6B2 BYT0+1:LOW_CELL BYT2+3:HIGH_CELL BYT4:HEALTH BYT5+6:CYCLES
//  ID 0x0A9 BYT0:RELAY_STATE BYT1:CCL BYT2:DCL BYT3+4:PACK_AH BYT5+6:AVG_AMP
//  ID 0x0BD BYT0+1:CUSTOM_FLAGS BYT2:HI_TMP BYT3:LO_TMP BYT4:COUNTER BYT5:BMS_STATUS

//  Variables
unsigned int rawU;            // Voltage - multiplied by 10
int rawI;                     // Current - multiplied by 10 - negative value indicates charge
byte soc;                     // State of charge - multiplied by 2
byte wrench;                  // Wrench icon variable
byte angle;                   // Needle angle
unsigned int p;               // Watt reading
int m = 10;                   // Mapped values to fit needle
u8g2_uint_t av = 0;           // 8 bit unsigned int amperage and voltage needle angle
u8g2_uint_t xx = 0;           // 8 bit unsigned int watt needle angle
uint8_t c = 180;              // 8 bit unsigned integer range from 0-255 (low - high contrast)
byte dcl;                     // Discharge current limit (used on gauge and text page)

//  Button settings
const int buttonPin = 2;  // Pin assigned for button
long millis_held;         // 4 byte variable for storing duration button is held down
unsigned long firstTime;  // 4 byte variable for storing time button is first pushed
byte previous = HIGH;     // Pin state before pushing or releasing button
byte hits;                // Variable for how many times button has bin pushed
byte buttonState;         // Variable for button pushed or not

// ------------------------ setup ------------------------------

void setup() {
  // Start serial monitor communication
  if (debug == 1) {
    Serial.begin(9600);
  }
  else {
    Serial.begin(9600);
    Serial.println("Debugging Off");
    Serial.end();
  }

   // Initialise MCP2515 running at 8MHz and baudrate 250kb/s
  CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ);

  CAN0.setMode(MCP_NORMAL);
  
  // Configure pin for INT input
  pinMode(CAN0_INT, INPUT);

  // Starts display
  u8g2.begin();

  // Set standard font
  u8g2.setFont(u8g2_font_chikita_tf);
}
// -------------------- set contrast -----------------------------

void contrast(uint8_t c) {
  
  u8g2.setContrast(c);
}

// -------------------- amperage display * 2 bytes from rxBuf -------------------------

void amperage(uint8_t angle) {

  // Sort CANBus data buffer
  if(rxId == 0x03B) {
    rawI = ((rxBuf[2] << 8) + rxBuf[3]);
  }

  // Map various amp readings to between 0-50. 0 = discharge 50 = charge
  if (rawI > 1000 || rawI < -1000) {
    m = map(rawI,-2500,2500,50,0);
  }
  else if (rawI > 500 || rawI < -500) {
    m = map(rawI,-1000,1000,50,0);
  }
  else if (rawI > 100 || rawI < -100) {
    m = map(rawI, -500,500,50,0);
  }
  else  if (rawI > 50 || rawI < -50) {
    m = map(rawI, -100,100,50,0);
  }
  else {
    m = map(rawI,-50,50,50,0);
  }

  // Display dimensions
  int xmax = 128;
  int ymax = 64;
  int xcenter = xmax/2;
  int ycenter = 80;
  int arc = 64;

  // Draw arc and scale lines
  u8g2.drawCircle(xcenter, ycenter+4, arc+8);
  u8g2.drawCircle(xcenter, ycenter+4, arc+12);
  // Draw far left line
  u8g2.drawLine(6, 31, 12, 36);
  // Draw quarter left line
  u8g2.drawLine(33, 12, 36, 19);
  // Draw center line
  u8g2.drawVLine(64, 8, 4);
  // Draw quarter right line
  u8g2.drawLine(97, 13, 93, 20);
  // Draw far right line
  u8g2.drawLine(122, 31, 116, 36);
  
    
  // Draw the needle and disc
  float x1 = sin(2*angle*2*3.14/360);
  float y1 = cos(2*angle*2*3.14/360); 
  u8g2.drawLine(xcenter, ycenter, xcenter+arc*x1, ycenter-arc*y1);
  u8g2.drawDisc(xcenter, ymax+10, 20, U8G2_DRAW_UPPER_LEFT);
  u8g2.drawDisc(xcenter, ymax+10, 20, U8G2_DRAW_UPPER_RIGHT);

  // Draw 3 different scale labels
  u8g2.setFont(u8g2_font_chikita_tf);
  // Scale from -250 till 250
  if (rawI > 1000 || rawI < -1000) {
    u8g2.drawStr(0, 22, "250");                   
    u8g2.drawStr(106, 22, "-250");
  }
  // Scale from -100 till 100
  else if (rawI > 500 || rawI < -500) {
    u8g2.drawStr(0, 24, "100");                   
    u8g2.drawStr(110, 24, "-100");
  }
  // Scale from -50 till 50
  else if (rawI > 100 || rawI <-100) {
    u8g2.drawStr(0, 26, "50");                   
    u8g2.drawStr(112, 26, "-50");
  }
  // Scale from -10 till 10
  else if (rawI > 50 || rawI < -50) {
    u8g2.drawStr(0, 28, "10");                   
    u8g2.drawStr(115, 28, "-10");
  }
  // Scale from -5 till 5
  else {
    u8g2.drawStr(0, 29, "5");                   
    u8g2.drawStr(117, 29, "-5");
  }
  // Zero never changes
  u8g2.drawStr(62, 5, "0");

  // Draw unit
  u8g2.drawStr(48, 38, "Ampere");

}

// --------------------- volt display * 2 bytes from rxBuf-----------------------

void voltage(uint8_t angle) {

  // Sort CANBus data buffer
  if(rxId == 0x03B) {
    rawU = ((rxBuf[0] << 8) + rxBuf[1]);  
  }
  // Map voltage from 44,0V - 64,0V between 0 - 50
  m = map(rawU, 440,640,0,50);

  // Display dimensions
  int xmax = 128;
  int ymax = 64;
  int xcenter = xmax/2;
  int ycenter = 80;
  int arc = 64;

  // Draw arc and scale lines
  u8g2.drawCircle(xcenter, ycenter+4, arc+8);
  u8g2.drawCircle(xcenter, ycenter+4, arc+12);
  // Draw far left line
  u8g2.drawLine(6, 31, 12, 36);
  // Draw left line
  u8g2.drawLine(12, 30, 14, 32);
  // Draw quarter left line
  u8g2.drawLine(31, 13, 34, 20);
  // Draw center line
  u8g2.drawVLine(64, 7, 7);
  // Draw quarter right line
  u8g2.drawLine(95, 12, 91, 19);
  // Draw right line
  u8g2.drawLine(99, 17, 97, 20);
  // Draw far right line
  u8g2.drawLine(122, 31, 116, 36);
  
  
 // Min shading lines
  u8g2.drawLine(12, 30, 9, 33);
  u8g2.drawLine(13, 30, 10, 33);
  u8g2.drawLine(13, 31, 10, 34);
  u8g2.drawLine(14, 31, 11, 34);
  u8g2.drawLine(14, 32, 11, 35);
  // Max shading lines
  u8g2.drawLine(94, 15, 99, 18);
  u8g2.drawLine(94, 16, 98, 18);
  u8g2.drawLine(94, 17, 98, 19);
  u8g2.drawLine(93, 17, 97, 19);
  u8g2.drawLine(93, 18, 98, 20);
  
  // Draw the needle and disc
  float x1 = sin(2*angle*2*3.14/360);
  float y1 = cos(2*angle*2*3.14/360); 
  u8g2.drawLine(xcenter, ycenter, xcenter+arc*x1, ycenter-arc*y1);
  u8g2.drawDisc(xcenter, ymax+10, 20, U8G2_DRAW_UPPER_LEFT);
  u8g2.drawDisc(xcenter, ymax+10, 20, U8G2_DRAW_UPPER_RIGHT);
  // Draw scale labels
  u8g2.drawStr(0, 29, "44"); 
  u8g2.drawStr(24, 11, "49");                  
  u8g2.drawStr(59, 5, "54");
  u8g2.drawStr(92, 10, "59");
  u8g2.drawStr(118, 29, "64");
  
  // Draw unit
  u8g2.drawStr(48, 38, "Voltage");

}

// --------------------- gauge display * 12 bytes from rxBuf ----------------------

void gauge(uint8_t angle) {

  // Fault messages & status from CANBus for displaying wrench icon
  int fs;
  // Relay status for determining when to show lightening bolt and sun icon respectively
  byte ry;
  // Average current for clock and sun symbol calculations
  int avgI;
  
  // Sort CANBus data buffer
  if(rxId == 0x03B) {
    rawU = ((rxBuf[0] << 8) + rxBuf[1]);
    rawI = ((rxBuf[2] << 8) + rxBuf[3]);
    soc = (rxBuf[6]);
  }
  if(rxId == 0x0BD) {
    fs = (rxBuf[0] + rxBuf[1] + rxBuf[5]);
  }
  if(rxId == 0x0A9) {
    ry = (rxBuf[0]);
    dcl = (rxBuf[2]);
    avgI = ((rxBuf[5] << 8) + rxBuf[6]);
  }

  // Map watt readings 0-10000 to between 0-90
  m = map(p,0,10000,0,90);

  // Watt calculation
  p = (abs(rawI)/10.0)*rawU/10.0;
  
  // Display dimensions
  byte xmax = 128;
  byte ymax = 64;
  byte xcenter = xmax/2;
  byte ycenter = ymax/2+10;
  byte arc = ymax/2;

  // Draw border of the gauge
  u8g2.drawCircle(xcenter, ycenter, arc+6, U8G2_DRAW_UPPER_RIGHT);
  u8g2.drawCircle(xcenter, ycenter, arc+4, U8G2_DRAW_UPPER_RIGHT);
  u8g2.drawCircle(xcenter, ycenter, arc+6, U8G2_DRAW_UPPER_LEFT);
  u8g2.drawCircle(xcenter, ycenter, arc+4, U8G2_DRAW_UPPER_LEFT);

  // Draw the needle
  float x1 = sin(2*angle*2*PI/360);
  float y1 = cos(2*angle*2*PI/360); 
  u8g2.drawLine(xcenter, ycenter, xcenter+arc*x1, ycenter-arc*y1);
  u8g2.drawDisc(xcenter, ycenter, 5, U8G2_DRAW_UPPER_LEFT);
  u8g2.drawDisc(xcenter, ycenter, 5, U8G2_DRAW_UPPER_RIGHT);
 
  // Draw scale labels
  u8g2.drawStr(20, 42, "0");                   
  u8g2.drawStr(24, 18, "25");
  u8g2.drawStr(60, 14, "50");
  u8g2.drawStr(95, 18, "75");
  u8g2.drawStr(105, 42, "100");

  // Draw gauge label
  u8g2.drawStr(45, 32, "% POWER");

  // Draw unit description
  u8g2.drawStr(88, 60, " WATT");
     
  // Draw digital value and align its position
  u8g2.setFont(u8g2_font_profont22_tn);             
  u8g2.setCursor(65,60);
  // Draw leading 0 when values below 10W
  if (p < 10){
    u8g2.print("0");
  }
  // Shift position of values above 99W
  if (p > 99 && p <= 999) {
    u8g2.setCursor(53, 60);
  }
  // Shift position of values above 999W
  else if (p > 999 && p <= 9999) {
    u8g2.setCursor(41, 60);
  }
  // Shift position of values above 9999W
  else if (p > 9999) {
    u8g2.setCursor(29,60);
  }
  u8g2.print(p);
  
  // Check if charge current is below 0A (negative A indicates charge)
  if (rawI < 0) {
    // Display lightening symbol if charge safety relay is closed
    if ((ry & 0b00000100) == 0b00000100) {
      u8g2.setFont(u8g2_font_open_iconic_embedded_2x_t);
      u8g2.drawGlyph(4, 40, 67);
    }
    // Display sun symbol if avgI is below 0A
    else if (avgI < 0) {
      u8g2.setFont(u8g2_font_open_iconic_weather_2x_t);
      u8g2.drawGlyph(4, 39, 69);
    }
  }
  // Draw warning symbol at and below 20% State of Charge if charge safety relay is open
  if (soc <= 40 && (ry & 0b00000100) != 0b00000100) {   // soc from canbus is multiplied by 2
    u8g2.setFont(u8g2_font_open_iconic_embedded_2x_t);
    u8g2.drawGlyph(4, 39, 71);
  }
  // Draw wrench icon if BMS flags have not been seen
  if (fs != wrench) {
    u8g2.setFont(u8g2_font_open_iconic_embedded_2x_t);
    u8g2.drawGlyph(3, 62, 72);
  }
  // Draw stop icon if rawI (x10) is above DCL
  if ((rawI / 10) > dcl) {
    u8g2.setFont(u8g2_font_open_iconic_check_2x_t);
    u8g2.drawGlyph(110, 30, 66);
  }

  // Draw battery icon
  u8g2.drawFrame( 1, 0, 22, 9);
  u8g2.drawFrame( 23, 2, 2, 5);
  u8g2.drawBox( 3, 2, soc/2*0.18, 5);

  // Draw state of charge
  u8g2.setCursor(4,16);
  u8g2.setFont(u8g2_font_chikita_tf);
  u8g2.print(soc/2); u8g2.print('%');

  // Draw clock
  int h;
  int m;
  // Discharge
  if (avgI > 0) {
    h = soc / (avgI/10.0);
    m = (soc / (avgI/10.0) - h) * 60;
    char t[11];
    sprintf(t, "%02d:%02dhrs", h, m);
    if (h > 99) {
      u8g2.setCursor(84, 5);
    }
    else {
      u8g2.setCursor(88, 5);
    }
    u8g2.print(t);
  }
  // Charge
  else {
    h = (200 - soc) / (abs(avgI)/10.0);
    m = ((200 - soc) / (abs(avgI)/10.0) - h) * 60;
    char t[11];
    sprintf(t, "%02d:%02dhrs", h, m);
    if (h > 99) {
      u8g2.setCursor(84, 5);
    }
    else {
      u8g2.setCursor(88, 5);
    }
    u8g2.print(t);
  }
}
 
// ------------------------ bars gauge * 7 bytes from rxBuf ------------------------

void bars() {
  
  // Variables from CANBus
  int hC;        // High Cell Voltage in 0,0001V
  int lC;        // Low Cell Voltage in 0,0001V
  int h;         // Health
  
  // Sort CANBus data buffer
  if(rxId == 0x03B) {
    rawU = ((rxBuf[0] << 8) + rxBuf[1]);
    rawI = ((rxBuf[2] << 8) + rxBuf[3]);
  }
  if(rxId == 0x6B2) {
    lC = ((rxBuf[0] << 8) + rxBuf[1]);
    hC = ((rxBuf[2] << 8) + rxBuf[3]);
    h = (rxBuf[4]);
  }

  // Draw pack volt bar
  int pV = ((rawU/10.0-43.2)*2.083); // Box length 35/16.8 (volt difference max to min)
  u8g2.setCursor(2, 5);
  u8g2.print(rawU/10.0, 1); // One decimal
  u8g2.drawStr(1, 56, "Pack");
  u8g2.drawStr(2, 63, "Volt");
  u8g2.drawFrame(5, 8, 11, 38);
  u8g2.drawBox(7, 44-pV, 7, pV);
  
  // Draw Min and Max cell voltage bars
  int hCb = ((hC/1000.00)-2.7)*26.9;
  int lCb = ((lC/1000.00)-2.7)*26.9;
  u8g2.setCursor(28, 5);
  u8g2.print(hC/1000.00, 2); // Two decimals
  u8g2.drawStr(28, 56, "High");
  u8g2.drawStr(29, 63, "Cell");
  u8g2.drawFrame(31, 8, 11, 38);
  if (hC <= 4000 && hC >= 2700) {
    u8g2.drawBox(33, 44-hCb, 7, hCb);
  }
  u8g2.setCursor(54, 5);
  u8g2.print(lC/1000.00, 2);  // Two decimals
  u8g2.drawStr(54, 56, "Low");
  u8g2.drawStr(55, 63, "Cell");
  u8g2.drawFrame(57, 8, 11, 38);
  if (lC <= 4000 && lC >= 2700) {
    u8g2.drawBox(59, 44-lCb, 7, lCb);
  }
  
  // Draw health bar
  
  int hBar = h*0.34;
  if (h >= 0 && h <= 9) {
    u8g2.setCursor(88, 5);  // Shift position at and above 0%
  }
  else {                    // Shift position above 9%
    u8g2.setCursor(84, 5);
  }
  u8g2.print(h);
  u8g2.drawStr(76, 56, "Health");
  u8g2.drawStr(86, 63, "%");
  u8g2.drawFrame(84, 8, 11, 38);
  if (h <= 100 && h >= 0) {
    u8g2.drawBox(86, 44-hBar, 7, hBar);
  }
  
  // Draw ampere bar
  float aBar = abs(rawI)*0.0137;
  if (rawI >= 0 && rawI <= 90) {               // Shift position at and above 0A
    u8g2.setCursor(111, 5);
  }
  else if (rawI > 90 && rawI <= 199) {         // Shift position above 9A
    u8g2.setCursor(113, 5);
  }
  else if (rawI > 190 && rawI <= 999) {        // Shift position above 19A
    u8g2.setCursor(111, 5);
  }  
  else if (rawI > 999 && rawI <= 1999) {       // Shift position above 99A
    u8g2.setCursor(110, 5);
  }
  else if (rawI > 1999) {                    // Shift position above 199A
    u8g2.setCursor(108, 5);
  }
  else if (rawI < 0 && rawI >= -99) {        // Shift position below 0A
    u8g2.setCursor(105, 5);
  }
  else if (rawI < -99 && rawI > -200) {       // Shift position below -9A
    u8g2.setCursor(107, 5);
  }
  else if (rawI < -199 && rawI >= -999) {     // Shift position below -19A
    u8g2.setCursor(106, 5);
  }
  else if (rawI < -999) {                  // Shift position below -99A
    u8g2.setCursor(104, 5);
  }
  if (rawI < 100 && rawI > -100) {            // Prints single decimal above -10A and below 10A
    u8g2.print(rawI/10.0, 1);
  }
  else {                                // Prints amp without decimal at and above 10A and at and below -10A
    u8g2.print(rawI/10.0, 0);
  }
  u8g2.drawStr(108, 56, "Amp");
  u8g2.drawFrame(111, 8, 11, 38);
  // No bar displayed below 7,3A
  if (abs(rawI) > 72 && abs(rawI) <= 250) {
    u8g2.drawBox(113, 45-aBar, 7, aBar);
  }
  // Full bar at 250A
  else if (abs(rawI) > 250) {
    u8g2.drawBox(113, 10, 7, 34);
  }
}
// ------------------------ text display * 13 bytes from rxBuf ---------------------

void text() {

  // Variables from CANBus
  int fu;                 // BMS faults
  byte tH;                // Highest cell temperature *was int
  byte tL;                // Lowest cell temperature * was int
  float ah;               // Amp hours
  byte ry;                // Relay status
  byte ccl;               // Charge current limit * was unsigned int
  byte ct;                // Counter to observe data received
  byte st;                // BMS Status
  int cc;                 // Total pack cycles

  // Sort CANBus data buffer
  if(rxId == 0x0A9) {
    ry = (rxBuf[0]);
    ccl = (rxBuf[1]);
    dcl = (rxBuf[2]);
    ah = ((rxBuf[3] << 8) + rxBuf[4]);
  }
  if(rxId == 0x6B2) {
    cc = ((rxBuf[5] << 8) + rxBuf[6]);
  }
  if(rxId == 0x0BD) {
    fu = ((rxBuf[0] << 8) + rxBuf[1]);
    tH = (rxBuf[2]);
    tL = (rxBuf[3]);
    ct = (rxBuf[4]);
    st = (rxBuf[5]);
    // Saves fault & status to "wrench" after reviewing text page
    wrench = (rxBuf[0] + rxBuf[1] + rxBuf[5]);
  }
  
  // Draw horisontal lines
  u8g2.drawHLine(0, 7, 128); u8g2.drawHLine(0, 37, 62); u8g2.drawHLine(0, 46, 128); u8g2.drawHLine(62, 55, 128); u8g2.drawHLine(97, 16, 128); u8g2.drawHLine(97, 26, 128); u8g2.drawHLine(97, 37, 128);
  
  // Draw vertical lines
  u8g2.drawVLine(62, 0, 64); u8g2.drawVLine(97, 0, 64);
    
  // Draw relay status
  u8g2.drawStr(0, 5, "Relay Status");
  u8g2.drawStr(0, 16, "Discharge");
  u8g2.setFont(u8g2_font_open_iconic_check_1x_t);
  if ((ry & 0b00000001) == 0b00000001) {
    u8g2.drawGlyph(52, 18, 64);
  }
  else {
    u8g2.drawGlyph(52, 18, 68);
  }
  u8g2.setFont(u8g2_font_chikita_tf);
  u8g2.drawStr(0, 25, "Charge");
  u8g2.setFont(u8g2_font_open_iconic_check_1x_t);
  if ((ry & 0b00000010) == 0b00000010) {
    u8g2.drawGlyph(52, 27, 64);
  }
  else {
    u8g2.drawGlyph(52, 27, 68);
  }
  u8g2.setFont(u8g2_font_chikita_tf);
  u8g2.drawStr(0, 34, "Chg Safety");
  u8g2.setFont(u8g2_font_open_iconic_check_1x_t);
  if ((ry & 0b00000100) == 0b00000100) {
    u8g2.drawGlyph(52, 36, 64);
  }
  else {
    u8g2.drawGlyph(52, 36, 68);
  }

  // Current limit 
  u8g2.setFont(u8g2_font_chikita_tf); 
  u8g2.drawStr(0, 44, "Current Limit"); 
  // Discharge current limit 
  u8g2.drawStr(0, 55, "DCH"); 
  if (dcl >= 0 && dcl < 10) { 
    u8g2.setCursor(47, 55); 
  }
  else if (dcl >= 10 && dcl < 20) {
    u8g2.setCursor(42, 64);
  }
  else if (dcl >= 20 && dcl < 100) { 
    u8g2.setCursor(40, 55); 
  } 
  else if (dcl >= 100 && dcl < 200) { 
    u8g2.setCursor(36, 55); 
  }
  else {
    u8g2.setCursor(34, 55);
  }
  u8g2.print(dcl); u8g2.print(" A");
     
  // Charge current limit
  u8g2.drawStr(0, 64, "CHG"); 
  if (ccl >= 0 && ccl < 10) { 
    u8g2.setCursor(47, 64); 
  } 
  else if (ccl >= 10 && ccl < 20) { 
    u8g2.setCursor(42, 64); 
  } 
  else if (ccl >= 20 && ccl < 100) { 
    u8g2.setCursor(40, 64); 
  }
  else {
    u8g2.setCursor(36, 64);
  }
  u8g2.print(ccl); u8g2.print(" A"); 
  
  // Draw fault and bms status flags
  int x = 66;         // x position for flags
  int y = 0;          // variable y position for flags
  u8g2.drawStr(69, 5, "Flags");

  // Flag internal communication fault
  if (((fu & 0x0100) == 0x0100) && y <= 28) {
    u8g2.drawStr(x, 16+y, "intCom");
    y += 7;
  }
  // Flag internal convertions fault
  if (((fu & 0x0200) == 0x0200) && y <= 28) {
    u8g2.drawStr(x-2, 16+y, "intConv");
    y += 7;
  }
  // Flag weak cell fault
  if (((fu & 0x0400) == 0x0400) && y <= 28) {
    u8g2.drawStr(x, 16+y, "wkCell");
    y += 7;
  }
  // Flag low cell fault
  if (((fu & 0x0800) == 0x0800) && y <= 28) {
    u8g2.drawStr(x, 16+y, "lowCell");
    y += 7;
  }
  // Flag open wire fault
  if (((fu & 0x1000) == 0x1000) && y <= 28) {
    u8g2.drawStr(x-2, 16+y, "opnWire");
    y += 7;
  }
  // Flag current sense fault
  if (((fu & 0x2000) == 0x2000) && y <= 28) {
    u8g2.drawStr(x-1, 16+y, "crrSns");
    y += 7;
  }
  // Flag volt sense fault
  if (((fu & 0x4000) == 0x4000) && y <= 28) {
    u8g2.drawStr(x, 16+y, "vltSns");
    y += 7;
  }
  // Flag volt redundancy fault
  if (((fu & 0x8000) == 0x8000) && y <= 28) {
    u8g2.drawStr(x-2, 16+y, "vltRdcy");
    y += 7;
  }
  // Flag weak pack fault
  if (((fu & 0x0001) == 0x0001) && y <= 28) {
    u8g2.drawStr(x-2, 16+y, "wkPack");
    y += 7;
  }
  // Flag thermistor fault
  if (((fu & 0x0002) == 0x0002) && y <= 28) {
    u8g2.drawStr(x, 16+y, "xThrm");
    y += 7;
  }
  // Flag charge limit enforcement fault
  if (((fu & 0x0004) == 0x0004) && y <= 28) {
    u8g2.drawStr(x-2, 16+y, "chgRly");
    y += 7;
  }
  // Flag discharge limit enforcement fault
  if (((fu & 0x0008) == 0x0008) && y <= 28) {
    u8g2.drawStr(x, 16+y, "dchRly");
    y += 7;
  }
  // Flag charge safety relay fault
  if (((fu & 0x0010) == 0x0010) && y <= 28) {
    u8g2.drawStr(x-2, 16+y, "sftyRly");
    y += 7;
  }
  // Flag internal memory fault
  if (((fu & 0x0020) == 0x0020) && y <= 28) {
    u8g2.drawStr(x, 16+y, "intMem");
    y += 7;
  }
  // Flag internal thermistor fault
  if (((fu & 0x0040) == 0x0040) && y <= 28) {
    u8g2.drawStr(x, 16+y, "intThm");
    y += 7;
  }
  // Flag internal logic fault
  if (((fu & 0x0080) == 0x0080) && y <= 28) {
    u8g2.drawStr(x, 16+y, "intLog");
    y += 7;
  }

  // Flag BMS status
  if (((st & 0x01) == 0x01) && y <= 28){
    u8g2.drawStr(x, 16+y, "VoltFS");
    y += 7;
  }
  if (((st & 0x02) == 0x02) && y <= 28) {
    u8g2.drawStr(x-2, 16+y, "CurrFS");
    y += 7;
  }
  if (((st & 0x04) == 0x04) && y <= 28) {
    u8g2.drawStr(x-1, 16+y, "RelyFS");
    y += 7;
  }
  if (((st & 0x08) == 0x08) && y <= 28) {
    u8g2.drawStr(x-2, 16+y, "CellBlcg");
    y += 7;
  }

  // Draw count
  if (ct < 10) {
    u8g2.setCursor(111, 5);
  }
  else if (ct >= 10 && ct < 100) {
    u8g2.setCursor(109, 5);
  }
  else if (ct >= 100 && ct < 120) {
    u8g2.setCursor(107, 5);
  }
  else if (ct >= 120 && ct < 200) {
    u8g2.setCursor(106, 5);
  }
  else {
    u8g2.setCursor(105, 5);
  }
  u8g2.print(ct);

  // Draw total pack cycles
  u8g2.drawStr(100, 14, "Cycles");
  if (cc < 10) {
    u8g2.setCursor(111, 24);
  }
  else if (cc >= 10 && cc < 100) {
    u8g2.setCursor(109, 24);
  }
  else if (cc >= 100 && cc < 120) {
    u8g2.setCursor(107, 24);
  }
  else if (cc >= 120 && cc < 200) {
    u8g2.setCursor(106, 24);
  }
  else {
    u8g2.setCursor(105, 24);
  }
  u8g2.print(cc);

  // Draw Ah 
  u8g2.drawStr(109, 35, "Ah");
  if (ah < 1000) {
    u8g2.setCursor(108, 44);
  }
  if (ah >= 1000 && ah < 10000) { 
    u8g2.setCursor(105, 44); 
  } 
  else if (ah > 11094 && ah < 11195) { 
    u8g2.setCursor(106,  44); 
  } 
  else { 
    u8g2.setCursor(103,44); 
  } 
  u8g2.print(ah/100, 1); 

  // Draws pack temp
  u8g2.drawStr(66, 53, "TempH"); 
  u8g2.drawStr(100, 53, "TempL"); 
  // Highest temperature
  if (tH >= 0 && tH < 10) { 
    u8g2.setCursor(78, 64); 
  } 
  else if (tH >= 10) { 
    u8g2.setCursor(75, 64); 
  } 
  else if (tH < 0 && tH > -10) { 
    u8g2.setCursor(71, 64); 
  } 
  else { 
    u8g2.setCursor(68, 64); 
  } 
  u8g2.print(tH); 
  // Lowest temperature
  if (tL >= 0 && tL < 10) { 
    u8g2.setCursor(112, 64); 
  } 
  else if (tL >= 10) { 
    u8g2.setCursor(109, 64); 
  } 
  else if (tL < 0 && tL > -10) { 
    u8g2.setCursor(105, 64); 
  } 
  else { 
    u8g2.setCursor(102, 64); 
  } 
  u8g2.print(tL); 
} 
// -------------------------- loop -------------------------

void loop() {

  do {
    contrast(c);
  }
  while(u8g2.nextPage());
  
  // Read MCP2515
  if(!digitalRead(CAN0_INT)) {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
  }

  // needle position calculations for amperage and voltage
  // 155 = zero position, 180 = just before middle, 0 = middle, 25 = max
  av = m;
  // position correction
  if (av < 25){
    av += 155;
  }
  else {
    av -= 25;
  }
  
  // needle position calculations watt gauge
  // 135 = zero position, 180 = just before middle, 0 = middle, 45 = max
  xx = m;
  // position correction
  if (xx < 45){
    xx += 135;
  }
  else {
    xx -= 45;
  }

  // Check the status of the button
  buttonState = digitalRead(buttonPin);

  // How long is the button held down
  if (buttonState == HIGH && previous == LOW) {
    firstTime = millis();
  }
  if (buttonState == LOW && previous == HIGH) {
    millis_held = millis() - firstTime;
  }

  // Require more than 200ms push to qualify as a button "hit"
  if (millis_held > 200) {
    if (buttonState == LOW && previous == HIGH) {

      // Long push over 3 sec sends MPO signal to clear BMS faults ** needs to be connected with 10kOhm pull up resistor to BAT+ and MPI **
      if (millis_held > 3000) {
        CAN0.sendMsgBuf(0x32, 0, 8, mpo);
        u8g2.setPowerSave(1);
        delay(100);
        u8g2.setPowerSave(0);
        
      }
      
      /*// Long push over 2 sec sends MPE signal ** not yet assigned task **
      else if (millis_held > 2000) {
        CAN0.sendMsgBuf(0x32, 0, 8, mpe);
      }*/

      // Long push for 0,5 sec changes contrast
      else if (millis_held >= 500) {
        if (c == 255) {
          c = 100;
        }
        else if (c == 100) {
          c = 180;
        }
        else {
          c = 255;
        }
      }
      
      // Short button press changes between pages
      else {
        if (hits < 5) {
          hits += 1;  // adds 1 to hits
        }
        else {
          hits = 1;
        }
      }
    }
    // Save button state
    previous = buttonState;
  }

  // Display voltage page
  if (hits == 1) {
    u8g2.firstPage(); 
    do {             
      voltage(av);
    }
    while(u8g2.nextPage());
  }
  
  // Display amperage page
  else if (hits == 2) {
    u8g2.firstPage(); 
    do {             
      amperage(av);
    }
    while(u8g2.nextPage());
  }
  
  // Display gauge page
  else if (hits == 3) {
    u8g2.firstPage(); 
    do {             
      gauge(xx);
    }
    while(u8g2.nextPage());
  }

  // Display bars page
  else if (hits == 4) {
    u8g2.firstPage();
    do {
      bars();
    }
    while(u8g2.nextPage());
  }

  // Display text page
  else {
    u8g2.firstPage();
    do {
      text();
    }
    while(u8g2.nextPage());
  }
}
