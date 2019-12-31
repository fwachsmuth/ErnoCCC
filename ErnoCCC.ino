/*
 * - D5 Pullup
 * - D9 STOP auswerten
 * - LED Ports down
 * - 5 LEDs ansteuern
 * - D6 SSR ansteuern
 * - Button an D4 abfragen
 * - D5 LED PWM
 * - D10 for a Mosfet or PWM to the LDR-LED?
 * 
 * Schematics:
 * - Fiducials
 * 
 */

#include <Encoder.h>
#include <FreqMeasure.h>

#include <Arduino.h>
#include <U8g2lib.h>             // Display Driver

// Instantuiate soem Objects
//U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);   

Encoder myEnc(3, 2);


// ---- Define useful time constants and macros ------------------------------------
//
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)  
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN) 
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)

#define charwidth 9

unsigned long totalSeconds = 0;
long currentFrameCount;
float filmFps = 25;
float playbackFps = 18;
uint8_t hours   = 0;
uint8_t minutes = 0;
uint8_t seconds = 0;
uint8_t currentSubFrame = 0;

uint8_t rightSecDigit = 0;
uint8_t leftSecDigit = 0;
uint8_t rightMinDigit = 0;
uint8_t leftMinDigit = 0;
uint8_t hourDigit = 0;

uint8_t prevPaintedRightSecDigit = 99;
uint8_t prevPaintedLeftSecDigit = 99;
uint8_t prevPaintedRightMinDigit = 99;
uint8_t prevPaintedLeftMinDigit =99;
uint8_t prevPaintedHourDigit = 99;



void setup() {
  Serial.begin(115200);
//  Serial.println("Erno Framecounter:");
//  u8g2.setI2CAddress(0x7a); // This would set a 2nd Display to a dedictaed I2C Adress
//  u8x8.setBusClock(800000);
//  u8g2.begin();
  u8x8.begin();
  u8x8.setFont(u8x8_font_courB18_2x3_n);
  u8x8.setCursor(4,3);
  u8x8.print(":");
  u8x8.setCursor(10,3);
  u8x8.print(":");

//  u8g2.firstPage();
//  do {
//    u8g2.setFont(u8g2_font_10x20_mn);
//    u8g2.setCursor(0, 16);
//    u8g2.print("Hello");
//  } while ( u8g2.nextPage() );
  FreqMeasure.begin();
}

long prevFrameCount  = -999;
double sum=0;
int count=0;
float frequency;
unsigned int currentFilmSecond; // Good for films up to 18 hours, should be enough 


void loop() {
  currentFrameCount = myEnc.read() / 4;
  if (currentFrameCount != prevFrameCount) {
    prevFrameCount = currentFrameCount;
    drawCurrentTime();
  }

  if (FreqMeasure.available()) {
    // average several reading together
    sum = sum + FreqMeasure.read();
    count = count + 1;
    if (count > 9) {
      frequency = FreqMeasure.countToFrequency(sum / count);
      sum = 0;
      count = 0;
    }
  }

  
}

void drawCurrentTime() {
  currentFilmSecond = currentFrameCount / filmFps ;
  currentSubFrame = currentFrameCount % int(filmFps); 
  hours   = numberOfHours(currentFilmSecond);
  minutes = numberOfMinutes(currentFilmSecond);
  seconds = numberOfSeconds(currentFilmSecond);

  u8x8.setFont(u8x8_font_courB18_2x3_n); // u8x8_font_inr21_2x4_n, u8x8_font_profont29_2x3_n
  u8x8.setCursor(0,0);
  u8x8.print(currentFrameCount);
  rightSecDigit = seconds % 10;

// Only paint the glyphs that have changed, this improves the display framerate a lot
  if (rightSecDigit != prevPaintedRightSecDigit) {
    prevPaintedRightSecDigit = rightSecDigit;
    u8x8.setCursor(14,3);
    u8x8.print(rightSecDigit);
    leftSecDigit = seconds / 10;
    if (leftSecDigit != prevPaintedLeftSecDigit) {
      prevPaintedLeftSecDigit = leftSecDigit;
      u8x8.setCursor(12,3);
      u8x8.print(leftSecDigit);
      rightMinDigit = minutes % 10;
      if (rightMinDigit != prevPaintedRightMinDigit) {
        prevPaintedRightMinDigit = rightMinDigit;
        u8x8.setCursor(8,3);
        u8x8.print(rightMinDigit);
        leftMinDigit = minutes / 10;
        if (leftMinDigit != prevPaintedLeftMinDigit) {
          prevPaintedLeftMinDigit = leftMinDigit;
          u8x8.setCursor(6,3);
          u8x8.print(leftMinDigit);
          hourDigit = hours % 10;
          if (hourDigit != prevPaintedHourDigit) {
            prevPaintedHourDigit = hourDigit;
            u8x8.setCursor(2,3);
            u8x8.print(hourDigit);
          }
        }
      }
    }
  }
  
  
//    u8x8.setCursor(0,3);
//    if (currentFrameCount < 0) { 
//         u8x8.print("-");
//      } else {
//        u8x8.print(" ");
//    }
//    u8x8.print(hours);
//    u8x8.print(":");
//    if (minutes < 10) u8x8.print("0");
//    u8x8.print(minutes);
//    u8x8.print(":");
//    if (seconds < 10) u8x8.print("0");
//    u8x8.print(seconds);
//
//    u8x8.setFont(u8x8_font_8x13_1x2_r);
//    u8x8.setCursor(0,6);
//    u8x8.print(frequency);
//    u8x8.print("fps   ");
//    u8x8.print(currentSubFrame);
//        u8x8.print(" ");
//

//    u8x8.print("0123456");
//    u8x8.setCursor(0,3);
//    u8x8.print("-0:12:34");
//    u8x8.setFont(u8x8_font_8x13_1x2_r);
//    u8x8.setCursor(0,6);
//    u8x8.print("18.45 fps");
//        
//    u8g2.firstPage();
//    do {
//      u8g2.setFont(u8g2_font_10x20_mn);
//      u8g2.setCursor(0, 16);
//      u8g2.print(currentFrameCount);
//
//      u8g2.drawStr(3 * charwidth,32,":");
//      u8g2.drawStr(6 * charwidth,32,":");
//      u8g2.drawStr(9 * charwidth,32,".");
//      
//      u8g2.setCursor(0,32);
//      if (currentFrameCount < 0) { 
//         u8g2.print("-");
//      } else {
//        u8g2.print(" ");
//      }
//      
//      if (hours < 10) u8g2.print("0");
//      // u8g2.setCursor(8,32);
//      u8g2.print(hours);
//      u8g2.setCursor(4 * charwidth,32);
//      if (minutes < 10) u8g2.print("0");
//      u8g2.print(minutes);
//      u8g2.setCursor(7 * charwidth,32);
//      if (seconds < 10) u8g2.print("0");
//      u8g2.print(seconds);
//      u8g2.setCursor(10 * charwidth,32);
//      
//      if (currentSubFrame < 10) u8g2.print("0");
//      u8g2.print(currentSubFrame);
//        
//      
//      u8g2.setCursor(0, 48);
//      u8g2.print(frequency);
//   } while ( u8g2.nextPage() );

  //Serial.println(currentFrameCount);
//   if (Serial.available()) {
//    Serial.read();
//    Serial.println("Reset to zero");
//    myEnc.write(0);
//  }

}
