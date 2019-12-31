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
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

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
float filmFps = 18;
float playbackFps = 18;
uint8_t hours   = 0;
uint8_t minutes = 0;
uint8_t seconds = 0;
uint8_t currentSubFrame = 0;



void setup() {
  Serial.begin(115200);
//  Serial.println("Erno Framecounter:");
//  u8g2.setI2CAddress(0x7a); // This would set a 2nd Display to a dedictaed I2C Adress
  u8g2.setBusClock(400000);
  u8g2.begin();
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
  long currentFrameCount = myEnc.read();
  Serial.println(currentFrameCount); // debug, remove me
  if (currentFrameCount != prevFrameCount) {
    // Serial.println(currentFrameCount); // debug, remove me
    prevFrameCount = currentFrameCount;

    currentFilmSecond = currentFrameCount / (filmFps * 4);
    currentSubFrame = (currentFrameCount / 4) % int(filmFps); 
    hours   = numberOfHours(currentFilmSecond);
    minutes = numberOfMinutes(currentFilmSecond);
    seconds = numberOfSeconds(currentFilmSecond);
        
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_10x20_mn);
      u8g2.setCursor(0, 16);
      u8g2.print(currentFrameCount);

      u8g2.drawStr(3 * charwidth,32,":");
      u8g2.drawStr(6 * charwidth,32,":");
      u8g2.drawStr(9 * charwidth,32,".");
      
      u8g2.setCursor(0,32);
      if (currentFrameCount < 0) { 
         u8g2.print("-");
      } else {
        u8g2.print(" ");
      }
      
      if (hours < 10) u8g2.print("0");
      // u8g2.setCursor(8,32);
      u8g2.print(hours);
      u8g2.setCursor(4 * charwidth,32);
      if (minutes < 10) u8g2.print("0");
      u8g2.print(minutes);
      u8g2.setCursor(7 * charwidth,32);
      if (seconds < 10) u8g2.print("0");
      u8g2.print(seconds);
      u8g2.setCursor(10 * charwidth,32);
      
      if (currentSubFrame < 10) u8g2.print("0");
      u8g2.print(currentSubFrame);
        
      
      u8g2.setCursor(0, 48);
      u8g2.print(frequency);
   } while ( u8g2.nextPage() );
  
    Serial.println(currentFrameCount);
  }
//   if (Serial.available()) {
//    Serial.read();
//    Serial.println("Reset to zero");
//    myEnc.write(0);
//  }

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
