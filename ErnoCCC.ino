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

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);


Encoder myEnc(3, 2);

void setup() {
//  Serial.begin(115200);
//  Serial.println("Erno Framecounter:");
//  u8g2.setI2CAddress(0x7a);
  u8g2.begin();
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_10x20_mn);
    u8g2.setCursor(0, 16);
    u8g2.print("Hello");
  } while ( u8g2.nextPage() );
  FreqMeasure.begin();
}

long oldPosition  = -999;
double sum=0;
int count=0;
float frequency;


void loop() {
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_10x20_mn);
      u8g2.setCursor(0, 16);
      u8g2.print(newPosition);
      u8g2.setCursor(0, 32);
      u8g2.print(frequency);
   } while ( u8g2.nextPage() );
  
//    Serial.println(newPosition);
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
    if (count > 20) {
      frequency = FreqMeasure.countToFrequency(sum / count);
      sum = 0;
      count = 0;
    }
  }

  
}
