/* 

Projektor Bildfrequenzmessung

*/

#include <FreqMeasure.h>
#include <Arduino.h>
#include <U8x8lib.h>

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);         

void setup() {
  Serial.begin(115200);
  FreqMeasure.begin();
  u8x8.begin();
  u8x8.setPowerSave(0);
}

double sum=0;
int count=0;

void loop() {
  if (FreqMeasure.available()) {
    // average several reading together
    sum = sum + FreqMeasure.read();
    count = count + 1;
    if (count > 100) {
      float frequency = FreqMeasure.countToFrequency(sum / count * 4 ); // Sektorencount
//      Serial.println(frequency,5);

//      u8x8.setFont(u8x8_font_px437wyse700b_2x2_r);
      u8x8.setFont(u8x8_font_profont29_2x3_n);
//      u8x8.setFont(u8x8_font_inb21_2x4_n);
      u8x8.setCursor(1, 3);
      u8x8.print(frequency, 3);
//      u8x8.refreshDisplay();    // only required for SSD1606/7  

      sum = 0;
      count = 0;
    }
  }
}
