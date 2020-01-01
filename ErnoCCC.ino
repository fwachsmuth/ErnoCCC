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
#include <U8g2lib.h>          // Display Driver
#include <SwitchManager.h>    // Button Handling and Debouncing

// Instantuiate some Objects
//
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);   
Encoder myEnc(3, 2);
SwitchManager theButton;

// ---- Define uC Pins ------------------------------------
//
#define theButtonPin  4
#define stopPin       9
#define ssrPin        6
#define ledPwmPin     5


// ---- Define useful time constants and macros ------------------------------------
//
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)  
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN) 
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)

// ---- Button state machine -------------------------------------------------------
//
#define BTN_STATE_UNLOCKED     0
#define BTN_STATE_LOCKED       1
#define BTN_STATE_9            2
#define BTN_STATE_16_2_3       3
#define BTN_STATE_18           4
#define BTN_STATE_24           5
#define BTN_STATE_25           6

uint8_t buttonState = 0;

bool stopped = false;
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
bool sign = false;

uint8_t prevPaintedRightSecDigit = 99;
uint8_t prevPaintedLeftSecDigit = 99;
uint8_t prevPaintedRightMinDigit = 99;
uint8_t prevPaintedLeftMinDigit =99;
uint8_t prevPaintedHourDigit = 99;
bool prevPaintedSign = false;

// Make some nice pixel gfx

const uint8_t unlockedLockTop[24] = {
  0b00000000
, 0b00000000
, 0b10000000
, 0b10000000
, 0b10000000
, 0b10000000
, 0b10000000
, 0b10000000

, 0b10000000
, 0b10000000
, 0b10000000
, 0b11100000
, 0b11111000
, 0b10001100
, 0b00000110
, 0b00000110

, 0b00000110
, 0b00000110
, 0b00001100
, 0b11111000
, 0b11100000
, 0b00000000
, 0b00000000
, 0b00000000
};

const uint8_t lockedLockTop[16] = {
  0b00000000
, 0b00000000
, 0b10000000
, 0b11100000
, 0b11111000
, 0b10001100
, 0b10000110
, 0b10000110

, 0b10000110
, 0b10000110
, 0b10001100
, 0b11111000
, 0b11100000
, 0b10000000
, 0b00000000
, 0b00000000
};

const uint8_t lockBottom[16] = {
  0b00000000
, 0b00000000
, 0b01111111
, 0b01111111
, 0b01111111
, 0b01111111
, 0b01011001
, 0b01000000

, 0b01000000
, 0b01011001
, 0b01111111
, 0b01111111
, 0b01111111
, 0b01111111
, 0b00000000
, 0b00000000
};

const uint8_t twoThirdsTop[8] = {
  0b01000010
, 0b01100001
, 0b01010001
, 0b01001110
, 0b10000000
, 0b01000000
, 0b00100000
, 0b00010000
};

const uint8_t twoThirdsBottom[8] = {
  0b00001000
, 0b00000100
, 0b00000010
, 0b00000001
, 0b00100010
, 0b01001001
, 0b01001001
, 0b00110110
};

void setup() {
  pinMode(stopPin, INPUT_PULLUP);
  pinMode(ssrPin, OUTPUT);
  pinMode(ledPwmPin, OUTPUT);

  theButton.begin(theButtonPin, onButtonPress);
  Serial.begin(115200);
//  u8x8.setI2CAddress(0x7a); // This would set a 2nd Display to a dedictaed I2C Adress
//  u8x8.setBusClock(800000); // overclocking doesn't seem to be necessary yet
  u8x8.begin();
  u8x8.setFont(u8x8_font_courB18_2x3_n);
  u8x8.setCursor(2,3);
  u8x8.print(":  :");

  u8x8.drawTile(12,7,2,lockBottom);
  unlockLock();

  FreqMeasure.begin();  // This should go into the state machine and only start once the viewer is not stopped
}

long prevFrameCount  = -999; // Magic Number to make sure we immediately draw a frame count
double freqSum=0;
int freqCount=0;
float frequency;
unsigned int currentFilmSecond; // Good for films up to 18 hours, should be enough 

float prevPaintedFrequency = -999;


void loop() {
  theButton.check();
  if (digitalRead(stopPin) == LOW) onStopSignal();
  
  currentFrameCount = myEnc.read() / 4;
  Serial.println(currentFrameCount);
  if (currentFrameCount != prevFrameCount) {
    prevFrameCount = currentFrameCount;
    drawCurrentTime();
  }

  if (FreqMeasure.available()) {
    // average several reading together
    freqSum = freqSum + FreqMeasure.read();
    freqCount = freqCount + 1;
    if (freqCount > 9) {
      frequency = FreqMeasure.countToFrequency(freqSum / freqCount);
      freqSum = 0;
      freqCount = 0;
    }
  }

  if (frequency != prevPaintedFrequency) drawCurrentFrequency();
}


void onButtonPress(const byte newState, const unsigned long interval, const byte whichPin) {
//  if (interval >= 1000) return; // This would detect a long press
  if (newState == LOW && interval < 1500) {
    Serial.println("The Button has been pressed!");
    if (buttonState == BTN_STATE_UNLOCKED) lockLock();
    else buttonState++;
    if (buttonState >= BTN_STATE_9) drawCurrentFps();
  } else if (newState == LOW && interval >= 1500) {
    
  }
}

void onStopSignal() {
  u8x8.setFont(u8x8_font_7x14B_1x2_r);
  u8x8.setCursor(1,6);
  u8x8.print("shot @     fps ");
  if (buttonState == BTN_STATE_UNLOCKED) buttonState = BTN_STATE_LOCKED;
  stopped = true;
}

void drawCurrentFps() {
  u8x8.setFont(u8x8_font_7x14B_1x2_n);
  
  if (stopped) {
    u8x8.setCursor(10,6);
    u8x8.print(" ");
    u8x8.setCursor(8,6);
  } else {
    u8x8.setCursor(2,6);
    u8x8.print("     ");
    u8x8.setCursor(2,6);
  }
  
  switch (buttonState) {
    default:
      buttonState = BTN_STATE_9;
    case BTN_STATE_9:
      u8x8.print(" 9");
      return;
    case BTN_STATE_16_2_3:
      u8x8.print(16);
      u8x8.drawTile(((stopped) ? 10 : 5),6,1,twoThirdsTop);
      u8x8.drawTile(((stopped) ? 10 : 5),7,1,twoThirdsBottom);
      return;
    case BTN_STATE_18:
      u8x8.print(18);
      return;
    case BTN_STATE_24:
      u8x8.print(24);
      return;
    case BTN_STATE_25:
      u8x8.print(25);
  }
}

void drawCurrentFrequency() {
  prevPaintedFrequency = frequency;

  u8x8.setFont(u8x8_font_7x14B_1x2_r);
  u8x8.setCursor(2,6);
  u8x8.print(frequency);
  u8x8.print(" fps");
}

void drawCurrentTime() {
  u8x8.setFont(u8x8_font_courB18_2x3_n);
  
  currentFilmSecond = abs(currentFrameCount) / filmFps ;
  currentSubFrame = currentFrameCount % int(filmFps - 1); 
  hours   = numberOfHours(currentFilmSecond);
  minutes = numberOfMinutes(currentFilmSecond);
  seconds = numberOfSeconds(currentFilmSecond);
  
// Only paint the glyphs that have changed, this improves the display framerate a lot
//
  rightSecDigit = seconds % 10;
  if (rightSecDigit != prevPaintedRightSecDigit) {
    prevPaintedRightSecDigit = rightSecDigit;
    u8x8.setCursor(12 + (sign ? 2 : 0),3);
    u8x8.print(rightSecDigit);
    leftSecDigit = seconds / 10;
    if (leftSecDigit != prevPaintedLeftSecDigit) {
      prevPaintedLeftSecDigit = leftSecDigit;
      u8x8.setCursor(10 + (sign ? 2 : 0),3);
      u8x8.print(leftSecDigit);
      rightMinDigit = minutes % 10;
      if (rightMinDigit != prevPaintedRightMinDigit) {
        prevPaintedRightMinDigit = rightMinDigit;
        u8x8.setCursor(6 + (sign ? 2 : 0),3);
        u8x8.print(rightMinDigit);
        leftMinDigit = minutes / 10;
        if (leftMinDigit != prevPaintedLeftMinDigit) {
          prevPaintedLeftMinDigit = leftMinDigit;
          u8x8.setCursor(4 + (sign ? 2 : 0),3);
          u8x8.print(leftMinDigit);
          hourDigit = hours % 10;
          if (hourDigit != prevPaintedHourDigit) {
            prevPaintedHourDigit = hourDigit;
            u8x8.setCursor(0 + (sign ? 2 : 0),3);
            u8x8.print(hourDigit);
          }
        }
      }
    }
  }
  
  // Handle negative frame counts and time nicely
  //
  if (rightSecDigit == 0) {
    if (currentFrameCount < 0) sign = true;
    else sign = false;
    if (sign != prevPaintedSign) {
      prevPaintedSign = sign;
      
      if (sign) {
        // when tc is negative, do not render sub frame cont, but a leading minus sign
        u8x8.setCursor(0,3);
        u8x8.print("-");
        u8x8.setCursor(0,3);
        u8x8.print("-0:00:00");
      } else {
        u8x8.setCursor(0,3);
        u8x8.print(" ");
        u8x8.setCursor(0,3);
        u8x8.print("0:00:00 ");
      }
    }
  } 
  if (currentFrameCount != 0) u8x8.setCursor(16 - (int(log10(abs(currentFrameCount)) + (sign ? 3 : 2)) << 1),0); // Kalle magic!
  else u8x8.setCursor(12,0);
  u8x8.print(" ");
  u8x8.print(currentFrameCount);

  // Print current Subframe for SMPTE compliance
  //
  if (currentFrameCount >= 0) {
    u8x8.setFont(u8x8_font_7x14B_1x2_n);
    u8x8.setCursor(14,3);
    if (currentSubFrame < 10) u8x8.print("0");
    u8x8.print(currentSubFrame);
  } 
 
}

void lockLock() {
  buttonState = BTN_STATE_LOCKED;
  u8x8.drawTile(12,6,2,lockedLockTop);
  u8x8.setCursor(14,6);
  u8x8.setFont(u8x8_font_7x14B_1x2_n);
  u8x8.print(" ");
  FreqMeasure.end();
}

void unlockLock() {
  buttonState = BTN_STATE_UNLOCKED;
  u8x8.drawTile(12,6,3,unlockedLockTop);
}
