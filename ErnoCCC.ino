/*
 * ***** Erno Crystal Counter Controller written in 2019, 2020 by Friedemann and Kalle Wachsmuth *****
 * 
 * ErnoCCC – a modern controller for Erno Motor film viewers, is intended to work in Erno EM-1801 or Goko MM-1 Motorized Film Viewers. 
 * 
 * To Do: 
 * - Create Timers for the reference pulses
 * - Control the DC motor (via DAC/FET?)
 * - Write Controller Calibration routine
 * - Exit RESETCOUNTER after timeout OR when the impulse counter changes
 * - Control the Power LED (Stopped Mode)
 * - Store last used shot/playback FPS in EEPROM (after n seconds or Mode change)
 * - 
 * 
 * Future Thoughts:
 * - Take over DC Motor entirely (H-Bridge, mechanical selenoid clutch)
 * - Synchronuous Mode via Impulses
 * - Add an accelerated rotary Encoder to target arbitrary film positions
 * - support electronic cutting lists to set all the marks
 * 
 * Notes: 
 * 
 * - D5 Pullup
 * - D9 STOP auswerten
 * - LED Ports down
 * - 5 LEDs ansteuern
 * - D6 SSR ansteuern
 * - Button an D4 abfragen
 * - D5 LED PWM
 * - D10 for a Mosfet or PWM to the LDR-LED?
 * 
 */

#include <Encoder.h>
#include <FreqMeasure.h>

#include <Arduino.h>
#include <U8x8lib.h>          // Display Driver
#include <SwitchManager.h>    // Button Handling and Debouncing, http://www.gammon.com.au/switches

// Instantiate some Objects
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
#define numberOfHours(_time_) ((_time_ % SECS_PER_DAY) / SECS_PER_HOUR)

// ---- Running screen state machine --------------------------------------
//
#define FPS_UNLOCKED     0

// ---- Multiple state machines (^) -----------------------
//
#define FPS_18           1
#define FPS_24           2
#define FPS_25           3
#define FPS_9            4
#define FPS_16_2_3       5

uint8_t fpsState       = 0;
uint8_t prevPaintedFps = 0;

// ---- Display state machine ----------------------------------
//
#define STATE_STOPPED     0     // 0x00       1st hexadecimal number = is running
#define STATE_RESET       1     // 0x01       state >> 4             = is running
#define STATE_RUNNING     16    // 0x10

#define isRunning         (state >> 4)
#define wasCounterVisible (prevPaintedState % 16 == 0)

uint8_t state            = 0;
uint8_t prevPaintedState = 99;

bool ignoreNextButtonPress   = false;
unsigned long requiredMillis = 0;
bool checkMillisInLoop       = 0;
unsigned long totalSeconds   = 0;
long currentFrameCount;
float filmFps                = 18;
float playbackFps            = 18;
uint8_t hours                = 0;
uint8_t minutes              = 0;
uint8_t seconds              = 0;
uint8_t currentSubFrame      = 0;

uint8_t rightSecDigit = 0;
uint8_t leftSecDigit  = 0;
uint8_t rightMinDigit = 0;
uint8_t leftMinDigit  = 0;
uint8_t hourDigit     = 0;
bool sign             = false;

uint8_t prevPaintedRightSecDigit = 99;
uint8_t prevPaintedLeftSecDigit  = 99;
uint8_t prevPaintedRightMinDigit = 99;
uint8_t prevPaintedLeftMinDigit  = 99;
uint8_t prevPaintedHourDigit     = 99;
bool prevPaintedSign             = false;

// Make some nice pixel gfx
//
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
const uint8_t emptyTile[8] = {0,0,0,0,0,0,0,0};

void setup() {
  pinMode(stopPin, INPUT_PULLUP);
  pinMode(ssrPin, OUTPUT);
  pinMode(ledPwmPin, OUTPUT);

  theButton.begin(theButtonPin, onButtonPress);
  Serial.begin(115200);
  u8x8.begin();
  //u8x8.setI2CAddress(0x7a); // This would set a 2nd Display to a dedictaed I2C Adress
  //u8x8.setBusClock(800000); // overclocking doesn't seem to be necessary yet
  drawState();
}

long prevFrameCount  = -999; // Magic Number to make sure we immediately draw a frame count
double freqSum=0;
int freqCount=0;
float frequency;
unsigned int currentFilmSecond; // Good for films up to 18 hours, should be enough 

float prevPaintedFrequency = -999;


void loop() {
  theButton.check();
  if (digitalRead(stopPin) == LOW && isRunning == 1) {   // Use MSBs to determine the run state
    state = STATE_STOPPED;
    drawState();
  } else if (digitalRead(stopPin) == HIGH && isRunning == 0) {
    state = STATE_RUNNING;
    drawState();
  }
  
  currentFrameCount = myEnc.read() / 4;
  if (currentFrameCount != prevFrameCount) {
    if (wasCounterVisible) {
      prevFrameCount = currentFrameCount;
      drawCurrentTime(false);
    } else {
      state = STATE_STOPPED;
      drawState();
    }
  }
  
  if (checkMillisInLoop) {
    switch (state) {
      case STATE_STOPPED:
        if (digitalRead(theButtonPin) == HIGH) checkMillisInLoop = false;
        else if (digitalRead(theButtonPin) == LOW && millis() >= requiredMillis) {
          checkMillisInLoop = false;
          state = STATE_RESET;
          drawState();
        }
        break;
      case STATE_RESET:
        if (millis() >= requiredMillis) {
          checkMillisInLoop = false;
          state = STATE_STOPPED;
          drawState();
        }
        break;
      default:
        break;
    }
  }

  if (FreqMeasure.available()) {
    // average several reading together
    freqSum = freqSum + FreqMeasure.read();
    freqCount++;
    if (freqCount > 9) {
      frequency = FreqMeasure.countToFrequency(freqSum / freqCount);
      freqSum = 0;
      freqCount = 0;
    }
    if (frequency != prevPaintedFrequency && state == STATE_RUNNING) drawCurrentCustomFrequency();
  }
}

void drawState() {
  checkMillisInLoop = false;
  if (!wasCounterVisible) {
    u8x8.clearDisplay();
    drawCurrentTime(true);
  }
  switch (state) {
    default:
      notFound();
      break;
    case STATE_STOPPED:
      if (prevPaintedState == STATE_RESET) ignoreNextButtonPress = true;
      FreqMeasure.end();
      u8x8.setFont(u8x8_font_7x14B_1x2_r);
      u8x8.setCursor(1,6);
      u8x8.print(F("shot @     fps"));
      switch (int(filmFps)) {
        case 18:
          fpsState = FPS_18;
          break;
        case 24:
          fpsState = FPS_24;
          break;
        case 25:
          fpsState = FPS_25;
          break;
        case 9:
          fpsState = FPS_9;
          break;
        case 16:
          fpsState = FPS_16_2_3;
      }
      drawCurrentFps(false);
      break;
    case STATE_RESET:
      u8x8.clearDisplay();
      u8x8.setFont(u8x8_font_courB18_2x3_r);
      u8x8.setCursor(3,0);
      u8x8.print(F("Reset"));
      u8x8.setCursor(0,3);
      u8x8.print(F("counter?"));
      u8x8.setFont(u8x8_font_7x14B_1x2_r);
      u8x8.setCursor(1,6);
      u8x8.print(F("push = confirm"));
      requiredMillis = millis() + 4000;
      checkMillisInLoop = true;
      break;
    case STATE_RUNNING:
      checkMillisInLoop = false;
      u8x8.setFont(u8x8_font_7x14B_1x2_r);
      u8x8.setCursor(8,6);
      u8x8.print("fps");
      u8x8.setCursor(1,6);
      u8x8.print(" ");
      u8x8.drawTile(14,7,1,emptyTile);
      u8x8.drawTile(12,7,2,lockBottom);
      u8x8.drawTile(12,6,3,unlockedLockTop);
      fpsState = FPS_UNLOCKED;
      FreqMeasure.begin();
      drawCurrentCustomFrequency();
  }
  prevPaintedState = state;
}

void onButtonPress(const byte newState, const unsigned long interval, const byte whichPin) {
  // newState will be LOW or HIGH (the is the state the switch is now in)
  // interval will be how many ms between the opposite state and this one
  // whichPin will be which pin caused this change (so you can share the function amongst multiple switches)

  if (newState == HIGH && interval < 1500) { //on button release
    if (!ignoreNextButtonPress) {
      switch (state) {
        case STATE_RUNNING:
          fpsState++;
          if (fpsState == FPS_UNLOCKED + 1) {
            u8x8.drawTile(12,6,2,lockedLockTop);
            u8x8.drawTile(14,6,1,emptyTile);
            u8x8.setFont(u8x8_font_7x14B_1x2_n);
            u8x8.setCursor(5,6);
            u8x8.print("  ");
            FreqMeasure.end();
          }
          drawCurrentFps(false);
          break;
        case STATE_STOPPED:
          fpsState++;
          drawCurrentFps(true);
        default:
          break;
      }
    } else ignoreNextButtonPress = false; 
  } else if (newState == LOW) { //on button press
    switch (state) {
      case STATE_STOPPED:
        requiredMillis = millis() + 1500;
        checkMillisInLoop = true;
        break;
      case STATE_RESET:
        myEnc.write(0);
        currentFrameCount = 0;
        state = STATE_STOPPED;
        drawState();
    }
  }
}

void drawCurrentFps(bool redrawCurrentTime) {
  float fps = 18;
  float prevFilmFps = filmFps;
  
  u8x8.setFont(u8x8_font_7x14B_1x2_n);
  u8x8.setCursor(((state == STATE_RUNNING) ? 4 : 10),6);
  u8x8.print(" ");
  u8x8.setCursor(((state == STATE_RUNNING) ? 2 : 8),6);
  
  switch (fpsState) {
    default:
      fpsState = FPS_18;
    case FPS_18:
      u8x8.print(18);
      break;
    case FPS_24:
      fps = 24;
      u8x8.print(24);
      break;
    case FPS_25:
      fps = 25;
      u8x8.print(25);
      break;
    case FPS_9:
      fps = 9;
      u8x8.print(" 9");
      break;
    case FPS_16_2_3:
      fps = 50/3;
      u8x8.print(16);
      u8x8.drawTile(((state == STATE_RUNNING) ? 4 : 10),6,1,twoThirdsTop);
      u8x8.drawTile(((state == STATE_RUNNING) ? 4 : 10),7,1,twoThirdsBottom);
  }
  if (state == STATE_RUNNING) playbackFps = fps;
  else {
    filmFps = fps;
    if (redrawCurrentTime && (currentFrameCount >= filmFps || currentFrameCount >= prevFilmFps || prevFilmFps == 9 || filmFps == 9)) drawCurrentTime(true);
  }
  prevPaintedFps = fpsState;
}

void drawCurrentCustomFrequency() {
  prevPaintedFrequency = frequency;

  u8x8.setFont(u8x8_font_7x14B_1x2_n);
  u8x8.setCursor(2,6);
  if (frequency < 10) u8x8.print(" ");
  u8x8.print(frequency);
}

void drawCurrentTime(bool forceFullRedraw) {
  u8x8.setFont(u8x8_font_courB18_2x3_n);
  
  if (filmFps != 50/3) {
    currentSubFrame = currentFrameCount % int(filmFps);
    currentFilmSecond = abs(currentFrameCount) / filmFps;
  } else {               // for 16 2/3 fps (Kalle magic)
      currentSubFrame = currentFrameCount % 50;
      currentFilmSecond = (abs(currentFrameCount) - currentSubFrame) * 3/50;
    if (currentFrameCount % 50 >= 16) {
      currentSubFrame = (currentSubFrame - 16) % 17;
      currentFilmSecond++;
      if (currentFrameCount % 50 > 32) currentFilmSecond++;
    }
  }

  hours   = numberOfHours(currentFilmSecond);
  minutes = numberOfMinutes(currentFilmSecond);
  seconds = numberOfSeconds(currentFilmSecond);
  rightSecDigit = seconds % 10;

  // Handle negative frame counts and time nicely
  //
  if (rightSecDigit == 0 || forceFullRedraw) {
    if (currentFrameCount < 0) sign = true;
    else sign = false;
    if (sign != prevPaintedSign || forceFullRedraw) {
      forceFullRedraw = true;
      prevPaintedSign = sign;
      u8x8.setCursor(((sign) ? 4 : 2),3);
      u8x8.print(F(":  :  -"));    // when tc is negative, do not render sub frame count, but a leading minus sign
      if (!sign && filmFps == 9) {
        u8x8.drawTile(15,3,1,emptyTile);
        u8x8.drawTile(15,4,1,emptyTile);
      }
    }
  }

  // Only paint the glyphs that have changed, this improves the display framerate a lot
  //
  if (rightSecDigit != prevPaintedRightSecDigit || forceFullRedraw) {
    prevPaintedRightSecDigit = rightSecDigit;
    u8x8.setCursor(12 + (sign ? 2 : 0),3);
    u8x8.print(rightSecDigit);
    leftSecDigit = seconds / 10;
    if (leftSecDigit != prevPaintedLeftSecDigit || forceFullRedraw) {
      prevPaintedLeftSecDigit = leftSecDigit;
      u8x8.setCursor(10 + (sign ? 2 : 0),3);
      u8x8.print(leftSecDigit);
      rightMinDigit = minutes % 10;
      if (rightMinDigit != prevPaintedRightMinDigit || forceFullRedraw) {
        prevPaintedRightMinDigit = rightMinDigit;
        u8x8.setCursor(6 + (sign ? 2 : 0),3);
        u8x8.print(rightMinDigit);
        leftMinDigit = minutes / 10;
        if (leftMinDigit != prevPaintedLeftMinDigit || forceFullRedraw) {
          prevPaintedLeftMinDigit = leftMinDigit;
          u8x8.setCursor(4 + (sign ? 2 : 0),3);
          u8x8.print(leftMinDigit);
          hourDigit = hours % 10;
          if (hourDigit != prevPaintedHourDigit || forceFullRedraw) {
            prevPaintedHourDigit = hourDigit;
            u8x8.setCursor(0 + (sign ? 2 : 0),3);
            u8x8.print(hourDigit);
          }
        }
      }
    }
  }
  if (!forceFullRedraw) {
    if (currentFrameCount != 0) u8x8.setCursor(16 - (int(log10(abs(currentFrameCount)) + (sign ? 3 : 2)) << 1),0); // Kalle magic!
    else u8x8.setCursor(12,0);
    u8x8.print(" ");
    u8x8.print(currentFrameCount);
  }

  // Print current Subframe for SMPTE compliance
  //
  if (currentFrameCount >= 0) {
    u8x8.setFont(u8x8_font_7x14B_1x2_n);
    u8x8.setCursor(14,3);
    if (currentSubFrame < 10 && filmFps != 9) u8x8.print(0);
    u8x8.print(currentSubFrame);
  } 
 
}

void notFound() {
  u8x8.home();
  u8x8.print(404);
}
