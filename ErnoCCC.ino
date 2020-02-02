/*
 * ***** Erno Crystal Counter Controller written in 2019, 2020 by Friedemann and Kalle Wachsmuth *****
 * 
 * ErnoCCC – a modern controller for Erno Motor film viewers, is intended to work in Erno EM-1801 or Goko MM-1 Motorized Film Viewers. 
 * 
 * To Do: 
 * - Do not devide Encoder Impulses and take both edges; recalculate Timer Dividers -> 4x faster controlling!
 * - Turn on LED during Claibration
 * - Flash LED after Claibration
 * - Draw some Display STuff when Calibrating
 * - Take the smallest diff value once testVoltage keeps repeating
 * - Fix Kalles 16 fps bug
 * - Poll Button in Setup()
 * - Find out decent LED current
 * - Create Timers for the 9 reference pulse
 * - Write Controller Calibration routine
 * - Exit RESETCOUNTER after timeout OR when the impulse counter changes
 * 
 * - Control the Power LED (Stopped Mode)
  
 * 
 * Future Thoughts:
 * - Take over DC Motor entirely (H-Bridge, mechanical selenoid clutch)
 * - Synchronuous Mode via Impulses
 * - Add an accelerated rotary Encoder to target arbitrary film positions
 * - support electronic cutting lists to set all the marks
 * 
 * PCB (Rev.B):
 * - R9/10 falsche Werte bei Mouser
 * 
 * Notes: 
 * 
 * - D5 Pullup (Pulldown?)
 * - LED Ports down
 * - 5 LEDs ansteuern
 * - D6 SSR ansteuern
 * - D5 LED PWM
 * 
 */

#include <Wire.h>
#include <Encoder.h>
#include <FreqMeasure.h>

#include <Arduino.h>
#include <EEPROM.h>
#include <U8x8lib.h>          // Display Driver
#include <Adafruit_MCP4725.h> // Fancy DAC for voltage control

#include <SwitchManager.h>    // Button Handling and Debouncing, http://www.gammon.com.au/switches

// Instantiate some Objects
//
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);   
Encoder myEnc(2, 3);
Adafruit_MCP4725 dac;                                   // Instantiate the DAC
SwitchManager theButton;

// ---- Define uC Pins ------------------------------------
//
#define impDetectorPin    3  // Here goes the encoder impulses
#define theButtonPin      4
#define ssrPin            6
#define ledPwmPin         5

#define ledSlowerRed      A0  //  --
#define ledSlowerYellow   A1  //  -
#define ledGreen          A2  //  o
#define ledFasterYellow   A3  //  +
#define ledFasterRed      7  //  ++



// ---- Define useful time constants and macros ------------------------------------
//
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)  
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN) 
#define numberOfHours(_time_) ((_time_ % SECS_PER_DAY) / SECS_PER_HOUR)


// Timer Variables

int timerFactor = 0;      // this is used for the Timer1 postscaler, since multiples of 18 and 24 Hz give better accuracy
volatile int timerDivider = 0; // For Modulo in the ISR
volatile unsigned long timerFrames = 0;

// Projector Variables

byte segmentCount = 4;    // What kind of Shutter Blade do we have?
volatile int projectorDivider = 0; // For Modulo in the ISR
volatile unsigned long projectorFrames = 0;


// Other variables
byte selectedSpeed;       // Takes 16, 16 2/3, 18, 24 or 25 fps right now
unsigned long millisNow = 0;
unsigned long lastMillis = 0;
long frameDifference = 0;

int lastCorrection = 0;


// Global Vars for Motor Control Voltage Calibration  
float motorVoltageScaleFactor = -78.00; // Final Factor and Offset are calcuclated during Calibration
float motorVoltageOffset = 4100.00;      
const float hiFpsTestFreq = 25.00; 
const float loFpsTestFreq = 9.00;




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


long prevFrameCount  = -999; // Magic Number to make sure we immediately draw a frame count
double freqSum=0;
int freqCount=0;
float frequency;
unsigned int currentFilmSecond; // Good for films up to 18 hours, should be enough 

float prevPaintedFrequency = -999;

uint8_t state            = 0;
uint8_t lastState       = 0;
uint8_t prevPaintedState = 99;

bool ignoreNextButtonPress     = false;
unsigned long requiredMillis   = 0;
unsigned long writeToEEPROM    = 0;
unsigned long lastEncoderPos;
unsigned long lastEncoderChangeTs;
bool checkRequiredMillisInLoop = false;
bool checkWriteToEEPROMInLoop  = false;
unsigned long totalSeconds     = 0;
long currentFrameCount         = 0;
float filmFps                  = 18;
float playbackFps              = 18;
float fps                      = 18;
uint8_t hours                  = 0;
uint8_t minutes                = 0;
uint8_t seconds                = 0;
uint8_t currentSubFrame        = 0;

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
  dac.begin(0x60);
  pinMode(ssrPin, OUTPUT);
  pinMode(ledPwmPin, OUTPUT);

  pinMode(ledSlowerRed, OUTPUT);     //  --
  pinMode(ledSlowerYellow, OUTPUT);  //  -
  pinMode(ledGreen, OUTPUT);         //  o
  pinMode(ledFasterYellow, OUTPUT);  //  +
  pinMode(ledFasterRed, OUTPUT);     //  ++
  

  analogWrite(ledPwmPin, 1);


  theButton.begin(theButtonPin, onButtonPress);
  Serial.begin(115200);
  
  u8x8.begin();
  
  //u8x8.setI2CAddress(0x7a); // This would set a 2nd Display to a dedictaed I2C Adress
  //u8x8.setBusClock(800000); // overclocking doesn't seem to be necessary yet

  filmFps = readEEPROMValue(0);

  playbackFps = readEEPROMValue(1);
  
  drawState();  

  // check if we need to calibrate Voltages
  //
  if (digitalRead(theButtonPin) == HIGH) { // SET BACK TO LOW
    Serial.println(F("In Setup"));
    calibrateCtrlVoltage();
  }
}

int calculateVoltageForFPS(byte fps) {
  // Factor and Offset are calcuclated during Calibration.
  //
  return fps * motorVoltageScaleFactor + motorVoltageOffset;
}

void calibrateCtrlVoltage() {
  digitalWrite(ssrPin, HIGH);
  float actualFrequency;
  int lastTooFast = 0;
  int lastTooSlow = 4095;
  int testVoltage = 500;
  int prevTestVoltage;
  int hiSpeedVoltage;
  int loSpeedVoltage;

  /*
  0    = fast
  4095 = slow

  voltage = ((Vhi - Vlo) / (HF - LF)) * desiredFPS + (Vhi - HF * ((Vhi - Vlo) / (HF - LF))
            _____________A___________                            _____________A___________ 
                                                      _________________B__________________ 
  */

  FreqMeasure.begin();
  
  Serial.println(F("Searching 25 fps Ctrl Value in 600 Measurements: "));
  do {
    dac.setVoltage(testVoltage, false);
    while (freqCount <= 600) { 
      if (FreqMeasure.available()) {
        // average several reading together
        freqSum = freqSum + FreqMeasure.read();
        freqCount++;
      }
    }
    actualFrequency = (FreqMeasure.countToFrequency(freqSum / freqCount) / 2.00 );
    freqSum = 0;
    freqCount = 0;
    Serial.print(F("DAC Wert:"));
    Serial.print(testVoltage);
    Serial.print(F(" - gemessene Frequenz: "));
    Serial.println(actualFrequency);
    if (abs(actualFrequency - hiFpsTestFreq) < 0.1) break;
    if (testVoltage == prevTestVoltage) break;
    prevTestVoltage = testVoltage;   
    if (actualFrequency > hiFpsTestFreq) { // too fast
      lastTooFast = testVoltage;
      testVoltage = (testVoltage + lastTooSlow) / 2;
    }
    if (actualFrequency < hiFpsTestFreq) { // too slow
      lastTooSlow = testVoltage;
      testVoltage = (testVoltage + lastTooFast) / 2;
    }
  } while (true);
  hiSpeedVoltage = testVoltage;
  Serial.print(F("Hi-Speed Voltage Found: "));
  Serial.println(hiSpeedVoltage);
  Serial.println(F("******"));

  // prepare to determine the low speed ctrl voltage
  
  lastTooFast = 0;
  lastTooSlow = 4095;
  testVoltage = 3000;
  Serial.println(F("Searching 9 fps Ctrl Value in 200 Measurements: "));
  do {
    dac.setVoltage(testVoltage, false);
    while (freqCount <= 200) { 
      if (FreqMeasure.available()) {
        // average several reading together
        freqSum = freqSum + FreqMeasure.read();
        freqCount++;
      }
    }
    actualFrequency = (FreqMeasure.countToFrequency(freqSum / freqCount) / 2.00 );
    freqSum = 0;
    freqCount = 0;
    Serial.print(F("DAC Wert:"));
    Serial.print(testVoltage);
    Serial.print(F(" - gemessene Frequenz: "));
    Serial.println(actualFrequency);
    if (abs(actualFrequency - loFpsTestFreq) < 0.1) break;
    if (testVoltage == prevTestVoltage) break;
    prevTestVoltage = testVoltage;   
    if (actualFrequency > loFpsTestFreq) { // too fast
      lastTooFast = testVoltage;
      testVoltage = (testVoltage + lastTooSlow) / 2;
    }
    if (actualFrequency < loFpsTestFreq) { // too slow
      lastTooSlow = testVoltage;
      testVoltage = (testVoltage + lastTooFast) / 2;
    }
  } while (true);
  loSpeedVoltage = testVoltage;
  Serial.print(F("Lo-Speed Voltage Found: "));
  Serial.println(loSpeedVoltage);
  Serial.println(F("******"));

  motorVoltageScaleFactor = (hiSpeedVoltage - loSpeedVoltage) / (hiFpsTestFreq - loFpsTestFreq) ; 
  motorVoltageOffset = hiSpeedVoltage - hiFpsTestFreq * motorVoltageScaleFactor;      
  Serial.print(F("A: "));
  Serial.println(motorVoltageScaleFactor);
  Serial.print(F("B: "));
  Serial.println(motorVoltageOffset);
  // write testVoltage to EEPROM and global vars
    
    
  FreqMeasure.end;
}

// ******************************************************************************
void loop() {

  // Lets contrl the Motor!
  //
  millisNow = millis();

  frameDifference = timerFrames - projectorFrames;
  controlProjector(frameDifference);

  if ((millisNow % 1000 == 0) && (lastMillis != millisNow)) {
    Serial.print("Timer: ");
    Serial.print(timerFrames);
    Serial.print(", Projektor: ");
    Serial.print(projectorFrames);
    Serial.print(", Differenz ***: ");
    Serial.println(frameDifference);
    lastMillis = millisNow;
  }  

  
  // determine if the viewer is running or not, since we actually do not have a Stop Pin anymore 
  //
  if (myEnc.read() != lastEncoderPos) {
    state = STATE_RUNNING;
    lastEncoderChangeTs = millis();
    lastEncoderPos = myEnc.read();
  } else {
    if (millis() - lastEncoderChangeTs > 100) {
      state = STATE_STOPPED;
      lastEncoderPos = myEnc.read();
    }
  }

  theButton.check();
  if (state != lastState) {
    drawState();
    lastState = state;
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
  
  if (checkRequiredMillisInLoop) {
    switch (state) {
      case STATE_STOPPED:
        if (digitalRead(theButtonPin) == HIGH) checkRequiredMillisInLoop = false;
        else if (digitalRead(theButtonPin) == LOW && millis() >= requiredMillis) {
          checkRequiredMillisInLoop = false;
          state = STATE_RESET;
          drawState();
        }
        break;
      case STATE_RESET:
        if (millis() >= requiredMillis) {
          checkRequiredMillisInLoop = false;
          state = STATE_STOPPED;
          drawState();
        }
        break;
      default:
        break;
    }
  }

  if (checkWriteToEEPROMInLoop && millis() >= writeToEEPROM) {
    checkWriteToEEPROMInLoop = false;
    Serial.println(F("Writing to EEPROM"));
    EEPROM.write(isRunning, fps);
  }

  if (FreqMeasure.available()) {
    // average several reading together
    freqSum = freqSum + FreqMeasure.read();
    freqCount++;
    if (freqCount > 30) {
      frequency = (FreqMeasure.countToFrequency(freqSum / freqCount) / 2 );
      freqSum = 0;
      freqCount = 0;
    }
    if (frequency != prevPaintedFrequency && state == STATE_RUNNING) drawCurrentCustomFrequency();
  }
}

// End Loop -----------------------------------------------

ISR(TIMER1_COMPA_vect) {
  if (timerDivider == 0) {
    //    digitalWrite(ledPin, digitalRead(ledPin) ^ 1);
    //    digitalWrite(ledPin, HIGH);
    //    digitalWrite(ledPin, LOW);
    timerFrames++;
  }
  timerDivider++;
  timerDivider %= timerFactor;

}

void projectorCountISR() {
  if (projectorDivider == 0) {
    projectorFrames++;
  }
  projectorDivider++;
  projectorDivider %= (segmentCount * 2); // we are triggering on CHANGE
}

void setLeds(int bargraph) {
  switch(bargraph) {
    case -2:
      digitalWrite(ledSlowerRed, HIGH);
      digitalWrite(ledSlowerYellow, LOW);
      digitalWrite(ledGreen, LOW);
      digitalWrite(ledFasterYellow, LOW);
      digitalWrite(ledFasterRed, LOW);
      break;
    case -1:  
      digitalWrite(ledSlowerRed, LOW);
      digitalWrite(ledSlowerYellow, HIGH);
      digitalWrite(ledGreen, LOW);
      digitalWrite(ledFasterYellow, LOW);
      digitalWrite(ledFasterRed, LOW);
      break;
    case 0:
      digitalWrite(ledSlowerRed, LOW);
      digitalWrite(ledSlowerYellow, LOW);
      digitalWrite(ledGreen, HIGH);
      digitalWrite(ledFasterYellow, LOW);
      digitalWrite(ledFasterRed, LOW);
      break;
    case 1:
      digitalWrite(ledSlowerRed, LOW);
      digitalWrite(ledSlowerYellow, LOW);
      digitalWrite(ledGreen, LOW);
      digitalWrite(ledFasterYellow, HIGH);
      digitalWrite(ledFasterRed, LOW);
      break;
    case 2:
      digitalWrite(ledSlowerRed, LOW);
      digitalWrite(ledSlowerYellow, LOW);
      digitalWrite(ledGreen, LOW);
      digitalWrite(ledFasterYellow, LOW);
      digitalWrite(ledFasterRed, HIGH);
      break;
    default:
      break;

  }
}


void controlProjector(int correction) {
  if (correction != lastCorrection) {
    Serial.print(F("Frmaes off: "));
    Serial.println(correction);
    if (correction <= -2) {
      setLeds(-2);
      dac.setVoltage(2300, false);
    } else if (correction == -1) {
      setLeds(-1);
      dac.setVoltage(2200, false);
    } else if (correction == 0) {
      setLeds(0);
      dac.setVoltage(2100, false);
    } else if (correction == 1) {
      setLeds(1);
      dac.setVoltage(2000, false);  
    } else if (correction >= 2) {
      setLeds(2);
      dac.setVoltage(1900, false);
    }
    lastCorrection = correction;
  }
}

void stopTimer1() {
  // TCCR1B &= ~(1 << CS11);
  noInterrupts();
  TIMSK1 &= ~(1 << OCIE1A);
  interrupts();
}

bool setupTimer1forFps(byte sollfps) {
  // start with a new sync point, no need to catch up differences from before.
  timerFrames = 0;
  projectorFrames = 0;
  timerDivider = 0;
  
  if (sollfps == 16 || sollfps == 1666 || sollfps ==  18 || sollfps == 24 || sollfps == 25) {
    noInterrupts();
    // Clear registers
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    // CTC
    TCCR1B |= (1 << WGM12);

    switch (sollfps) {
      case 16:
        OCR1A = 15624;    // 16 Hz (16000000/((15624+1)*64))
        TCCR1B |= (1 << CS11) | (1 << CS10); // Prescaler 64
        timerFactor = 1;

        break;
      case 1666:
        OCR1A = 14999;    // 16 2/3 Hz (16000000/((14999+1)*64))
        TCCR1B |= (1 << CS11) | (1 << CS10); // Prescaler 64
        timerFactor = 1;

        break;
      case 18:
        OCR1A = 10100;    // 198.000198000198 Hz (16000000/((10100+1)*8)),
        //              divided by 11 is 18.000018.. Hz
        //              or 18 2/111,111
        //              or 2,000,000/111,111
        //
        TCCR1B |= (1 << CS11);  // Prescaler 8
        timerFactor = 11;

        break;
      case 24:
        OCR1A = 60605;    // 264.000264000264 Hz (16000000/((60605+1)*1)),
        //               divided by 11 is 24.000024.. Hz
        //               or 24 8/333,333
        //               or 8,000,000 / 333,333
        //
        TCCR1B |= (1 << CS10);  // Prescaler 1
        timerFactor = 11;

        break;
      case 25:
        OCR1A = 624;      // 25 Hz (16000000/((624+1)*1024))
        TCCR1B |= (1 << CS12) | (1 << CS10); // Prescaler 1024
        timerFactor = 1;

        break;
      default:
        break;
    }
    // Output Compare Match A Interrupt Enable
    TIMSK1 |= (1 << OCIE1A);
    interrupts();
  } else {
    // invalid fps requested
    Serial.println(F("Invalid FPS request"));
    return false;
  }
}

void drawState() {
  checkRequiredMillisInLoop = false;
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
      Serial.println(F("Freq-Measuer OFF, but do NOT start the Timer1!"));
      FreqMeasure.end();
      u8x8.setFont(u8x8_font_7x14B_1x2_r);
      u8x8.setCursor(1,6);
      u8x8.print(F("shot @     fps"));
      fpsState = getFpsState(filmFps);
      drawCurrentFps(false, false);
      if (checkWriteToEEPROMInLoop) {
        checkWriteToEEPROMInLoop = false;
        Serial.println(F("Writing to EEPROM"));
        EEPROM.write(1, playbackFps);
      }
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
      checkRequiredMillisInLoop = true;
      break;
    case STATE_RUNNING:
      checkRequiredMillisInLoop = false;
      u8x8.setFont(u8x8_font_7x14B_1x2_r);
      u8x8.setCursor(8,6);
      u8x8.print("fps");
      u8x8.setCursor(1,6);
      u8x8.print(" ");
      u8x8.drawTile(14,7,1,emptyTile);
      u8x8.drawTile(12,7,2,lockBottom);
      u8x8.drawTile(12,6,3,unlockedLockTop);
      fpsState = FPS_UNLOCKED;

      Serial.println(F("Freq-Measuer ON, Timer1 STOP"));
      stopTimer1();
      detachInterrupt(digitalPinToInterrupt(impDetectorPin));
      Serial.println(F("Crystal Control OFF"));
      digitalWrite(ssrPin, LOW);
      FreqMeasure.begin();
      
      drawCurrentCustomFrequency();
      if (checkWriteToEEPROMInLoop) {
        checkWriteToEEPROMInLoop = false;
        Serial.println(F("Writing to EEPROM"));
        EEPROM.write(0, filmFps);
      }
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
            fpsState = getFpsState(playbackFps);
            Serial.println(F("Freq-Measuer OFF, starting Timer1!"));
            FreqMeasure.end();
            
            setupTimer1forFps(18); // takes 16, 1666, 18, 24 or 25 only
            attachInterrupt(digitalPinToInterrupt(impDetectorPin), projectorCountISR, CHANGE);
            Serial.println(F("Crystal Control ON"));
            digitalWrite(ssrPin, HIGH);
            
            
            drawCurrentFps(false, false);
          } else drawCurrentFps(false, true);
          break;
        case STATE_STOPPED:
          fpsState++;
          drawCurrentFps(true, true);
        default:
          break;
      }
    } else ignoreNextButtonPress = false; 
  } else if (newState == LOW) { //on button press
    switch (state) {
      case STATE_STOPPED:
        requiredMillis = millis() + 1500;
        checkRequiredMillisInLoop = true;
        break;
      case STATE_RESET:
        myEnc.write(0);
        currentFrameCount = 0;
        state = STATE_STOPPED;
        drawState();
    }
  }
}

void drawCurrentFps(const bool redrawCurrentTime, const bool updateEEPROM) {
  float prevFilmFps = filmFps;
  
  u8x8.setFont(u8x8_font_7x14B_1x2_n);
  u8x8.setCursor(((state == STATE_RUNNING) ? 4 : 10),6);
  u8x8.print(" ");
  u8x8.setCursor(((state == STATE_RUNNING) ? 2 : 8),6);
  
  switch (fpsState) {
    default:
      fpsState = FPS_18;
    case FPS_18:
      fps = 18;
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
      fps = 50/3.0;
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
  if (updateEEPROM) {
    writeToEEPROM = millis() + 10000;
    checkWriteToEEPROMInLoop = true;
  }
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
  
  if (filmFps != 50/3.0) {
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

uint8_t getFpsState(float inputFps) {
  switch (int(inputFps)) {
    case 18: return FPS_18;
    case 24: return FPS_24;
    case 25: return FPS_25;
    case 9:  return FPS_9;
    case 16: return FPS_16_2_3;
  }
}

float readEEPROMValue(uint8_t address) {
  uint8_t value = 0;
  value = EEPROM.read(address);
  if (value == 0) return 1;
  else if (value == 16) return 50/3.0;
  else return value;
}

void notFound() {
  u8x8.home();
  u8x8.print(404);
}
