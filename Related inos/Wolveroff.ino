/* 

Turn off the crappy Wolverine Scanner when it stops moving 

*/

#define impDetectorPin  3
#define buzzerPin       5
#define relaisPin       12

#define blueLedPin      10 
#define greenLedPin     11
#define redLedPin       13

#define SCANNER_RUNNING          1
#define SCANNER_GETTING_LOADED   2
#define SCAN_FINISHED            3

uint8_t myState;
uint8_t prevState;
unsigned long millisNow = 0;
unsigned long lastMillis = 0;
volatile unsigned long impCounter = 0;
unsigned long lastFrameCount = 0;

bool scannerEverRan = false; // Don't beep right away

float lastFreq;

void setup() {
  pinMode(impDetectorPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(relaisPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  
  Serial.begin(115200);
  tone(buzzerPin, 6000, 200); // Hello!
  delay(250);

  digitalWrite(relaisPin, LOW);
  redLight();
  
  attachInterrupt(digitalPinToInterrupt(impDetectorPin), countISR, CHANGE);

}

void loop() {
  measureScannerFreq(10); // measure speed over 10 seconds
  if ((lastFreq <= 1.7) || (lastFreq >= 2.3)) {
    if (lastFreq == 0.0) {
      if (scannerEverRan) {
        myState = SCAN_FINISHED;
      }
    } else {
      myState = SCANNER_GETTING_LOADED;
    }
  } else {
    myState = SCANNER_RUNNING;
    scannerEverRan = true;
  }

  if (prevState != myState) {
    switch (myState) {
      case SCANNER_RUNNING:
        greenLight();
        Serial.println("Scan läuft.");
        break;
      case SCANNER_GETTING_LOADED:
        blueLight();
        Serial.println("Scanner wird geladen.");
        break;
      case SCAN_FINISHED:
        Serial.println("Scan ist fertig.");
        redLight();
        powerOff();
        scannerEverRan = false;
        myState = 0;
        break;
      default:
      break;
    }
    prevState = myState;
  }
}

void measureScannerFreq(int checkIntervalSec) {
  millisNow = millis();
  if (((millisNow % (checkIntervalSec * 1000)) == 0) && (lastMillis != millisNow)) {
    lastFreq = float((impCounter - lastFrameCount)) / checkIntervalSec;
    lastFrameCount = impCounter;
    lastMillis = millisNow;
    Serial.print(lastFreq);
    Serial.println(" B/s");
    Serial.println("");
  }
}

void redLight() {
  digitalWrite(redLedPin, HIGH);
  digitalWrite(greenLedPin, LOW);
  digitalWrite(blueLedPin, LOW);
}

void greenLight() {
  digitalWrite(redLedPin, LOW);
  digitalWrite(greenLedPin, HIGH);
  digitalWrite(blueLedPin, LOW);
}

void blueLight() {
  digitalWrite(redLedPin, LOW);
  digitalWrite(greenLedPin, LOW);
  digitalWrite(blueLedPin, HIGH);
}

void yellowLight() {
  digitalWrite(redLedPin, HIGH);
  digitalWrite(greenLedPin, HIGH);
  digitalWrite(blueLedPin, LOW);
}


void countISR() {
  impCounter++;
}

void powerOff() {
  digitalWrite(relaisPin, HIGH);
  delay(3000);
  digitalWrite(relaisPin, LOW);
  
  tone(buzzerPin, 6000, 1000); // Hello!
  delay(2000);
  tone(buzzerPin, 6000, 1000); // Hello!
  delay(2000);
  tone(buzzerPin, 6000, 1000); // Hello!
  delay(2000);
  tone(buzzerPin, 6000, 1000); // Hello!
  delay(2000);
  tone(buzzerPin, 6000, 1000); // Hello!
  delay(2000);
  tone(buzzerPin, 6000, 1000); // Hello!
  delay(2000);
  tone(buzzerPin, 6000, 1000); // Hello!
  delay(2000);
  tone(buzzerPin, 6000, 1000); // Hello!
  delay(2000);
  tone(buzzerPin, 6000, 1000); // Hello!
  delay(2000);
  tone(buzzerPin, 6000, 1000); // Hello!
  delay(2000);
  
}
