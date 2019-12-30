#include <avr/interrupt.h>   

int t2Speed = 50;

const int LED = 13;
const int channelA = 2;
const int channelB = 3;
int bttnPin = 9;
int sensorPin = A0;
int analogValue;
int prevAnalogValue;
int bttnState;

volatile byte tmrCount = 0;
//Timer2 overflow interrupt vector handler
ISR(TIMER2_OVF_vect) {
  TCNT2 = t2Speed;      //reset timer
  // digitalWrite(LED,HIGH);
  if (bttnState == 1) {
    switch (tmrCount) {
    case 0:
      digitalWrite(channelA,HIGH);
      break;
    case 1:
      digitalWrite(channelB,HIGH);
      break;
    case 2:
      digitalWrite(channelA,LOW);
      break;
    case 3:
      digitalWrite(channelB,LOW);
    }
  } else {
    switch (tmrCount) {
      case 0:
        digitalWrite(channelB,HIGH); // count backwards
        break;
      case 1:
        digitalWrite(channelA,HIGH);
        break;
      case 2:
        digitalWrite(channelB,LOW);
        break;
      case 3:
        digitalWrite(channelA,LOW);
      }    
    }
  tmrCount++;
  if (tmrCount >= 4) {
    tmrCount = 0;
  }
  digitalWrite(LED,LOW);
  TIFR2 = 0x00;
};  

void setup() {
  Serial.begin(115200);
  pinMode(LED,OUTPUT);   // UNO LED, just for show
  pinMode(channelA,OUTPUT);
  pinMode(channelB,OUTPUT);
  pinMode(bttnPin,INPUT_PULLUP );

  TCCR2A = 0;           //Timer2 Settings: WGM mode 0
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);   //Timer2 Settings: Timer Prescaler 1024
  TIMSK2 = _BV(TOIE2);  //Timer2 Overflow Interrupt Enable 
  TCNT2 = t2Speed;      //reset timer
}

void loop() {
  analogValue = analogRead(sensorPin)/7; // this sets the t2Speed to 0-146, equalling fps of 15.3 - 34.9 fps
  if (analogValue != prevAnalogValue) {
    // Serial.println(analogValue);
    t2Speed = analogValue;
    prevAnalogValue = analogValue;
  }
  bttnState = digitalRead(bttnPin);
}
