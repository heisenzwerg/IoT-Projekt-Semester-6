//www.elegoo.com
//2016.12.08

#include "pitches.h"

int pin_rot = 5;
int pin_gruen = 6;
int pin_blau = 7;

int alarm[] = {
  NOTE_F5, NOTE_C6
};
int alarm_on[] = {
  NOTE_D4, NOTE_D5
};
int alarm_off[] = {
  NOTE_D5, NOTE_D4
};
int duration = 500;  // 500 miliseconds

void setup() {
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(10, INPUT);
  pinMode(pin_rot, OUTPUT);
  pinMode(pin_gruen, OUTPUT);
  pinMode(pin_blau, OUTPUT);
}

void loop() {

  while (digitalRead(10) == LOW) {
    digitalWrite(pin_gruen, 0);
    digitalWrite(pin_blau , 0);
    for (int x = 0; x < 2; x++) {
      if (x == 0) {
        digitalWrite(pin_rot, 1);
      }
      else {
        digitalWrite(pin_rot, 0);
      }
      // pin8 output the voice, every scale is 0.5 sencond
      tone(8, alarm[x], duration);
      // Output the voice after several minutes
      delay(500);
    }

    // restart after two seconds

  }


while (digitalRead(2) == HIGH)  {
  digitalWrite(pin_gruen, 0);
  digitalWrite(pin_blau , 0);
  for (int x = 0; x < 2; x++) {
    if (x == 0) {
      digitalWrite(pin_rot, 1);
    }
    else {
      digitalWrite(pin_rot, 0);
    }
    // pin8 output the voice, every scale is 0.5 sencond
    tone(8, alarm[x], duration);
    // Output the voice after several minutes
    delay(500);
  }

  // restart after two seconds

}

if (digitalRead(3) == HIGH)  {
  digitalWrite(pin_gruen, 0);
  digitalWrite(pin_blau , 1);
  digitalWrite(pin_rot, 0);
  for (int x = 0; x < 2; x++) {
    // pin8 output the voice, every scale is 0.5 sencond
    tone(8, alarm_on[x], 100);

    // Output the voice after several minutes
    delay(200);
  }
  delay(500);
}

if (digitalRead(4) == HIGH)  {
  digitalWrite(pin_gruen, 1);
  digitalWrite(pin_blau , 0);
  digitalWrite(pin_rot, 0);
  for (int x = 0; x < 2; x++) {
    // pin8 output the voice, every scale is 0.5 sencond
    tone(8, alarm_off[x], 100);

    // Output the voice after several minutes
    delay(200);

  }
  delay(500);
}

}
