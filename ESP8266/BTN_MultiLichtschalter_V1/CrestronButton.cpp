#include "Arduino.h"
#include "CrestronButton.h"

DebouncedButton::DebouncedButton() {

}

void DebouncedButton::initialize(byte pin, unsigned long debounceDelay, short crestronId) {
  pinMode(pin, INPUT_PULLUP);
  this->pin = pin;
  this->debounceDelay = debounceDelay;
  this->crestronId = crestronId;
  this->lastState = HIGH;
  this->state = HIGH;
  this->lastDebounceTime = 0;
}

change DebouncedButton::process() {
  change res = none;

  int reading = digitalRead(pin);
  
  if (reading != lastState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != state) {
      state = reading;

      if (state == HIGH) {
        res = high;
      }
      if (state == LOW) {
        res = low;
      }
    }
  }

  /*Serial.print("button ");
  Serial.print(pin);
  Serial.print(" laststate ");
  Serial.print(lastState);
  Serial.print(" lastDebounceTime ");
  Serial.print(lastDebounceTime);
  Serial.print(" state ");
  Serial.println(reading);*/

  lastState = reading;

  return res;
}

short DebouncedButton::getCrestronId() {
  return this->crestronId;
}
