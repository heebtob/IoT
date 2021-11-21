#ifndef CrestronButton_h
#define CrestronButton_h

#include "Arduino.h"

enum change {
  none,
  high,
  low
};

class DebouncedButton
{
  public:
    DebouncedButton();
    void initialize(byte pin, unsigned long debounceDelay, short crestronId);
    change process();
    short getCrestronId();
  private:
    byte pin;
    int lastState;
    int state;
    unsigned long lastDebounceTime;
    unsigned long debounceDelay;
    short crestronId;
};

#endif
