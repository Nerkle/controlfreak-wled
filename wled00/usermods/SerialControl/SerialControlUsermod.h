// SerialControlUsermod.h

#pragma once
#include "wled.h"

#define USERMOD_ID_SERIAL_CONTROL  0xD00A

class SerialControlUsermod : public Usermod {
private:
  String inputLine;
  void handleCommand(const String &cmdLine);

public:
  // These two are pure‐virtual in Usermod and must be provided:
  void setup() override;
  void loop() override;

};
