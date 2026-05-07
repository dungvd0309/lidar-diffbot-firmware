#include <Arduino.h>

extern "C" {

bool arduino_transport_open(struct uxrCustomTransport * transport)
{
  Serial.begin(921600);
  return true;
}

}
