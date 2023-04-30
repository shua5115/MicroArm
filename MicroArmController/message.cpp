#include "message.h"

Message readMessage(Stream& stream) {
  char buf[4] = {0};
  Message m = {0};
  stream.readBytes(buf, 2);
  m.mode = *((uint16_t*)buf);
  stream.readBytes(buf, 4);
  m.x = *((uint16_t*)buf);
  stream.readBytes(buf, 4);
  m.y = *((uint16_t*)buf);
  stream.readBytes(buf, 4);
  m.z = *((uint16_t*)buf);
  stream.readBytes(buf, 4);
  m.w = *((uint16_t*)buf);
  stream.readStringUntil('\n');
  return m;
} 