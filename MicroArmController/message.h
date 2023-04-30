#ifndef MESSAGE_H
#define MESSAGE_H

#include <Arduino.h>
#include <stdint.h>

typedef struct {
  uint16_t x, y, z, w;
  uint16_t mode;
} Message;

Message readMessage(Stream& stream);

#endif