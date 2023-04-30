// Copyright (c) 2023 Yehoshua Halle
// See copyright statement at bottom of document.

#include "SyncServo.h"
#include <Arduino.h>

long unsigned int SyncServo::startUS = micros();
long unsigned int SyncServo::deltaUS = 0;

SyncServo::SyncServo(uint8_t pin)
  : SyncServo(pin, SYNC_SERVO_MIN_PW, SYNC_SERVO_MAX_PW) {}

SyncServo::SyncServo(uint8_t pin, uint16_t lowPulseWidth, uint16_t highPulseWidth)
  : pin(pin), low_pulseWidth(lowPulseWidth), high_pulseWidth(highPulseWidth) {
    this->setSpeedRaw(0xffff); // Set speed to maximum by default
    pinMode(pin, OUTPUT);
  }

void SyncServo::sync(SyncServo servo_list[], uint8_t nservos, void (*fun) () = NULL, uint16_t how_long_fun_lasts_micros = 0xffff) {
  uint8_t i;
  // exit early if there are no servos to sync
  if (nservos == 0 || servo_list == NULL) return;
  // wait until the sync period finishes
  while(micros() < SyncServo::startUS + SYNC_SERVO_SIGNAL_PERIOD_US + how_long_fun_lasts_micros) {
    if(fun != NULL) {
      fun();
    }
  }
  // use more precise timing when we are close to the signal time
  while (micros() < SyncServo::startUS + SYNC_SERVO_SIGNAL_PERIOD_US) {
  }
  // calculate the next position of each of the servos
  SyncServo::deltaUS = micros() - SyncServo::startUS;
  SyncServo::startUS = micros();
  unsigned long deltaMS = SyncServo::deltaUS/1000;
  for (i = 0; i < nservos; i++) {
    servo_list[i].update(deltaMS);
  }
  // Accurately send a the correct length pulse width for each servo
  for (i = 0; i < nservos; i++) {
    SyncServo s = servo_list[i];
    unsigned long mystart = micros();
    digitalWrite(s.pin, HIGH);
    while(micros() < mystart + s.getPulseWidth()) {}
    digitalWrite(s.pin, LOW);
    // Wait for an equal amount of time per servo for consistency
    while(micros() < mystart + SYNC_SERVO_MAX_PW) {}
  }
  // for(i = 0; i < nservos; i++) {
  //   Serial.print(i);
  //   Serial.print(F(": "));
  //   Serial.println(servo_list[i].getPulseWidth());
  // }
}

uint16_t SyncServo::degreesToTicks(float degrees) {
  degrees /= 180.0;  // convert to range "0-1"
  return (uint16_t)(degrees * 65536);
}

uint16_t SyncServo::radiansToTicks(float radians) {
  radians /= PI;  // convert to range "0-1"
  return (uint16_t)(radians * 65536);
}

void SyncServo::setTargetRaw(uint16_t ticks) {
  this->target = ticks;
}

void SyncServo::setTargetDegrees(float degrees) {
  this->setTargetRaw(degreesToTicks(degrees));
}

void SyncServo::setTargetRadians(float radians) {
  this->setTargetRaw(radiansToTicks(radians));
}

void SyncServo::setSpeedRaw(uint16_t ticksPerMS) {
  this->speed = ticksPerMS;
}

void SyncServo::setSpeedDegreesPerSecond(float degreesPerSecond) {
  this->speed = degreesToTicks(degreesPerSecond) / 1000;  // convert to ticks per ms
}

void SyncServo::setSpeedRadiansPerSecond(float radiansPerSecond) {
  this->speed = radiansToTicks(radiansPerSecond) / 1000;  // convert to ticks per ms
}

bool SyncServo::targetReached() {
  return this->angle == this->target;
}

void SyncServo::update(long unsigned int deltaTimeMS) {
  int32_t newangle = this->angle;
  int32_t delta = ((int32_t)this->speed) * deltaTimeMS;
  int32_t maxdelta = (this->target<this->angle ? (this->angle-this->target) : (this->target-this->angle));
  delta = min(delta, maxdelta);
  if (this->target < this->angle) {
    delta = -delta;
  }
  newangle += delta;
  if (newangle < 0) newangle = 0;
  if (newangle > 65535) newangle = 65535;
  this->angle = (uint16_t)newangle;
  this->pulseWidth = map(newangle, 0, 65535, this->low_pulseWidth, this->high_pulseWidth);
}

uint16_t SyncServo::getPulseWidth() {
  return this->pulseWidth;
}

uint16_t SyncServo::getTargetRaw() {
  return this->target;
}

float SyncServo::getTargetDegrees() {
  return (this->target / 65535.0) * 180.0;
}

float SyncServo::getTargetRadians() {
  return (this->target / 65535.0) * PI;
}

uint16_t SyncServo::getAngleRaw() {
  return this->angle;
}

float SyncServo::getAngleDegrees() {
  return (this->angle / 65535.0) * 180.0;
}

float SyncServo::getAngleRadians() {
return (this->angle / 65535.0) * PI;
}

// Copyright (c) 2023 Yehoshua Halle
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.