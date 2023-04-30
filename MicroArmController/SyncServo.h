// Copyright (c) 2023 Yehoshua Halle
// See copyright statement at bottom of document.

#ifndef SYNC_SERVO_H
#define SYNC_SERVO_H

#include <stdint.h>
#include <stdlib.h>

#define SYNC_SERVO_SIGNAL_PERIOD_US 10000
// #define SYNC_SERVO_DEFAULT_PW 1500
#define SYNC_SERVO_MIN_PW 550
#define SYNC_SERVO_MAX_PW 2300
#define SYNC_SERVO_TARGET_REACHED 0x1

/// A manager for a micro servo to make it move with more control over speed.
/// The servos are assumed to have a motion range of 0-180 degrees,
/// with a signal period of 20 ms.
///
/// Servos store raw angle units of "ticks", where there are 65535 ticks in 180 degrees.
class SyncServo {
  public:
  SyncServo(uint8_t pin);
  SyncServo(uint8_t pin, uint16_t lowPulseWidth, uint16_t highPulseWidth);
  
  /// This function completes the 20 millisecond actuation cycle for all micro servos,
  /// so it should be called often, somewhere in loop().
  /// This list passed to this function should contain ALL SERVOS in use, 
  /// otherwise servos will not be actuated correctly.
  ///
  /// In order, the function performs the following actions:
  /// 1. Waits until 20 ms have passed since the last call to sync()
  ///    - Calls fun() if it exists and more than `how_long_fun_lasts_micros` time is remaining in the loop
  /// 2. Calls update() on all servos in the list, which updates their internal target
  /// 3. Writes the correct length pulse width to each servo's pin;
  ///    this section lasts SYNC_SERVO_MAX_PW microseconds times the number of servos in the list.
  static void sync(SyncServo* servo_list, uint8_t n_servos, void (*fun) () = NULL, uint16_t how_long_fun_lasts_micros = 0xffff);

  static uint16_t degreesToTicks(float degrees);

  static uint16_t radiansToTicks(float radians);
  
  void setTargetDegrees(float degrees);
  
  void setTargetRadians(float radians);
  
  /// Sets the target angle of the servo directly with units of ticks,
  /// where there are 65535 ticks in 180 degrees.
  void setTargetRaw(uint16_t ticks);
  
  void setSpeedDegreesPerSecond(float degreesPerSecond);
  
  void setSpeedRadiansPerSecond(float radiansPerSecond);
  
  /// Set the speed of the servo directly with units ticks per millisecond,
  /// where there are 65535 ticks in 180 degrees.
  void setSpeedRaw(uint16_t ticksPerMS);
  
  /// Whether the internal position of this servo has reached its target position.
  bool targetReached();
  
  /// Returns the pulse width in microseconds used for the most recent actuation.
  uint16_t getPulseWidth();

  uint16_t getTargetRaw();

  float getTargetDegrees();

  float getTargetRadians();

  uint16_t getAngleRaw();

  float getAngleDegrees();

  float getAngleRadians();

  /// Updates the stored position (angle) of the servo.
  /// Does not actuate the servo (see SyncServo::sync()).
  void update(long unsigned int deltaTimeMS);

  protected:
  static long unsigned int startUS;
  static long unsigned int deltaUS;
  uint16_t target; // Where the servo wants to go
  uint16_t angle; // Where the servo is "now"
  uint16_t speed; // How much angle should change every ms to reach the target
  uint16_t pulseWidth;
  uint16_t low_pulseWidth;
  uint16_t high_pulseWidth;

  public:
  const uint8_t pin;
};

#endif

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