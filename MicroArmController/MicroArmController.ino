#include "SyncServo.h"
#include "message.h"
#include <stdint.h>
#define arrayLength(arr, t) (sizeof(arr) / sizeof(t))
#define MSG_QUEUE_SIZE 32
SyncServo servos[] = { SyncServo(4), SyncServo(5), SyncServo(6), SyncServo(7), SyncServo(8) };  // base, shoulder, elbow, wrist, gripper
Message msgQueue[MSG_QUEUE_SIZE] = { 0 };                                                       // beware: compiler allocates this on heap, so it doesn't show up as a global variable
uint8_t msgQueueIndex = 0;
uint8_t msgQueueLength = 0;

void setup() {
  pinMode(2, INPUT_PULLUP);
  Serial.begin(9600);
  Serial.setTimeout(50);
  for (SyncServo s : servos) {
    s.setSpeedDegreesPerSecond(1);
  }
  servos[0].setTargetRaw(32768);
  servos[1].setTargetRaw(4096);
  servos[2].setTargetRaw(12203);
  servos[3].setTargetRaw(16384);
}

void offerMessage(Message* m) {
  if (msgQueueLength >= MSG_QUEUE_SIZE) return;
  uint8_t insertIndex = (msgQueueIndex + msgQueueLength) % MSG_QUEUE_SIZE;
  msgQueue[insertIndex] = *m;
  m = msgQueue + insertIndex;
  msgQueueLength += 1;
}

Message pollMessage() {
  if (msgQueueLength <= 0) return Message{ 0, 0, 0, 0, 0 };
  Message m = msgQueue[msgQueueIndex];
  msgQueueIndex = (msgQueueIndex + 1) % MSG_QUEUE_SIZE;
  if (msgQueueLength > 0) { msgQueueLength -= 1; }
  return m;
}

void readMessages() {
  if (Serial.available() >= 19) {
    Message m = readMessage(Serial);
    offerMessage(&m);
  }
}

struct JoyState {
  uint16_t x, y, z;
  uint8_t button;
};

void loop() {
  SyncServo::sync(servos, arrayLength(servos, SyncServo), readMessages, 5000);
  // uint16_t potx = analogRead(0);
  // servos[0].setTargetRaw(map(1024+512, 0, 8192, servos[0].getTargetRaw(), potx*64));
  Message m = pollMessage();
  // mode = 0 -> no op, don't do anything for 20 ms
  // if (m.mode != 0) {
  //   Serial.println(m.mode);
  // }
  if (m.mode == 1) {  // fw kinematics
    servos[0].setTargetRaw((uint16_t)m.x);
    servos[1].setTargetRaw((uint16_t)m.y);
    servos[2].setTargetRaw((uint16_t)m.z);
    servos[3].setTargetRaw((uint16_t)m.w);
  } else if (m.mode == 2) {  // gripper
    servos[4].setTargetRaw((uint16_t)m.x);
  }
  // send joystick info
  JoyState joyState = JoyState{
    analogRead(0),
    analogRead(1),
    analogRead(2),
    (1-digitalRead(2))
  };
  Serial.write((const char*)(&joyState), sizeof(joyState));
  Serial.write('\n');
}
