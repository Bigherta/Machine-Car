#include <Servo.h>
#include <PS2X_lib.h>

/*
  正式版：按你当前控制板的真实映射控制 0~5 号舵机接口

  已验证映射：
  0号接口 -> D7，用 十字键 LEFT / RIGHT 控制
  1号接口 -> D3，用 L1 / R1 控制
  2号接口 -> D5，用 TRIANGLE / CROSS 控制
  3号接口 -> D6，用 十字键 UP / DOWN 控制
  4号接口 -> D9，用 SQUARE / CIRCLE 控制
  5号接口 -> D8，用 L2 / R2 控制

  SELECT -> 全部回中

  PS2 接线：
  CLK = 11
  CMD = A0
  ATT = A3
  DAT = 12
*/

PS2X ps2x;
Servo servos[6];

const uint8_t SERVO_PINS[6] = {
  7,  // 0号接口
  3,  // 1号接口
  5,  // 2号接口
  6,  // 3号接口
  9,  // 4号接口
  8   // 5号接口
};

const uint8_t PS2_CLK = 11;
const uint8_t PS2_CMD = A0;
const uint8_t PS2_ATT = A3;
const uint8_t PS2_DAT = 12;

const int SERVO_MIN_US    = 1000;
const int SERVO_CENTER_US = 1500;
const int SERVO_MAX_US    = 2000;

// 按住按键时，每次目标值变化的幅度
const int TARGET_STEP_US = 8;
// 多久允许更新一次目标值
const unsigned long TARGET_UPDATE_MS = 80;
// 舵机每次逼近目标值的步长
const int MOVE_STEP_US = 2;
// 多久真正写一次舵机
const unsigned long MOVE_UPDATE_MS = 20;
// PS2 断线后重连周期
const unsigned long RECONNECT_MS = 1500;

int servoTargetUs[6] = {
  SERVO_CENTER_US, SERVO_CENTER_US, SERVO_CENTER_US,
  SERVO_CENTER_US, SERVO_CENTER_US, SERVO_CENTER_US
};

int servoCurrentUs[6] = {
  SERVO_CENTER_US, SERVO_CENTER_US, SERVO_CENTER_US,
  SERVO_CENTER_US, SERVO_CENTER_US, SERVO_CENTER_US
};

bool ps2Ready = false;
unsigned long lastReconnect = 0;
unsigned long lastTargetUpdate = 0;
unsigned long lastMoveUpdate = 0;

void centerAllServos(bool writeNow) {
  for (int i = 0; i < 6; i++) {
    servoTargetUs[i] = SERVO_CENTER_US;
    if (writeNow) {
      servoCurrentUs[i] = SERVO_CENTER_US;
      servos[i].writeMicroseconds(SERVO_CENTER_US);
    }
  }
}

void clampAllTargets() {
  for (int i = 0; i < 6; i++) {
    if (servoTargetUs[i] < SERVO_MIN_US) servoTargetUs[i] = SERVO_MIN_US;
    if (servoTargetUs[i] > SERVO_MAX_US) servoTargetUs[i] = SERVO_MAX_US;
  }
}

void initPS2() {
  ps2Ready = false;

  for (int i = 0; i < 10; i++) {
    int error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, false, false);
    if (error == 0) {
      ps2Ready = true;
      return;
    }
    delay(300);
  }
}

void updateOneServo(uint8_t index) {
  if (servoCurrentUs[index] < servoTargetUs[index]) {
    servoCurrentUs[index] += MOVE_STEP_US;
    if (servoCurrentUs[index] > servoTargetUs[index]) {
      servoCurrentUs[index] = servoTargetUs[index];
    }
  } else if (servoCurrentUs[index] > servoTargetUs[index]) {
    servoCurrentUs[index] -= MOVE_STEP_US;
    if (servoCurrentUs[index] < servoTargetUs[index]) {
      servoCurrentUs[index] = servoTargetUs[index];
    }
  }

  servos[index].writeMicroseconds(servoCurrentUs[index]);
}

void updateServosSmoothly() {
  if (millis() - lastMoveUpdate < MOVE_UPDATE_MS) return;
  lastMoveUpdate = millis();

  for (int i = 0; i < 6; i++) {
    updateOneServo(i);
  }
}

void handlePadControl() {
  if (!ps2Ready) return;

  ps2x.read_gamepad(false, 0);

  if (millis() - lastTargetUpdate >= TARGET_UPDATE_MS) {
    lastTargetUpdate = millis();

    // 0号接口：十字键左右
    if (ps2x.Button(PSB_PAD_LEFT))  servoTargetUs[0] += TARGET_STEP_US;
    if (ps2x.Button(PSB_PAD_RIGHT)) servoTargetUs[0] -= TARGET_STEP_US;

    // 1号接口：L1 / R1
    if (ps2x.Button(PSB_L1)) servoTargetUs[1] += TARGET_STEP_US;
    if (ps2x.Button(PSB_R1)) servoTargetUs[1] -= TARGET_STEP_US;

    // 2号接口：三角 / 叉
    if (ps2x.Button(PSB_TRIANGLE)) servoTargetUs[2] += TARGET_STEP_US;
    if (ps2x.Button(PSB_CROSS))    servoTargetUs[2] -= TARGET_STEP_US;

    // 3号接口：十字键上下
    if (ps2x.Button(PSB_PAD_UP))   servoTargetUs[3] += TARGET_STEP_US;
    if (ps2x.Button(PSB_PAD_DOWN)) servoTargetUs[3] -= TARGET_STEP_US;

    // 4号接口：方块 / 圆圈
    if (ps2x.Button(PSB_SQUARE)) servoTargetUs[4] += TARGET_STEP_US;
    if (ps2x.Button(PSB_CIRCLE)) servoTargetUs[4] -= TARGET_STEP_US;

    // 5号接口：L2 / R2
    if (ps2x.Button(PSB_L2)) servoTargetUs[5] += TARGET_STEP_US;
    if (ps2x.Button(PSB_R2)) servoTargetUs[5] -= TARGET_STEP_US;

    // SELECT：全部回中
    if (ps2x.ButtonPressed(PSB_SELECT)) {
      centerAllServos(false);
    }

    clampAllTargets();
  }
}

void setup() {
  for (int i = 0; i < 6; i++) {
    servos[i].attach(SERVO_PINS[i], 500, 2500);
  }

  centerAllServos(true);
  delay(500);
  initPS2();
}

void loop() {
  if (!ps2Ready) {
    if (millis() - lastReconnect >= RECONNECT_MS) {
      lastReconnect = millis();
      initPS2();
    }
    updateServosSmoothly();
    return;
  }

  handlePadControl();
  updateServosSmoothly();
}
