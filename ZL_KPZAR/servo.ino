#include <PS2X_lib.h>
#include <Servo.h>

extern bool g_ps2_link_ok;
extern PS2X ps2;

Servo servos[6];

const uint8_t SERVO_PINS[6] = {
    7, // 0号接口
    3, // 1号接口
    5, // 2号接口
    6, // 3号接口
    9, // 4号接口
    8  // 5号接口
};

const int SERVO_MIN_US = 1000;
const int SERVO_CENTER_US = 1500;
const int SERVO_MAX_US = 2000;

// 按住按键时，每次目标值变化的幅度
const int TARGET_STEP_US = 8;
// 多久允许更新一次目标值
const unsigned long TARGET_UPDATE_MS = 80;
// 舵机每次逼近目标值的步长
const int MOVE_STEP_US = 2;
// 多久真正写一次舵机
const unsigned long MOVE_UPDATE_MS = 20;

int servoTargetUs[6] = {SERVO_CENTER_US, SERVO_CENTER_US, SERVO_CENTER_US,
                        SERVO_CENTER_US, SERVO_CENTER_US, SERVO_CENTER_US};

int servoCurrentUs[6] = {SERVO_CENTER_US, SERVO_CENTER_US, SERVO_CENTER_US,
                         SERVO_CENTER_US, SERVO_CENTER_US, SERVO_CENTER_US};

unsigned long lastTargetUpdate = 0;
unsigned long lastMoveUpdate = 0;
bool servo_targets_centered = true;

void centerAllServos(bool writeNow) {
  for (int i = 0; i < 6; i++) {
    servoTargetUs[i] = SERVO_CENTER_US;
    if (writeNow) {
      servoCurrentUs[i] = SERVO_CENTER_US;
      servos[i].writeMicroseconds(SERVO_CENTER_US);
    }
  }
  servo_targets_centered = true;
}

void clampAllTargets() {
  for (int i = 0; i < 6; i++) {
    if (servoTargetUs[i] < SERVO_MIN_US)
      servoTargetUs[i] = SERVO_MIN_US;
    if (servoTargetUs[i] > SERVO_MAX_US)
      servoTargetUs[i] = SERVO_MAX_US;
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
  if (millis() - lastMoveUpdate < MOVE_UPDATE_MS)
    return;
  lastMoveUpdate = millis();

  for (int i = 0; i < 6; i++) {
    updateOneServo(i);
  }
}

void handlePadControl() {
  if (!g_ps2_link_ok)
    return;

  if (millis() - lastTargetUpdate >= TARGET_UPDATE_MS) {
    lastTargetUpdate = millis();
    if (ps2.ButtonPressed(PSB_SELECT)) {
      centerAllServos(false);
      clampAllTargets();
      return;
    }

    bool changed = false;

    // 0号接口：十字键左右
    if (ps2.Button(PSB_PAD_LEFT)) {
      servoTargetUs[0] += TARGET_STEP_US;
      changed = true;
    }
    if (ps2.Button(PSB_PAD_RIGHT)) {
      servoTargetUs[0] -= TARGET_STEP_US;
      changed = true;
    }

    // 1号接口：L1 / R1
    if (ps2.Button(PSB_L1)) {
      servoTargetUs[1] += TARGET_STEP_US;
      changed = true;
    }
    if (ps2.Button(PSB_R1)) {
      servoTargetUs[1] -= TARGET_STEP_US;
      changed = true;
    }

    // 2号接口：三角 / 叉
    if (ps2.Button(PSB_TRIANGLE)) {
      servoTargetUs[2] += TARGET_STEP_US;
      changed = true;
    }
    if (ps2.Button(PSB_CROSS)) {
      servoTargetUs[2] -= TARGET_STEP_US;
      changed = true;
    }

    // 3号接口：十字键上下
    if (ps2.Button(PSB_PAD_UP)) {
      servoTargetUs[3] += TARGET_STEP_US;
      changed = true;
    }
    if (ps2.Button(PSB_PAD_DOWN)) {
      servoTargetUs[3] -= TARGET_STEP_US;
      changed = true;
    }

    // 4号接口：方块 / 圆圈
    if (ps2.Button(PSB_SQUARE)) {
      servoTargetUs[4] += TARGET_STEP_US;
      changed = true;
    }
    if (ps2.Button(PSB_CIRCLE)) {
      servoTargetUs[4] -= TARGET_STEP_US;
      changed = true;
    }

    // 5号接口：L2 / R2
    if (ps2.Button(PSB_L2)) {
      servoTargetUs[5] += TARGET_STEP_US;
      changed = true;
    }
    if (ps2.Button(PSB_R2)) {
      servoTargetUs[5] -= TARGET_STEP_US;
      changed = true;
    }

    clampAllTargets();
    if (changed) {
      servo_targets_centered = false;
    }
  }
}

void setup_servo(void) {
  for (int i = 0; i < 6; i++) {
    servos[i].attach(SERVO_PINS[i], 500, 2500);
  }

  centerAllServos(true);
}

void loop_servo(void) {
  if (!g_ps2_link_ok) {
    if (!servo_targets_centered) {
      centerAllServos(false);
    }
  } else {
    handlePadControl();
  }

  updateServosSmoothly();
}
