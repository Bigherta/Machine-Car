#include <PS2X_lib.h>
#include <Servo.h>

extern bool g_ps2_link_ok;
extern PS2X ps2;

Servo servos[6];

const uint8_t SERVO_PINS[6] = {
  7,  // 0号接口
  3,  // 1号接口
  5,  // 2号接口
  6,  // 3号接口
  9,  // 4号接口
  8   // 5号接口
};

const int SERVO_MIN_US = 600;
const int SERVO_CENTER_US = 1500;
const int SERVO_MAX_US = 2400;

// 每个接口的单独最小/最大脉宽（微秒），可为某个舵机单独放宽范围
const int SERVO_MIN_US_PER[6] = { 600, 600, 600, 600, 600, 600 };
// 将 1 号接口（索引 1）上限放宽到 2500us，其他默认使用 2400us
const int SERVO_MAX_US_PER[6] = { 2400, 2500, 2400, 2400, 2400, 2400 };

// 按住按键时，每次目标值变化的幅度
const int TARGET_STEP_US = 20;
// 多久允许更新一次目标值
const unsigned long TARGET_UPDATE_MS = 10;
// 舵机每次逼近目标值的步长
const int MOVE_STEP_US = 25;
// 多久真正写一次舵机
const unsigned long MOVE_UPDATE_MS = 10;

int servoTargetUs[6] = { SERVO_CENTER_US, SERVO_CENTER_US, SERVO_CENTER_US,
                         SERVO_CENTER_US, SERVO_CENTER_US, SERVO_CENTER_US };

int servoCurrentUs[6] = { SERVO_CENTER_US, SERVO_CENTER_US, SERVO_CENTER_US,
                          SERVO_CENTER_US, SERVO_CENTER_US, SERVO_CENTER_US };

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
    if (servoTargetUs[i] < SERVO_MIN_US_PER[i])
      servoTargetUs[i] = SERVO_MIN_US_PER[i];
    if (servoTargetUs[i] > SERVO_MAX_US_PER[i])
      servoTargetUs[i] = SERVO_MAX_US_PER[i];
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

void move_to_determined_pos(int pos1, int pos2, int pos3, int pos4, int pos5,
                            int pos6) {
  servoTargetUs[0] = pos1;
  servoTargetUs[1] = pos2;
  servoTargetUs[2] = pos3;
  servoTargetUs[3] = pos4;
  servoTargetUs[4] = pos5;
  servoTargetUs[5] = pos6;

  clampAllTargets();
  servo_targets_centered = false;
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
    bool l1Pressed = ps2.Button(PSB_L1);
    bool r1Pressed = ps2.Button(PSB_R1);
    bool l2Pressed = ps2.Button(PSB_L2);
    bool r2Pressed = ps2.Button(PSB_R2);
    bool pad_down = ps2.Button(PSB_PAD_DOWN);
    bool pad_cross = ps2.Button(PSB_CROSS);
    bool pad_right = ps2.Button(PSB_PAD_RIGHT);
    bool pad_square = ps2.Button(PSB_SQUARE);
    bool pad_up = ps2.Button(PSB_PAD_UP);
    bool pad_triangle = ps2.Button(PSB_TRIANGLE);
    bool pad_left = ps2.Button(PSB_PAD_LEFT);
    bool pad_circle = ps2.Button(PSB_CIRCLE);
    // 组合键：同时按下 L1 + R1 时复位，且不触发单独 L1/R1 效果
    if (l1Pressed && r1Pressed) {
      move_to_determined_pos(1500, 1500, 780, 1500, 1500, 1500);
      return;
    }
    if (l2Pressed && r2Pressed) {
      move_to_determined_pos(1500, 1500, 2080, 1950, 2400, 1500);
      return;
    }
    if (l1Pressed && l2Pressed) {
      move_to_determined_pos(1500, 1500, 1500, 1650, 2200, 1500);
    }
    if (pad_down && pad_cross) {
      move_to_determined_pos(1500, 1500, 1800, 1850, 2400, 1500);
      return;
    }
    if (pad_right && pad_square) {
      move_to_determined_pos(1500, 1500, 1000, 2200, 1400, 1500);
      return;
    }
    if (pad_up && pad_triangle) {
      move_to_determined_pos(1500, 1500, 1100, 1500, 1800, 1500);
      return;
    }
    if (pad_left && pad_circle) {
      move_to_determined_pos(1500, 1500, 1900, 2300, 2000, 1500);
      return;
    }
    // 0号接口：十字键左右
    if (pad_left && !pad_circle) {
      servoTargetUs[0] += TARGET_STEP_US;
      changed = true;
    }
    if (pad_right && !pad_square) {
      servoTargetUs[0] -= TARGET_STEP_US;
      changed = true;
    }

    // 1号接口：L1 / R1
    if (l1Pressed && !r1Pressed && !l2Pressed) {
      servoTargetUs[1] -= TARGET_STEP_US;
      changed = true;
    }
    if (r1Pressed && !l1Pressed) {
      servoTargetUs[1] += TARGET_STEP_US;
      changed = true;
    }

    // 2号接口：三角 / 叉
    if (pad_cross && !pad_down) {
      servoTargetUs[2] += TARGET_STEP_US;
      changed = true;
    }
    if (pad_triangle && !pad_up) {
      servoTargetUs[2] -= TARGET_STEP_US;
      changed = true;
    }

    // 3号接口：十字键上下
    if (pad_up && !pad_triangle) {
      servoTargetUs[3] += TARGET_STEP_US;
      changed = true;
    }
    if (pad_down && !pad_cross) {
      servoTargetUs[3] -= TARGET_STEP_US;
      changed = true;
    }

    // 4号接口：方块 / 圆圈
    if (pad_square && !pad_right) {
      servoTargetUs[4] += TARGET_STEP_US;
      changed = true;
    }
    if (pad_circle && !pad_left) {
      servoTargetUs[4] -= TARGET_STEP_US;
      changed = true;
    }

    // 5号接口：L2 / R2
    if (l2Pressed && !r2Pressed && !l1Pressed) {
      servoTargetUs[5] += TARGET_STEP_US;
      changed = true;
    }
    if (r2Pressed && !l2Pressed) {
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