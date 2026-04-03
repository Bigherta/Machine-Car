# Machine-Car
A machine car control coding repository.
项目目标： 制作一个控制电机与舵机的程序，使得PS2手柄可以通过蓝牙连接来操控小车前进、后退、转弯与机械臂进行操作。
依赖库：PS2X_lib.h，已从外部引入。
编译与烧录：通过Arduino IDE进行。

## 代码优化说明
- 优化了 `setup_ps2()` 初始化流程，避免重复调用 `config_gamepad`。
- 优化了串口输出函数签名：`uart_send_str` 支持 `const char*`，减少字符串字面量类型告警。
- 优化了摇杆控制逻辑：
  - 引入摇杆死区（`JOYSTICK_DEADZONE`）减少中位抖动导致的误动作。
  - 抽取电机速度上下限与缩放常量，提升可读性与可调性。
  - 统一速度限幅函数 `clamp_motor_speed`，降低重复代码。

## 当前控制逻辑
- 左摇杆纵向主导：前进/后退速度调节。
- 左摇杆横向主导：左右转向速度差分。
- 摇杆回中（含死区）：重置电机速度为 0。
