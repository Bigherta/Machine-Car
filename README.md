# Machine-Car
A machine car control coding repository.
项目目标： 制作一个控制电机与舵机的程序，使得PS2手柄可以通过蓝牙连接来操控小车前进、后退、转弯与机械臂进行操作。
依赖库：PS2X_lib.h，已从外部引入。
编译与烧录：通过Arduino IDE进行。

## 代码优化说明
- 优化了 `setup_ps2()` 初始化流程，避免重复调用 `config_gamepad`。
- 优化了串口输出函数签名：`uart_send_str` 支持 `const char*`，减少字符串字面量类型告警。
- 优化了摇杆控制逻辑：
  - 摇杆偏移采用二次曲线映射，偏移越大速度增幅越大。
  - 引入摇杆死区（`JOYSTICK_DEADZONE`）减少中位抖动导致的误动作。
  - 增加最小有效输出（`MOTOR_SPEED_MIN_EFFECTIVE`），降低低速推不动问题。
  - 采用目标速度追踪与斜率限制（加速/减速步进），控制更平顺。
  - 使用差速混控（`throttle ± steering`）实现行进中连续转向。
  - 统一速度限幅函数 `clamp_motor_speed`，确保最高速度限制。

## 当前控制逻辑
- 左摇杆 Y 轴映射为油门（前进/后退目标速度）。
- 左摇杆 X 轴映射为转向（转向目标速度差）。
- 左右轮目标速度采用差速混控：`left = throttle + steering`，`right = throttle - steering`。
- 实际电机速度按斜率限制逐步逼近目标速度，且始终受上下限保护。
