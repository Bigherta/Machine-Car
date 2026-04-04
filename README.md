# Machine-Car
A machine car control coding repository.
项目目标： 制作一个控制电机与舵机的程序，使得PS2手柄可以通过蓝牙连接来操控小车前进、后退、转弯与机械臂进行操作。
依赖库：PS2X_lib.h，已从外部引入。
编译与烧录：通过Arduino IDE进行。

## 当前控制逻辑
- 左摇杆 Y 轴映射为油门（前进/后退目标速度）。
- 左摇杆 X 轴映射为转向（转向目标速度差）。
- 左右轮目标速度采用差速混控：`left = throttle + steering`，`right = throttle - steering`。
- 实际电机速度按斜率限制逐步逼近目标速度，且始终受上下限保护。

## 电机电路直连测试（本分支）
- 本分支 `feat/direct-slow-motor-test` 默认开启“直接缓慢驱动电机”模式，用于排查 Arduino 主板到电机驱动/电机的线路问题。
- 固件会按周期自动执行：慢速前进（2s）→停止（2s）→慢速后退（2s）→停止（2s）循环。
- 测试模式下不依赖手柄摇杆输入。
- 按下 KEY2（低电平）可立即急停两个电机。
- 如需恢复手柄控制，将 `ZL_KPZAR/loopkey.ino` 中 `DIRECT_MOTOR_TEST_MODE` 改为 `false`。
