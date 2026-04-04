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
