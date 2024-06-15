# Machine_hand

这是中南大学自动化学院智能2103班大三下机器人学课程设计。主要内容为多种方式实现机械手实时控制。主要包含上位机控制、体感手套控制和基于Mediapipe视觉控制三部分的代码。

## 目录

- [上位机控制](#上位机控制)
- [体感手套控制](#体感手套控制)
- [视觉控制](#视觉控制)

## 上位机控制

上位机部分的代码用于与机械手进行通信和控制。本项目实现了一个图形用户界面（GUI），用户可以通过该界面发送指令、接收反馈并监控机械手的状态。除此之外还可通过USB将保存好的动作组文件下载到机械手Flash中，实现脱机运行。

### 功能

- 发送控制指令
- 接收并显示反馈
- 实时监控机械手状态

### 环境配置

- 采用Visual stduio 2015的WPF框架实现，只需下载VS2015社区版，打开RobotArm.sln运行即可，推荐在Debug或Release生成解决方案，打开生成的exe文件即可使用。


## 体感手套控制

体感手套控制采用蓝牙通信完成，使用者佩戴手套使用，可以实现机械手手指的弯曲和手掌的旋转。

### 功能

- 通过体感手套实时操纵机械手

### 环境配置

- 使用Keil打开STM32目录下的OpenArmSTM32.uvproj文件，设置target路径后translate并build生成hex文件。

- 将生成的hex文件通过USB烧录到机械手中，本项目采用mcuisp进行烧录。

- 使用Arduino打开lehend目录下的lehend.ino,选择相应端口后编译，无误后烧录即可。

## 视觉控制

采用摄像头获取视频，初始化计算关节角度所使用的的手部节点，初始化Mediapipe和串口、摄像头后，检测右手再进行处理。此功能还在进行调试中，相关代码在基于Mediapipe的视觉控制目录的main.py中。
