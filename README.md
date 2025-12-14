# SJTU Engineering Practice: Line Follower Robot (循线小车)

本项目是上海交通大学（SJTU）工程实践课程的循线小车控制系统。基于 KRduino UNO (ATmega328P) 开发板，实现了基于 5 路灰度传感器的循线算法，包含状态记忆与差速转向控制，并配备了 Python 实时数据可视化调试工具。

## 📂 项目结构

```text
.
├── src/
│   └── main.cpp        # 核心控制算法 (状态机、传感器读取、电机控制)
├── tools/
│   └── monitor.py      # Python 串口数据可视化工具 (Matplotlib)
├── include/            # 头文件
├── lib/                # 私有库
├── platformio.ini      # PlatformIO 项目配置文件
└── README.md           # 项目说明文档
