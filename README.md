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
````

## ✨ 功能特性

* **状态记忆算法**：当小车冲出跑道（所有传感器丢失黑线）时，系统会自动记忆最后一次的偏差方向（左或右），并持续向该方向修正，防止“无头苍蝇”式乱撞。
* **差速转向控制**：根据偏离程度（一级/二级偏差），自动调整左右轮差速，甚至在急弯时反转内侧轮，实现平滑且快速的过弯。
* **实时可视化调试**：提供基于 `matplotlib` 的 Python 脚本，通过串口实时绘制 5 路传感器的波形图，直观展示黑白阈值与传感器噪声。
* **KRduino 适配**：针对 KRduino UNO R3 的电机驱动引脚进行了封装适配。

## 🛠️ 硬件接线说明

### 传感器 (5路灰度)

| 传感器位置 | 变量名 | Arduino 引脚 |
| :--- | :--- | :--- |
| 最左 (←←) | `SENSOR_L2` | A4 |
| 左 (←) | `SENSOR_L1` | A3 |
| 中 | `SENSOR_M` | A2 |
| 右 (→) | `SENSOR_R1` | A1 |
| 最右 (→→) | `SENSOR_R2` | A0 |

*注：`main.cpp` 中定义 `THRESHOLD` 为 740（低于此值为黑线）。*

### 电机驱动 (KRduino 板载)

| 电机组 | 功能 | 变量名 | 引脚 |
| :--- | :--- | :--- | :--- |
| **B组 (左轮)** | 方向 | `B_DIR` | D4 |
| | PWM速度 | `B_PWM` | D5 |
| **A组 (右轮)** | PWM速度 | `A_PWM` | D6 |
| | 方向 | `A_DIR` | D7 |

### 舵机

* **信号线**: D8 (`PIN_SERVO`)

## 🚀 快速开始

### 1\. 固件烧录

本项目使用 **PlatformIO** 进行开发。

1. 在 VSCode 中安装 PlatformIO 插件。
2. 打开本文件夹。
3. 连接开发板（确保已安装 CH340 或对应驱动）。
4. 点击底部状态栏的 `→` (Upload) 按钮进行烧录。

### 2\. 调试参数

在 `src/main.cpp` 顶部可调整核心参数：

```cpp
const int THRESHOLD = 700;    // 黑白阈值 (根据场地光线调整)
const int BASE_SPEED = 140;   // 基础巡航速度
const int DIFF_SPEED = 60;    // 转向差速力度
const int SERVO_CENTER = 98;  // 舵机物理中值
```

### 3\. 使用可视化工具

在调试阈值时，建议使用 Python 脚本观察传感器数据。

**前置要求**:

* Python 3.x
* 安装依赖: `pip install pyserial matplotlib`

**运行**:

1. 确保开发板已连接，且 **VSCode 的串口监视器已关闭**（防止端口占用）。
2. 在终端运行：

    ```bash
    python tools/monitor.py
    ```

3. 脚本会自动连接 COM 口（需在脚本中修改 `PORT` 变量），并显示 5 条传感器曲线及实时平均值。

## 📝 开发日志

* **V1.0**: 完成基础循线逻辑，集成 KRduino 电机驱动。
* **V1.1**: 引入 Python 可视化工具，优化阈值判定逻辑。
* **V1.2**: 增加“状态记忆”功能，解决急弯丢线问题。

## 📄 License

MIT License
