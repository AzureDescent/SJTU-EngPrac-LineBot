import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading

# ================= 配置区域 =================
# 串口设置 (请根据实际情况修改)
PORT = "COM8"  # 你的开发板端口
BAUD_RATE = 9600  # 必须与 Arduino Serial.begin() 一致

# 绘图设置
MAX_POINTS = 200  # 图表上保留的历史数据点数 (X轴长度)
AVG_WINDOW = 20  # 平均值采样窗口 (取最近多少个点取平均)
Y_MIN, Y_MAX = 0, 1024  # 传感器数值范围 (0-1023)
REFRESH_INTERVAL = 30  # 刷新间隔 (毫秒)
# ===========================================

# 初始化串口
try:
    ser = serial.Serial(PORT, BAUD_RATE, timeout=0.1)
    print(f"✅ 成功连接到 {PORT}")
    print(f"📊 正在等待数据... (按 Ctrl+C 退出)")
except serial.SerialException as e:
    print(f"❌ 无法打开串口 {PORT}: {e}")
    print("提示: 请检查端口号，或确认是否已关闭其他占用该端口的软件 (如 VSCode 监视器)")
    exit()

# 数据存储 (使用 deque 自动丢弃旧数据)
# data_queues[0] 对应 Sensor 1, data_queues[4] 对应 Sensor 5
data_queues = [deque([0] * MAX_POINTS, maxlen=MAX_POINTS) for _ in range(5)]

# 创建图表
fig, ax = plt.subplots(figsize=(10, 6))
fig.canvas.manager.set_window_title("Line Follower Sensor Monitor") # type: ignore

# 初始化 5 条曲线，使用不同颜色
colors = ["#FF0000", "#FFA500", "#008000", "#0000FF", "#800080"]  # 红, 橙, 绿, 蓝, 紫
lines = []
for i in range(5):
    (line,) = ax.plot(
        [], [], label=f"Sensor {i+1} (A{i})", color=colors[i], linewidth=1.5
    )
    lines.append(line)

# 设置图表样式
ax.set_ylim(Y_MIN, Y_MAX)
ax.set_xlim(0, MAX_POINTS)
ax.set_title(f"Real-time Sensor Data (Moving Average Window: {AVG_WINDOW})")
ax.set_ylabel("Analog Value (0-1023)")
ax.set_xlabel("Time (Samples)")
ax.grid(True, linestyle="--", alpha=0.5)
ax.legend(loc="upper right")

# bbox 参数用于绘制边框和背景色
info_text = ax.text(
    0.02,
    0.95,
    "",
    transform=ax.transAxes,
    verticalalignment="top",
    bbox=dict(boxstyle="round,pad=0.5", fc="white", ec="gray", alpha=0.9),
)


def calculate_average(queue, window_size):
    """计算队列中最后 N 个点的平均值"""
    # 获取最近的数据切片
    recent_data = list(queue)[-window_size:]
    if not recent_data:
        return 0.0
    return sum(recent_data) / len(recent_data)


def update_plot(frame):
    # --- 1. 读取串口数据 ---
    # 循环读取缓冲区直到清空，防止绘图跟不上串口速度
    while ser.in_waiting:
        try:
            line = ser.readline().decode("utf-8").strip()
            if not line:
                continue

            # 解析数据 (期望格式: 100,200,300,400,500)
            parts = line.split(",")
            if len(parts) == 5:
                vals = [int(p) for p in parts]
                # 更新队列
                for i in range(5):
                    data_queues[i].append(vals[i])
        except (ValueError, UnicodeDecodeError):
            continue  # 忽略坏数据

    # --- 2. 更新曲线 ---
    for i, line in enumerate(lines):
        line.set_ydata(data_queues[i])
        line.set_xdata(range(len(data_queues[i])))

    # --- 3. 计算平均值并更新文本框 ---
    status_str = "🔍 Current Averages:\n"
    status_str += "-" * 20 + "\n"

    for i in range(5):
        avg_val = calculate_average(data_queues[i], AVG_WINDOW)
        # 获取当前瞬时值
        current_val = data_queues[i][-1]

        # 格式化字符串: Sensor 1:  102.5 (Now: 100)
        status_str += f"S{i+1}: {avg_val:>6.1f}  (Now: {current_val:>4})\n"

    info_text.set_text(status_str)

    return lines + [info_text]


# 启动动画
ani = animation.FuncAnimation(fig, update_plot, interval=REFRESH_INTERVAL, blit=True)

# 显示图表
plt.tight_layout()
plt.show()

# 关闭程序时释放串口
ser.close()
