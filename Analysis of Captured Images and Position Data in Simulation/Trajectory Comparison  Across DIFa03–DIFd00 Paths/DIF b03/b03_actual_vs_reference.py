import csv
import matplotlib.pyplot as plt
import os
import numpy as np
import matplotlib

# === 设置全局字体为 Calibri，字号为五号（约 10.5pt） ===
matplotlib.rcParams['font.family'] = 'Calibri'
matplotlib.rcParams['font.size'] = 10.5

# === 文件路径设置 ===
csv_file_name = "log_3"  # 替换为你的 CSV 文件名
csv_file_path = os.path.join(os.getcwd(), csv_file_name + ".csv")

# ✅ 检查文件是否存在
if not os.path.exists(csv_file_path):
    print(f"Error: CSV file '{csv_file_path}' not found. Please check the file path.")
    exit()


# === 读取 CSV 数据函数（仅提取 x, y）===
def read_position_data(csv_file_path):
    timestamps = []
    x_positions = []
    y_positions = []

    with open(csv_file_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            try:
                timestamp = float(row[0].split()[0])
                x = float(row[1].split()[0])
                y = float(row[2].split()[0])

                timestamps.append(timestamp)
                x_positions.append(x)
                y_positions.append(y)
            except (ValueError, IndexError):
                continue

    return x_positions, y_positions


# === 绘图函数（透明背景、无边框、无标题）===
def plot_position_data(x_positions, y_positions):
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(-6, 6)
    ax.set_ylim(-6, 6)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.grid(False)

    # 去除四条边框线
    for spine in ax.spines.values():
        spine.set_visible(False)

    # 绘制轨迹线
        # 绘制轨迹线（深橙色）
        ax.plot(x_positions, y_positions, color='darkblue', linewidth=3)

    # 保存为 SVG 矢量图，透明背景
    plt.savefig("trajectory_plot_3.svg", format="svg", transparent=True, bbox_inches='tight')
    plt.close()
    print("Trajectory plot saved as 'trajectory_plot_3.svg'")


# === 主程序入口 ===
if __name__ == "__main__":
    x_positions, y_positions = read_position_data(csv_file_path)
    plot_position_data(x_positions, y_positions)
