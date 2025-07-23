import csv
import matplotlib.pyplot as plt
import os

# File path for CSV
csv_file_name = "log_03_2"
csv_file_path = os.path.join(os.getcwd(), csv_file_name + ".csv")

def read_position_data(csv_file_path):
    timestamps = []
    x_positions = []
    y_positions = []

    with open(csv_file_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            try:
                timestamp = float(row[0].split()[0])  # Parse timestamp
                x = float(row[1].split()[0])          # Parse position.x
                y = float(row[2].split()[0])          # Parse position.y

                timestamps.append(timestamp)
                x_positions.append(x)
                y_positions.append(y)
            except (ValueError, IndexError):
                continue  # Skip invalid rows

    return timestamps, x_positions, y_positions

def plot_position_data(x_positions, y_positions):
    if not x_positions or not y_positions:
        print("Error: No valid position data to plot.")
        return

    # 定义物理参数
    fig_width_m = 0.192  # 画布宽度（米）
    fig_height_m = 0.108  # 画布高度（米）

    # 计算画布尺寸（英寸）
    inches_per_meter = 1 / 0.002  # 1 米 = 39.37 英寸
    fig_width_inch = fig_width_m * inches_per_meter
    fig_height_inch = fig_height_m * inches_per_meter

    # 设置画布和DPI
    dpi = 100  # 可根据显示需求调整
    plt.figure(figsize=(fig_width_inch, fig_height_inch), dpi=dpi, facecolor='none')

    # 绘制无人机飞行轨迹（深绿色，粗线，实线）
    plt.plot(x_positions, y_positions, color=(0, 139 / 255, 0), linewidth=20, linestyle='-')

    # 绘制参考线（深灰色，细线，虚线）
    plt.plot([0.006, 0.006], [0.006, 0.074], color=(105 / 255, 105 / 255, 105 / 255), linewidth=10, linestyle='--')

    # 绘制原点
    plt.scatter(0, 0, color='black', s=5)

    # 坐标系范围
    plt.xlim(0, fig_width_m)
    plt.ylim(0, fig_height_m)

    # 隐藏坐标轴刻度
    plt.xticks([])
    plt.yticks([])

    # 隐藏边框
    ax = plt.gca()
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_visible(False)
    ax.spines['bottom'].set_visible(False)
    ax.set_facecolor('none')

    # 保存透明背景的图片（新增代码）
    output_image_path = os.path.join(os.getcwd(), csv_file_name + "_plot.png")
    plt.savefig(output_image_path, transparent=True, bbox_inches='tight', pad_inches=0)
    print(f"图片已保存至: {output_image_path}")
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # 读取数据并调整坐标
    timestamps, x_raw, y_raw = read_position_data(csv_file_path)

    # 坐标变换（单位：米）
    x_adjusted = [(9.6 - y) * 0.01 for y in y_raw]  # 缩放后的x坐标
    y_adjusted = [(x + 5.4) * 0.01 for x in x_raw]  # 缩放后的y坐标

    # 绘制图形
    plot_position_data(x_adjusted, y_adjusted)
