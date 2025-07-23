import csv
import matplotlib.pyplot as plt
import os
import numpy as np

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
                timestamp = float(row[0].split()[0])
                x = float(row[1].split()[0])
                y = float(row[2].split()[0])

                timestamps.append(timestamp)
                x_positions.append(x)
                y_positions.append(y)
            except (ValueError, IndexError):
                continue
    return timestamps, x_positions, y_positions


def compute_errors(x_positions, y_positions, timestamps):
    """计算 X 方向误差 + 实际终点和参考终点的欧式距离（考虑 X 和 Y）"""

    reference_x = 0.6
    reference_y = 7.4

    x_errors = np.array([x - reference_x for x in x_positions])

    rmse_x = np.sqrt(np.mean(x_errors ** 2))
    max_error_x = np.max(np.abs(x_errors))
    mean_error_x = np.mean(np.abs(x_errors))
    std_error_x = np.std(x_errors)

    reference_end = (reference_x, reference_y)
    actual_end = (x_positions[-1], y_positions[-1])
    euclidean_distance = np.sqrt((actual_end[0] - reference_end[0]) ** 2 +
                                 (actual_end[1] - reference_end[1]) ** 2)

    return {
        "rmse_x": rmse_x,
        "max_error_x": max_error_x,
        "mean_error_x": mean_error_x,
        "std_error_x": std_error_x,
        "x_errors": x_errors,
        "timestamps": timestamps,
        "reference_x": reference_x,
        "reference_end": reference_end,
        "actual_end": actual_end,
        "euclidean_distance": euclidean_distance
    }


def plot_error_vs_time_save_svg(errors_dict, csv_file_path):
    import matplotlib.pyplot as plt
    import numpy as np
    import os

    # 从字典中提取误差和时间数据
    x_errors = np.array(errors_dict["x_errors"])
    timestamps = np.array(errors_dict["timestamps"])

    last_time = timestamps[-1] if len(timestamps) > 0 else 0
    max_error = x_errors.max() if x_errors.size > 0 else 0
    y_max = max_error * 1.1 if max_error > 0 else 1

    # ✅ 创建正方形图像，背景设为浅灰色
    fig = plt.figure(figsize=(5, 5), facecolor='white')
    ax = fig.add_subplot(1, 1, 1)
    ax.set_facecolor('white')

    # ✅ 绘图
    ax.plot(timestamps, x_errors, color='red', linestyle='-')

    # ✅ 横轴设置：仅显示起点、中点、终点
    if last_time > 0:
        mid_time = round(last_time / 2.0, 1)
        ax.set_xticks([0, mid_time, round(last_time, 1)])
        ax.set_xticklabels([f"{0:g}", f"{mid_time:g}", f"{last_time:g}"], fontsize=20)
    else:
        ax.set_xticks([0])
        ax.set_xticklabels(["0"], fontsize=20)

    # ✅ 纵轴设置：只显示3个刻度，不含起点0.00
    y_ticks = np.linspace(0, y_max, 4)[1:]  # 分成4段，去掉0段
    ax.set_yticks(y_ticks)
    ax.set_yticklabels([f"{tick:.2f}" for tick in y_ticks], fontsize=20)
    # ✅ 绘图 + 图例说明
    ax.plot(timestamps, x_errors, color='red', linestyle='-', label=r'$error_x$')
    ax.legend(
        loc='upper right',
        fontsize=24,  # 控制文字大小
        handlelength=0.5,  # 控制图例线条的长度
        handletextpad=0.5,  # 控制线条和文字之间的间距
        borderpad=0.5,  # 控制图例内容与边框之间的距离
        labelspacing=0.5,  # 控制多行之间的垂直间距
        borderaxespad=0.5  # 控制图例边框与坐标轴之间的距离
    )

    # ✅ 设置坐标轴标签，字体加大
    ax.set_xlabel('$time\ (s)$', fontsize=24)
    ax.set_ylabel(r'$error_x\ (m)$', fontsize=24)

    # ✅ 强制坐标从原点开始对齐
    ax.set_xlim(0, last_time * 1.02)

    ax.set_ylim(0, y_max)

    # ✅ 设置四边边框颜色
    # === 坐标轴样式 ===
    for side in ['top', 'bottom', 'left', 'right']:
        ax.spines[side].set_visible(True)
        ax.spines[side].set_color('black')
        ax.spines[side].set_linewidth(1.2)

    # ✅ 放大刻度线样式
    ax.tick_params(axis='both', direction='in', length=6, width=1.2)

    # 保存图像
    save_dir = os.path.dirname(os.path.abspath(csv_file_path))
    save_path = os.path.join(save_dir, "course0_x_error_plot.svg")
    plt.tight_layout()
    plt.savefig(save_path, format='svg')
    plt.close(fig)

if __name__ == "__main__":
    # 读取数据
    timestamps, x_raw, y_raw = read_position_data(csv_file_path)

    # 坐标变换
    x_adjusted = [(9.6 - y) for y in y_raw]
    y_adjusted = [(x + 5.4) for x in x_raw]

    # 计算误差
    errors = compute_errors(x_adjusted, y_adjusted, timestamps)


    # 打印误差信息
    print("[Error Analysis Results]")
    print(f"Reference X: {errors['reference_x']}")
    print(f"Actual Trajectory Start: ({x_adjusted[0]:.4f}, {y_adjusted[0]:.4f})")
    print(f"Actual Trajectory End:   ({x_adjusted[-1]:.4f}, {y_adjusted[-1]:.4f})")
    print(f"Reference End: {errors['reference_end']}")
    print(f"Actual End:    {errors['actual_end']}")
    print(f"Euclidean Distance (2D): {errors['euclidean_distance']:.4f} m")
    print(f"RMSE_X: {errors['rmse_x']:.4f} m")
    print(f"Max Error_X: {errors['max_error_x']:.4f} m")
    print(f"Mean Error_X: {errors['mean_error_x']:.4f} m")
    print(f"STD_X: {errors['std_error_x']:.4f} m")

    # 绘图
    plot_error_vs_time_save_svg(errors, csv_file_path)


