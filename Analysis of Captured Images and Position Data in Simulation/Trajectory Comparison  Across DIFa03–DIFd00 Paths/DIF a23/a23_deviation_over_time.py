import csv
import matplotlib.pyplot as plt
import os
import numpy as np

# File path for CSV
csv_file_name = "log_23_2"
csv_file_path = os.path.join(os.getcwd(), csv_file_name + ".csv")


def read_position_data(csv_file_path):
    import pandas as pd

    # 使用 GBK 编码读取 CSV（因为是中文环境）
    df = pd.read_csv(csv_file_path, encoding='gbk')

    # 提取时间戳、位置坐标，去掉单位
    timestamps = df['Timestamp (s)'].astype(str).str.replace('s', '').astype(float).tolist()
    x_positions = df['Position_X (m)'].astype(str).str.replace('m', '').astype(float).tolist()
    y_positions = df['Position_Y (m)'].astype(str).str.replace('m', '').astype(float).tolist()

    # 提取 rotation_end_time（s） 列的第一个非空值
    rot_col_name = [col for col in df.columns if 'rotation_end_time' in col][0]
    rotation_end_time = df[rot_col_name].dropna().astype(float).values[0] if df[rot_col_name].notna().any() else None

    return timestamps, x_positions, y_positions, rotation_end_time



def compute_errors(x_positions, y_positions, timestamps, rotation_end_time):
    """根据旋转结束时间划分两个阶段，并计算误差"""
    reference_y = []
    reference_x = []

    # 找到关键时间索引
    t_array = np.array(timestamps)
    if rotation_end_time is None:
        raise ValueError("Rotation End Time not found in CSV")

    idx_rot_end = np.where(t_array > rotation_end_time)[0]
    if len(idx_rot_end) < 2:
        raise ValueError("Not enough timestamps after Rotation End Time")

    idx_split1 = idx_rot_end[0]     # Rotation End Time 后第一时刻索引
    idx_split2 = idx_rot_end[1]     # Rotation End Time 后第二时刻索引

    # 构建参考轨迹
    for i, t in enumerate(timestamps):
        if i <= idx_split1:
            reference_y.append(0.6)
            reference_x.append(x_positions[i])  # 保持原值
        else:
            if i == len(timestamps) - 1:
                reference_y.append(7.4)  # ✅ 最后一项强制为 7.4
            else:
                reference_y.append(y_positions[i])
            reference_x.append(7.8)

    # 误差计算
    x_errors = np.array([x_act - x_ref for x_act, x_ref in zip(x_positions, reference_x)])
    y_errors = np.array([y_act - y_ref for y_act, y_ref in zip(y_positions, reference_y)])

    # 指标统计
    rmse_x = np.sqrt(np.mean(x_errors ** 2))
    rmse_y = np.sqrt(np.mean(y_errors ** 2))
    max_error_x = np.max(np.abs(x_errors))
    max_error_y = np.max(np.abs(y_errors))
    mean_error_x = np.mean(np.abs(x_errors))
    mean_error_y = np.mean(np.abs(y_errors))
    std_error_x = np.std(x_errors)
    std_error_y = np.std(y_errors)

    reference_end = (reference_x[-1], reference_y[-1])
    actual_end = (x_positions[-1], y_positions[-1])
    euclidean_distance = np.sqrt((reference_end[0] - actual_end[0]) ** 2 + (reference_end[1] - actual_end[1]) ** 2)

    return {
        "rmse_x": rmse_x, "rmse_y": rmse_y,
        "max_error_x": max_error_x, "max_error_y": max_error_y,
        "mean_error_x": mean_error_x, "mean_error_y": mean_error_y,
        "std_error_x": std_error_x, "std_error_y": std_error_y,
        "x_positions": x_positions,
        "y_positions": y_positions,
        "timestamps": timestamps,
        "reference_x": reference_x,
        "reference_y": reference_y,
        "reference_start": (reference_x[0], reference_y[0]),
        "reference_end": reference_end,
        "actual_end": actual_end,
        "euclidean_distance": euclidean_distance,
        "idx_split1": idx_split1,
        "idx_split2": idx_split2,
        "rotation_end_time": rotation_end_time
    }


def plot_merged_error_svg(errors, csv_file_path):
    import matplotlib.pyplot as plt
    import numpy as np
    import os
    from matplotlib.ticker import FormatStrFormatter

    # 提取数据
    timestamps = np.array(errors["timestamps"])
    y_positions = np.array(errors["y_positions"])
    x_positions = np.array(errors["x_positions"])
    reference_y = np.array(errors["reference_y"])
    reference_x = np.array(errors["reference_x"])
    rotation_end_time = errors["rotation_end_time"]

    # === 构造误差 ===
    y_errors = np.abs(y_positions - reference_y)
    x_errors = np.abs(x_positions - reference_x)

    idx_split = errors["idx_split1"]  # 用 split1 作为切换点
    t_y = timestamps[:idx_split + 1]
    err_y = y_errors[:idx_split + 1]
    t_x = timestamps[idx_split + 1:]
    err_x = x_errors[idx_split + 1:]

    # === 时间轴刻度设置 ===
    t_start = round(timestamps[0], 2)
    t_rot = round(rotation_end_time, 2)
    t_end = round(timestamps[-1], 2)
    xticks = [t_start, t_rot, t_end]

    # === 最大误差决定Y轴范围 ===
    all_errors = np.concatenate([err_y, err_x])
    y_max = max(all_errors) * 1.1 if len(all_errors) > 0 else 1
    y_max = round(y_max, 2)

    # === 创建正方形画布 ===
    fig, ax = plt.subplots(figsize=(6, 6), facecolor='white')
    ax.set_facecolor('white')

    # === 绘图 ===
    ax.plot(t_y, err_y, color='blue', linestyle='-', linewidth=2, label=r'$error_y$')
    ax.plot(t_x, err_x, color='red', linestyle='-', linewidth=2, label=r'$error_x$')
    # ✅ 添加旋转结束时刻的竖线
    ax.axvline(rotation_end_time, color='black', linestyle=':', linewidth=1.2)

    # === 坐标轴设置 ===
    ax.set_xlim(0, t_end * 1.02)
    ax.set_ylim(0, y_max)
    ax.set_xlabel("$time\ (s)$", fontsize=24)
    ax.set_ylabel(r"$error_{x/y}\ (m)$", fontsize=24)

    # === 坐标刻度简化 ===
    ax.set_xticks(xticks)
    ax.set_xticklabels([f"{v:g}" for v in xticks], fontsize=20)
    yticks = np.linspace(0, y_max, 4)[1:]
    ax.set_yticks(yticks)
    ax.set_yticklabels([f"{v:.2f}" for v in yticks], fontsize=20)
    ax.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))

    # === 坐标轴样式 ===
    for side in ['top', 'bottom', 'left', 'right']:
        ax.spines[side].set_visible(True)
        ax.spines[side].set_color('black')
        ax.spines[side].set_linewidth(1.2)

    ax.tick_params(axis='both', direction='in', length=6, width=1.2)

    # === 图例右上角 ===
    ax.legend(
        loc='upper left',
        fontsize=24,  # 控制文字大小
        handlelength=0.5,  # 控制图例线条的长度
        handletextpad=0.5,  # 控制线条和文字之间的间距
        borderpad=0.5,  # 控制图例内容与边框之间的距离
        labelspacing=0.5,  # 控制多行之间的垂直间距
        borderaxespad=0.5  # 控制图例边框与坐标轴之间的距离
    )

    # === 保存图像为SVG ===
    save_dir = os.path.dirname(os.path.abspath(csv_file_path))
    save_path = os.path.join(save_dir, "course2_error_plot.svg")
    plt.tight_layout(pad=2.0)
    plt.savefig(save_path, format='svg')
    plt.close(fig)
    print(f"✅ 图像已保存至: {save_path}")

if __name__ == "__main__":
    # 读取数据
    timestamps, x_raw, y_raw, rotation_end_time = read_position_data(csv_file_path)

    # 坐标变换
    x_adjusted = [(9.6 - y) for y in y_raw]
    y_adjusted = [(x + 5.4) for x in x_raw]

    # 误差计算
    errors = compute_errors(x_adjusted, y_adjusted, timestamps, rotation_end_time)

    # 打印分析
    print("[Error Analysis Results]")
    print(f"Reference Trajectory Start: {errors['reference_start']}")
    print(f"Reference Trajectory End: {errors['reference_end']}")
    print(f"Actual Trajectory Start: ({x_adjusted[0]:.4f}, {y_adjusted[0]:.4f})")
    print(f"Actual Trajectory End: ({x_adjusted[-1]:.4f}, {y_adjusted[-1]:.4f})")
    print(f"Euclidean Distance between Reference and Actual Endpoints: {errors['euclidean_distance']:.4f} m")
    print(f"RMSE_X: {errors['rmse_x']:.4f} m, RMSE_Y: {errors['rmse_y']:.4f} m")
    print(f"Max Error_X: {errors['max_error_x']:.4f} m, Max Error_Y: {errors['max_error_y']:.4f} m")
    print(f"Mean Error_X: {errors['mean_error_x']:.4f} m, Mean Error_Y: {errors['mean_error_y']:.4f} m")
    print(f"STD_X: {errors['std_error_x']:.4f} m, STD_Y: {errors['std_error_y']:.4f} m")

    # 绘图

    plot_merged_error_svg(errors, csv_file_path)

