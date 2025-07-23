import cv2  # cv2 用于读取和处理图像文件，opencv读取的格式是BGR
import numpy as np
import matplotlib.pyplot as plt
import os
from collections import deque
from statistics import mode, StatisticsError


lower_yellow = np.array([18, 84, 140])
upper_yellow = np.array([175, 255, 255])
# 定义黄色在 HSV 色彩空间中的范围,lower_yellow 和 upper_yellow 分别是黄色的下限和上限 HSV 值。

bin_size = 10  # 设置 bin_size，可以根据需要调整


def calculate_ratio_dict(image, bin_size):
    # 获取图像的高度 (height) 和宽度 (width)。
    height, width = image.shape
    # 将图像的宽度和高度分别除以 bin_size，得到宽度方向和高度方向的块数量 (bin_width 和 bin_height)。
    bin_width = width // bin_size
    bin_height = height // bin_size
    # 初始化两个空字典，分别用于存储宽度方向和高度方向的白色像素比例。
    white_pixel_ratios_width = {}
    white_pixel_ratios_height = {}

    # 开始一个循环，用于计算每一块的白色像素比例。
    # start 变量表示当前块的起始列，end 变量表示当前块的结束列（保证不超过图像的宽度）。
    for i in range(bin_width):
        start = i * bin_size
        end = min((i + 1) * bin_size, width)
        # 计算当前块中的白色像素数（像素值等于255）。
        white_pixels = np.sum(image[:, start:end] == 255)
        # 计算当前块中的总像素数。
        total_pixels = (end - start) * height
        # 计算白色像素比例，并将其四舍五入到小数点后三位。
        ratio = round(white_pixels / total_pixels, 3)
        # 将计算出的白色像素比例存储在 white_pixel_ratios_width 字典中，键为当前块的索引 i。
        white_pixel_ratios_width[i] = ratio

    # 开始另一个循环，用于计算高度方向每一块的白色像素比例。
    # start 变量表示当前块的起始行，end 变量表示当前块的结束行（保证不超过图像的高度）。
    for i in range(bin_height):
        start = i * bin_size
        end = min((i + 1) * bin_size, height)
        # 计算当前块中的白色像素数（像素值等于255）。
        white_pixels = np.sum(image[start:end, :] == 255)
        # 计算当前块中的总像素数。
        total_pixels = (end - start) * width
        # 计算白色像素比例，并将其四舍五入到小数点后三位。
        ratio = round(white_pixels / total_pixels, 3)
        # 将计算出的白色像素比例存储在 white_pixel_ratios_height 字典中，键为当前块的索引 i。
        white_pixel_ratios_height[i] = ratio
    # 返回两个字典，分别包含宽度方向和高度方向每一块的白色像素比例。
    return white_pixel_ratios_width, white_pixel_ratios_height


def analyze_trends_in_bins(bins, ratios):
    """
    分析白色像素比率的变化趋势：从一个簇到下一个簇的变化是增加、减少，还是保持不变。
    簇是指至少有2个连续数据点的相同白色像素比率。
    """
    trends = []

    # Step 1: 过滤掉白色像素比率为0的数据点
    non_zero_bins = [bin for bin, ratio in zip(bins, ratios) if ratio != 0]
    non_zero_ratios = [ratio for ratio in ratios if ratio != 0]

    # Step 2: 检查是否有足够的数据点
    if len(non_zero_ratios) < 2:  # 需要至少2个非零点
        return ['constant'] if len(non_zero_ratios) == 1 else []

    # Step 3: 识别簇：至少2个连续的白色像素比率相同的数据点
    clusters = []
    current_cluster = [non_zero_ratios[0]]  # 初始化第一个簇

    for i in range(1, len(non_zero_ratios)):
        if non_zero_ratios[i] == non_zero_ratios[i - 1]:
            current_cluster.append(non_zero_ratios[i])  # 连续相同比率，加入当前簇
        else:
            # 簇结束时检查是否满足至少两个数据点
            if len(current_cluster) >= 2:
                clusters.append(current_cluster)
            current_cluster = [non_zero_ratios[i]]  # 开始新簇

    # 检查最后一个簇是否符合条件
    if len(current_cluster) >= 2:
        clusters.append(current_cluster)

    # 输出簇的数量
    print(f"簇的数量: {len(clusters)}")

    # Step 4: 如果只有一个簇，返回 constant
    if len(clusters) == 1:
        return ['constant']

    # Step 5: 计算簇与簇之间的变化趋势
    for i in range(1, len(clusters)):
        prev_cluster_mean = sum(clusters[i - 1]) / len(clusters[i - 1])  # 前一个簇的平均比率
        curr_cluster_mean = sum(clusters[i]) / len(clusters[i])  # 当前簇的平均比率

        # 判断簇的变化趋势
        if curr_cluster_mean > prev_cluster_mean:
            trends.append('increase')
        elif curr_cluster_mean < prev_cluster_mean:
            trends.append('decrease')
        else:
            trends.append('constant')

    return trends

#当标识为Line时，用数字1表示，当标识为Cross时，用数字2表示，标识为L且朝向为Left时，用数字3表示，标识为L且朝向为Right时，用数字4表示，
# 当标识为T且T的朝向为Up时，用数字5表示，当标识为T且T的朝向为Down时，用数字6表示，当标识为T且T的朝向为Left时，用数字7表示，当标识为T且T的朝向为Right时，用数字8表示，
# 当形状标识为“Unkown”，用数字0表示
#def identify_shape(image, width_ratios, height_ratios):
def identify_marker_shape_and_orientation(width_trends, height_trends):
    # 定义趋势判定结果的初始值，默认为 "Unknown"
    shape_orientation = "Unknown"
    shape_code = 1  # Unknown 对应的数字编码

    # 判断标识的形状及朝向
    if (width_trends == ['constant'] or width_trends == []) and height_trends == ['constant']:
        shape_orientation = "Line"
        shape_code = 2  # Line
    elif width_trends == ['increase', 'decrease'] and height_trends == ['increase', 'decrease']:
        shape_orientation = "Cross"
        shape_code = 3
    elif width_trends == ['decrease'] and (height_trends == ['increase'] or height_trends == ['decrease']):
        shape_orientation = "L (Right)"
        shape_code = 4  # L 朝右
    elif width_trends == ['increase'] and (height_trends == ['decrease'] or height_trends == ['increase']):
        shape_orientation = "L (Left)"
        shape_code = 5  # L 朝左
    elif width_trends == ['increase', 'decrease'] and height_trends == ['increase']:
        shape_orientation = "T (Up)"
        shape_code = 6  # T 朝上
    elif width_trends == ['increase', 'decrease'] and height_trends == ['decrease']:
        shape_orientation = "T (Down)"
        shape_code = 7  # T 朝下
    elif width_trends == ['decrease'] and height_trends == ['increase', 'decrease']:
        shape_orientation = "T (Right)"
        shape_code = 8  # T 朝右
    elif width_trends == ['increase'] and height_trends == ['increase', 'decrease']:
        shape_orientation = "T (Left)"
        shape_code = 9  # T 朝左
    else:
        shape_orientation = "Unknown"
        shape_code = 1  # Unknown

    # 输出形状和朝向
    print(f"Marker shape and orientation: {shape_orientation}")
    print(f"Shape code: {shape_code}")

    return shape_code

def plot_white_pixel_ratios(white_pixel_ratios_width, white_pixel_ratios_height, image, filename):
    # 获取当前工作目录
    current_directory = os.getcwd()

    # 定义保存图像的文件夹路径（在当前目录下的 white_pixel_ratios_images 文件夹）
    folder_path = os.path.join(current_directory, 'white_pixel_ratios_images')

    # 确保文件夹存在，如果不存在则创建它
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    # 创建一个新的图像窗口，指定 num 参数来重用窗口，避免创建过多的窗口
    plt.figure(figsize=(12, 6), num=1)

    # 清除图像内容，确保图形窗口被重新使用时没有残留数据
    plt.clf()

    # 将字典转换为列表形式，方便处理
    width_bins = list(white_pixel_ratios_width.keys())
    width_ratios = list(white_pixel_ratios_width.values())

    height_bins = list(white_pixel_ratios_height.keys())
    height_ratios = list(white_pixel_ratios_height.values())

    # 对数据进行修正处理，处理连续值相差小于0.01的情况（宽度方向）
    def process_ratios(bins, ratios):
        corrected_bins = []
        corrected_ratios = []
        temp_bins = []
        temp_ratios = []

        for i in range(len(ratios)):
            if not temp_ratios or abs(ratios[i] - temp_ratios[-1]) < 0.01:
                temp_bins.append(bins[i])
                temp_ratios.append(ratios[i])
            else:
                # 对连续相差小于0.01的部分取平均值
                avg_ratio = sum(temp_ratios) / len(temp_ratios)
                corrected_bins.extend(temp_bins)
                corrected_ratios.extend([avg_ratio] * len(temp_ratios))

                # 清空临时存储，开始新的分组
                temp_bins = [bins[i]]
                temp_ratios = [ratios[i]]

        # 最后剩余的部分
        if temp_ratios:
            avg_ratio = sum(temp_ratios) / len(temp_ratios)
            corrected_bins.extend(temp_bins)
            corrected_ratios.extend([avg_ratio] * len(temp_ratios))

        return corrected_bins, corrected_ratios

    # 修正后的宽度和高度数据
    corrected_width_bins, corrected_width_ratios = process_ratios(width_bins, width_ratios)
    corrected_height_bins, corrected_height_ratios = process_ratios(height_bins, height_ratios)

    # 调用 analyze_trends_in_bins 处理修正后的宽度和高度数据
    width_trends = analyze_trends_in_bins(corrected_width_bins, corrected_width_ratios)
    height_trends = analyze_trends_in_bins(corrected_height_bins, corrected_height_ratios)

    # 输出趋势
    print(f"Width trends: {width_trends}")
    print(f"Height trends: {height_trends}")

    # 调用 identify_marker_shape_and_orientation 来识别标志形状和朝向
    shape_code = identify_marker_shape_and_orientation(width_trends, height_trends)

    # 输出形状和朝向编码
    print(f"Identified marker shape code: {shape_code}")

    # 绘制宽度方向的白色像素比例折线图
    plt.subplot(1, 2, 1)
    plt.plot(corrected_width_bins, corrected_width_ratios, marker='o', linestyle='-', color='b', linewidth=2)
    plt.title('White Pixel Ratio along Width')
    plt.xlabel('Bin Index (Width)')
    plt.ylabel('White Pixel Ratio')
    plt.grid(True)

    # 绘制高度方向的白色像素比例折线图
    plt.subplot(1, 2, 2)
    plt.plot(corrected_height_bins, corrected_height_ratios, marker='o', linestyle='-', color='r', linewidth=2)
    plt.title('White Pixel Ratio along Height')
    plt.xlabel('Bin Index (Height)')
    plt.ylabel('White Pixel Ratio')
    plt.grid(True)

    # 调整子图布局，避免重叠
    plt.tight_layout()

    # 生成图像的文件名，确保每个图像有唯一的名称
    base_filename = os.path.splitext(filename)[0]  # 去掉扩展名
    file_name = f"{base_filename}_white_pixel_ratios.jpg"
    save_path = os.path.join(folder_path, file_name)

    # 保存图像
    plt.savefig(save_path)
    print(f"Image saved to {save_path}")

    # 显式关闭图像，避免打开过多图像导致内存问题
    plt.close()

    return shape_code




class ShapeCodeProcessor:
    def __init__(self, window_size=3):
        self.window_size = window_size
        self.window = []

    def process_code(self, new_code):
        # 添加新代码到窗口
        self.window.append(new_code)
        if len(self.window) > self.window_size:
            self.window.pop(0)

        try:
            # 尝试计算众数
            dominant_code = mode(self.window)
        except StatisticsError:
            # 如果没有唯一众数，返回窗口中的第一个值
            dominant_code = self.window[0]

        return dominant_code


def process_images_and_plot():
    # 指定图片所在的文件夹路径
    image_folder = 'course1yuan_PID_images'

    # 获取文件夹中所有文件名
    all_files = os.listdir(image_folder)

    # 过滤出以 'image_' 开头的 jpg 图片文件
    image_filenames = [f for f in all_files if f.startswith('image_') and f.endswith('.jpg')]

    # 提取文件名中的数字部分，用来排序
    image_filenames.sort(key=lambda f: int(f.split('_')[1].split('.')[0]))

    # 输出调试信息，检查图像总数
    print(f"找到 {len(image_filenames)} 张图像进行处理")

    # 结合文件夹路径和文件名，生成图像的完整路径
    image_paths = [os.path.join(image_folder, filename) for filename in image_filenames]

    # 存储图像识别结果
    shape_codes = []

    # 引入滑动窗口处理器
    processor = ShapeCodeProcessor(window_size=3)

    # 引入变量记录上一帧的形状代码
    last_shape_code = None

    # 对每张图像进行处理
    for image_path, filename in zip(image_paths, image_filenames):
        print(f"处理图像: {filename}")
        # 读取图像
        img = cv2.imread(image_path)

        # 保存读取的原始图像
        save_path = os.path.join(image_folder, 'processed_' + filename)
        cv2.imwrite(save_path, img)

        ### Step 0: 检测是否存在红色 ###
        # 转换为 HSV 色彩空间
        hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 定义红色的 HSV 范围
        lower_red1 = np.array([0, 100, 100])    # 红色低范围1
        upper_red1 = np.array([10, 255, 255])  # 红色高范围1
        lower_red2 = np.array([170, 100, 100]) # 红色低范围2
        upper_red2 = np.array([180, 255, 255]) # 红色高范围2

        # 创建红色掩码
        mask_red1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        # 计算红色像素的数量
        red_pixel_count = cv2.countNonZero(mask_red)

        # 如果红色像素数量超过一定阈值，标记为初始位置
        if red_pixel_count > 350:  # 假设 350 是一个合理的阈值，根据需要调整
            print(f"图像 {filename} 检测到红色，标记为无人机初始位置 (0)")
            shape_codes.append(0)
            last_shape_code = 0  # 更新上一帧形状代码
            continue  # 跳过后续处理

        ### 后续的图像处理步骤 ###
        # Step 1: 将图像从 BGR 转换为 HSV
        mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        # Step 2: 对掩码应用中值滤波
        mask_blurred = cv2.medianBlur(mask, 5)

        # Step 3: 形态学闭运算
        kernel = np.ones((5, 5), np.uint8)
        mask_morph = cv2.morphologyEx(mask_blurred, cv2.MORPH_CLOSE, kernel)

        # 保存处理后的掩码图像
        result_path = os.path.join(image_folder, 'processed_' + filename)
        cv2.imwrite(result_path, mask_morph)

        # 重新读取保存的图像
        processed_image = cv2.imread(result_path, cv2.IMREAD_GRAYSCALE)

        # 调用 calculate_ratio_dict 函数计算白色像素比例
        white_pixel_ratios_width, white_pixel_ratios_height = calculate_ratio_dict(processed_image, bin_size)

        # 调用 plot_white_pixel_ratios 并接收返回的 shape_code
        shape_code = plot_white_pixel_ratios(white_pixel_ratios_width, white_pixel_ratios_height, processed_image,
                                             filename)

        # 检查是否为 Unknown 形状 (代码 1)
        if shape_code == 1:
            if last_shape_code is not None:
                print(f"图像 {filename} 识别为未知形状，将形状代码调整为上一帧的形状代码 {last_shape_code}")
                shape_code = last_shape_code  # 调整为上一帧的形状代码
            else:
                print(f"图像 {filename} 识别为未知形状，但无上一帧数据，保持为 1")

        smoothed_shape_code = processor.process_code(shape_code)
        shape_codes.append(smoothed_shape_code)
        last_shape_code = smoothed_shape_code

        print(f"图像 {filename}: 原始形状代码={shape_code}, 平滑后={smoothed_shape_code}")
        # 最终输出所有形状代码
    #print(f"最终平滑后的形状代码序列: {shape_codes}")


    # 生成图像范围和形状代码的输出
    output = []
    start = 0
    for i in range(1, len(shape_codes)):
        if shape_codes[i] != shape_codes[start]:
            output.append(f"[{start}-{i - 1}, {shape_codes[start]}]")
            start = i
    output.append(f"[{start}-{len(shape_codes) - 1}, {shape_codes[start]}]")  # 最后一段
    # 输出结果
    print("调整后的形状代码范围和代码:")
    print(", ".join(output))

    # 继续绘图
    plt.figure(figsize=(10, 6))
    plt.plot(range(1, len(shape_codes) + 1), shape_codes,
             marker='o', linestyle='-', color='b', label='Shape Label')
    plt.title('Image Recognition Labels Over Numbers')
    plt.xlabel('Image Processing Order')
    plt.ylabel('Recognition Label (0-9)')
    plt.yticks(range(10))
    plt.grid(True)
    plt.show()


# 主函数，只有在脚本直接执行时才会运行
if __name__ == "__main__":
    process_images_and_plot()
