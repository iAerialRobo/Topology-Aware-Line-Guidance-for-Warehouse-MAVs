# Python标准库模块
import time
import math
import threading
import os
from collections import deque
from statistics import mode, StatisticsError
import csv

# 第三方库
import numpy as np
import cv2
import matplotlib.pyplot as plt

# 特定项目的库和模块
import rospy
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rospy import Rate
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
# 验证导入是否成功
print("cv2 imported successfully")
print("cv bridge imported successfully")

# Connect to the simulated vehicle in Gazebo
print("Connecting to the vehicle...")
connection_string = 'tcp:127.0.0.1:5762'
vehicle = connect(connection_string, wait_ready=True, baud=57600)
print("Connected to IRIS")



def condition_yaw(vehicle, heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    """
    if relative:
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle

    if heading > 0:
        direction = 1
    else:
        direction = -1

    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        abs(heading),  # param 1, yaw in degrees
        0,  # param 2, yaw speed deg/s
        direction,  # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)  # param 5 ~ 7 not used

    # send command to vehicle
    vehicle.send_mavlink(msg)


def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration, frequency=10):
    """
    Move vehicle in direction based on specified velocity vectors.
    Args:
        vehicle: Dronekit vehicle object.
        velocity_x (float): Forward velocity in m/s.
        velocity_y (float): Sideward velocity in m/s.
        velocity_z (float): Downward/upward velocity in m/s.
        duration (float): Duration to send velocity commands.
        frequency (int): Command update frequency in Hz (default: 20 Hz).
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # Send the velocity command repeatedly for the specified duration
    interval = 1.0 / frequency
    start_time = time.time()
    while time.time() - start_time < duration:
        vehicle.send_mavlink(msg)
        time.sleep(interval)  # Send at defined frequency


def arm_and_takeoff(target_altitude):
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off")
    vehicle.simple_takeoff(target_altitude)

    while True:
        altitude = vehicle.location.global_relative_frame.alt
        print("Altitude: ", altitude)
        if altitude >= target_altitude * 0.9:
            print("Reached target altitude")
            break
        time.sleep(1)

    print("Hovering for 2 seconds")
    time.sleep(2)
    print("Hover complete")


def hover(vehicle, hover_time):


    print("Hovering in place for", hover_time, "seconds.")

    # 发送零速度指令以停止无人机的所有移动（悬停）
    send_ned_velocity(vehicle, 0, 0, 0, hover_time)

    # 悬停计时
    time.sleep(hover_time)

    print("Hover complete.")


def land(camera_subscriber):
    print("Landing")
    vehicle.mode = VehicleMode("LAND")

    # 等待无人机降落并解除武装
    while vehicle.armed:
        time.sleep(1)

    print("Disarming motors")
    vehicle.armed = False

    # 取消订阅摄像头话题
    if camera_subscriber:
        camera_subscriber.unregister()  # 取消订阅
        print("Camera subscriber unregistered.")

# 全局变量定义
data_lock = threading.Lock()  # 数据访问锁
rotation_start_time = None
rotation_end_time = None
rotation_duration = None
is_rotating = False

def rotate_clockwise_90(vehicle):
    """顺时针旋转90度（带完整时间记录）"""
    global rotation_start_time, rotation_end_time, rotation_duration, is_rotating

    with data_lock:
        rotation_start_time = time.time()  # 记录绝对开始时间
        is_rotating = True

    current_yaw = math.degrees(vehicle.attitude.yaw)
    target_yaw = (current_yaw + 90) % 360
    condition_yaw(vehicle, heading=target_yaw, relative=False)

    YAW_TOLERANCE = 5
    TIMEOUT = 15
    start_time = time.time()

    while True:
        current_yaw = math.degrees(vehicle.attitude.yaw) % 360
        error = (target_yaw - current_yaw) % 360
        if error > 180: error -= 360

        if abs(error) <= YAW_TOLERANCE:
            print(f"Reached target yaw: {current_yaw:.1f}°")
            break

        if time.time() - start_time > TIMEOUT:
            print(f"Rotation timeout at {current_yaw:.1f}°")
            break

        time.sleep(0.2)

    with data_lock:
        rotation_end_time = time.time()
        rotation_duration = rotation_end_time - rotation_start_time
        is_rotating = False
        print(f"Rotation completed in {rotation_duration:.2f}s")

    return rotation_start_time, rotation_end_time, rotation_duration  # ✅ **添加返回值**

def rotate_counterclockwise_90(vehicle):
    """逆时针旋转90度（带完整时间记录）"""
    global rotation_start_time, rotation_end_time, rotation_duration, is_rotating

    with data_lock:
        rotation_start_time = time.time()  # 记录绝对开始时间
        is_rotating = True

    current_yaw = math.degrees(vehicle.attitude.yaw)
    target_yaw = (current_yaw - 90) % 360
    condition_yaw(vehicle, heading=target_yaw, relative=False)

    YAW_TOLERANCE = 5
    TIMEOUT = 15
    start_time = time.time()

    while True:
        current_yaw = math.degrees(vehicle.attitude.yaw) % 360
        error = (current_yaw - target_yaw) % 360
        if error > 180: error -= 360

        if abs(error) <= YAW_TOLERANCE:
            print(f"Reached target yaw: {current_yaw:.1f}°")
            break

        if time.time() - start_time > TIMEOUT:
            print(f"Rotation timeout at {current_yaw:.1f}°")
            break

        time.sleep(0.2)

    with data_lock:
        rotation_end_time = time.time()
        rotation_duration = rotation_end_time - rotation_start_time
        is_rotating = False
        print(f"Rotation completed in {rotation_duration:.2f}s")

    return rotation_start_time, rotation_end_time, rotation_duration  # ✅ **添加返回值**

sensitivity = 20	# higher number, sensitive to small deviation
def calculate_yaw_angle(frame_width, deviation, sensitivity, max_yaw_angle=30):
    """
       Calculate yaw angle based on deviation from center.
       Args:
           frame_width (int): Width of the image frame.
           deviation (float): Deviation of the detected center from the frame center.
           sensitivity (float): Sensitivity factor to control the yaw response.
           max_yaw_angle (float): Maximum yaw angle to prevent oversteering (default: 30 degrees).

       Returns:
           float: Calculated yaw angle in degrees.
       """
    yaw_angle = math.atan(deviation / (frame_width * sensitivity))
    yaw_angle = math.degrees(yaw_angle)
    # Clamp the yaw angle within the max limits
    return max(-max_yaw_angle, min(yaw_angle, max_yaw_angle))


def ascend_to_height(vehicle, target_height):
    current_altitude = vehicle.location.global_relative_frame.alt
    target_altitude = target_height - current_altitude

    print(f"Current altitude: {current_altitude} m. Ascending to {target_altitude} m.")
    while vehicle.location.global_relative_frame.alt < target_altitude - 0.1:  # 给一点误差范围
        send_ned_velocity(vehicle, velocity_x=0, velocity_y=0, velocity_z=-0.5, duration=1)  # 负值表示上升
        print(f"Ascending... Current altitude: {vehicle.location.global_relative_frame.alt:.2f} m")

    print(f"Reached target altitude: {target_altitude} m.")
    hover(vehicle, hover_time=5)  # 稳定悬停

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

def plot_white_pixel_ratios(white_pixel_ratios_width, white_pixel_ratios_height, image):


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


    # 调用 identify_marker_shape_and_orientation 来识别标志形状和朝向
    shape_code = identify_marker_shape_and_orientation(width_trends, height_trends)

    return shape_code



# 全局变量用于 PID 控制器
integral_error = 0
last_error = 0
# 添加积分限幅以避免积分饱和
MAX_INTEGRAL = 50  # 最大积分值限制
def pid_control_with_dead_zone(deviation, Kp, Ki, Kd, dead_zone=0.05):
    global integral_error, last_error

    # 死区判断
    if abs(deviation) <= dead_zone:
        integral_error = 0  # 偏差在死区范围内，清空积分项
        return 0

    # PID 计算
    integral_error += deviation
    integral_error = max(min(integral_error, MAX_INTEGRAL), -MAX_INTEGRAL)  # 限幅
    derivative_error = deviation - last_error
    last_error = deviation

    pid_output = (Kp * deviation + Ki * integral_error + Kd * derivative_error)
    return pid_output



def detect_yellow_line(frame, lower_yellow, upper_yellow):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # 应用形态学操作以去噪
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    moments = cv2.moments(mask)
    if moments["m00"] > 0:
        cx = int(moments["m10"] / moments["m00"])
    else:
        cx = -1
    return cx

# 无人机沿黄线控制函数
def follow_yellow_line(vehicle, frame):


    # 黄色范围的 HSV 值
    lower_yellow = np.array([18, 84, 140])
    upper_yellow = np.array([175, 255, 255])

    # 检测黄色线的中心位置
    cx = detect_yellow_line(frame, lower_yellow, upper_yellow)

    # 获取图像宽度和中心位置
    frame_width = frame.shape[1]
    frame_center = frame_width // 2

    # 在图像上绘制 frame_center 的红色点
    frame_height = frame.shape[0]
    cv2.circle(frame, (frame_center, frame_height // 2), 5, (0, 0, 255), -1)  # 红色点 (B, G, R)

    if cx != -1:
        # 在图像上绘制 cx 的蓝色点
        cv2.circle(frame, (cx, frame_height // 2), 5, (255, 0, 0), -1)  # 蓝色点

    # 如果未检测到黄色线，悬停
    if cx == -1:
        print("Yellow line not detected. Hovering...")
        hover(vehicle, hover_time=0)  # 示例：无人机悬停
        return

    # 计算偏差值
    deviation = cx - frame_center

    # 归一化偏差值到 [-1, 1]，避免速度映射超出无人机能力
    normalized_deviation = deviation / frame_center

    # PID 参数
    Kp, Ki, Kd = 1.0, 0.02, 0.5
    dead_zone = 0.05  # 死区也按归一化值调整

    # 使用 PID 计算横向调整
    lateral_adjustment = pid_control_with_dead_zone(normalized_deviation, Kp, Ki, Kd, dead_zone)

    # 调整横向速度
    forward_speed = 0.1  # 前进速度增加以测试效果
    lateral_speed = lateral_adjustment * 0.5  # 根据无人机特性缩放
    send_ned_velocity(vehicle, velocity_x=forward_speed, velocity_y=lateral_speed, velocity_z=0, duration=0.1)

    print(f"Normalized Deviation: {normalized_deviation:.2f}, Lateral Adjustment: {lateral_speed:.2f}")

def calculate_yellow_line_offset(frame):
    # 黄色范围的 HSV 值
    lower_yellow = np.array([18, 84, 140])
    upper_yellow = np.array([175, 255, 255])

    # 检测黄色线的中心位置
    cx = detect_yellow_line(frame, lower_yellow, upper_yellow)

    # 获取图像宽度和中心位置
    frame_width = frame.shape[1]
    frame_center = frame_width // 2

    if cx == -1:
        # 如果未检测到黄线，则返回偏移量为 0
        return 0

    # 计算偏差值
    deviation = cx - frame_center
    return deviation


class ShapeCodeProcessor:
    def __init__(self, window_size=5):
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

# 引入滑动窗口处理器
shape_code_processor = ShapeCodeProcessor(window_size=5)

def process_image_and_get_shape_code(image):
    # 使用局部变量而非全局变量
    local_last_shape_code = last_shape_code

    # Resize the image to a smaller resolution
    new_width, new_height = 400, 400  # 目标尺寸
    small_image = cv2.resize(image, (new_width, new_height))

    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(small_image, cv2.COLOR_BGR2HSV)

    # 定义红色的 HSV 范围
    lower_red1, upper_red1 = np.array([0, 100, 100]), np.array([10, 255, 255])
    lower_red2, upper_red2 = np.array([170, 100, 100]), np.array([180, 255, 255])

    # 创建红色掩码
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # 计算红色像素的数量
    red_pixel_count = cv2.countNonZero(mask_red)


    # 如果检测到足够的红色像素，返回形状代码 0，不做后续操作
    if red_pixel_count > 350:  # 阈值可调整
        print("Red detected: Marking as shape code 0")
        local_last_shape_code = 0  # 更新局部变量
        smoothed_code = shape_code_processor.process_code(0)
        return 0, smoothed_code

    # 定义黄色在 HSV 色彩空间中的范围,lower_yellow 和 upper_yellow 分别是黄色的下限和上限 HSV 值。
    lower_yellow = np.array([18, 84, 140])
    upper_yellow = np.array([175, 255, 255])

    # 黄色掩码及形态学处理
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask_blurred = cv2.medianBlur(mask_yellow, 5)
    kernel = np.ones((5, 5), np.uint8)
    mask_morph = cv2.morphologyEx(mask_blurred, cv2.MORPH_CLOSE, kernel)

    # 调用自定义函数计算白色像素比例
    bin_size = 10  # 分区大小
    white_pixel_ratios_width, white_pixel_ratios_height = calculate_ratio_dict(mask_morph, bin_size)

    # 调用自定义函数绘图并返回形状代码
    shape_code = plot_white_pixel_ratios(
        white_pixel_ratios_width, white_pixel_ratios_height, mask_morph
    )

    # 如果形状代码为 1，调整为上一帧的形状代码
    if shape_code == 1:
        if last_shape_code is not None:  # 如果上一帧形状代码存在
            print(f"Unknown shape detected. Adjusting to last shape code: {last_shape_code}")
            shape_code = last_shape_code  # 调整为上一帧形状代码
        else:
            print("Unknown shape detected. No last shape code available. Keeping as 1.")

    # 对形状代码进行平滑处理
    smoothed_code = shape_code_processor.process_code(shape_code)

    # 最后更新局部变量
    local_last_shape_code = smoothed_code

    # 返回原始形状代码和平滑后的形状代码
    return shape_code, smoothed_code


# Create a CvBridge object to convert ROS Image messages to OpenCV images
bridge = CvBridge()
forward_speed = 0.1  # Default forward speed
# 引入变量记录上一帧的形状代码
last_shape_code = 0
# 引入计数变量

t_left_count = 0
l_left_count = 0
l_right_count = 0
t_right_count = 0
# 全局变量用于控制主循环
pause_movement = False
exit_flag = False  # 用于标记是否退出主循环
# 引入变量记录形状跟踪状态
tracking_active = False
tracking_state = None  # 跟踪的目标状态（'T_left' 或 'Cross'等等）
image_counter = 0
phase = 1
# 初始化 ROS 节点
rospy.init_node('drone_data_logger', anonymous=True)

# 定义全局变量
current_pose = None
current_twist = None
current_pitch = None
current_roll = None
current_yaw = None




directory_1 = '/home/dym/course1_data_1'
data_file_path_1 = os.path.join(directory_1, 'log_1.csv')
# 检查并创建目录
if not os.path.exists(directory_1):
    os.makedirs(directory_1)
# 初始化计时器标志
start_timer_1_initialized = False
start_time_1 = None
# 初始化 CSV 文件并写入表头（含单位）
with open(data_file_path_1, 'w', newline='') as csvfile_1:
    csv_writer_1 = csv.writer(csvfile_1)
    csv_writer_1.writerow([
        "Timestamp (s)", "Position_X (m)", "Position_Y (m)", "Position_Z (m)",
        "Speed_X (m/s)", "Speed_Y (m/s)", "Speed_Z (m/s)",
        "Pitch (rad)", "Roll (rad)", "Yaw (rad)", "Image_Center_Offset (pixels)"
    ])


directory_2 = '/home/dym/course2_data_2'
data_file_path_2 = os.path.join(directory_2, 'log_2.csv')
# 检查并创建目录
if not os.path.exists(directory_2):
    os.makedirs(directory_2)
# 初始化计时器标志
start_timer_2_initialized = False
start_time_2 = None
# 初始化 CSV 文件并写入表头（含单位）
with open(data_file_path_2, 'w', newline='') as csvfile_2:
    csv_writer_2 = csv.writer(csvfile_2)
    csv_writer_2.writerow([
        "Timestamp (s)", "Position_X (m)", "Position_Y (m)", "Position_Z (m)",
        "Speed_X (m/s)", "Speed_Y (m/s)", "Speed_Z (m/s)",
        "Pitch (rad)", "Roll (rad)", "Yaw (rad)", "Image_Center_Offset (pixels)"
    ])


directory_3 = '/home/dym/course3_data_3'
data_file_path_3 = os.path.join(directory_3, 'log_3.csv')
# 检查并创建目录
if not os.path.exists(directory_3):
    os.makedirs(directory_3)
# 初始化计时器标志
start_timer_3_initialized = False
start_time_3 = None
# 初始化 CSV 文件并写入表头（含单位）
with open(data_file_path_3, 'w', newline='') as csvfile_3:
    csv_writer_3 = csv.writer(csvfile_3)
    csv_writer_3.writerow([
        "Timestamp (s)", "Position_X (m)", "Position_Y (m)", "Position_Z (m)",
        "Speed_X (m/s)", "Speed_Y (m/s)", "Speed_Z (m/s)",
        "Pitch (rad)", "Roll (rad)", "Yaw (rad)", "Image_Center_Offset (pixels)"
    ])


def model_states_callback(data):
    global current_pose, current_twist, current_pitch, current_roll, current_yaw
    drone_name = "iris"  # 替换为您的无人机模型名称

    if drone_name in data.name:
        index = data.name.index(drone_name)
        current_pose = data.pose[index]
        current_twist = data.twist[index]

        # 提取四元数
        quaternion = (
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w,
        )

        # 四元数转换为欧拉角（弧度）
        current_roll, current_pitch, current_yaw = euler_from_quaternion(quaternion)

# 订阅 Gazebo 的 /gazebo/model_states 话题
rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)


def image_callback_0(msg):
    global camera_subscriber

    print("Received image in image_callback_0.")

    # 转换ROS图像消息到OpenCV格式
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    small_image = cv2.resize(cv_image, (400, 400))

    # 图像识别：提取形状代码
    shape_code, smoothed_shape_code = process_image_and_get_shape_code(cv_image)

    # 确保是整数类型
    smoothed_shape_code = int(smoothed_shape_code)

    print(f"Detected smoothed shape code: {smoothed_shape_code}")

    # 检测到shape code = 0时触发动作
    if smoothed_shape_code == 0:
        print("Shape code 0 detected. Executing special maneuvers.")

        # 取消订阅器，防止后续图像继续触发
        if camera_subscriber:
            camera_subscriber.unregister()
            print("Camera subscriber stopped.")

        # 执行逆时针旋转90度
        rotate_counterclockwise_90(vehicle)
        print("Rotation complete.")

        # 等待用户输入目标上升高度
        while True:
            try:
                user_input = input("Enter target ascent height in meters: ").strip()
                target_height = float(user_input)
                if target_height > 0:
                    break
                else:
                    print("Height must be a positive number.")
            except ValueError:
                print("Invalid input. Please enter a valid number.")

        # 上升到指定高度
        ascend_to_height(vehicle, target_height)

        # 悬停10秒
        hover(vehicle, hover_time=5)

        # 缓慢降落
        print("Initiating slow descent...")
        send_ned_velocity(vehicle, velocity_x=0, velocity_y=0, velocity_z=0.2, duration=5)  # 0.2 m/s下降
        vehicle.mode = VehicleMode("LAND")
        print("Landing initiated.")

        # 完成后退出
        print("Mission complete for image_callback_0.")


def image_callback_1(msg):
    global camera_subscriber, t_left_count, last_shape_code, pause_movement, exit_flag, image_counter
    global tracking_active, tracking_state
    global start_timer_1_initialized, start_time_1
    global current_pose, current_twist, current_pitch, current_roll, current_yaw

    print(f"Last shape code before processing: {last_shape_code}")

    # === 初始化时间戳 ===
    if not start_timer_1_initialized:
        start_time_1 = time.time()
        start_timer_1_initialized = True
        print("Timer initialized for image_callback_1.")

    timestamp = time.time() - start_time_1

    # === 获取无人机状态 ===
    position = current_pose.position if current_pose else None
    velocity = current_twist.linear if current_twist else None
    pitch = current_pitch if current_pitch is not None else None
    roll = current_roll if current_roll is not None else None
    yaw = current_yaw if current_yaw is not None else None

    # === 图像处理 ===
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    small_image = cv2.resize(cv_image, (400, 400))
    yellow_line_offset = calculate_yellow_line_offset(small_image)

    # 保存数据到 CSV 文件
    with open(data_file_path_1, 'a', newline='') as csvfile_1:
        csv_writer_1 = csv.writer(csvfile_1)
        csv_writer_1.writerow([
            f"{round(timestamp, 2)} s",
            f"{position.x:.2f} m" if position else "N/A",
            f"{position.y:.2f} m" if position else "N/A",
            f"{position.z:.2f} m" if position else "N/A",
            f"{velocity.x:.2f} m/s" if velocity else "N/A",
            f"{velocity.y:.2f} m/s" if velocity else "N/A",
            f"{velocity.z:.2f} m/s" if velocity else "N/A",
            f"{pitch:.2f} rad" if pitch is not None else "N/A",
            f"{roll:.2f} rad" if roll is not None else "N/A",
            f"{yaw:.2f} rad" if yaw is not None else "N/A",
            f"{yellow_line_offset} pixels"
        ])

    print("Data recorded to CSV.")

    # Define the folder to save captured images (JPG format before converting to EPS)
    folder_path = '/home/dym/course1_PID_images'  # course0bian_images,course0yuan_images

    # Create the folder if it does not exist
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    # Generate a unique filename based on the counter value (JPG format)
    image_filename = f"image_{image_counter}.jpg"

    # Increment the counter for the next image
    image_counter += 1

    # Construct the full file path to save the image in JPG format
    file_path = os.path.join(folder_path, image_filename)

    # Save the captured image in JPG format (for now)
    cv2.imwrite(file_path, small_image)
    print(f"Image saved as JPG: {file_path}")

    # === Shape Code识别 ===
    frame = np.array(small_image)
    shape_code, smoothed_shape_code = process_image_and_get_shape_code(cv_image)

    last_shape_code = int(last_shape_code)
    smoothed_shape_code = int(smoothed_shape_code)
    print(f"Checking transition: last_shape_code={last_shape_code}, smoothed_shape_code={smoothed_shape_code}")

    # === 控制逻辑 ===
    # (2 ➔ 5)：检测L左转
    if not tracking_active and smoothed_shape_code == 2:
        tracking_active = True
        tracking_state = "L_left_detecting"
        print("Started tracking L left transition.")

    if tracking_active and tracking_state == "L_left_detecting" and smoothed_shape_code == 5:
        print("L left detected. Rotating counterclockwise 90 degrees.")
        tracking_active = False
        pause_movement = True
        rotate_counterclockwise_90(vehicle)
        print("Moving forward after rotation.")
        send_ned_velocity(vehicle, velocity_x=0.1, velocity_y=0, velocity_z=0, duration=1)  # 前进 1秒，速度0.1m/s，可根据实际调整
        pause_movement = False
        tracking_state = None
        return

    # (5 ➔ 9)：检测T左转
    if not tracking_active and smoothed_shape_code == 5:
        tracking_active = True
        tracking_state = "T_left_detecting"
        print("Started tracking T left transition.")

    if tracking_active and tracking_state == "T_left_detecting" and smoothed_shape_code == 9:
        t_left_count += 1
        print(f"T left transition completed. Count: {t_left_count}")
        tracking_active = False
        tracking_state = None

        if t_left_count == 1:
            print("T left completed. Hovering...")
            pause_movement = True
            hover(vehicle, hover_time=0)
            camera_subscriber.unregister()
            print("Camera subscriber stopped.")

            while True:
                try:
                    user_input = input("Enter command (0: Forward, 1: Rotate clockwise 90, 2: Rotate counterclockwise 90): ").strip()
                    user_input = int(user_input)
                    if user_input == 0:
                        print("Continue forward.")
                        send_ned_velocity(vehicle, velocity_x=0.1, velocity_y=0, velocity_z=0, duration=1)
                        break
                    elif user_input == 1:
                        rotate_clockwise_90(vehicle)
                        try:
                            target_height = float(input("Enter target height in meters (or press Enter to land): ") or "0")
                            if target_height > 0:
                                ascend_to_height(vehicle, target_height)
                            else:
                                land(vehicle)
                        except ValueError:
                            print("Invalid input. Landing the drone.")
                            land(vehicle)
                        break
                    elif user_input == 2:
                        rotate_counterclockwise_90(vehicle)
                        try:
                            target_height = float(input("Enter target height in meters (or press Enter to land): ") or "0")
                            if target_height > 0:
                                ascend_to_height(vehicle, target_height)
                            else:
                                land(vehicle)
                        except ValueError:
                            print("Invalid input. Landing the drone.")
                            land(vehicle)
                        break
                    else:
                        print("Invalid input. Please enter 0, 1, or 2.")
                except ValueError:
                    print("Invalid input. Please enter a number (0, 1, or 2).")

            pause_movement = False
            exit_flag = True

    # === 如果识别到直线 (2)，执行路径跟随 ===
    if smoothed_shape_code == 2:
        follow_yellow_line(vehicle, frame)

    # === 图像显示 ===
    cv2.imshow("frame", frame)
    cv2.waitKey(10)

    # 更新 last_shape_code
    last_shape_code = smoothed_shape_code
    print(f"Last shape code after update: {last_shape_code}")

def image_callback_2(msg):
    global camera_subscriber, t_left_count, last_shape_code, pause_movement, exit_flag, image_counter
    global tracking_active, tracking_state, phase
    global start_timer_2_initialized, start_time_2
    global current_pose, current_twist, current_pitch, current_roll, current_yaw

    print(f"Last shape code before processing: {last_shape_code}")

    # === 初始化时间戳 ===
    if not start_timer_2_initialized:
        start_time_2 = time.time()
        start_timer_2_initialized = True
        print("Timer initialized for image_callback_2.")

    timestamp = time.time() - start_time_2

    # === 获取无人机状态 ===
    position = current_pose.position if current_pose else None
    velocity = current_twist.linear if current_twist else None
    pitch = current_pitch if current_pitch is not None else None
    roll = current_roll if current_roll is not None else None
    yaw = current_yaw if current_yaw is not None else None

    # === 图像处理 ===
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    small_image = cv2.resize(cv_image, (300, 300))
    yellow_line_offset = calculate_yellow_line_offset(small_image)

    # 保存数据到 CSV 文件
    with open(data_file_path_2, 'a', newline='') as csvfile_2:
        csv_writer_2 = csv.writer(csvfile_2)
        csv_writer_2.writerow([
            f"{round(timestamp, 2)} s",
            f"{position.x:.2f} m" if position else "N/A",
            f"{position.y:.2f} m" if position else "N/A",
            f"{position.z:.2f} m" if position else "N/A",
            f"{velocity.x:.2f} m/s" if velocity else "N/A",
            f"{velocity.y:.2f} m/s" if velocity else "N/A",
            f"{velocity.z:.2f} m/s" if velocity else "N/A",
            f"{pitch:.2f} rad" if pitch is not None else "N/A",
            f"{roll:.2f} rad" if roll is not None else "N/A",
            f"{yaw:.2f} rad" if yaw is not None else "N/A",
            f"{yellow_line_offset} pixels"
        ])

    print("Data recorded to CSV.")

    # Define the folder to save captured images (JPG format before converting to EPS)
    folder_path = '/home/dym/course2_PID_images'  # course0bian_images,course0yuan_images

    # Create the folder if it does not exist
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    # Generate a unique filename based on the counter value (JPG format)
    image_filename = f"image_{image_counter}.jpg"

    # Increment the counter for the next image
    image_counter += 1

    # Construct the full file path to save the image in JPG format
    file_path = os.path.join(folder_path, image_filename)

    # Save the captured image in JPG format (for now)
    cv2.imwrite(file_path, small_image)
    print(f"Image saved as JPG: {file_path}")

    # === Shape Code识别 ===
    frame = np.array(small_image)
    shape_code, smoothed_shape_code = process_image_and_get_shape_code(cv_image)

    last_shape_code = int(last_shape_code)
    smoothed_shape_code = int(smoothed_shape_code)
    print(f"Checking transition: last_shape_code={last_shape_code}, smoothed_shape_code={smoothed_shape_code}")

    # === 核心控制逻辑开始 ===
    if phase == 1:
        # 第1阶段: 检测 L形 2 ➔ 5
        if smoothed_shape_code == 2:
            follow_yellow_line(vehicle, frame)
            if not tracking_active:
                tracking_active = True
                tracking_state = "Left_turn_detecting"
                print("Started tracking Left turn.")
        elif tracking_active and tracking_state == "Left_turn_detecting" and smoothed_shape_code == 5:
            print("Detected Left turn. Rotating counterclockwise 90 degrees.")
            pause_movement = True
            rotate_counterclockwise_90(vehicle)
            send_ned_velocity(vehicle, velocity_x=0.1, velocity_y=0, velocity_z=0, duration=1)
            pause_movement = False
            tracking_active = False
            phase = 2
            print("Phase 1 complete. Moving to Phase 2.")

    elif phase == 2:
        # 第2阶段: 检测 T形 5 ➔ 9
        if smoothed_shape_code == 5 and not tracking_active:
            tracking_active = True
            tracking_state = "T_detecting"
            print("Started tracking T marker.")
        elif tracking_active and tracking_state == "T_detecting" and smoothed_shape_code == 9:
            print("Detected T marker. Moving forward.")
            send_ned_velocity(vehicle, velocity_x=0.1, velocity_y=0, velocity_z=0, duration=1)
            tracking_active = False
            phase = 3
            print("Phase 2 complete. Moving to Phase 3.")
        elif smoothed_shape_code == 2:
            follow_yellow_line(vehicle, frame)

    elif phase == 3:
        # 第3阶段: 检测 L形 2 ➔ 5
        if smoothed_shape_code == 2:
            follow_yellow_line(vehicle, frame)
            if not tracking_active:
                tracking_active = True
                tracking_state = "Left_turn_detecting"
                print("Started tracking Left turn in Phase 3.")
        elif tracking_active and tracking_state == "Left_turn_detecting" and smoothed_shape_code == 5:
            print("Detected Left turn in Phase 3. Rotating counterclockwise 90 degrees.")
            pause_movement = True
            rotate_counterclockwise_90(vehicle)
            send_ned_velocity(vehicle, velocity_x=0.1, velocity_y=0, velocity_z=0, duration=1)
            pause_movement = False
            tracking_active = False
            phase = 4
            print("Phase 3 complete. Moving to Phase 4.")

    elif phase == 4:
        # 第4阶段: 检测 T形 5 ➔ 9，到达终点
        if smoothed_shape_code == 5 and not tracking_active:
            tracking_active = True
            tracking_state = "T_detecting"
            print("Started tracking T marker in Phase 4.")
        elif tracking_active and tracking_state == "T_detecting" and smoothed_shape_code == 9:
            print("Final T marker detected. Reached destination '3'.")
            pause_movement = True
            hover(vehicle, hover_time=0)
            camera_subscriber.unregister()
            print("Camera subscriber stopped.")

            while True:
                try:
                    user_input = input("Enter command (0: Forward, 1: Rotate clockwise 90, 2: Rotate counterclockwise 90): ").strip()
                    user_input = int(user_input)
                    if user_input == 0:
                        print("Continue forward.")
                        send_ned_velocity(vehicle, velocity_x=0.1, velocity_y=0, velocity_z=0, duration=1)
                        break
                    elif user_input == 1:
                        rotate_clockwise_90(vehicle)
                        break
                    elif user_input == 2:
                        rotate_counterclockwise_90(vehicle)
                        try:
                            height_input = input("Enter target height in meters (or press Enter to land): ").strip()
                            if height_input:
                                target_height = float(height_input)
                                ascend_to_height(vehicle, target_height)
                            else:
                                print("No height input. Landing drone.")
                                land(vehicle)
                        except ValueError:
                            print("Invalid input. Landing drone.")
                            land(vehicle)
                        break
                    else:
                        print("Invalid input. Please enter 0, 1, or 2.")
                except ValueError:
                    print("Invalid input. Please enter a number (0, 1, or 2).")

            pause_movement = False
            exit_flag = True
            print("Exit flag set. Mission complete.")
        elif smoothed_shape_code == 2:
            # ✅ 新增：如果检测到黄线，继续进行路径跟随
            follow_yellow_line(vehicle, frame)
    # === 图像显示 ===
    cv2.imshow("frame", frame)
    cv2.waitKey(10)

    # 更新 last_shape_code
    last_shape_code = smoothed_shape_code
    print(f"Last shape code after update: {last_shape_code}")

def image_callback_3(msg):
    global camera_subscriber, t_right_count, last_shape_code, pause_movement, exit_flag, image_counter
    global tracking_active, tracking_state
    global start_timer_3_initialized, start_time_3
    global current_pose, current_twist, current_pitch, current_roll, current_yaw

    print(f"Last shape code before processing: {last_shape_code}")

    # === 初始化时间戳 ===
    if not start_timer_3_initialized:
        start_time_3 = time.time()
        start_timer_3_initialized = True
        print("Timer initialized for image_callback_3.")

    timestamp = time.time() - start_time_3

    # === 获取无人机状态 ===
    position = current_pose.position if current_pose else None
    velocity = current_twist.linear if current_twist else None
    pitch = current_pitch if current_pitch is not None else None
    roll = current_roll if current_roll is not None else None
    yaw = current_yaw if current_yaw is not None else None

    # === 图像处理 ===
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    small_image = cv2.resize(cv_image, (400, 400))
    yellow_line_offset = calculate_yellow_line_offset(small_image)

    # 保存数据到 CSV 文件
    with open(data_file_path_3, 'a', newline='') as csvfile_3:
        csv_writer_3 = csv.writer(csvfile_3)
        csv_writer_3.writerow([
            f"{round(timestamp, 2)} s",
            f"{position.x:.2f} m" if position else "N/A",
            f"{position.y:.2f} m" if position else "N/A",
            f"{position.z:.2f} m" if position else "N/A",
            f"{velocity.x:.2f} m/s" if velocity else "N/A",
            f"{velocity.y:.2f} m/s" if velocity else "N/A",
            f"{velocity.z:.2f} m/s" if velocity else "N/A",
            f"{pitch:.2f} rad" if pitch is not None else "N/A",
            f"{roll:.2f} rad" if roll is not None else "N/A",
            f"{yaw:.2f} rad" if yaw is not None else "N/A",
            f"{yellow_line_offset} pixels"
        ])

    print("Data recorded to CSV.")

    # Define the folder to save captured images (JPG format before converting to EPS)
    folder_path = '/home/dym/course3_PID_images'  # course0bian_images,course0yuan_images

    # Create the folder if it does not exist
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    # Generate a unique filename based on the counter value (JPG format)
    image_filename = f"image_{image_counter}.jpg"

    # Increment the counter for the next image
    image_counter += 1

    # Construct the full file path to save the image in JPG format
    file_path = os.path.join(folder_path, image_filename)

    # Save the captured image in JPG format (for now)
    cv2.imwrite(file_path, small_image)
    print(f"Image saved as JPG: {file_path}")

    # === Shape Code识别 ===
    frame = np.array(small_image)
    shape_code, smoothed_shape_code = process_image_and_get_shape_code(cv_image)

    last_shape_code = int(last_shape_code)
    smoothed_shape_code = int(smoothed_shape_code)
    print(f"Checking transition: last_shape_code={last_shape_code}, smoothed_shape_code={smoothed_shape_code}")

    # === 控制逻辑 ===
    # (2 ➔ 4)：检测L右转
    if not tracking_active and smoothed_shape_code == 2:
        tracking_active = True
        tracking_state = "L_Right_detecting"
        print("Started tracking L Right transition.")

    if tracking_active and tracking_state == "L_Right_detecting" and smoothed_shape_code == 4:
        print("L left detected. Rotating counterclockwise 90 degrees.")
        tracking_active = False
        pause_movement = True
        rotate_clockwise_90(vehicle)
        print("Moving forward after rotation.")
        send_ned_velocity(vehicle, velocity_x=0.1, velocity_y=0, velocity_z=0, duration=1)  # 前进 1秒，速度0.1m/s，可根据实际调整
        pause_movement = False
        tracking_state = None
        return

    # (4 ➔ 8)：检测T右转
    if not tracking_active and smoothed_shape_code == 4:
        tracking_active = True
        tracking_state = "T_right_detecting"
        print("Started tracking T right transition.")

    if tracking_active and tracking_state == "T_right_detecting" and smoothed_shape_code == 8:
        t_right_count += 1
        print(f"T Right transition completed. Count: {t_right_count}")
        tracking_active = False
        tracking_state = None

        if t_right_count == 1:
            print("T left completed. Hovering...")
            pause_movement = True
            hover(vehicle, hover_time=0)
            camera_subscriber.unregister()
            print("Camera subscriber stopped.")

            while True:
                try:
                    user_input = input("Enter command (0: Forward, 1: Rotate clockwise 90, 2: Rotate counterclockwise 90): ").strip()
                    user_input = int(user_input)
                    if user_input == 0:
                        print("Continue forward.")
                        send_ned_velocity(vehicle, velocity_x=0.1, velocity_y=0, velocity_z=0, duration=1)
                        break
                    elif user_input == 1:
                        rotate_clockwise_90(vehicle)
                        try:
                            target_height = float(input("Enter target height in meters (or press Enter to land): ") or "0")
                            if target_height > 0:
                                ascend_to_height(vehicle, target_height)
                            else:
                                land(vehicle)
                        except ValueError:
                            print("Invalid input. Landing the drone.")
                            land(vehicle)
                        break
                    elif user_input == 2:
                        rotate_counterclockwise_90(vehicle)
                        try:
                            target_height = float(input("Enter target height in meters (or press Enter to land): ") or "0")
                            if target_height > 0:
                                ascend_to_height(vehicle, target_height)
                            else:
                                land(vehicle)
                        except ValueError:
                            print("Invalid input. Landing the drone.")
                            land(vehicle)
                        break
                    else:
                        print("Invalid input. Please enter 0, 1, or 2.")
                except ValueError:
                    print("Invalid input. Please enter a number (0, 1, or 2).")

            pause_movement = False
            exit_flag = True

    # === 如果识别到直线 (2)，执行路径跟随 ===
    if smoothed_shape_code == 2:
        follow_yellow_line(vehicle, frame)

    # === 图像显示 ===
    cv2.imshow("frame", frame)
    cv2.waitKey(10)

    # 更新 last_shape_code
    last_shape_code = smoothed_shape_code
    print(f"Last shape code after update: {last_shape_code}")


# 位置字典（同样可以使用之前定义的positions字典）
positions = {
    0: 0,
    1: 1,
    2: 2,
    3: 3
}


def fly_to_0():
    global camera_subscriber, pause_movement, exit_flag

    print("Flying logic for position 0")

    target_altitude = 0.5  # 起飞目标高度（米）

    # 确保进入 GUIDED 模式
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.mode.name == "GUIDED":
        print("Waiting for vehicle to enter GUIDED mode...")
        time.sleep(1)

    # 无人机解锁并起飞
    arm_and_takeoff(target_altitude)
    print("Vehicle armed and took off to 0.5 meters.")

    # 设置相机话题订阅，绑定 image_callback_0
    camera_topic = '/camera/image_raw'
    camera_subscriber = rospy.Subscriber(camera_topic, Image, image_callback_0)
    print("Camera subscriber initialized: /camera/image_raw")

    # 设置主循环频率
    loop_rate = rospy.Rate(30)  # 30 Hz

    try:
        # 只维持 ROS 主循环，不发送任何运动指令
        while not rospy.is_shutdown() and not exit_flag:
            loop_rate.sleep()  # 每秒循环30次，保持程序运行

    except rospy.ROSInterruptException:
        print("ROS shutdown detected.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        # 收尾处理
        print("Cleaning up...")

        # 取消订阅器
        if camera_subscriber:
            camera_subscriber.unregister()
            print("Camera subscriber stopped.")

        # 安全降落
        if vehicle:
            vehicle.mode = VehicleMode("LAND")
            print("Landing vehicle.")

        print("Fly to 0 complete.")

def fly_to_1():
    global camera_subscriber, pause_movement, exit_flag

    print("Flying logic for position 1")

    target_altitude = 0.5  # 起飞目标高度（米）
    forward_speed = 0.1    # 前进速度（米/秒）

    # 确保进入 GUIDED 模式
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.mode.name == "GUIDED":
        print("Waiting for vehicle to enter GUIDED mode...")
        time.sleep(1)

    # 无人机解锁并起飞
    arm_and_takeoff(target_altitude)
    print("Vehicle armed and took off.")

    # 起飞后稳定悬停2秒（无旋转）
    hover(vehicle, hover_time=2)

    # 设置相机订阅，接收图像消息
    camera_topic = '/camera/image_raw'
    camera_subscriber = rospy.Subscriber(camera_topic, Image, image_callback_1)
    print("Camera subscriber initialized: /camera/image_raw")

    # 主循环频率
    loop_rate = rospy.Rate(30)  # 30 Hz

    try:
        while not rospy.is_shutdown() and not exit_flag:
            if not pause_movement:
                send_ned_velocity(vehicle, velocity_x=forward_speed, velocity_y=0, velocity_z=0, duration=0.1)
                print(f"Drone moving forward at {forward_speed} m/s")
            loop_rate.sleep()

    except rospy.ROSInterruptException:
        print("ROS shutdown detected.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        print("Cleaning up...")

        # 停止无人机移动
        send_ned_velocity(vehicle, velocity_x=0, velocity_y=0, velocity_z=0, duration=1)
        print("Drone stopped.")

        # 停止订阅图像
        if camera_subscriber:
            camera_subscriber.unregister()
            print("Camera subscriber stopped.")

        # 切换降落模式
        if vehicle:
            vehicle.mode = VehicleMode("LAND")
            print("Landing vehicle.")

        print("Fly to 1 complete.")

def fly_to_2():
    global camera_subscriber, pause_movement, exit_flag

    print("Flying logic for position 2")

    target_altitude = 0.5  # 起飞目标高度（米）
    forward_speed = 0.1    # 前进速度（米/秒）

    # 确保进入 GUIDED 模式
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.mode.name == "GUIDED":
        print("Waiting for vehicle to enter GUIDED mode...")
        time.sleep(1)

    # 无人机解锁并起飞
    arm_and_takeoff(target_altitude)
    print("Vehicle armed and took off.")

    # 起飞后稳定悬停2秒（无旋转）
    hover(vehicle, hover_time=2)

    # 设置相机订阅，接收图像消息
    camera_topic = '/camera/image_raw'
    camera_subscriber = rospy.Subscriber(camera_topic, Image, image_callback_2)
    print("Camera subscriber initialized: /camera/image_raw")

    # 主循环频率
    loop_rate = rospy.Rate(30)  # 30 Hz

    try:
        while not rospy.is_shutdown() and not exit_flag:
            if not pause_movement:
                send_ned_velocity(vehicle, velocity_x=forward_speed, velocity_y=0, velocity_z=0, duration=0.1)
                print(f"Drone moving forward at {forward_speed} m/s")
            loop_rate.sleep()

    except rospy.ROSInterruptException:
        print("ROS shutdown detected.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        print("Cleaning up...")

        # 停止无人机移动
        send_ned_velocity(vehicle, velocity_x=0, velocity_y=0, velocity_z=0, duration=1)
        print("Drone stopped.")

        # 停止订阅图像
        if camera_subscriber:
            camera_subscriber.unregister()
            print("Camera subscriber stopped.")

        # 切换降落模式
        if vehicle:
            vehicle.mode = VehicleMode("LAND")
            print("Landing vehicle.")

        print("Fly to 2 complete.")

def fly_to_3():
    global camera_subscriber, pause_movement, exit_flag

    print("Flying logic for position 3")

    target_altitude = 0.5  # 起飞目标高度（米）
    forward_speed = 0.1    # 前进速度（米/秒）

    # 确保进入 GUIDED 模式
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.mode.name == "GUIDED":
        print("Waiting for vehicle to enter GUIDED mode...")
        time.sleep(1)

    # 无人机解锁并起飞
    arm_and_takeoff(target_altitude)
    print("Vehicle armed and took off.")

    # 起飞后稳定悬停2秒（无旋转）
    hover(vehicle, hover_time=2)

    # 连续两次逆时针旋转90度
    print("Starting first 90 degree counterclockwise rotation.")
    rotate_counterclockwise_90(vehicle)

    print("Starting second 90 degree counterclockwise rotation.")
    rotate_counterclockwise_90(vehicle)

    # 设置相机订阅，接收图像消息
    camera_topic = '/camera/image_raw'
    camera_subscriber = rospy.Subscriber(camera_topic, Image, image_callback_3)
    print("Camera subscriber initialized: /camera/image_raw")

    # 主循环频率
    loop_rate = rospy.Rate(30)  # 30 Hz

    try:
        while not rospy.is_shutdown() and not exit_flag:
            if not pause_movement:
                send_ned_velocity(vehicle, velocity_x=forward_speed, velocity_y=0, velocity_z=0, duration=0.1)
                print(f"Drone moving forward at {forward_speed} m/s")
            loop_rate.sleep()

    except rospy.ROSInterruptException:
        print("ROS shutdown detected.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        print("Cleaning up...")

        # 停止无人机移动
        send_ned_velocity(vehicle, velocity_x=0, velocity_y=0, velocity_z=0, duration=1)
        print("Drone stopped.")

        # 停止订阅图像
        if camera_subscriber:
            camera_subscriber.unregister()
            print("Camera subscriber stopped.")

        # 切换降落模式
        if vehicle:
            vehicle.mode = VehicleMode("LAND")
            print("Landing vehicle.")

        print("Fly to 3 complete.")


# 飞行路线映射
flight_routes = {
    '0': fly_to_0,
    '1': fly_to_1,
    '2': fly_to_2,
    '3': fly_to_3,
}

# 主程序
def main():
    global vehicle

    while True:
        target_position = input("Enter the target position (0, 1, 2, 3): ").strip()
        if target_position in flight_routes:
            break
        print("Invalid input. Please enter a valid position code (0, 1, 2, 3).")

    print(f"Flying to target position {target_position}")
    flight_routes[target_position]()

if __name__ == "__main__":
    main()

