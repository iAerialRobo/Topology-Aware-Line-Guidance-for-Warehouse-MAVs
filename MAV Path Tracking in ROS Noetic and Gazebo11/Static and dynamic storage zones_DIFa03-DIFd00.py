# 第一部分模块导入区（Import Section）
import time
import math
import threading
from statistics import mode, StatisticsError
import numpy as np
import cv2
import rospy
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rospy import Rate
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion



# 第二部分：连接 Gazebo 仿真中的无人机（IRIS 机型），使 Python 脚本可以通过 DroneKit 控制仿真中的飞行器
# Connect to the simulated vehicle in Gazebo
print("Connecting to the vehicle...")
connection_string = 'tcp:127.0.0.1:5762'
vehicle = connect(connection_string, wait_ready=True, baud=57600)
print("Connected to IRIS")


# （第三部分无人机控制指令模块）Drone Commands
# 包括condition_yaw，send_ned_velocity，arm_and_takeoff，hover，land
# rotate_clockwise_90，rotate_counterclockwise_90


# condition_yaw ：Set yaw angle via MAVLink
# Rotate the MAV to a specified yaw angle, supporting either absolute or relative heading.
# Set relative=True for relative heading, and relative=False for absolute heading.
def condition_yaw(vehicle, heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    """
    if relative:
        is_relative = 1
    else:
        is_relative = 0

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

# Send NED velocity to control motion
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

# Arm and take off to target altitude
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

# Hover in place for given time
def hover(vehicle, hover_time):

    print("Hovering in place for", hover_time, "seconds.")

    # 发送零速度指令以停止无人机的所有移动（悬停）
    send_ned_velocity(vehicle, 0, 0, 0, hover_time)

    # 悬停计时
    time.sleep(hover_time)

    print("Hover complete.")

# Land and stop image subscription
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


def rotate_clockwise_90(vehicle):
    """顺时针旋转90度"""
    current_yaw = math.degrees(vehicle.attitude.yaw)
    target_yaw = (current_yaw + 90) % 360
    condition_yaw(vehicle, heading=target_yaw, relative=False)

    YAW_TOLERANCE = 5
    TIMEOUT = 15
    start_time = time.time()

    while True:
        current_yaw = math.degrees(vehicle.attitude.yaw) % 360
        error = (target_yaw - current_yaw) % 360
        if error > 180:
            error -= 360

        if abs(error) <= YAW_TOLERANCE:
            print(f"Reached target yaw: {current_yaw:.1f}°")
            break

        if time.time() - start_time > TIMEOUT:
            print(f"Rotation timeout at {current_yaw:.1f}°")
            break

        time.sleep(0.2)


# Rotate 90° CCW and record time
def rotate_counterclockwise_90(vehicle):
    """逆时针旋转90度（带完整时间记录）"""
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

def rotate_clockwise_45(vehicle):
    """顺时针旋转45度（带完整时间记录）"""
    current_yaw = math.degrees(vehicle.attitude.yaw)
    target_yaw = (current_yaw + 45) % 360
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

def rotate_counterclockwise_45(vehicle):
    """逆时针旋转45度（带完整时间记录）"""
    current_yaw = math.degrees(vehicle.attitude.yaw)
    target_yaw = (current_yaw - 45) % 360
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


def calculate_yaw_angle(deviation, altitude_m, max_yaw_angle=None):
    altitude_m = max(altitude_m, 0.01)
    deviation_m = (deviation * altitude_m) / FOCAL_LENGTH_PX
    yaw_angle_rad = math.atan(deviation_m / altitude_m)
    yaw_angle_deg = math.degrees(yaw_angle_rad)
    return yaw_angle_deg



# 图像处理与识别模块（Image Processing）
# 包括detect_yellow_line,follow_yellow_line,calculate_yellow_line_offset,calculate_ratio_dict
# analyze_trends_in_bins,identify_marker_shape_and_orientation,plot_white_pixel_ratios,ShapeCodeProcessor 类
# process_image_and_get_shape_code

# Calculate white pixel ratio per bin
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

# Analyze pixel trend for junction detection
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

# Identify shape code and orientation
def identify_junction_shape_and_orientation(width_trends, height_trends):
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
    print(f"junction shape and orientation: {shape_orientation}")
    print(f"Shape code: {shape_code}")

    return shape_code

# Wrap trend analysis and return shape code
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
    shape_code = identify_junction_shape_and_orientation(width_trends, height_trends)

    return shape_code



# Lateral PID control with dead zone
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


# Get yellow line center (cx)
def detect_yellow_line(frame, lower_yellow, upper_yellow):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_raw = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask_blurred = cv2.medianBlur(mask_raw, 3)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask_blurred, cv2.MORPH_CLOSE, kernel)
    moments = cv2.moments(mask)
    if moments["m00"] > 0:
        cx = int(moments["m10"] / moments["m00"])
    else:
        cx = -1
    return cx

# Follow yellow line using PID control
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


# Class for smoothing shape codes
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

# Extract smoothed shape code from image
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


# 创建 CvBridge 对象用于 ROS 图像与 OpenCV 图像转换
bridge = CvBridge()

# 无人机控制默认参数
forward_speed = 0.1  # Default forward speed

# 当前图像帧计数
image_counter = 0

# 上一帧的形状代码
last_shape_code = 0

# 姿态与位置相关
current_pose = None
current_twist = None
current_pitch = None
current_roll = None
current_yaw = None

# 红色检测与圆形路径逻辑控制
initial_red_detected = False
left_start_point = False
has_completed_circle = False
last_turn_direction = None
start_time_after_left_point = None
last_forward_time = 0
start_time = None
red_detect_counter = 0
red_centered_counter = 0
nonline_counter = 0
t_marker_5_counter = 0

# 图像识别相关状态
tracking_active = False  # 是否正在跟踪（例如形状过渡）
tracking_state = None  # 当前跟踪状态，如 'T_left' 或 'Cross'

# 图像识别与 PID 跟踪阶段控制
pid_phase_started = False
pid_phase_completed = False
first_nonzero_code_seen = False
center_tracking_enabled = True
t_marker_phase_ready = False
second_pid_phase_started = False
second_pid_phase_completed = False
yaw_rotation_completed = False
yaw_aligned_phase2 = False  # 初始为未对准
t_marker_counted = False
pending_pid_restart = False
estimated_yellow_width = None

# 相机订阅器
camera_subscriber = None

# 形状识别计数器
t_left_count = 0
t_right_count = 0
l_left_count = 0
l_right_count = 0
cross_count = 0

# 路径阶段控制
phase = 1
t_detection_round = 1

# 圆环路径追踪参数
FOCAL_LENGTH_PX = 320
yaw_threshold = 45 # 自定义阈值
# 控制主循环运行状态
pause_movement = False
exit_flag = False


# 初始化 ROS 节点
rospy.init_node('drone_data_logger', anonymous=True)

# ROS callback to update MAV pose/yaw
def model_states_callback(data):
    global current_pose, current_twist, current_pitch, current_roll, current_yaw
    drone_name = "iris"

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

def reset_state_for_next_round():
    global pid_phase_started, pid_phase_completed, first_nonzero_code_seen
    global t_marker_phase_ready, second_pid_phase_started, second_pid_phase_completed
    global yaw_rotation_completed, t_detection_round

    pid_phase_started = False
    pid_phase_completed = False
    first_nonzero_code_seen = False
    center_tracking_enabled = True
    t_marker_phase_ready = False
    second_pid_phase_started = False
    second_pid_phase_completed = False
    yaw_rotation_completed = False

    t_detection_round += 1
    print(f"[Info] State reset. Starting round {t_detection_round}.")


# Image callback of flight path 1
def image_callback_a03(msg):
    global camera_subscriber, t_right_count, last_shape_code, pause_movement, exit_flag, tracking_active, image_counter
    global current_pose, current_twist, current_pitch, current_roll, current_yaw  # 使用新的全局变量

    print(f"Last shape code before update: {last_shape_code}")

    # 获取无人机状态信息
    position = current_pose.position if current_pose else None
    velocity = current_twist.linear if current_twist else None
    # 使用 Gazebo 提取的姿态信息
    pitch = current_pitch if current_pitch is not None else None
    roll = current_roll if current_roll is not None else None
    yaw = current_yaw if current_yaw is not None else None

    # 转换图像
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    small_image = cv2.resize(cv_image, (400, 400))


    # Convert the OpenCV image to a NumPy array
    frame = np.array(small_image)  # Use the resized image

    # 获取原代码及平滑后的形状代码
    shape_code, smoothed_shape_code = process_image_and_get_shape_code(cv_image)

    # 确保数据类型一致
    last_shape_code = int(last_shape_code)
    smoothed_shape_code = int(smoothed_shape_code)

    print(f"Checking condition: last_shape_code={last_shape_code}, smoothed_shape_code={smoothed_shape_code}")
    # 如果形状代码为4，进入阶段
    if smoothed_shape_code == 4 and not tracking_active:
        print("Shape code 4 detected. Starting transition phase.")
        tracking_active = True

    elif smoothed_shape_code == 8 and tracking_active:
        t_right_count += 1
        tracking_active = False  # 重置跟踪状态

        print(f"T right transition detected. Current count: {t_right_count}")

        if t_right_count == 1:
            print("First T right detected.")
            send_ned_velocity(vehicle, velocity_x=forward_speed, velocity_y=0, velocity_z=0, duration=0.1)
        elif t_right_count == 2:
            print("Second T right detected.")
            send_ned_velocity(vehicle, velocity_x=forward_speed, velocity_y=0, velocity_z=0, duration=0.1)
        elif t_right_count == 3:
            print("Third T right detected. Position '03' reached.")

            # 暂停主循环
            pause_movement = True

            # 无人机悬停
            hover(vehicle, hover_time=5)

            # 停止相机订阅
            camera_subscriber.unregister()
            print("Camera subscriber stopped.")


            # 设置退出标志，退出主循环
            exit_flag = True

    elif smoothed_shape_code == 2:

        # 调用路径跟踪函数
        follow_yellow_line(vehicle, frame)

    # Display the image
    cv2.imshow("frame", frame)
    cv2.waitKey(10)

    # 更新上一帧形状代码
    last_shape_code = smoothed_shape_code
    print(f"Last shape code after update: {last_shape_code}")

# Image callback of flight path 2
def image_callback_a13(msg):
    global camera_subscriber, t_left_count, cross_count, last_shape_code, pause_movement, exit_flag, image_counter
    global tracking_active, tracking_state
    global current_pose, current_twist, current_pitch, current_roll, current_yaw  # 使用新的全局变量

    print(f"Last shape code before processing: {last_shape_code}")

    # 获取无人机状态信息
    position = current_pose.position if current_pose else None
    velocity = current_twist.linear if current_twist else None
    # 使用 Gazebo 提取的姿态信息
    pitch = current_pitch if current_pitch is not None else None
    roll = current_roll if current_roll is not None else None
    yaw = current_yaw if current_yaw is not None else None

    # Convert the ROS image message to an OpenCV image
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    small_image = cv2.resize(cv_image, (300, 300))

    # Convert the OpenCV image to a NumPy array
    frame = np.array(small_image)  # Use the resized image

    # 获取平滑后的形状代码
    shape_code, smoothed_shape_code = process_image_and_get_shape_code(cv_image)

    # 确保数据类型一致
    last_shape_code = int(last_shape_code)
    smoothed_shape_code = int(smoothed_shape_code)
    print(f"Checking transition condition: last_shape_code={last_shape_code}, smoothed_shape_code={smoothed_shape_code}")

    ### 步骤 1: 检测并处理 T 形标志 ###
    if not tracking_active and smoothed_shape_code == 5:
        # 开始跟踪 T 左过渡
        tracking_active = True
        #tracking_state = 'T_left'
        print("Started tracking T left transition.")

    if tracking_active and smoothed_shape_code == 9:
        t_left_count += 1
        print(f"T left transition completed. T left count: {t_left_count}")
        tracking_active = False
        #tracking_state = None

        if t_left_count == 1:
           print("First T left detected")
           pause_movement = True  # 暂停主循环
           # ✅ 正确调用旋转函数，获取时间数据
           rotate_counterclockwise_90(vehicle)
           pause_movement = False  # 恢复主循环
           return

    ### 步骤 2: 检测并处理交叉标志 ###
    if not tracking_active and smoothed_shape_code == 7:
        # 开始跟踪交叉过渡
        tracking_active = True
        #tracking_state = 'Cross'
        print("Started tracking Cross transition.")

    if tracking_active and smoothed_shape_code == 3:
        cross_count += 1
        print(f"Cross transition completed. Cross count: {cross_count}")
        tracking_active = False
        #tracking_state = None

        if cross_count == 1:
            print("First cross detected.")
            send_ned_velocity(vehicle, velocity_x=forward_speed, velocity_y=0, velocity_z=0, duration=0.1)
        elif cross_count == 2:
            print("Second cross detected.")
            send_ned_velocity(vehicle, velocity_x=forward_speed, velocity_y=0, velocity_z=0, duration=0.1)
        elif cross_count == 3:
            print("Third cross detected. Position '13' reached.")

            # 暂停主循环
            pause_movement = True

            # 无人机悬停
            hover(vehicle, hover_time=5)

            # 停止相机订阅
            camera_subscriber.unregister()
            print("Camera subscriber stopped.")


            exit_flag = True  # 设置退出标志
    elif smoothed_shape_code == 2 :
        # 调用路径跟踪函数
        follow_yellow_line(vehicle, frame)

    # Display the image
    cv2.imshow("frame", frame)
    cv2.waitKey(10)
    # 更新上一帧形状代码
    last_shape_code = smoothed_shape_code
    print(f"Last shape code after update: {last_shape_code}")

# Image callback of flight path 3
def image_callback_a23(msg):
    global camera_subscriber, t_left_count, cross_count, last_shape_code, pause_movement, exit_flag, image_counter
    global tracking_active, tracking_state
    global current_pose, current_twist, current_pitch, current_roll, current_yaw
    print(f"Last shape code before processing: {last_shape_code}")
    # 获取无人机状态信息
    position = current_pose.position if current_pose else None
    velocity = current_twist.linear if current_twist else None
    pitch = current_pitch if current_pitch is not None else "N/A"
    roll = current_roll if current_roll is not None else "N/A"
    yaw = current_yaw if current_yaw is not None else "N/A"

    # Convert the ROS image message to an OpenCV image
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    small_image = cv2.resize(cv_image, (400, 400))
    # Convert the OpenCV image to a NumPy array
    frame = np.array(small_image)

    # 获取平滑后的形状代码
    shape_code, smoothed_shape_code = process_image_and_get_shape_code(cv_image)

    last_shape_code = int(last_shape_code)
    smoothed_shape_code = int(smoothed_shape_code)

    ### 步骤 1: 处理 T 形标志 ###
    if not tracking_active and smoothed_shape_code == 5:
        tracking_active = True
        tracking_state = 'T_left'
        print("Started tracking T left transition.")

    if tracking_active and tracking_state == 'T_left' and smoothed_shape_code == 9:
        t_left_count += 1
        print(f"T left transition completed. T left count: {t_left_count}")
        tracking_active = False
        tracking_state = None

        if t_left_count == 1:
            print("First T left detected.")
            send_ned_velocity(vehicle, velocity_x=forward_speed, velocity_y=0, velocity_z=0, duration=0.1)
        elif t_left_count == 2:
            print("Second T left detected, rotating counterclockwise 90 degrees.")
            pause_movement = True
            # ✅ 正确调用旋转函数，获取时间数据
            rotate_counterclockwise_90(vehicle)
            pause_movement = False


        return

    ### 步骤 2: 处理交叉标志 ###
    if not tracking_active and smoothed_shape_code == 7:
        tracking_active = True
        tracking_state = 'Cross'
        print("Started tracking Cross transition.")

    if tracking_active and tracking_state == 'Cross' and smoothed_shape_code == 3:
        cross_count += 1
        print(f"Cross transition completed. Cross count: {cross_count}")
        tracking_active = False
        tracking_state = None

        if cross_count == 1:
            send_ned_velocity(vehicle, velocity_x=forward_speed, velocity_y=0, velocity_z=0, duration=0.1)
        elif cross_count == 2:
            send_ned_velocity(vehicle, velocity_x=forward_speed, velocity_y=0, velocity_z=0, duration=0.1)
        elif cross_count == 3:
            print("Third cross detected. Position '23' reached.")
            pause_movement = True
            hover(vehicle, hover_time=5)
            camera_subscriber.unregister()
            print("Camera subscriber stopped.")


            exit_flag = True

    elif smoothed_shape_code == 2:
        follow_yellow_line(vehicle, frame)

    cv2.imshow("frame", frame)
    cv2.waitKey(10)

    last_shape_code = smoothed_shape_code
    print(f"Last shape code after update: {last_shape_code}")

# Image callback of flight path4
def image_callback_a53(msg):
    global camera_subscriber, t_left_count, phase, last_shape_code, pause_movement, exit_flag, image_counter
    global tracking_active, tracking_state
    global current_pose, current_twist, current_pitch, current_roll, current_yaw  # 使用新的全局变量
    global yaw_aligned_phase2  # ✅ 新增变量用于标志是否已对准
    print(f"Last shape code before processing: {last_shape_code}")

    # 获取无人机状态信息
    position = current_pose.position if current_pose else None
    velocity = current_twist.linear if current_twist else None
    # 使用 Gazebo 提取的姿态信息
    pitch = current_pitch if current_pitch is not None else None
    roll = current_roll if current_roll is not None else None
    yaw = current_yaw if current_yaw is not None else None

    # 转换 ROS 图像消息为 OpenCV 图像
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    small_image = cv2.resize(cv_image, (400, 400))
    # Convert the OpenCV image to a NumPy array
    frame = np.array(small_image)  # Use the resized image
    # 获取平滑后的形状代码
    shape_code, smoothed_shape_code = process_image_and_get_shape_code(cv_image)

    # 确保数据类型一致
    last_shape_code = int(last_shape_code)
    smoothed_shape_code = int(smoothed_shape_code)

    print(f"Last shape code: {last_shape_code}, Smoothed shape code: {smoothed_shape_code}")

    # 第一阶段逻辑：从 5 到 9
    if phase == 1:
        if not tracking_active and smoothed_shape_code == 5:
            tracking_active = True
            tracking_state = "T_detecting"

        if tracking_active and tracking_state == "T_detecting" and smoothed_shape_code == 9:
            t_left_count += 1
            tracking_active = False  # 停止跟踪
            print(f"T-shaped marker {t_left_count} detected in phase 1.")

            if t_left_count < 4:
                send_ned_velocity(vehicle, velocity_x=forward_speed, velocity_y=0, velocity_z=0, duration=0.1)
            elif t_left_count == 4:
                print("Fourth T-shaped marker detected. Proceeding to phase 2.")
                t_left_count = 0  # 重置计数器
                phase = 2  # 进入第二阶段
        elif smoothed_shape_code == 2:
             # 调用路径跟踪函数
            follow_yellow_line(vehicle, frame)


    # 第二阶段逻辑：从 2 到 5
    elif phase == 2:
        if smoothed_shape_code == 2:
            # ✅ 如果尚未Yaw对准，执行Yaw调整
            if not yaw_aligned_phase2:
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                lower_yellow = np.array([20, 100, 100])
                upper_yellow = np.array([30, 255, 255])
                mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
                kernel = np.ones((5, 5), np.uint8)
                yellow_mask = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel)

                M = cv2.moments(yellow_mask)
                if M["m00"] > 0:
                    cX = int(M["m10"] / M["m00"])
                    frame_center_x = frame.shape[1] // 2
                    deviation = cX - frame_center_x

                    # 获取当前高度
                    altitude_m = current_pose.position.z if current_pose else 0.5
                    yaw_angle = calculate_yaw_angle(deviation, altitude_m, max_yaw_angle=20)

                    if abs(deviation) > 10:  # 设置一个小阈值避免微调
                        print(f"[Yaw Correction] Adjusting yaw by {yaw_angle:.2f}° for alignment.")
                        condition_yaw(vehicle, yaw_angle, relative=True)
                    else:
                        print("[Yaw Correction] Already aligned with yellow line.")

                    yaw_aligned_phase2 = True  # ✅ 设置为已对准，避免后续重复调整

        if not tracking_active and smoothed_shape_code == 2:
            tracking_active = True
            tracking_state = "Left_turn_detecting"
            
        if tracking_active and tracking_state == "Left_turn_detecting" and smoothed_shape_code == 5:
            print("Left turn detected. Rotating counterclockwise 90 degrees.")
            tracking_active = False  # 停止跟踪
            pause_movement = True
            # ✅ 正确调用旋转函数，获取时间数据
            rotate_counterclockwise_90(vehicle)
            send_ned_velocity(vehicle, velocity_x=forward_speed, velocity_y=0, velocity_z=0, duration=0.1)
            pause_movement = False
            phase = 3  # 进入第三阶段

    # 第三阶段逻辑：从 5 到 9
    elif phase == 3:

        if not tracking_active and smoothed_shape_code == 5:
            tracking_active = True
            tracking_state = "T_detecting"

        if tracking_active and tracking_state == "T_detecting" and smoothed_shape_code == 9:
            t_left_count += 1
            tracking_active = False  # 停止跟踪
            print(f"T-shaped marker {t_left_count} detected in phase 3.")

            if t_left_count < 3:
                send_ned_velocity(vehicle, velocity_x=forward_speed, velocity_y=0, velocity_z=0, duration=0.1)
            elif t_left_count == 3:

                print("Third T right detected. Position '53' reached.")

                # 暂停主循环
                pause_movement = True

                # 无人机悬停
                hover(vehicle, hover_time=0)

                # 停止相机订阅
                camera_subscriber.unregister()
                print("Camera subscriber stopped.")


                # 设置退出标志，退出主循环
                exit_flag = True
        elif smoothed_shape_code == 2 :

            # 调用路径跟踪函数
            follow_yellow_line(vehicle, frame)

    # Display the image
    cv2.imshow("frame", frame)
    cv2.waitKey(10)
    # 更新上一帧形状代码
    last_shape_code = smoothed_shape_code
    print(f"Last shape code after update: {last_shape_code}")

def image_callback_b00(msg):
    global camera_subscriber

    print("Received image in image_callback_200.")

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


        # 悬停10秒
        hover(vehicle, hover_time=5)

        # 缓慢降落
        print("Initiating slow descent...")
        send_ned_velocity(vehicle, velocity_x=0, velocity_y=0, velocity_z=0.2, duration=5)  # 0.2 m/s下降
        vehicle.mode = VehicleMode("LAND")
        print("Landing initiated.")

        # 完成后退出
        print("Mission complete for image_callback_200.")


def image_callback_b01(msg):
    global camera_subscriber, t_left_count, last_shape_code, pause_movement, exit_flag, image_counter
    global tracking_active, tracking_state
    global current_pose, current_twist, current_pitch, current_roll, current_yaw

    print(f"Last shape code before processing: {last_shape_code}")


    # === 获取无人机状态 ===
    position = current_pose.position if current_pose else None
    velocity = current_twist.linear if current_twist else None
    pitch = current_pitch if current_pitch is not None else None
    roll = current_roll if current_roll is not None else None
    yaw = current_yaw if current_yaw is not None else None

    # === 图像处理 ===
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    small_image = cv2.resize(cv_image, (400, 400))
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
            hover(vehicle, hover_time=5)
            camera_subscriber.unregister()
            print("Camera subscriber stopped.")
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

def image_callback_b02(msg):
    global camera_subscriber, t_left_count, last_shape_code, pause_movement, exit_flag, image_counter
    global tracking_active, tracking_state, phase
    global current_pose, current_twist, current_pitch, current_roll, current_yaw

    print(f"Last shape code before processing: {last_shape_code}")


    # === 获取无人机状态 ===
    position = current_pose.position if current_pose else None
    velocity = current_twist.linear if current_twist else None
    pitch = current_pitch if current_pitch is not None else None
    roll = current_roll if current_roll is not None else None
    yaw = current_yaw if current_yaw is not None else None

    # === 图像处理 ===
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    small_image = cv2.resize(cv_image, (300, 300))


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

def image_callback_b03(msg):
    global camera_subscriber, t_right_count, last_shape_code, pause_movement, exit_flag, image_counter
    global tracking_active, tracking_state
    global current_pose, current_twist, current_pitch, current_roll, current_yaw

    print(f"Last shape code before processing: {last_shape_code}")

    # === 获取无人机状态 ===
    position = current_pose.position if current_pose else None
    velocity = current_twist.linear if current_twist else None
    pitch = current_pitch if current_pitch is not None else None
    roll = current_roll if current_roll is not None else None
    yaw = current_yaw if current_yaw is not None else None

    # === 图像处理 ===
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    small_image = cv2.resize(cv_image, (400, 400))
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



def image_callback_c00(msg):
    global camera_subscriber

    print("Received image in image_callback_300.")

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

        # 悬停10秒
        hover(vehicle, hover_time=5)

        # 缓慢降落
        print("Initiating slow descent...")
        send_ned_velocity(vehicle, velocity_x=0, velocity_y=0, velocity_z=0.2, duration=5)  # 0.2 m/s下降
        vehicle.mode = VehicleMode("LAND")
        print("Landing initiated.")

        # 完成后退出
        print("Mission complete for image_callback_300.")


def image_callback_c01(msg):
    global camera_subscriber, t_left_count, last_shape_code, pause_movement, exit_flag, image_counter
    global tracking_active, tracking_state
    global current_pose, current_twist, current_pitch, current_roll, current_yaw
    global pid_phase_started, pid_phase_completed, first_nonzero_code_seen, center_tracking_enabled
    global t_marker_phase_ready
    global second_pid_phase_started, second_pid_phase_completed
    global yaw_rotation_completed  # ✅ 新增，表示是否已完成一次旋转

    # === 状态获取 ===
    position = current_pose.position if current_pose else None
    velocity = current_twist.linear if current_twist else None
    pitch = current_pitch if current_pitch is not None else None
    roll = current_roll if current_roll is not None else None
    yaw = current_yaw if current_yaw is not None else None

    # === 图像处理 ===
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    small_image = cv2.resize(cv_image, (400, 400))
    frame = np.array(small_image)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([18, 94, 140])
    upper_yellow = np.array([48, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # === 图像识别 ===
    shape_code, smoothed_shape_code = process_image_and_get_shape_code(cv_image)
    print(f"[Shape] last={last_shape_code}, current={smoothed_shape_code}")

    # === 起点标志 ===
    if smoothed_shape_code == 0:
        print("[Info] Detected start marker (code 0). Skipping tracking and yaw control.")
        last_shape_code = smoothed_shape_code
        return

    # === 控制逻辑 ===
    if center_tracking_enabled:
        moments = cv2.moments(mask)
        if moments["m00"] != 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            frame_width = frame.shape[1]
            frame_center = frame_width // 2
            frame_height = frame.shape[0]

            deviation = cx - frame_center
            normalized_deviation = deviation / frame_center

            if not first_nonzero_code_seen and smoothed_shape_code != 0:
                first_nonzero_code_seen = True
                pid_phase_started = True
                print("[PID Phase] First non-zero shape code seen. Starting lateral PID adjustment.")

            # === 第一轮 PID 控制 ===
            if pid_phase_started and not pid_phase_completed:
                Kp, Ki, Kd = 1.0, 0.02, 0.5
                pid_dead_zone = 0.05
                lateral_adjustment = pid_control_with_dead_zone(normalized_deviation, Kp, Ki, Kd, dead_zone=pid_dead_zone)

                forward_speed = 0.2
                lateral_speed = lateral_adjustment * 0.5
                send_ned_velocity(vehicle, velocity_x=forward_speed, velocity_y=lateral_speed, velocity_z=0, duration=0.1)

                print(f"[1st PID] Normalized Deviation: {normalized_deviation:.2f}, Lateral Adjustment: {lateral_speed:.2f}")

                if abs(normalized_deviation) <= pid_dead_zone:
                    pid_phase_completed = True
                    pid_phase_started = False
                    print("[1st PID] Deviation within dead zone. PID complete. Preparing for yaw control.")

            # === 自适应 Yaw 控制阶段 ===
            elif pid_phase_completed and not yaw_rotation_completed:
                altitude_m = current_pose.position.z
                sensitivity = round(altitude_m, 2)

                if yaw_threshold/(2 * sensitivity) <= abs(deviation):
                    print(
                        f"[Yaw] Deviation {abs(deviation)} within threshold ({yaw_threshold} / (2 * {sensitivity})). Rotating 45 degrees.")
                    rotate_counterclockwise_45(vehicle)
                    yaw_rotation_completed = True
                    second_pid_phase_started = True
                    print("[Yaw] Rotation complete. Starting 2nd PID phase.")
                else:
                    print(f"[Yaw] Deviation {abs(deviation)} not in range. Waiting...")

            # === 第二轮 PID 控制 ===
            elif second_pid_phase_started and not second_pid_phase_completed:
                Kp, Ki, Kd = 1.0, 0.02, 0.5
                pid_dead_zone = 0.05
                lateral_adjustment = pid_control_with_dead_zone(normalized_deviation, Kp, Ki, Kd, dead_zone=pid_dead_zone)

                forward_speed = 0.2
                lateral_speed = lateral_adjustment * 0.5
                send_ned_velocity(vehicle, velocity_x=forward_speed, velocity_y=lateral_speed, velocity_z=0, duration=0.1)

                print(f"[2nd PID] Normalized Deviation: {normalized_deviation:.2f}, Lateral Adjustment: {lateral_speed:.2f}")

                if abs(normalized_deviation) <= pid_dead_zone:
                    second_pid_phase_completed = True
                    second_pid_phase_started = False
                    t_marker_phase_ready = True
                    print("[2nd PID] Deviation within threshold. Ready for T marker detection.")

            # === 可视化红点（中心）与质心 ===
            cv2.circle(frame, (frame_center, frame_height // 2), 8, (0, 0, 255), -1)
            cv2.circle(frame, (cx, cy), 8, (255, 0, 0), -1)

    elif tracking_active:
        send_ned_velocity(vehicle, velocity_x=0.1, velocity_y=0, velocity_z=0, duration=0.1)
        print("Tracking active: Moving forward at 0.1 m/s (no PID).")

    # === T 型转弯识别 ===
    if t_marker_phase_ready:
        if smoothed_shape_code == 5 and not tracking_active:
            tracking_active = True
            tracking_state = "T_left_detecting"
            center_tracking_enabled = False
            print("Started tracking T left transition.")
        elif tracking_active and tracking_state == "T_left_detecting" and smoothed_shape_code == 9:
            t_left_count += 1
            print(f"T left transition completed. Count: {t_left_count}")
            tracking_active = False
            tracking_state = None
            center_tracking_enabled = True
            t_marker_phase_ready = False

            if t_left_count == 1:
                pause_movement = True
                hover(vehicle, hover_time=5)
                if camera_subscriber:
                    camera_subscriber.unregister()
                    print("Camera subscriber stopped.")

    last_shape_code = smoothed_shape_code
    cv2.imshow("frame", frame)
    cv2.waitKey(10)


def image_callback_c02(msg):
    global camera_subscriber, t_left_count, last_shape_code, pause_movement, exit_flag, image_counter
    global tracking_active, tracking_state
    global current_pose, current_twist, current_pitch, current_roll, current_yaw
    global pid_phase_started, pid_phase_completed, first_nonzero_code_seen, center_tracking_enabled
    global t_marker_phase_ready
    global second_pid_phase_started, second_pid_phase_completed
    global yaw_rotation_completed
    global t_detection_round  # ✅ 新增，标记当前第几轮检测
    global yaw_threshold_list
    global estimated_yellow_width


    # === 状态获取 ===
    position = current_pose.position if current_pose else None
    velocity = current_twist.linear if current_twist else None
    pitch = current_pitch if current_pitch is not None else None
    roll = current_roll if current_roll is not None else None
    yaw = current_yaw if current_yaw is not None else None

    # === 图像处理 ===
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    small_image = cv2.resize(cv_image, (400, 400))
    frame = np.array(small_image)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([18, 94, 140])
    upper_yellow = np.array([48, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)


    # === 图像识别 ===
    shape_code, smoothed_shape_code = process_image_and_get_shape_code(cv_image)
    print(f"[Shape] last={last_shape_code}, current={smoothed_shape_code}")



    # === 起点标志 ===
    if smoothed_shape_code == 0:
       print("[Info] Detected start marker (code 0). Skipping tracking and yaw control.")
       last_shape_code = smoothed_shape_code
       return

    # === 控制逻辑 ===
    if center_tracking_enabled:
        moments = cv2.moments(mask)
        if moments["m00"] != 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])

            frame_width = frame.shape[1]
            frame_center = frame_width // 2
            frame_height = frame.shape[0]

            deviation = cx - frame_center
            normalized_deviation = deviation / frame_center

            if not first_nonzero_code_seen and smoothed_shape_code != 0:
                first_nonzero_code_seen = True
                pid_phase_started = True
                print(f"[Round {t_detection_round}] First non-zero shape code seen. Starting lateral PID adjustment.")

            # === 第一轮 PID ===
            if pid_phase_started and not pid_phase_completed:
                Kp, Ki, Kd = 1.0, 0.02, 0.5
                pid_dead_zone = 0.05

                lateral_adjustment = pid_control_with_dead_zone(
                    normalized_deviation, Kp, Ki, Kd, dead_zone=pid_dead_zone
                )

                forward_speed = 0.2
                lateral_speed = lateral_adjustment * 0.5
                send_ned_velocity(
                    vehicle,
                    velocity_x=forward_speed,
                    velocity_y=lateral_speed,
                    velocity_z=0,
                    duration=0.1
                )

                print(f"[1st PID] Normalized Deviation: {normalized_deviation:.2f}, Lateral Adjustment: {lateral_speed:.2f}")

                if abs(normalized_deviation) <= pid_dead_zone:
                    pid_phase_completed = True
                    pid_phase_started = False
                    print("[1st PID] Deviation within dead zone. PID complete. Preparing for Yaw control.")

            # === Yaw 阶段 ===
            elif pid_phase_completed and not yaw_rotation_completed:
                altitude_m = current_pose.position.z
                sensitivity = round(altitude_m, 2)

                if abs(deviation) >= yaw_threshold / (2 * sensitivity):
                    print(
                        f"[Yaw] Deviation {abs(deviation)} within threshold ({yaw_threshold} / (2 * {sensitivity})). Rotating 45 degrees.")
                    rotate_counterclockwise_45(vehicle)
                    yaw_rotation_completed = True
                    second_pid_phase_started = True
                    print("[Yaw] Rotation complete. Starting 2nd PID phase.")
                else:
                    print(f"[Yaw] Deviation {abs(deviation)} not in range. Waiting...")


            # === 第二轮 PID ===
            elif second_pid_phase_started and not second_pid_phase_completed:
                Kp, Ki, Kd = 1.0, 0.02, 0.5
                pid_dead_zone = 0.05

                lateral_adjustment = pid_control_with_dead_zone(
                    normalized_deviation, Kp, Ki, Kd, dead_zone=pid_dead_zone
                )

                forward_speed = 0.2
                lateral_speed = lateral_adjustment * 0.5
                send_ned_velocity(
                    vehicle,
                    velocity_x=forward_speed,
                    velocity_y=lateral_speed,
                    velocity_z=0,
                    duration=0.1
                )

                print(f"[2nd PID] Normalized Deviation: {normalized_deviation:.2f}, Lateral Adjustment: {lateral_speed:.2f}")

                if abs(normalized_deviation) <= pid_dead_zone:
                    second_pid_phase_completed = True
                    second_pid_phase_started = False
                    t_marker_phase_ready = True
                    print("[2nd PID] Deviation within threshold. Ready for T marker detection.")

            cv2.circle(frame, (frame_center, frame_height // 2), 8, (0, 0, 255), -1)
            cv2.circle(frame, (cx, cy), 8, (255, 0, 0), -1)

    elif tracking_active:
        send_ned_velocity(vehicle, velocity_x=0.2, velocity_y=0, velocity_z=0, duration=0.1)
        print("Tracking active: Moving forward at 0.1 m/s (no PID).")

    # === T型转弯检测阶段 ===
    if t_marker_phase_ready:
        if smoothed_shape_code == 5 and not tracking_active:
            tracking_active = True
            tracking_state = "T_left_detecting"
            center_tracking_enabled = False
            print(f"[Round {t_detection_round}] Started tracking T left transition.")

        elif tracking_active and tracking_state == "T_left_detecting" and smoothed_shape_code == 9:
            t_left_count += 1
            print(f"[Round {t_detection_round}] T left transition completed. Count: {t_left_count}")
            tracking_active = False
            tracking_state = None
            center_tracking_enabled = True
            t_marker_phase_ready = False

            if t_detection_round == 1:
                # ✅ 第一次检测完，重置状态继续飞行
                print("[Info] First T marker detected. Continuing flight (no pause).")
                reset_state_for_next_round()

            elif t_detection_round == 2:
                # ✅ 第二次检测完，悬停 + 停止订阅
                pause_movement = True
                hover(vehicle, hover_time=5)
                if camera_subscriber:
                    camera_subscriber.unregister()
                    print("Camera subscriber stopped.")
                exit_flag = True  # 可选：标记退出

    last_shape_code = smoothed_shape_code
    cv2.imshow("frame", frame)
    cv2.waitKey(10)


def image_callback_c04(msg):
    global camera_subscriber, t_left_count, last_shape_code, pause_movement, exit_flag, image_counter
    global tracking_active, tracking_state
    global current_pose, current_twist, current_pitch, current_roll, current_yaw
    global pid_phase_started, pid_phase_completed, first_nonzero_code_seen, center_tracking_enabled
    global t_marker_phase_ready
    global second_pid_phase_started, second_pid_phase_completed
    global yaw_rotation_completed
    global yaw_threshold_list
    global t_marker_counted, pending_pid_restart
    global nonline_counter, t_marker_5_counter  # 新增

    # === 状态获取 ===
    position = current_pose.position if current_pose else None
    velocity = current_twist.linear if current_twist else None
    pitch = current_pitch if current_pitch is not None else None
    roll = current_roll if current_roll is not None else None
    yaw = current_yaw if current_yaw is not None else None

    # === 图像处理 ===
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    small_image = cv2.resize(cv_image, (400, 400))
    frame = np.array(small_image)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([18, 94, 140])
    upper_yellow = np.array([48, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # === 图像识别 ===
    shape_code, smoothed_shape_code = process_image_and_get_shape_code(cv_image)
    print(f"[Shape] last={last_shape_code}, current={smoothed_shape_code}")

    if smoothed_shape_code == 0:
        print("[Info] Detected start marker (code 0). Skipping tracking and yaw control.")
        last_shape_code = smoothed_shape_code
        return

    if center_tracking_enabled:
        moments = cv2.moments(mask)
        if moments["m00"] != 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])

            frame_width = frame.shape[1]
            frame_center = frame_width // 2
            frame_height = frame.shape[0]

            deviation = cx - frame_center
            normalized_deviation = deviation / frame_center


            if not first_nonzero_code_seen and smoothed_shape_code != 0:
                first_nonzero_code_seen = True
                pid_phase_started = True
                print(f"[T Count {t_left_count}] First non-zero shape code seen. Starting lateral PID adjustment.")

            # === 第一轮 PID 控制 ===
            if pid_phase_started and not pid_phase_completed:
                Kp, Ki, Kd = 1.0, 0.02, 0.5
                pid_dead_zone = 0.05
                lateral_adjustment = pid_control_with_dead_zone(normalized_deviation, Kp, Ki, Kd, pid_dead_zone)
                forward_speed = 0.2
                lateral_speed = lateral_adjustment * 0.5
                send_ned_velocity(vehicle, forward_speed, lateral_speed, 0, 0.1)
                print(f"[1st PID] Normalized Deviation: {normalized_deviation:.2f}, Lateral Adjustment: {lateral_speed:.2f}")

                if abs(normalized_deviation) <= pid_dead_zone:
                    pid_phase_completed = True
                    pid_phase_started = False
                    print("[1st PID] Deviation within dead zone. PID complete. Preparing for Yaw control.")

            # === Yaw 调整阶段 ===
            elif pid_phase_completed and not yaw_rotation_completed:
                altitude_m = current_pose.position.z
                sensitivity = round(altitude_m, 2)

                if abs(deviation) >= yaw_threshold / (2 * sensitivity):
                    print(
                        f"[Yaw] Deviation {abs(deviation)} within threshold ({yaw_threshold} / (2 * {sensitivity})). Rotating 45 degrees.")
                    rotate_counterclockwise_45(vehicle)
                    yaw_rotation_completed = True
                    second_pid_phase_started = True
                    print("[Yaw] Rotation complete. Starting 2nd PID phase.")
                else:
                    print(f"[Yaw] Deviation {abs(deviation)} not in range. Waiting...")


            # === 第二轮 PID 控制 ===
            elif second_pid_phase_started and not second_pid_phase_completed:
                Kp, Ki, Kd = 1.0, 0.02, 0.5
                pid_dead_zone = 0.05
                lateral_adjustment = pid_control_with_dead_zone(normalized_deviation, Kp, Ki, Kd, pid_dead_zone)
                forward_speed = 0.2
                lateral_speed = lateral_adjustment * 0.5
                send_ned_velocity(vehicle, forward_speed, lateral_speed, 0, 0.1)
                print(f"[2nd PID] Normalized Deviation: {normalized_deviation:.2f}, Lateral Adjustment: {lateral_speed:.2f}")

                if abs(normalized_deviation) <= pid_dead_zone:
                    second_pid_phase_completed = True
                    second_pid_phase_started = False
                    t_marker_phase_ready = True
                    print("[2nd PID] Deviation within threshold. Ready for T marker detection.")

            cv2.circle(frame, (frame_center, frame_height // 2), 8, (0, 0, 255), -1)
            cv2.circle(frame, (cx, cy), 8, (255, 0, 0), -1)

    elif tracking_active:
        send_ned_velocity(vehicle, 0.2, 0, 0, 0.1)
        print("Tracking active: Moving forward at 0.1 m/s (no PID).")

    # === T型标志检测阶段 ===
    if t_marker_phase_ready:
        if smoothed_shape_code == 5 and not tracking_active:
                tracking_active = True
                tracking_state = "T_left_detecting"
                center_tracking_enabled = False
                print(f"[T Count {t_left_count}] Started tracking T left transition.")

        elif tracking_active and tracking_state == "T_left_detecting" and smoothed_shape_code == 9:
            t_left_count += 1
            print(f"[T Count {t_left_count}] T left transition completed.")
            tracking_active = False
            tracking_state = None
            center_tracking_enabled = True
            t_marker_phase_ready = False

            if t_left_count < 4:
                print(f"[Info] T marker #{t_left_count} detected. Continuing flight (no pause).")
                reset_state_for_next_round()
            elif t_left_count == 4:
                print(f"[Info] T marker #{t_left_count} detected. Hovering and stopping camera.")
                pause_movement = True
                hover(vehicle, hover_time=5)
                if camera_subscriber:
                    camera_subscriber.unregister()
                    print("Camera subscriber stopped.")
                exit_flag = True  # 可选

    last_shape_code = smoothed_shape_code
    cv2.imshow("frame", frame)
    cv2.waitKey(10)


def image_callback_d00(msg):
    global initial_red_detected, has_completed_circle, left_start_point
    global image_counter, current_pose, current_twist, current_pitch, current_roll, current_yaw
    global start_time_after_left_point, last_forward_time, start_time, red_detect_counter, red_centered_counter

    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    small_image = cv2.resize(cv_image, (400, 400))
    hsv = cv2.cvtColor(small_image, cv2.COLOR_BGR2HSV)

    # 红色区域提取
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
    red_pixel_count = cv2.countNonZero(mask_red)

    # 红点质心计算与是否居中判断
    red_M = cv2.moments(mask_red)
    if red_M["m00"] > 0:
        red_cX = int(red_M["m10"] / red_M["m00"])
        red_cY = int(red_M["m01"] / red_M["m00"])
        frame_center_x = small_image.shape[1] // 2
        frame_center_y = small_image.shape[0] // 2
        in_center_zone = (abs(red_cX - frame_center_x) < 50) and (abs(red_cY - frame_center_y) < 50)
    else:
        in_center_zone = False

    # 红点是否检测到
    is_red_detected = (red_pixel_count > 500 and in_center_zone) or (red_pixel_count > 1000)

    if is_red_detected:
        red_detect_counter += 1
        if in_center_zone:
            red_centered_counter += 1
        else:
            red_centered_counter = 0
    else:
        red_detect_counter = 0
        red_centered_counter = 0

    # 初次检测红点 → 前进离开起点
    if not initial_red_detected and is_red_detected:
        initial_red_detected = True
        print("[Action] Moving forward 1 meter to leave the start point.")
        send_ned_velocity(vehicle, velocity_x=0.2, velocity_y=0, velocity_z=0, duration=5)
        return

    # ✅ 加入基于时间的冗余判断
    if initial_red_detected and not left_start_point:
        if red_detect_counter == 0:
            left_start_point = True
            start_time_after_left_point = time.time()
            print("[Info] Left the start point (red disappeared).")
        elif time.time() - start_time > 8:  # 起飞后超过8秒，强制跳出
            left_start_point = True
            start_time_after_left_point = time.time()
            print("[Bypass] Forced exit from start point due to timeout.")

    # 已离开起点 + 返回起点条件满足 → 执行降落
    MIN_FLIGHT_DURATION = 15
    if (initial_red_detected and left_start_point and red_detect_counter >= 3 and
            red_centered_counter >= 3 and not has_completed_circle and
            start_time_after_left_point is not None and
            (time.time() - start_time_after_left_point) > MIN_FLIGHT_DURATION):
        has_completed_circle = True
        print("[End] Returned to center of start point. Initiating landing sequence.")
        hover(vehicle, 5)
        land()
        return

    # 跟踪黄线逻辑
    if left_start_point:
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        M = cv2.moments(mask_yellow)
        if M["m00"] > 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            frame_center_x = small_image.shape[1] // 2
            x_deviation = cX - frame_center_x
            if current_pose:
                altitude_m = current_pose.position.z
                sensitivity = round(altitude_m, 2)

                max_deviation = 55
                actual_deviation = max(-max_deviation, min(x_deviation, max_deviation))


                yaw_adjustment = calculate_yaw_angle(actual_deviation, altitude_m)

                if abs(actual_deviation) >= yaw_threshold/(2 * sensitivity):
                    print(f"[Debug] Altitude: {altitude_m:.2f} m, Sensitivity: {sensitivity}, Deviation: {actual_deviation}")
                    print(f"[Yaw] Continuous correction: {actual_deviation}px, adjusting {yaw_adjustment:.1f}°")
                    condition_yaw(vehicle, yaw_adjustment, relative=True)

                if time.time() - last_forward_time > 0.5:
                    print(f"[Move] Moving forward with deviation: {actual_deviation}px.")
                    send_ned_velocity(vehicle, 0.2, 0, 0, 0.5)
                    last_forward_time = time.time()

            # 可视化红点与质心
            cv2.circle(small_image, (frame_center_x, small_image.shape[0] // 2), 8, (0, 0, 255), -1)
            cv2.circle(small_image, (cX, cY), 8, (255, 0, 0), -1)

    # 图像显示
    cv2.imshow("Tracking", small_image)
    cv2.waitKey(1)

# 位置字典（同样可以使用之前定义的positions字典）
positions = {
    'a03': (1, 3),
    'a13': (1, 4),
    'a23': (2, 3),
    'a33': (3, 3),
    'a43': (4, 3),
    'a53': (5, 3),
    'b00': (2, 0),
    'b01': (2, 1),
    'b02': (2, 2),
    'b03': (2, 3),
    'c00': (3, 0),
    'c01': (3, 1),
    'c02': (3, 2),
    'c04': (3, 4),
    'd00': (4, 0),
}


def fly_to_a03():
    global camera_subscriber, pause_movement, exit_flag

    print("Flying logic for position 03")

    target_altitude = 0.5 # 起飞目标高度（米）
    forward_speed = 0.1  # 前进速度（米/秒）

    # 确保 GUIDED 模式
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.mode.name == "GUIDED":
        print("Waiting for vehicle to enter GUIDED mode")
        time.sleep(1)

    # 无人机解锁并起飞
    arm_and_takeoff(target_altitude)
    print("Vehicle armed and took off")

    # Set up the subscriber to receive camera messages
    camera_topic = '/camera/image_raw'
    camera_subscriber = rospy.Subscriber(camera_topic, Image, image_callback_a03)


    # 设置主循环频率
    loop_rate = rospy.Rate(30)  # 主循环频率（10 Hz）

    try:
        # 在主循环中持续前进并处理图像数据
        while not rospy.is_shutdown() and not exit_flag:  # 检查 exit_flag 控制退出主循环
            if not pause_movement:
                send_ned_velocity(vehicle, velocity_x=forward_speed, velocity_y=0, velocity_z=0, duration=0.1)
                print(f"Drone moving forward at {forward_speed} m/s")

            # 保持主循环运行
            loop_rate.sleep()

    except rospy.ROSInterruptException:
        print("ROS shutdown")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # 停止无人机运动
        send_ned_velocity(vehicle, velocity_x=0, velocity_y=0, velocity_z=0, duration=1)
        print("Drone stopped.")

        # 停止相机订阅
        if camera_subscriber:
            camera_subscriber.unregister()
            print("Camera subscriber stopped.")

        # 清理资源并结束程序
        if vehicle:
            vehicle.mode = VehicleMode("LAND")  # 降落
            print("Landing vehicle.")

        print("Fly to a03 complete.")


def fly_to_a13():
    global camera_subscriber, pause_movement, exit_flag

    print("Flying logic for position 13")

    target_altitude = 0.5  # 起飞目标高度（米）
    forward_speed = 0.1  # 前进速度（米/秒）

    # 确保 GUIDED 模式
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.mode.name == "GUIDED":
        print("Waiting for vehicle to enter GUIDED mode")
        time.sleep(1)

    # 无人机解锁并起飞
    arm_and_takeoff(target_altitude)
    print("Vehicle armed and took off")

    # 顺时针旋转90度
    rotate_clockwise_90(vehicle)  # 调用顺时针旋转函数
    print("顺时针旋转完成，继续向前移动...")
    hover(vehicle, hover_time=2)

    # Set up the subscriber to receive camera messages
    camera_topic = '/camera/image_raw'
    camera_subscriber = rospy.Subscriber(camera_topic, Image, image_callback_a13)


    # 设置主循环频率
    loop_rate = rospy.Rate(30)  # 主循环频率（10 Hz）

    try:
        # 在主循环中持续前进并处理图像数据
        while not rospy.is_shutdown() and not exit_flag:  # 检查 exit_flag 控制退出主循环
            if not pause_movement:
                send_ned_velocity(vehicle, velocity_x=forward_speed, velocity_y=0, velocity_z=0, duration=0.1)
                print(f"Drone moving forward at {forward_speed} m/s")

            # 保持主循环运行
            loop_rate.sleep()


    except rospy.ROSInterruptException:
        print("ROS shutdown")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # 停止无人机运动
        send_ned_velocity(vehicle, velocity_x=0, velocity_y=0, velocity_z=0, duration=1)
        print("Drone stopped.")

        # 停止相机订阅
        if camera_subscriber:
            camera_subscriber.unregister()
            print("Camera subscriber stopped.")

        # 清理资源并结束程序
        if vehicle:
            vehicle.mode = VehicleMode("LAND")  # 降落
            print("Landing vehicle.")

        print("Fly to a13 complete.")



def fly_to_a23():
    global camera_subscriber, pause_movement, exit_flag

    print("Flying logic for position 23")

    target_altitude = 0.5 # 起飞目标高度（米）
    forward_speed = 0.1  # 前进速度（米/秒）
    # 确保 GUIDED 模式
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.mode.name == "GUIDED":
        print("Waiting for vehicle to enter GUIDED mode")
        time.sleep(1)
    # 无人机解锁并起飞
    arm_and_takeoff(target_altitude)
    print("Vehicle armed and took off")

    # 顺时针旋转90度
    rotate_clockwise_90(vehicle)  # 调用顺时针旋转函数
    print("顺时针旋转完成，继续向前移动...")

    # Set up the subscriber to receive camera messages
    camera_topic = '/camera/image_raw'
    camera_subscriber = rospy.Subscriber(camera_topic, Image, image_callback_a23)


    # 设置主循环频率
    loop_rate = rospy.Rate(30)  # 主循环频率（10 Hz）

    try:
        # 在主循环中持续前进并处理图像数据
        while not rospy.is_shutdown() and not exit_flag:  # 检查 exit_flag 控制退出主循环
            if not pause_movement:
                send_ned_velocity(vehicle, velocity_x=forward_speed, velocity_y=0, velocity_z=0, duration=0.1)
                print(f"Drone moving forward at {forward_speed} m/s")

            # 保持主循环运行
            loop_rate.sleep()

    except rospy.ROSInterruptException:
        print("ROS shutdown")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # 停止无人机运动
        send_ned_velocity(vehicle, velocity_x=0, velocity_y=0, velocity_z=0, duration=1)
        print("Drone stopped.")

        # 停止相机订阅
        if camera_subscriber:
            camera_subscriber.unregister()
            print("Camera subscriber stopped.")

        # 清理资源并结束程序
        if vehicle:
            vehicle.mode = VehicleMode("LAND")  # 降落
            print("Landing vehicle.")

        print("Fly to a23 complete.")


def fly_to_a33():
    print("Flying logic for position a33")
    # Add your drone control code here
    pass


def fly_to_a43():
    print("Flying logic for position a43")
    # Add your drone control code here
    pass

def fly_to_a53():
    global camera_subscriber, pause_movement, exit_flag

    print("Flying logic for position 53")

    target_altitude = 0.5  # 起飞目标高度（米）
    forward_speed = 0.1  # 前进速度（米/秒）

    # 确保 GUIDED 模式
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.mode.name == "GUIDED":
        print("Waiting for vehicle to enter GUIDED mode")
        time.sleep(1)
    # 无人机解锁并起飞
    arm_and_takeoff(target_altitude)
    print("Vehicle armed and took off")

    # 顺时针旋转90度
    rotate_clockwise_90(vehicle)  # 调用顺时针旋转函数
    print("顺时针旋转完成，继续向前移动...")


    # Set up the subscriber to receive camera messages
    camera_topic = '/camera/image_raw'
    camera_subscriber = rospy.Subscriber(camera_topic, Image, image_callback_a53)
    

    # 设置主循环频率
    loop_rate = rospy.Rate(30)  # 主循环频率（10 Hz）

    try:
        # 在主循环中持续前进并处理图像数据
        while not rospy.is_shutdown() and not exit_flag:  # 检查 exit_flag 控制退出主循环
            if not pause_movement:
                send_ned_velocity(vehicle, velocity_x=forward_speed, velocity_y=0, velocity_z=0, duration=0.1)
                print(f"Drone moving forward at {forward_speed} m/s")

            # 保持主循环运行
            loop_rate.sleep()

    except rospy.ROSInterruptException:
        print("ROS shutdown")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # 停止无人机运动
        send_ned_velocity(vehicle, velocity_x=0, velocity_y=0, velocity_z=0, duration=1)
        print("Drone stopped.")

        # 停止相机订阅
        if camera_subscriber:
            camera_subscriber.unregister()
            print("Camera subscriber stopped.")

        # 清理资源并结束程序
        if vehicle:
            vehicle.mode = VehicleMode("LAND")  # 降落
            print("Landing vehicle.")

        print("Fly to a53 complete.")

def fly_to_b00():
    global camera_subscriber, pause_movement, exit_flag

    print("Flying logic for position 200")

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
    camera_subscriber = rospy.Subscriber(camera_topic, Image, image_callback_b00)
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

        print("Fly to b00 complete.")

def fly_to_b01():
    global camera_subscriber, pause_movement, exit_flag

    print("Flying logic for position 201")

    target_altitude = 0.5# 起飞目标高度（米）
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
    camera_subscriber = rospy.Subscriber(camera_topic, Image, image_callback_b01)
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

        print("Fly to b01 complete.")

def fly_to_b02():
    global camera_subscriber, pause_movement, exit_flag

    print("Flying logic for position 202")

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
    camera_subscriber = rospy.Subscriber(camera_topic, Image, image_callback_b02)
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

        print("Fly to b02 complete.")

def fly_to_b03():
    global camera_subscriber, pause_movement, exit_flag

    print("Flying logic for position 203")

    target_altitude = 0.5 # 起飞目标高度（米）
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
    camera_subscriber = rospy.Subscriber(camera_topic, Image, image_callback_b03)
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

        print("Fly to b03 complete.")

def fly_to_c00():
    global camera_subscriber, pause_movement, exit_flag

    print("Flying logic for position 300")

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
    camera_subscriber = rospy.Subscriber(camera_topic, Image, image_callback_c00)
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

        print("Fly to c00 complete.")

def fly_to_c01():
    global camera_subscriber, pause_movement, exit_flag

    print("Flying logic for position 301")

    target_altitude = 0.5 # 起飞目标高度（米）
    forward_speed = 0.1  # 前进速度（米/秒）

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
    camera_subscriber = rospy.Subscriber(camera_topic, Image, image_callback_c01)
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

        print("Fly to c01 complete.")

def fly_to_c02():
    global camera_subscriber, pause_movement, exit_flag

    print("Flying logic for position 302")

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
    camera_subscriber = rospy.Subscriber(camera_topic, Image, image_callback_c02)
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

        print("Fly to c02 complete.")

def fly_to_c04():
    global camera_subscriber, pause_movement, exit_flag

    print("Flying logic for position 304")

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
    camera_subscriber = rospy.Subscriber(camera_topic, Image, image_callback_c04)
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

        print("Fly to c04 complete.")

def fly_to_d00():

    global camera_subscriber, start_time
    start_time = time.time()
    arm_and_takeoff(0.5)
    camera_subscriber = rospy.Subscriber('/camera/image_raw', Image, image_callback_d00)
    rospy.spin()






flight_routes = {
    'a03': fly_to_a03,
    'a13': fly_to_a13,
    'a23': fly_to_a23,
    'a33': fly_to_a33,
    'a43': fly_to_a43,
    'a53': fly_to_a53,
    'b00': fly_to_b00,
    'b01': fly_to_b01,
    'b02': fly_to_b02,
    'b03': fly_to_b03,
    'c00': fly_to_c00,
    'c01': fly_to_c01,
    'c02': fly_to_c02,
    'c04': fly_to_c04,
    'd00': fly_to_d00,
}


def main():
    global vehicle

    # 提示用户输入新的三位编码（如a03, b02, d00）
    print("Available positions:", list(flight_routes.keys()))

    while True:
        target_position = input("Enter the target position (e.g., a03, b03, d00): ").strip()
        if target_position in flight_routes:
            break
        print("Invalid input. Please enter a valid position code from the list above.")

    print(f"Flying to target position {target_position}")
    flight_routes[target_position]()



if __name__ == "__main__":
    main()

