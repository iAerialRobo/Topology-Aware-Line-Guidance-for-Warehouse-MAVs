# Python标准库模块
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


def rotate_counterclockwise_90(vehicle):
    """逆时针旋转45度（带完整时间记录）"""
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


# 全局变量用于 PID 控制器
integral_error = 0
last_error = 0
# 添加积分限幅以避免积分饱和
MAX_INTEGRAL = 20  # 最大积分值限制
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
camera_subscriber = None
# 引入变量记录形状跟踪状态
tracking_active = False
tracking_state = None  # 跟踪的目标状态（'T_left' 或 'Cross'等等）
image_counter = 0
phase = 1


pid_phase_started = False  # 是否已进入PID阶段
pid_phase_completed = False  # PID是否已经完成
first_nonzero_code_seen = False
center_tracking_enabled = True
t_marker_phase_ready = False  # ✅ 新增全局变量
second_pid_phase_started = False
second_pid_phase_completed = False
yaw_rotation_completed = False
t_detection_round = 1  # 初始化从第 1 轮开始
yaw_threshold = 45 # 自定义阈值

# 初始化 ROS 节点
rospy.init_node('drone_data_logger', anonymous=True)

# 定义全局变量
current_pose = None
current_twist = None
current_pitch = None
current_roll = None
current_yaw = None


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
    global current_pose, current_twist, current_pitch, current_roll, current_yaw
    global pid_phase_started, pid_phase_completed, first_nonzero_code_seen, center_tracking_enabled
    global t_marker_phase_ready
    global second_pid_phase_started, second_pid_phase_completed
    global yaw_rotation_completed  # ✅ 新增
    global yaw_threshold_list



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
            # 👇 每轮动态获取 yaw_threshold
            current_yaw_threshold = yaw_threshold_list[min(t_left_count, len(yaw_threshold_list) - 1)]
            if not first_nonzero_code_seen and smoothed_shape_code != 0:
                first_nonzero_code_seen = True
                pid_phase_started = True
                print("[PID Phase] First non-zero shape code seen. Starting lateral PID adjustment.")

            # === 第一轮 PID（直到偏差足够小）===
            if pid_phase_started and not pid_phase_completed:
                Kp, Ki, Kd = 1.0, 0.02, 0.5
                pid_dead_zone = 0.05

                lateral_adjustment = pid_control_with_dead_zone(
                    normalized_deviation, Kp, Ki, Kd, dead_zone=pid_dead_zone
                )

                forward_speed = 0.1
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

            # === Yaw 控制阶段（持续检测直到旋转完成）===
            elif pid_phase_completed and not yaw_rotation_completed:
                altitude_m = current_pose.position.z
                sensitivity = round(altitude_m, 2)

                if yaw_threshold / (2 * sensitivity) <= abs(deviation):
                    print(
                        f"[Yaw] Deviation {abs(deviation)} within threshold ({yaw_threshold} / (2 * {sensitivity})). Rotating 45 degrees.")
                    rotate_counterclockwise_45(vehicle)
                    yaw_rotation_completed = True
                    second_pid_phase_started = True
                    print("[Yaw] Rotation complete. Starting 2nd PID phase.")
                else:
                    print(
                        f"[Yaw] Deviation {abs(deviation)} <= threshold {current_yaw_threshold}. Waiting to rotate...")

            # === 第二轮 PID（直到收敛）===
            elif second_pid_phase_started and not second_pid_phase_completed:
                Kp, Ki, Kd = 1.0, 0.02, 0.5
                pid_dead_zone = 0.05

                lateral_adjustment = pid_control_with_dead_zone(
                    normalized_deviation, Kp, Ki, Kd, dead_zone=pid_dead_zone
                )

                forward_speed = 0.1
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
        send_ned_velocity(vehicle, velocity_x=0.1, velocity_y=0, velocity_z=0, duration=0.1)
        print("Tracking active: Moving forward at 0.1 m/s (no PID).")

    # === T型转弯识别 ===
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



def image_callback_2(msg):
    global camera_subscriber, t_left_count, last_shape_code, pause_movement, exit_flag, image_counter
    global tracking_active, tracking_state
    global current_pose, current_twist, current_pitch, current_roll, current_yaw
    global pid_phase_started, pid_phase_completed, first_nonzero_code_seen, center_tracking_enabled
    global t_marker_phase_ready
    global second_pid_phase_started, second_pid_phase_completed
    global yaw_rotation_completed
    global t_detection_round  # ✅ 新增，标记当前第几轮检测
    global yaw_threshold_list



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
            # 👇 每轮动态获取 yaw_threshold
            current_yaw_threshold = yaw_threshold_list[min(t_left_count, len(yaw_threshold_list) - 1)]
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

                forward_speed = 0.1
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

                if yaw_threshold / (2 * sensitivity) <= abs(deviation):
                    print(
                        f"[Yaw] Deviation {abs(deviation)} within threshold ({yaw_threshold} / (2 * {sensitivity})). Rotating 45 degrees.")
                    rotate_counterclockwise_45(vehicle)
                    yaw_rotation_completed = True
                    second_pid_phase_started = True
                    print("[Yaw] Rotation complete. Starting 2nd PID phase.")
                else:
                    print(
                        f"[Yaw] Deviation {abs(deviation)} <= threshold {current_yaw_threshold}. Waiting to rotate...")


            # === 第二轮 PID ===
            elif second_pid_phase_started and not second_pid_phase_completed:
                Kp, Ki, Kd = 1.0, 0.02, 0.5
                pid_dead_zone = 0.05

                lateral_adjustment = pid_control_with_dead_zone(
                    normalized_deviation, Kp, Ki, Kd, dead_zone=pid_dead_zone
                )

                forward_speed = 0.1
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
        send_ned_velocity(vehicle, velocity_x=0.1, velocity_y=0, velocity_z=0, duration=0.1)
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


def image_callback_4(msg):
    global camera_subscriber, t_left_count, last_shape_code, pause_movement, exit_flag, image_counter
    global tracking_active, tracking_state
    global current_pose, current_twist, current_pitch, current_roll, current_yaw
    global pid_phase_started, pid_phase_completed, first_nonzero_code_seen, center_tracking_enabled
    global t_marker_phase_ready
    global second_pid_phase_started, second_pid_phase_completed
    global yaw_rotation_completed
    global yaw_threshold_list

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
            # 👇 每轮动态获取 yaw_threshold
            current_yaw_threshold = yaw_threshold_list[min(t_left_count, len(yaw_threshold_list) - 1)]
            if not first_nonzero_code_seen and smoothed_shape_code != 0:
                first_nonzero_code_seen = True
                pid_phase_started = True
                print(f"[T Count {t_left_count}] First non-zero shape code seen. Starting lateral PID adjustment.")

            # === 第一轮 PID ===
            if pid_phase_started and not pid_phase_completed:
                Kp, Ki, Kd = 1.0, 0.02, 0.5
                pid_dead_zone = 0.05

                lateral_adjustment = pid_control_with_dead_zone(
                    normalized_deviation, Kp, Ki, Kd, dead_zone=pid_dead_zone
                )

                forward_speed = 0.1
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

                if yaw_threshold / (2 * sensitivity) <= abs(deviation):
                    print(
                        f"[Yaw] Deviation {abs(deviation)} within threshold ({yaw_threshold} / (2 * {sensitivity})). Rotating 45 degrees.")
                    rotate_counterclockwise_45(vehicle)
                    yaw_rotation_completed = True
                    second_pid_phase_started = True
                    print("[Yaw] Rotation complete. Starting 2nd PID phase.")
                else:
                    print(
                        f"[Yaw] Deviation {abs(deviation)} <= threshold {current_yaw_threshold}. Waiting to rotate...")

            # === 第二轮 PID ===
            elif second_pid_phase_started and not second_pid_phase_completed:
                Kp, Ki, Kd = 1.0, 0.02, 0.5
                pid_dead_zone = 0.05

                lateral_adjustment = pid_control_with_dead_zone(
                    normalized_deviation, Kp, Ki, Kd, dead_zone=pid_dead_zone
                )

                forward_speed = 0.1
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
        send_ned_velocity(vehicle, velocity_x=0.1, velocity_y=0, velocity_z=0, duration=0.1)
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



# 位置字典（同样可以使用之前定义的positions字典）
positions = {
    0: 0,
    1: 1,
    2: 2,
    4: 4
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

def fly_to_4():
    global camera_subscriber, pause_movement, exit_flag

    print("Flying logic for position 4")

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
    camera_subscriber = rospy.Subscriber(camera_topic, Image, image_callback_4)
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
    '4': fly_to_4,
}

# 主程序
def main():
    global vehicle

    while True:
        target_position = input("Enter the target position (0, 1, 2, 4): ").strip()
        if target_position in flight_routes:
            break
        print("Invalid input. Please enter a valid position code (0, 1, 2, 4).")

    print(f"Flying to target position {target_position}")
    flight_routes[target_position]()

if __name__ == "__main__":
    main()

