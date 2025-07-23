import cv2
from cv_bridge import CvBridge
import time
import math
import numpy as np
import rospy
from dronekit import connect, VehicleMode
from sensor_msgs.msg import Image
from pymavlink import mavutil
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

print("Connecting to the vehicle...")
connection_string = 'tcp:127.0.0.1:5762'
vehicle = connect(connection_string, wait_ready=True, baud=57600)
print("Connected to IRIS")

FOCAL_LENGTH_PX = 320
bridge = CvBridge()
image_counter = 0
current_pose = None
current_twist = None
current_pitch = None
current_roll = None
current_yaw = None
camera_subscriber = None
initial_red_detected = False
left_start_point = False
has_completed_circle = False
last_turn_direction = None
start_time_after_left_point = None
last_forward_time = 0
start_time = None
red_detect_counter = 0  # 新增：统计红色标志连续观测次数
red_centered_counter = 0  # 新增：统计红点在图像中心的连续帧数


def arm_and_takeoff(target_altitude):
    global start_time
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
        if altitude >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
    print("Hovering for 3 seconds before starting logging...")
    time.sleep(3)
    start_time = time.time()
    print("Start time recorded. Beginning data logging from 0s.")


def condition_yaw(vehicle, heading, relative=False):
    is_relative = 1 if relative else 0
    direction = 1 if heading > 0 else -1
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        abs(heading),
        0,
        direction,
        is_relative,
        0, 0, 0)
    vehicle.send_mavlink(msg)


def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration, frequency=10):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        velocity_x, velocity_y, velocity_z,
        0, 0, 0,
        0, 0)
    interval = 1.0 / frequency
    start_time = time.time()
    while time.time() - start_time < duration:
        vehicle.send_mavlink(msg)
        time.sleep(interval)


def hover(vehicle, hover_time):
    print(f"Hovering for {hover_time} seconds.")
    send_ned_velocity(vehicle, 0, 0, 0, hover_time)
    time.sleep(hover_time)
    print("Hover complete.")


def land():
    global camera_subscriber
    print("Landing")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        time.sleep(1)
    print("Vehicle disarmed.")
    if camera_subscriber:
        camera_subscriber.unregister()
        print("Camera subscriber unregistered.")
    rospy.signal_shutdown("Landing complete.")


def calculate_yaw_angle(deviation, altitude_m, max_yaw_angle=None):
    altitude_m = max(altitude_m, 0.01)
    deviation_m = (deviation * altitude_m) / FOCAL_LENGTH_PX
    yaw_angle_rad = math.atan(deviation_m / altitude_m)
    yaw_angle_deg = math.degrees(yaw_angle_rad)
    return yaw_angle_deg


def model_states_callback(data):
    global current_pose, current_twist, current_pitch, current_roll, current_yaw
    drone_name = "iris"
    if drone_name in data.name:
        index = data.name.index(drone_name)
        current_pose = data.pose[index]
        current_twist = data.twist[index]
        quaternion = (
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w,
        )
        current_roll, current_pitch, current_yaw = euler_from_quaternion(quaternion)


rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)


def image_callback(msg):
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
        send_ned_velocity(vehicle, velocity_x=0.1, velocity_y=0, velocity_z=0, duration=5)
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

                if abs(actual_deviation) >= yaw_threshold / (2 * sensitivity):
                    print(
                        f"[Debug] Altitude: {altitude_m:.2f} m, Sensitivity: {sensitivity}, Deviation: {actual_deviation}")
                    print(f"[Yaw] Continuous correction: {actual_deviation}px, adjusting {yaw_adjustment:.1f}°")
                    condition_yaw(vehicle, yaw_adjustment, relative=True)

                if time.time() - last_forward_time > 0.5:
                    print(f"[Move] Moving forward with deviation: {actual_deviation}px.")
                    send_ned_velocity(vehicle, 0.1, 0, 0, 0.5)
                    last_forward_time = time.time()

            # 可视化红点与质心
            cv2.circle(small_image, (frame_center_x, small_image.shape[0] // 2), 8, (0, 0, 255), -1)
            cv2.circle(small_image, (cX, cY), 8, (255, 0, 0), -1)

    # 图像显示
    cv2.imshow("Tracking", small_image)
    cv2.waitKey(1)



def main():
    global camera_subscriber, start_time
    rospy.init_node('circle_path_follower')
    start_time = time.time()
    arm_and_takeoff(0.5)
    camera_subscriber = rospy.Subscriber('/camera/image_raw', Image, image_callback)
    rospy.spin()


if __name__ == "__main__":
    main()
