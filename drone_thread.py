import cv2
import mediapipe as mp
import math
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

#- Importing Tkinter: sudo apt-get install python-tk
#import tk
import tkinter as tk

#-- Connect to the vehicle
print('Connecting...')
vehicle = connect('COM3',baud=57600)
#-- Setup the commanded flying speed
gnd_speed = 1 # [m/s]

def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,
                         yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                         thrust = 0.5):
    """
    use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                  When one is used, the other is ignored by Ardupilot.
    thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
            Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
            the code for maintaining current altitude.
    """
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        1, # Target system
        1, # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
        0, # Body roll rate in radian
        0, # Body pitch rate in radian
        math.radians(yaw_rate), # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def set_attitude(roll_angle = 0.0, pitch_angle = 0.0,
                 yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                 thrust = 0.5):
    """
    Note that from AC3.3 the message should be re-sent more often than every
    second, as an ATTITUDE_TARGET order has a timeout of 1s.
    In AC3.2.1 and earlier the specified attitude persists until it is canceled.
    The code below should work on either version.
    Sending the message multiple times is the recommended way.
    """
    send_attitude_target(roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)

    time.sleep(0.1)
    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(0, 0,
                         0, 0, True,
                         0.5)
    time.sleep(0.1)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]


#-- Key event function
def key(event):
    if event.char == event.keysym: #-- standard keys
        if event.keysym == 'r':
            print("r pressed >> Set the vehicle to RTL")
            vehicle.mode = VehicleMode("RTL")
        elif event.keysym == 'o':
            vehicle.mode = VehicleMode("GUIDED_NOGPS")
            vehicle.armed = True
        elif event.keysym == 'p':
            vehicle.armed = False
        elif event.keysym == 'o':
            vehicle.mode = VehicleMode("GUIDED_NOGPS")
        elif event.keysym == 'w':
            #set_velocity_body(vehicle, gnd_speed, 0, 0)
            #send_ned_position(vehicle, 0.5, 0, 0)
            set_attitude(pitch_angle = -5, thrust = 0.5)
        elif event.keysym == 's':
            #set_velocity_body(vehicle,-gnd_speed, 0, 0)
            #send_ned_position(vehicle, -0.5, 0, 0)
            set_attitude(pitch_angle = 5, thrust = 0.5)
        elif event.keysym == 'a':
            #set_velocity_body(vehicle, 0, -gnd_speed, 0)
            #send_ned_position(vehicle, 0, -0.5, 0)
            set_attitude(roll_angle = -5, thrust = 0.5)
        elif event.keysym == 'd':
            #set_velocity_body(vehicle, 0, gnd_speed, 0)
            #send_ned_position(vehicle, 0, 0.5, 0)
            set_attitude(roll_angle = 5, thrust = 0.5)
        elif event.keysym == 'q':
            #set_velocity_body(vehicle, 0, -gnd_speed, 0)
            #send_ned_position(vehicle, 0, -0.5, 0)
            set_attitude(yaw_angle = -5, thrust = 0.5)
        elif event.keysym == 'e':
            #set_velocity_body(vehicle, 0, gnd_speed, 0)
            #send_ned_position(vehicle, 0, 0.5, 0)
            set_attitude(yaw_angle = 5, thrust = 0.5)

    else: #-- non standard keys
        if event.keysym == 'Up':
            print("111")
            #set_velocity_body(vehicle, 0, 0, -gnd_speed)
            #send_ned_position(vehicle, 0, 0, -0.5)
            set_attitude(thrust = 0.6)
        elif event.keysym == 'Down':
            #set_velocity_body(vehicle, 0, 0, gnd_speed)
            #send_ned_position(vehicle, 0, 0, 0.5)
            set_attitude(thrust = 0.2)
            print("222")


#---- MAIN FUNCTION
#- Takeoff
#arm_and_takeoff(10)
def keyboard():
    #- Read the keyboard with tkinter
    root = tk.Tk()
    print(">> Control the drone with the arrow keys. Press r for RTL mode")
    root.bind_all('<Key>', key)
    root.mainloop()

def vector_2d_angle(v1, v2):  # 求出v1,v2兩條向量的夾角
    v1_x = v1[0]
    v1_y = v1[1]
    v2_x = v2[0]
    v2_y = v2[1]
    try:
        angle_ = math.degrees(math.acos(
            (v1_x * v2_x + v1_y * v2_y) / (((v1_x ** 2 + v1_y ** 2) ** 0.5) * ((v2_x ** 2 + v2_y ** 2) ** 0.5))))
    except:
        angle_ = 100000.
    return angle_


def pose_angle(pose_):
    angle_list = []
    # ---------------------------- 右上臂角度
    angle_ = vector_2d_angle(
        ((int(pose_[11][0]) - int(pose_[12][0])), (int(pose_[11][1]) - int(pose_[12][1]))),
        ((int(pose_[13][0]) - int(pose_[11][0])), (int(pose_[13][1]) - int(pose_[11][1])))
    )
    angle_list.append(angle_)
    # ---------------------------- 左上臂角度
    angle_ = vector_2d_angle(
        ((int(pose_[12][0]) - int(pose_[11][0])), (int(pose_[12][1]) - int(pose_[11][1]))),
        ((int(pose_[14][0]) - int(pose_[12][0])), (int(pose_[14][1]) - int(pose_[12][1])))
    )
    angle_list.append(angle_)
    # ---------------------------- 右下臂角度
    angle_ = vector_2d_angle(
        ((int(pose_[11][0]) - int(pose_[12][0])), (int(pose_[11][1]) - int(pose_[12][1]))),
        ((int(pose_[15][0]) - int(pose_[13][0])), (int(pose_[15][1]) - int(pose_[13][1])))
    )
    angle_list.append(angle_)
    # ---------------------------- 左下臂角度
    angle_ = vector_2d_angle(
        ((int(pose_[12][0]) - int(pose_[11][0])), (int(pose_[12][1]) - int(pose_[11][1]))),
        ((int(pose_[16][0]) - int(pose_[14][0])), (int(pose_[16][1]) - int(pose_[14][1])))
    )
    angle_list.append(angle_)
    # ---------------------------- 右下臂方向
    angle_ = vector_2d_angle(
        ((int(pose_[35][0]) - int(pose_[33][0])), (int(pose_[35][1]) - int(pose_[33][1]))),
        ((int(pose_[15][0]) - int(pose_[13][0])), (int(pose_[15][1]) - int(pose_[13][1])))
    )
    angle_list.append(angle_)
    # ---------------------------- 左下臂方向
    angle_ = vector_2d_angle(
        ((int(pose_[35][0]) - int(pose_[33][0])), (int(pose_[35][1]) - int(pose_[33][1]))),
        ((int(pose_[16][0]) - int(pose_[14][0])), (int(pose_[16][1]) - int(pose_[14][1])))
    )
    angle_list.append(angle_)

    print(angle_list)
    return angle_list


def pose_gesture(angle_list):  # 偵測手勢(以操作者方向為主)
    if 100000. not in angle_list:
        if (angle_list[0] < 25) and (angle_list[1] < 25) and (70 < angle_list[2] < 110) and (
                70 < angle_list[3] < 110) and (160 < angle_list[4]) and (160 < angle_list[5]):
            print("take off")  # 雙手半舉

        elif (70 < angle_list[0]) and (70 < angle_list[1]) and (80 < angle_list[2]) and (
                80 < angle_list[3]) and (160 < angle_list[4]) and (160 < angle_list[5]):
            print("up")  # 雙手全舉

        elif (angle_list[0] < 25) and (angle_list[1] < 25) and (angle_list[2] < 20) and (
                angle_list[3] < 20):
            print("down")  # 雙手平舉

        elif (70 < angle_list[0]) and (angle_list[1] < 35) and (80 < angle_list[2]) and (
                angle_list[3] < 20) and (angle_list[4] < 35):
            print("left")  # 左手平舉

        elif (angle_list[0] < 35) and (70 < angle_list[1]) and (angle_list[2] < 20) and (
                80 < angle_list[3]) and (angle_list[5] < 35):
            print("right")  # 右手平舉

        elif (70 < angle_list[0]) and (60 < angle_list[1]) and (80 < angle_list[2]) and (
                65 < angle_list[3]) and (160 < angle_list[4]) and (angle_list[5] < 20):
            print("forward")  # 右手全舉

        elif (70 < angle_list[0]) and (60 < angle_list[1]) and (65 < angle_list[2]) and (
                80 < angle_list[3]) and (angle_list[4] < 20) and (160 < angle_list[5]):
            print("backward")  # 左手全舉

        elif (angle_list[0] < 25) and (angle_list[1] < 35) and (70 < angle_list[2] < 110) and (
                angle_list[3] < 20) and (160 < angle_list[4]):
            print("clockwise")  # 左手平舉、右手半舉

        elif (angle_list[0] < 35) and (angle_list[1] < 25) and (angle_list[2] < 20) and (
                70 < angle_list[3] < 110) and (160 < angle_list[5]):
            print("counterclockwise")  # 右手平舉、左手半舉


def detect():
    mp_drawing = mp.solutions.drawing_utils
    mp_pose = mp.solutions.pose
    pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

    # 開啟視訊鏡頭讀取器
    cap = cv2.VideoCapture(0)
    while True:
        # 偵測影像中的手部
        _, frame = cap.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = cv2.flip(frame, 1)
        results = pose.process(frame)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        keypoint_pos = ['' for i in range(33)]
        if results.pose_landmarks:
            mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
            for i in range(33):
                cx = int(results.pose_landmarks.landmark[i].x * frame.shape[1])
                cy = int(results.pose_landmarks.landmark[i].y * frame.shape[0])
                keypoint_pos[i] = (cx, cy)
            keypoint_pos.append((0, 0))
            keypoint_pos.append((1, 0))  # →
            keypoint_pos.append((0, 1))  # ↓
            # print(keypoint_pos)
            if keypoint_pos:
                # 得到各手指的夾角資訊
                angle_list = pose_angle(keypoint_pos)
                # 根據角度判斷此手勢是否為愛心
                pose_gesture(angle_list)

        cv2.imshow('MediaPipe Pose', frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break
    cap.release()


if __name__ == '__main__':
    detect()
