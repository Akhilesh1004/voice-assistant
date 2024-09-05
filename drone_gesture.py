import cv2
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import mediapipe as mp
import math
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
    start = time.time()
    send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, False,
                             thrust)
    time.sleep(0.1)
    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(0, 0,
                         0, 0, True,
                         thrust)
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


def hand_angle(hand_):
    angle_list = []
    # ---------------------------- thumb 大拇指角度
    angle_ = vector_2d_angle(
        ((int(hand_[0][0]) - int(hand_[2][0])), (int(hand_[0][1]) - int(hand_[2][1]))),
        ((int(hand_[3][0]) - int(hand_[4][0])), (int(hand_[3][1]) - int(hand_[4][1])))
    )
    angle_list.append(angle_)
    # ---------------------------- index 食指角度
    angle_ = vector_2d_angle(
        ((int(hand_[0][0]) - int(hand_[6][0])), (int(hand_[0][1]) - int(hand_[6][1]))),
        ((int(hand_[7][0]) - int(hand_[8][0])), (int(hand_[7][1]) - int(hand_[8][1])))
    )
    angle_list.append(angle_)
    # ---------------------------- middle 中指角度
    angle_ = vector_2d_angle(
        ((int(hand_[0][0]) - int(hand_[10][0])), (int(hand_[0][1]) - int(hand_[10][1]))),
        ((int(hand_[11][0]) - int(hand_[12][0])), (int(hand_[11][1]) - int(hand_[12][1])))
    )
    angle_list.append(angle_)
    # ---------------------------- ring 無名指角度
    angle_ = vector_2d_angle(
        ((int(hand_[0][0]) - int(hand_[14][0])), (int(hand_[0][1]) - int(hand_[14][1]))),
        ((int(hand_[15][0]) - int(hand_[16][0])), (int(hand_[15][1]) - int(hand_[16][1])))
    )
    angle_list.append(angle_)
    # ---------------------------- pink 小拇指角度
    angle_ = vector_2d_angle(
        ((int(hand_[0][0]) - int(hand_[18][0])), (int(hand_[0][1]) - int(hand_[18][1]))),
        ((int(hand_[19][0]) - int(hand_[20][0])), (int(hand_[19][1]) - int(hand_[20][1])))
    )
    angle_list.append(angle_)
    # ---------------------------- 食指上下方向
    angle_ = vector_2d_angle(
        ((int(hand_[23][0]) - int(hand_[21][0])), (int(hand_[23][1]) - int(hand_[21][1]))),  #→
        ((int(hand_[7][0]) - int(hand_[8][0])), (int(hand_[7][1]) - int(hand_[8][1])))
    )
    angle_list.append(angle_)
    # ---------------------------- 食指左右方向
    angle_ = vector_2d_angle(
        ((int(hand_[22][0]) - int(hand_[21][0])), (int(hand_[22][1]) - int(hand_[21][1]))),  #→
        ((int(hand_[7][0]) - int(hand_[8][0])), (int(hand_[7][1]) - int(hand_[8][1])))
    )
    angle_list.append(angle_)
    print(angle_list)
    return angle_list


def hand_gesture(angle_list):  # 偵測手勢
    if 100000. not in angle_list:
        if ((10 < angle_list[0] < 30) or (angle_list[0] > 40)) and (angle_list[1] < 40) and (angle_list[2] > 100) and (
                angle_list[3] > 100) and (angle_list[4] > 40) and (angle_list[5] < 30):
            set_attitude(thrust = 0.6)
            print("up")

        if ((10 < angle_list[0] < 30) or (angle_list[0] > 40)) and (angle_list[1] < 40) and (angle_list[2] > 100) and (
                angle_list[3] > 100) and (angle_list[4] > 40) and (angle_list[5] > 150):
            set_attitude(thrust = 0.3)
            print("down")

        if ((10 < angle_list[0] < 30) or (angle_list[0] > 40)) and (angle_list[1] < 40) and (angle_list[2] > 100) and (
                angle_list[3] > 100) and (angle_list[4] > 40) and (angle_list[6] < 30):
            set_attitude(roll_angle = -10, thrust = 0.5)
            print("left")

        if ((10 < angle_list[0] < 30) or (angle_list[0] > 40)) and (angle_list[1] < 40) and (angle_list[2] > 100) and (
                angle_list[3] > 100) and (angle_list[4] > 40) and (angle_list[6] > 150):
            set_attitude(roll_angle = 10, thrust = 0.5)
            print("right")

        if (150 < angle_list[1] ) and (150 < angle_list[2] ) and (140 < angle_list[3] ) and (135 < angle_list[4] ) and (
                angle_list[6] < 135):
            print("stable")

        if ((10<angle_list[0]<30) or (angle_list[0]>40)) and (angle_list[1]<40) and (angle_list[2]<15) and (      #二五切換
                angle_list[3]>100) and (angle_list[4]>40) :
            print("two")

        if (0 < angle_list[0] < 35) and (0 < angle_list[1] < 30) and (0 < angle_list[2] < 30) and (0 <
                angle_list[3] < 15) and (0 < angle_list[4] < 30):
            print("five")




def detect():
    mp_drawing = mp.solutions.drawing_utils
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        min_detection_confidence=0.75,
        min_tracking_confidence=0.75)
    # 開啟視訊鏡頭讀取器
    cap = cv2.VideoCapture(0)
    while True:
        # 偵測影像中的手部
        _, frame = cap.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = cv2.flip(frame, 1)
        results = hands.process(frame)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                keypoint_pos = []
                for i in range(21):
                    x = hand_landmarks.landmark[i].x * frame.shape[1]
                    y = hand_landmarks.landmark[i].y * frame.shape[0]
                    keypoint_pos.append((x, y))
                keypoint_pos.append((0, 0))
                keypoint_pos.append((1, 0))   #→
                keypoint_pos.append((0, 1))   #↓
                if keypoint_pos:
                    # 得到各手指的夾角資訊
                    angle_list = hand_angle(keypoint_pos)
                    # 根據角度判斷此手勢是否為愛心
                    hand_gesture(angle_list)

        cv2.imshow('MediaPipe Hands', frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break
    cap.release()


if __name__ == '__main__':
    detect()
