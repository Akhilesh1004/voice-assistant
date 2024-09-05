import asyncio
from mavsdk import System
import cv2
import mediapipe as mp
import math

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
            print("up")
            return "up"

        if ((10 < angle_list[0] < 30) or (angle_list[0] > 40)) and (angle_list[1] < 40) and (angle_list[2] > 100) and (
                angle_list[3] > 100) and (angle_list[4] > 40) and (angle_list[5] > 150):
            print("down")
            return "down"

        if ((10 < angle_list[0] < 30) or (angle_list[0] > 40)) and (angle_list[1] < 40) and (angle_list[2] > 100) and (
                angle_list[3] > 100) and (angle_list[4] > 40) and (angle_list[6] < 30):
            print("left")
            return "left"

        if ((10 < angle_list[0] < 30) or (angle_list[0] > 40)) and (angle_list[1] < 40) and (angle_list[2] > 100) and (
                angle_list[3] > 100) and (angle_list[4] > 40) and (angle_list[6] > 150):
            print("right")
            return "right"

        if (150 < angle_list[1] ) and (150 < angle_list[2] ) and (140 < angle_list[3] ) and (135 < angle_list[4] ) and (
                angle_list[6] < 135):
            print("stable")
            return "stable"

        if ((10<angle_list[0]<30) or (angle_list[0]>40)) and (angle_list[1]<40) and (angle_list[2]<15) and (      #二五切換
                angle_list[3]>100) and (angle_list[4]>40) :
            print("two")
            return "two"

        if (0 < angle_list[0] < 35) and (0 < angle_list[1] < 30) and (0 < angle_list[2] < 30) and (0 <
                angle_list[3] < 15) and (0 < angle_list[4] < 30):
            print("five")
            return "five"


async def drone_fly():

    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break
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
                    x = hand_gesture(angle_list)
                    if x == "two":
                        print("-- Arming")
                        await drone.action.arm()

                        print("-- Taking off")
                        await drone.action.takeoff()

                        await asyncio.sleep(10)
                        await drone.manual_control.set_manual_control_input(
                        float(0), float(0), float(0.5), float(0)
                    )
                        await drone.manual_control.start_position_control()
                    elif x== "five":
                        print("-- Landing")
                        await drone.action.land()
                    elif x == "right":
                        #await drone.manual_control.start_position_control()
                        # get current state of roll axis (between -1 and 1)
                        roll = float(0)
                        # get current state of pitch axis (between -1 and 1)
                        pitch = float(0.2)
                        # get current state of throttle axis (between -1 and 1, but between 0 and 1 is expected)
                        throttle = float(0.5)
                        # get current state of yaw axis (between -1 and 1)
                        yaw = float(0)
                        await drone.manual_control.set_manual_control_input(roll, pitch, throttle, yaw)
                        await asyncio.sleep(0.1)

                    elif x=="up":
                        #await drone.manual_control.start_position_control()
                        # get current state of roll axis (between -1 and 1)
                        roll = float(0)
                        # get current state of pitch axis (between -1 and 1)
                        pitch = float(0)
                        # get current state of throttle axis (between -1 and 1, but between 0 and 1 is expected)
                        throttle = float(0.6)
                        # get current state of yaw axis (between -1 and 1)
                        yaw = float(0)
                        await drone.manual_control.set_manual_control_input(roll, pitch, throttle, yaw)
                        await asyncio.sleep(0.1)

                    elif x== "down":
                        #await drone.manual_control.start_position_control()
                        # get current state of roll axis (between -1 and 1)
                        roll = float(0)
                        # get current state of pitch axis (between -1 and 1)
                        pitch = float(0)
                        # get current state of throttle axis (between -1 and 1, but between 0 and 1 is expected)
                        throttle = float(0.45)
                        # get current state of yaw axis (between -1 and 1)
                        yaw = float(0)
                        await drone.manual_control.set_manual_control_input(roll, pitch, throttle, yaw)
                        await asyncio.sleep(0.1)

                    elif x == "left":
                        #await drone.manual_control.start_position_control()
                        # get current state of roll axis (between -1 and 1)
                        roll = float(0)
                        # get current state of pitch axis (between -1 and 1)
                        pitch = float(-0.2)
                        # get current state of throttle axis (between -1 and 1, but between 0 and 1 is expected)
                        throttle = float(0.5)
                        # get current state of yaw axis (between -1 and 1)
                        yaw = float(0)
                        await drone.manual_control.set_manual_control_input(roll, pitch, throttle, yaw)
                        await asyncio.sleep(0.1)
                    else:
                        #await drone.manual_control.start_position_control()
                        # get current state of roll axis (between -1 and 1)
                        roll = float(0)
                        # get current state of pitch axis (between -1 and 1)
                        pitch = float(0)
                        # get current state of throttle axis (between -1 and 1, but between 0 and 1 is expected)
                        throttle = float(0.5)
                        # get current state of yaw axis (between -1 and 1)
                        yaw = float(0)
                        await drone.manual_control.set_manual_control_input(roll, pitch, throttle, yaw)
                        await asyncio.sleep(0.1)

        cv2.imshow('MediaPipe Hands', frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break
    cap.release()


loop = asyncio.get_event_loop()
loop.run_until_complete(drone_fly())

