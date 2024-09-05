# Import DroneKit-Python
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math
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



################################################################################################
# Settings
################################################################################################

connection_string       = '127.0.0.1:14540'
MAV_MODE_AUTO   = 4
# https://github.com/PX4/PX4-Autopilot/blob/master/Tools/mavlink_px4.py


# Parse connection argument
parser = argparse.ArgumentParser()
parser.add_argument("-c", "--connect", help="connection string")
args = parser.parse_args()

if args.connect:
    connection_string = args.connect


################################################################################################
# Init
################################################################################################

# Connect to the Vehicle
print("Connecting")
vehicle = connect(connection_string, wait_ready=True)


def PX4setMode(mavMode):
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                               mavMode,
                                               0, 0, 0, 0, 0, 0)



def get_location_offset_meters(original_location, dNorth, dEast, alt):



    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt+alt)





################################################################################################
# Listeners
################################################################################################

home_position_set = False

#Create a message listener for home position fix
@vehicle.on_message('HOME_POSITION')
def listener(self, name, home_position):
    global home_position_set
    home_position_set = True



################################################################################################
# Start mission example
################################################################################################

# wait for a home position lock
while not home_position_set:
    print("Waiting for home position...")
    time.sleep(1)

# Display basic vehicle state
print(" Type: %s" % vehicle._vehicle_type)
print(" Armed: %s" % vehicle.armed)
print(" System status: %s" % vehicle.system_status.state)
print(" GPS: %s" % vehicle.gps_0)
print(" Alt: %s" % vehicle.location.global_relative_frame.alt)

# Change to AUTO mode
PX4setMode(MAV_MODE_AUTO)
time.sleep(1)
cmds = vehicle.commands
home = vehicle.location.global_relative_frame

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        min_detection_confidence=0.75,
        min_tracking_confidence=0.75)
# 開啟視訊鏡頭讀取器
cap = cv2.VideoCapture(0)
h = 0
v = 5
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
                    cmds.clear()
                    # takeoff to 10 meters
                    wp = get_location_offset_meters(home, 0, 0, v);
                    cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
                    cmds.add(cmd)
                    # Upload mission
                    cmds.upload()
                    time.sleep(2)
                    vehicle.armed = True
                elif x== "five":
                    # land
                    wp = get_location_offset_meters(home, 0, 0, v);
                    cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
                    cmds.add(cmd)
                    # Upload mission
                    cmds.upload()
                    time.sleep(0.2)
                    # wait for the vehicle to land
                    while vehicle.commands.next > 0:
                        time.sleep(1)
                    # Disarm vehicle
                    vehicle.armed = False
                    time.sleep(1)
                elif x == "right":
                    h+=0.1
                    cmds.clear()
                    # move 10 meters east
                    wp = get_location_offset_meters(wp, 0, h, 0);
                    cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
                    cmds.add(cmd)
                    # Upload mission
                    cmds.upload()
                    time.sleep(0.2)
                elif x=="up":
                    v+=0.1
                    cmds.clear()
                    # move 10 meters east
                    wp = get_location_offset_meters(wp, 0, 0, v);
                    cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
                    cmds.add(cmd)
                    # Upload mission
                    cmds.upload()
                    time.sleep(0.2)
                elif x== "down":
                    v-=0.1
                    if v<=0:
                        v=0
                    cmds.clear()
                    # move 10 meters east
                    wp = get_location_offset_meters(wp, 0, 0, v);
                    cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
                    cmds.add(cmd)
                    # Upload mission
                    cmds.upload()
                    time.sleep(0.2)
                elif x == "left":
                    h-=0.1
                    cmds.clear()
                    # move 10 meters east
                    wp = get_location_offset_meters(wp, 0, h, 0);
                    cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
                    cmds.add(cmd)
                    # Upload mission
                    cmds.upload()
                    time.sleep(0.2)

    cv2.imshow('MediaPipe Hands', frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break
cap.release()


