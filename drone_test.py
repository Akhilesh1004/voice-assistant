import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import math

#- Importing Tkinter: sudo apt-get install python-tk
#import tk
import tkinter as tk


#-- Connect to the vehicle
print('Connecting...')
vehicle = connect('COM3',baud=57600)
#-- Setup the commanded flying speed
gnd_speed = 1 # [m/s]

#-- Define arm and takeoff
def arm_and_takeoff(altitude):

   while not vehicle.is_armable:
      print("waiting to be armable")
      time.sleep(1)

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print("Taking Off")
   vehicle.simple_takeoff(altitude)

   while True:
      v_alt = vehicle.location.global_relative_frame.alt
      print(">> Altitude = %.1f m"%v_alt)
      if v_alt >= altitude - 1.0:
          print("Target altitude reached")
          break
      time.sleep(1)

 #-- Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html

    Bitmask to indicate which dimensions should be ignored by the vehicle
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that
    none of the setpoint dimensions should be ignored). Mapping:
    bit 1: x,  bit 2: y,  bit 3: z,
    bit 4: vx, bit 5: vy, bit 6: vz,
    bit 7: ax, bit 8: ay, bit 9:
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def send_ned_position(vehicle, position_x, position_y, position_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # 時間戳(未使用)
        0,  # 目標系統ID(自動填充)
        0,  # 目標組件填充(一般為0，向所有組件發送)
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # 參考系
        0b110111111000,   # 使能速度控制或位置控制的bitmask，此處使能控制位置控制
        position_x, position_y, position_z,  # x, y, z軸位置
        0, 0, 0,  # x, y, z軸速度
        0, 0, 0,  # x, y, z加速度(ArduCopter固件暫不支持)
        0, 0)  # 目標航向角(yaw軸角度)，yaw軸加速度(ArduCopter固件暂不支持)

    vehicle.send_mavlink(msg)
    vehicle.flush()

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
            send_ned_position(vehicle, 0.5, 0, 0)
            #set_attitude(pitch_angle = -5, thrust = 0.5)
        elif event.keysym == 's':
            #set_velocity_body(vehicle,-gnd_speed, 0, 0)
            send_ned_position(vehicle, -0.5, 0, 0)
            #set_attitude(pitch_angle = 5, thrust = 0.5)
        elif event.keysym == 'a':
            #set_velocity_body(vehicle, 0, -gnd_speed, 0)
            send_ned_position(vehicle, 0, -0.5, 0)
            #set_attitude(roll_angle = -5, thrust = 0.5)
        elif event.keysym == 'd':
            #set_velocity_body(vehicle, 0, gnd_speed, 0)
            #send_ned_position(vehicle, 0, 0.5, 0)
            #set_attitude(roll_angle = 5, thrust = 0.5)
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

#- Read the keyboard with tkinter
root = tk.Tk()
print(">> Control the drone with the arrow keys. Press r for RTL mode")
root.bind_all('<Key>', key)
root.mainloop()
