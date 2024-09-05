'''
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math


#connection_string = '127.0.0.1:14551'

# Connect to the Vehicle
#print('Connecting to vehicle on: %s' % connection_string)
#vehicle = connect(connection_string, wait_ready=True)
vehicle = connect('/dev/cu.usbserial-D30AFE2G', wait_ready=True, baud = 57600)

print('Connect Successfully!')
print("Basic pre-arm checks")
while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)

vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
time.sleep(3)

print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
'''

from threading import Thread
import time

class test(Thread):
    def __init__(self, num):
        Thread.__init__(self)
        self.x = num
    def yyy(self):
        time.sleep(1)
        print(self.x)

def detect():
    print("uuu")
    thread1 = test(123)
    thread2 = test(321)
    thread1.start()
    thread2.start()
    while True:
        thread1.yyy()
        thread2.yyy()
def main():
    t = Thread(detect())
    t.start()
if __name__ == '__main__':
    main()
