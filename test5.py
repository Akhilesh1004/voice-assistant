print("Start simulator (SITL)")
#import dronekit_sitl
#sitl = dronekit_sitl.start_default()
#connection_string = sitl.connection_string()
connection_string =  '/dev/cu.usbserial-D30AFE2G'
#connection_string =  "127.0.0.1:14551"
# Import DroneKit-Python
from dronekit import connect, VehicleMode
import time

# Connect to the Vehicle.
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True, baud = 57600)
print("vehicle.is_armable:", vehicle.is_armable)
# Get some vehicle attributes (state)
print("Get some vehicle attribute values:")
print(" GPS: %s" % vehicle.gps_0)
print(" Battery: %s" % vehicle.battery)
print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
print(" Is Armable?: %s" % vehicle.is_armable)
print(" System status: %s" % vehicle.system_status.state)
print(" Mode: %s" % vehicle.mode.name)    # settable
def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    print(vehicle.mode.name)
    while vehicle.mode.name != "GUIDED":
        print(vehicle.mode.name)
        vehicle.mode    = VehicleMode("GUIDED")
        time.sleep(1)

    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    vehicle.armed   = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)


    try:
        print("Taking off!")
        vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
        # Check that vehicle has reached takeoff altitude
    except:
        print("ERROR")
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt )
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print("Reached target altitude")
            break
        if vehicle.location.global_relative_frame.alt<0:
            print("error")
            break
        time.sleep(1)

# Initialize the takeoff sequence to 20m
arm_and_takeoff(2.5)

print("Take off complete")

# Hover for 10 seconds
time.sleep(3)

print("Now let's land")
vehicle.mode = VehicleMode("LAND")
while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt )
    #Break and return from function just below target altitude.
    if vehicle.location.global_relative_frame.alt<=0:
        print("LAND FINISH")
        break
    time.sleep(1)

#vehicle.mode    = VehicleMode("GUIDED")

# Close vehicle object before exiting script
vehicle.close()

# Shut down simulator
#sitl.stop()
print("Completed")
