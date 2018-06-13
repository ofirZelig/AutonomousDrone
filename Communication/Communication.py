

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative


import argparse


#move to main

# parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
# parser.add_argument('--connect',
#                     help="Vehicle connection target string. If not specified, SITL automatically started and used.")
# args = parser.parse_args()
#
# connection_string = args.connect
#
#
# # Start SITL if no connection string specified
# if not connection_string:
#     import dronekit_sitl
#     sitl = dronekit_sitl.start_default()
#     connection_string = sitl.connection_string()
#     #tcp:127.0.0.1:5760


"""
Functions to make it easy to convert between the different frames-of-reference. In particular these
make it easy to navigate in terms of "metres from the current position" when using commands that take 
absolute positions in decimal degrees.

The methods are approximations only, and may be less accurate over longer distances, and when close 
to the Earth's poles.

Specifically, it provides:
* get_location_metres - Get LocationGlobal (decimal degrees) at distance (m) North & East of a given LocationGlobal.
* get_distance_metres - Get the distance between two LocationGlobal objects in metres
* get_bearing - Get the bearing in degrees to a LocationGlobal
"""
import math

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation = LocationGlobal(newlat, newlon, original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation = LocationGlobalRelative(newlat, newlon, original_location.alt)
    else:
        raise Exception("Invalid Location object passed")

    return targetlocation;


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


def get_bearing(aLocation1, aLocation2):
    """
    Returns the bearing between the two LocationGlobal objects passed as parameters.

    This method is an approximation, and may not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing



#Callback to print the location in global frames. 'value' is the updated value
def location_callback(self, attr_name, value):
    print ("Location (Global): ", value)


def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
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



class FlightControl:

    def __init__(self , connection_string):
        self.connection_string = connection_string
        print('Connecting to vehicle on: %s' % connection_string)
        self.vehicle = connect(connection_string, wait_ready=True)

    def arm_and_takeoff(self,aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """

        vehicle = self.vehicle

        print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        while not vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED mode
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True

        while not vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()
        print ('Home Location: ' ,  vehicle.home_location)


        print("Taking off!")
        vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print(" Altitude: ", vehicle.location.global_relative_frame.alt)
            if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:  # Trigger just below target alt.
                print("Reached target altitude")
                break
            time.sleep(1)

    def goto(self, dNorth, dEast):
        """
        Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

        The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for
        the target position. This allows it to be called with different position-setting commands.
        By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

        The method reports the distance to target every two seconds.
        """
        vehicle = self.vehicle
        gotoFunction = vehicle.simple_goto

        currentLocation = vehicle.location.global_relative_frame
        targetLocation = get_location_metres(currentLocation, dNorth, dEast)
        targetDistance = get_distance_metres(currentLocation, targetLocation)
        gotoFunction(targetLocation)

        # print "DEBUG: targetLocation: %s" % targetLocation
        # print "DEBUG: targetLocation: %s" % targetDistance

        while vehicle.mode.name == "GUIDED":  # Stop action if we are no longer in guided mode.
            # print "DEBUG: mode: %s" % vehicle.mode.name
            remainingDistance = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
            print("Distance to target: ", remainingDistance)
            if remainingDistance <= targetDistance * 0.01:  # Just below target, in case of undershoot.
                print("Reached target")
                break
            time.sleep(2)



    def Finish(self):
        print("Returning to Launch")
        self.vehicle.mode = VehicleMode("RTL")

        # Close vehicle object before exiting script
        print("Close vehicle object")
        self.vehicle.close()



    def addObserver(self):
        # Add a callback `location_callback` for the `global_frame` attribute.
        self.vehicle.add_attribute_listener('vehicle.location.global_relative_frame', location_callback)


    def ChangeSpeed(self , s):
        self.vehicle.airspeed = s

    def Rotate(self , pitch, roll , yaw):
        self.vehicle.gimbal.rotate(pitch , roll , yaw)

    def set_attitude(self, roll_angle=0.0, pitch_angle=0.0, yaw_rate=0.0, thrust=0.5, duration=0):
        """
        Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
        with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
        velocity persists until it is canceled. The code below should work on either version
        (sending the message multiple times does not cause problems).
        """

        """
        The roll and pitch rate cannot be controllbed with rate in radian in AC3.4.4 or earlier,
        so you must use quaternion to control the pitch and roll for those vehicles.
        """

        # Thrust >  0.5: Ascend
        # Thrust == 0.5: Hold the altitude
        # Thrust <  0.5: Descend
        msg = self.vehicle.message_factory.set_attitude_target_encode(
            0,  # time_boot_ms
            1,  # Target system
            1,  # Target component
            0b00000000,  # Type mask: bit 1 is LSB
            to_quaternion(roll_angle, pitch_angle),  # Quaternion
            0,  # Body roll rate in radian
            0,  # Body pitch rate in radian
            math.radians(yaw_rate),  # Body yaw rate in radian
            thrust  # Thrust
        )
        self.vehicle.send_mavlink(msg)

        start = time.time()
        while time.time() - start < duration:
            self.vehicle.send_mavlink(msg)
            time.sleep(0.1)

    def simple_goto1(self):
        print("Set default/target airspeed to 3")
        self.vehicle.airspeed = 10

        print("Going towards first point for 30 seconds ...")
        point1 = LocationGlobalRelative(-35.361354, 149.165218, 20)
        self.vehicle.simple_goto(point1)