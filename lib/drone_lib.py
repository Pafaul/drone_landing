# -*- coding: utf-8 -*-
import time
import dronekit
from pymavlink import mavutil
from math import cos, sin, pi

def connect(device, baud_rate):
    '''
    Осуществляет подключение к дрону
    :param device: устройство, к которому необходимо подключиться, например: /dev/ttyAMA0
    :param baud_rate: скорость подключения, например: 57600
    :return:
    '''
    vehicle = dronekit.connect(device, baud = baud_rate, wait_ready=True)
    vehicle.mode = dronekit.VehicleMode("GUIDED")
    return vehicle

def arm_vehicle(vehicle):

    while not vehicle.is_armable:
        print('Waiting for vehicle to initialize')
        time.sleep(1)

    print('Arming')

    vehicle.arm()

    for i in range(3):
        print('Waiting for arming')
        time.sleep(1)
        if (vehicle.armed):
            break

    if (vehicle.armed):
        return True
    else:
        return False

def px4_calc_location_lat_lon_meters(original_location, dNorth = 0, dEast = 0, alt = 0):
    earth_radius = 6378137.0
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*cos(pi*original_location.lat/180))

    new_lat = original_location.lat + (dLat * 180/pi)
    new_lon = original_location.lon + (dLon * 180/pi)

    return dronekit.LocationGlobal(new_lat, new_lon, original_location.alt + alt)

def px4_control(vehicle, command, dNorth = 0, dEast = 0, alt = 0):
    location = vehicle.location.global_reltive_frame
    move = px4_calc_location_lat_lon_meters(location, dNorth, dEast, alt)
    cmd = dronekit.Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                           command,
                           0, 1, 0, 0, 0, 0, move.lat, move.lon, move.alt)
    vehicle.commands.add(cmd)

def to_quaternion(angles=[0,0,0]):
    return [
        cos(angles[0] / 2.) * cos(angles[1] / 2.) * cos(angles[2] / 2.) - sin(angles[0] / 2.) * sin(angles[1] / 2.) * sin(angles[2] / 2.),
        cos(angles[0] / 2.) * cos(angles[1] / 2.) * sin(angles[2] / 2.) + sin(angles[0] / 2.) * sin(angles[1] / 2.) * cos(angles[2] / 2.),
        sin(angles[0] / 2.) * cos(angles[1] / 2.) * cos(angles[2] / 2.) + cos(angles[0] / 2.) * sin(angles[1] / 2.) * sin(angles[2] / 2.),
        cos(angles[0] / 2.) * sin(angles[1] / 2.) * cos(angles[2] / 2.) + sin(angles[0] / 2.) * cos(angles[1] / 2.) * sin(angles[2] / 2.)
    ]

def set_vehicle_attitude( vehicle, angles = [0, 0, 0], thrust = 0.5):
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, #time_boot_ms
        1, #target system
        1, #target component
        0b00000000,
        to_quaternion( angles ), #quaternion
        0, #body roll rate in radian
        0, #body pitch rate in radian
        0, #body aw rate n radian/second
        thrust #thrust
    )
    vehicle.send_mavlink(msg)

if __name__ == '__main__':
    pass