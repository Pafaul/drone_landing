# -*- coding: utf-8 -*-
import time
from math import cos, sin

import dronekit


def connect(connection_string, baud_rate):
    '''
    Осуществляет подключение к дрону
    :param connection_string: устройство, к которому необходимо подключиться, например: /dev/ttyAMA0
    :param baud_rate: скорость подключения, например: 57600
    :return:
    '''
    vehicle = dronekit.connect(connection_string, baud = baud_rate, wait_ready=True)
    vehicle.mode = dronekit.VehicleMode("GUIDED")
    return vehicle

def arm_vehicle(vehicle):

    while not vehicle.is_armable:
        print('Waiting for vehicle to initialize')
        time.sleep(5)

    print('Arming')

    vehicle.arm()

    for i in range(3):
        print('Waiting for arming')
        time.sleep(5)
        if (vehicle.armed):
            break

    if (vehicle.armed):
        return True
    else:
        return False



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