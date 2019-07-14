# -*- coding: utf-8 -*-
from time import time, sleep
from dronekit import connect, VehicleMode
import dronekit
from pymavlink import mavutil
from math import cos, sin, pi

def connect(device, baud_rate):
    #установка соединения с дроном
    vehicle = None
    if (baud_rate == 0):
        vehicle = dronekit.connect(device, wait_ready = True)
    else:
        vehicle = dronekit.connect(device, baud=baud_rate, wait_ready = True)

    return vehicle

def vehicle_arm(vehicle):
    #заармить дрон
    while( not vehicle.is_armable):
        sleep(1)

    vehicle.mode = VehicleMode('GUIDED_NOGPS')

    vehicle.arm()

    return vehicle.armed

def take_off(vehicle, alt):
    #взлёт на какую-либо высоту
    thrust = 0.6
    while (True):
        cur_alt = vehicle.global_relative_frame.alt
        if (cur_alt >= alt):
            break
        else:
            set_attitude(thrust=thrust)
        sleep(0.2)

def send_attitude(vehicle, angles = [0, 0, 0], thrust = 0.5):
    #формирование сообщения для отправки на дрон
    #angles: [0] - крен, [1] - тангаж, [2] - рысканье
    msg = vehicle.message_factory.set_attitude_target_encode(
        0,  # time_boot_ms
        1,  # Target system
        1,  # Target component
        0b00000100,
        #roll, pitch, yaw
        to_quaternion(angles),  # Quaternion
        0,  # Body roll rate in radian
        0,  # Body pitch rate in radian
        0,  # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)

def set_attitude(vehicle, angles = [0, 0, 0], thrust = 0.5):
    #сама отправка сообщения
    send_attitude(vehicle, angles=angles, thrust=thrust)
    sleep(0.04)
    #send_attitude(vehicle, angles=[0, 0, 0], thrust=thrust)

def to_quaternion(angles = [0, 0, 0]):
    #перевод уголов в кватернионы
    t0 = cos(angles[2] * 0.5)
    t1 = sin(angles[2] * 0.5)
    t2 = cos(angles[0] * 0.5)
    t3 = sin(angles[0] * 0.5)
    t4 = cos(angles[1] * 0.5)
    t5 = sin(angles[1] * 0.5)

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]


if __name__ == '__main__':
    pass