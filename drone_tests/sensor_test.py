# -*- coding: utf-8 -*-
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import time

import drone_lib


def test_sensors(vehicle):
    '''
    Расчёт мат ожидания и дисперсии по данным, полученным с
    датчиков высоты и измерителя углов
    в качестве параметров - подключённый дрон
    :param vehicle: подключенный дрон
    :return: [mean, dispersion]
    '''

    test_num = 1000

    alts = list();
    attitudes = list();

    for test in range(test_num):
        alts.append(vehicle.location.global_frame.alt)
        time.wait(0.1)
        attitudes.append(vehicle.attitude)
        time.wait(0.1)

    mean = [0] * 4;
    dispersion = [0] * 4;

    for alt in alts:
        mean[0] += alt

    for att in attitudes:
        mean[1] += att.roll
        mean[2] += att.pitch
        mean[3] += att.yaw

    mean = [m/test_num for m in mean]

    for alt in alts:
        dispersion[0] += (alt - mean[0])**2

    for att in attitudes:
        dispersion[1] += (att.roll - mean[1])**2
        dispersion[2] += (att.pitch - mean[2])**2
        dispersion[3] += (att.yaw - mean[3])**2

    dispersion = [d/(test_num-1) for d in dispersion]

    return [mean, dispersion]

def main():
    '''
    Выполняет подключение к дрону и вызов функции расчета
    статистических параметров датчиков высоты и углов
    :return:
    '''
    connection_string = '/dev/ttyAMA0'
    baud_rate         = 57600

    vehicle = drone_lib.startup(connection_string, baud_rate)
    mean, dispersion = test_sensors(vehicle)

    if __name__ == '__main__':
        print('mean altitude: %f' % (mean[0]))
        print('mean roll: %f' % (mean[1]))
        print('mean pitch: %f' % (mean[2]))
        print('mean yaw: %f' % (mean[3]))

        print('dispersion altitude: %f' % (dispersion[0]))
        print('dispersion roll: %f' % (dispersion[1]))
        print('dispersion pitch: %f' % (dispersion[2]))
        print('dispersion yaw: %f' % (dispersion[3]))

    vehicle.close()

if __name__ == '__main__':
    main()