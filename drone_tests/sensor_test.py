# -*- coding: utf-8 -*-
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import time
import sys
sys.path.append('../lib/')
import drone_lib
import arg_lib


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
    
    print('Calculating mean values and dispersion...')

    for test in range(test_num):
        alts.append(vehicle.location.global_relative_frame.alt)
        time.sleep(0.1)
        attitudes.append(vehicle.attitude)
        time.sleep(0.1)

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
    
    print('Calculation finished...')

    return [mean, dispersion]

def main():
    '''
    Выполняет подключение к дрону и вызов функции расчета
    статистических параметров датчиков высоты и углов
    :return:
    '''
    parser = arg_lib.create_arg_parser_drone()
    args = parser.parse_args()
    device = args.device
    baud_rate = args.baud
    
    print('Trying to connect to vehicle')
    vehicle = drone_lib.connect(device, baud_rate)
    if (vehicle):
        print('Connection successful. Starting sensors test.')
        mean, dispersion = test_sensors(vehicle)

        if __name__ == '__main__':
            print('Mean values of:')
            print('altitude: %f' % (mean[0]))
            print('roll: %f' % (mean[1]))
            print('pitch: %f' % (mean[2]))
            print('yaw: %f' % (mean[3]))

            print('Standart deviation of:')
            print('altitude: %f' % (dispersion[0]**0.5))
            print('roll: %f' % (dispersion[1]**0.5))
            print('pitch: %f' % (dispersion[2]**0.5))
            print('yaw: %f' % (dispersion[3]**0.5))
    else:
        print('Cannot connect to vehicle. Test Failed.')

    vehicle.close()

if __name__ == '__main__':
    main()
