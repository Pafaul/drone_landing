# -*- coding=utf-8 -*-
from __future__ import print_function

import sys
sys.path.append('../lib/')
import time

import drone_lib
import arg_lib

def check_arm(vehicle):
    '''
    Проверка на возможность приведения дрона в рабочее состояние
    :param vehicle: подключенный дрон
    :return:
    '''
    if (drone_lib.arm_vehicle(vehicle)):
        print('Vehicle can be armed!')
    else:
        print('Vehicle cannot be armed!')
        return

    if (vehicle.armed):
        print('Vehicle armed!')

    time.sleep(3)
    vehicle.disarm(5, 15)
    print('Vehicle disarmed!')

    return

def check_thrust(vehicle):
    '''
    проверка на изменение скорости вращения винтов дрона
    :param vehicle: подключенный дрон
    :return:
    '''
    thrust_values = [0.1, 0.2, 0.3]
    if (drone_lib.arm_vehicle(vehicle)):
        print('Starting thrust test!')
    else:
        print('Cannot arm vehicle!')

    if (vehicle.armed):
        for thrust_val in thrust_values:
            print('Trying to chech thrust=%f' % (thrust_val))
            drone_lib.set_vehicle_attitude(vehicle, thrust=thrust_val)
            time.sleep(2)
            drone_lib.set_vehicle_attitude(vehicle, thrust=0)
            time.sleep(2)

    vehicle.disarm()

    return


def main():
    
    parser = arg_lib.create_arg_parser_drone()
    args = parser.parse_args()
    
    device           = args.device
    baud_rate        = args.baud
    tests            = args.tests
    avaivalble_tests = ['arm', 'thrust']

    vehicle = drone_lib.connect(device, baud_rate)
    if (vehicle):
        print('Connection successful!')
        
        for test in tests:
            if (test not in available_tests):
                tests.remove(test)

        for test in tests:
            if (test == 'arm'):
                print('Testing arm function')
                check_arm(vehicle)

            if (test == 'thrust'):
                print('Testing thrust function')
                check_thrust(vehicle)

    vehicle.close()

if (__name__ == '__main__'):
    main()