# -*- coding: utf-8 -*-
from __future__ import print_function

import drone_lib


def main():

    connection_string = '/dev/ttyAMA0'
    baud_rate         = 57600

    vehicle = drone_lib.connect(connection_string, baud_rate)

    if (vehicle):
        print('connection successul!')
    else:
        print('connection failed!')

    vehicle.close()

if (__name__ == '__main__'):
    main()