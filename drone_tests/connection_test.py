# -*- coding: utf-8 -*-
from __future__ import print_function

import sys
sys.path.append('../lib/')
import drone_lib
import arg_lib


def main():
    
    parser    = arg_lib.create_arg_parser_drone()
    args      = parser.parse_args()
    device    = args.device
    baud_rate = args.baud

    vehicle = drone_lib.connect(device, baud_rate)

    if (vehicle):
        print('connection successul!')
    else:
        print('connection failed!')

    vehicle.close()

if (__name__ == '__main__'):
    main()