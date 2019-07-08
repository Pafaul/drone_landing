import argparse

def create_arg_parser_drone():
    parser = argparse.ArgumentParser()
    
    parser.add_argument("-b", "--baud", type = int, nargs='?', const = 57600, help = "connection baud rate (example: 9600, 57600, ...)")
    parser.add_argument("-d", "--device", type = str, nargs='?', const = '/dev/ttyACM0', help = "device that flight controller is connected to (example: /dev/ttyAMA0, /dev/ttyACM0, /dev/ttyS0, ...)")
    parser.add_argument("-t", "--test", type = str, nargs='?', const = '', action = "append", choices = ['arm', 'thrust'], help = "What tests will be maintained. Example: \"-t arm -t thrust\"")

    return parser

def create_arg_parser_video():
    parser = argparse.ArgumentParser()
    
    parser.add_argument("-o", "--orig", type = str, nargs='?', const = 'orig.avi', help = "Original video name (with no modifications)")
    parser.add_argument("-r", "--result", type = str, nargs='?', const = 'result.avi', help = "Result video name (with modifications)")
    parser.add_argument("-f", "--fps", type = int, nargs='?', const = 12, help = "Videos FPS")
    parser.add_argument("-t", "--time", type = int, nargs='?', const = 120, help = "Test time duration")
    parser.add_argument("-s", "--source", type = int, nargs='?', const = 0, help = "Video source code number")
    
    return parser

if (__name__ == "__main__"):
    print("This module create parsers for:\ndrone scripts\nvideo scripts")