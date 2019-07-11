import sys, time, dronekit
from pymavlink import mavutil
sys.path.append("../lib/")
import drone_lib, detect_qr, arg_lib, physics_lib

def connect_px4():
    parser = arg_lib.create_arg_parser_drone()
    args = parser.parse_args()

    vehicle = drone_lib.connect(args.device, args.baud)

    return vehicle

def auto_mode_px4(vehicle):
    auto_mode = 4
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                          mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                          auto_mode,
                                          0, 0, 0, 0, 0, 0)

def main():
    vehicle = connect_px4()

    home_position_set = False
    take_off = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    set_waypoint = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
    land = mavutil.mavlink.MAV_CMD_NAV_LAND

    vehicle_altitude = 10

    @vehicle.on_message('HOME_POSITION')
    def listener(self, name, home_position):
        home_position_set = True

    while (not home_position_set):
        print('Waiting for home position')
        time.sleep(1)

    auto_mode_px4(vehicle)

    start_point = vehicle.location.global_relative_frame

    drone_lib.px4_control(vehicle, take_off, 0, 0, vehicle_altitude)
    drone_lib.px4_control(vehicle, set_waypoint, 10, 0, 0)
    drone_lib.px4_control(vehicle, set_waypoint, 0, 10, 0)
    drone_lib.px4_control(vehicle, set_waypoint, 0, -10, 0)
    drone_lib.px4_control(vehicle, set_waypoint, -10, 0, 0)
    drone_lib.px4_control(vehicle, land, 0, 0, vehicle_altitude)

    vehicle.commands.upload()
    time.sleep(2)

    drone_lib.arm_vehicle(vehicle)

    next_waypoint = vehicle.commands.next
    while (next_waypoint < len(vehicle.commands)):
        if (vehicle.commands.next > next_waypoint):
            display_seq = vehicle.commands.next + 1
            print ("Moving to waypoint %s" %(display_seq))
            next_waypoint = vehicle.commands.next
        time.sleep(1)

    while (vehicle.commands.next > 0):
        time.sleep(1)

    vehicle.armed = False
    time.sleep(1)

    vehicle.close()
    time.sleep(1)
