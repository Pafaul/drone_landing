# coding=utf-8
import sys, dronekit, cv2
from time import time, sleep
from pymavlink import mavutil
sys.path.append("../lib/")
import drone_lib, detect_qr, arg_lib, physics_lib

phrases = dict()
phrases['landing'] = 'landing'

def connect_px4(device):
    #соединение с px4
    vehicle = drone_lib.connect(device, 0)

    return vehicle

def auto_mode_px4(vehicle):
    #установка нужного режима полёта
    auto_mode = 4
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                          mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                          auto_mode,
                                          0, 0, 0, 0, 0, 0)

def startup_videos(angle, source, orig, result, fps):

    camera_angle = angle

    #создаем источник получения видеосигнала
    cap = cv2.VideoCapture(source)
    flag, img = cap.read()

    #создание видеопотоков для записи
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    orig_video = cv2.VideoWriter(orig, fourcc, fps, (img.shape[1::-1]))
    result_video = cv2.VideoWriter(result, fourcc, fps, (img.shape[1::-1]))

    #возвращаем источник видеосигнала и потоки для записи
    return [cap, orig_video, result_video, camera_angle]

def finish_all(cap, orig_video, result_video, vehicle, log):
    #закрытие всего
    vehicle.close()
    cap.release()
    orig_video.release()
    result_video.release()
    log.close()


def landing_comparisson(text):
    #сравнение с фразой для посадки
    if (str(text) == phrases['landing']):
        return True
    else:
        return False

def detect_qr_code(qr_codes):
    for qr in qr_codes:
        if (landing_comparisson(qr.data)):
            return qr
    return None

def write_log(file, start_time, text):
    file.write("%f : %s" % (time() - start_time, text) + '\n')

def future_main():
    #будет управляющей функцией в ближайшем обновлении
    log = open('log.txt', 'a')
    startup_time = time()

    #задание параметров для работы программы
    vehicle_parameters = dict()
    vehicle_parameters['take_off'] = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    vehicle_parameters['set_waypoint'] = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
    vehicle_parameters['land'] = mavutil.mavlink.MAV_CMD_NAV_LAND
    vehicle_parameters['channel'] = 6
    vehicle_parameters['channel_val'] = 1200
    vehicle_parameters['landing_alt'] = 0.2
    landing_eps = 50

    #создание парсера
    parser = arg_lib.create_arg_parser_main()
    args = parser.parse_args()

    #установка соединения с дроном
    vehicle = connect_px4(args.device)

    if (vehicle):
        print("Drone connected")
        write_log(log, startup_time, "Drone connection successful")
    else:
        print("Cannot connect to drone")
        write_log(log, startup_time, "Drone connection failed. Exiting...")
        vehicle.close()
        return

    #создание видеокомпонент
    cap, orig_video, result_video, camera_angle = startup_videos(args.angle, args.source, args.orig, args.result, args.fps)
    if (cap and orig_video and result_video):
        print("Video components created")
        write_log(log, startup_time, "Video components created")
    else:
        print("Cannot create video components")
        write_log(log, startup_time, "Cannot create video components. Exiting...")
        vehicle.close()
        cap.release()

    img_shape = None
    run_flag = True
    control_flag = False

    print('Starting main cycle')
    write_log(log, startup_time, "Starting main cycle")

    while (True):
        flag, img = cap.read()
        orig_video.write(img)
        if (flag):
            print("Image read")
            write_log(log, startup_time, "Image read")
            img_shape = img.shape[1::-1]
            qr_codes = detect_qr.get_qr_codes(img)

            if (qr_codes):
                print("QR codes detected")
                write_log(log, startup_time, "Qr codes detected")
                # если найдены qr-коды: получение из параметро - геометрия, расположение, данные
                centers, polygons, texts = detect_qr.parse_decoded_qrs(qr_codes)
                # запись значений на изображение
                result_image = detect_qr.draw_qr_contour(img, centers, polygons, texts, landing_comparisson)
                result_video.write(result_image)
                for qr_num in range(len(texts)):
                    if (landing_comparisson(texts[qr_num][0])):
                        print("Landing qr code founded")
                        write_log(log, startup_time, "Landig qr code founded")
                        # если среди qr-кодов найден нужный (с надписью 'landing')
                        distance = ((centers[qr_num][0] - img_shape[0] / 2) ** 2 + (
                                    centers[qr_num][1] - img_shape[1] / 2) ** 2) ** 0.5

                        if (distance > landing_eps):
                            #если дистанция до qr больше, чем landign_eps
                            print("Moving to qr")
                            write_log(log, startup_time, "Moving to qr")
                            distance = physics_lib.calculate_distance_NEU_meters(centers[qr_num],
                                                                                 vehicle.location.global_relative_frame)
                            drone_lib.px4_control(vehicle, vehicle_parameters['set_waypoint'],
                                                  distance[0], distance[1], 0)
                        else:
                            print("Landing")
                            write_log(log, startup_time, "Landing")
                            drone_lib.px4_control(vehicle, vehicle_parameters['land'], 0, 0,
                                                  vehicle.location.global_relative_frame.alt)
                            run_flag = False
                        break
                    else:
                        print("Landing qr not found. Waiting")
                        write_log(log, startup_time, "Landing qr not found. Waiting")
                        drone_lib.px4_control(vehicle, vehicle_parameters['set_waypoint'], 0, 0, 0)
            else:
                print("qr not found. Waiting")
                write_log(log, startup_time, "qr not found. Waiting")
                drone_lib.px4_control(vehicle, vehicle_parameters['set_waypoint'], 0, 0, 0)
        else:
            print("Cannot read image. Landing.")
            write_log(log, startup_time, "Cannot read image. Landing.")
            drone_lib.px4_control(vehicle, vehicle_parameters['land'], 0, 0,
                                  vehicle.location.global_relative_frame.alt)
            run_flag = False

        vehicle.commands.upload()
        sleep(1)
        if (not run_flag):
            break

    finish_all(cap, orig_video, result_video, vehicle, log)

def main():
    #текущая функция main
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
        sleep(1)

    auto_mode_px4(vehicle)

    start_point = vehicle.location.global_relative_frame

    drone_lib.px4_control(vehicle, take_off, 0, 0, vehicle_altitude)
    drone_lib.px4_control(vehicle, set_waypoint, 10, 0, 0)
    drone_lib.px4_control(vehicle, set_waypoint, 0, 10, 0)
    drone_lib.px4_control(vehicle, set_waypoint, 0, -10, 0)
    drone_lib.px4_control(vehicle, set_waypoint, -10, 0, 0)
    drone_lib.px4_control(vehicle, land, 0, 0, vehicle_altitude)

    vehicle.commands.upload()
    sleep(2)

    drone_lib.arm_vehicle(vehicle)

    next_waypoint = vehicle.commands.next
    while (next_waypoint < len(vehicle.commands)):
        if (vehicle.commands.next > next_waypoint):
            display_seq = vehicle.commands.next + 1
            print ("Moving to waypoint %s" %(display_seq))
            next_waypoint = vehicle.commands.next
        sleep(1)

    while (vehicle.commands.next > 0):
        sleep(1)

    vehicle.armed = False
    sleep(1)

    vehicle.close()
    sleep(1)

if (__name__ == '__main__'):
    main()