# coding=utf-8
from time import time, sleep
import cv2
import dronekit
import sys
sys.path.append('../lib/')

import detect_qr, arg_lib, drone_lib, physics_lib

phrases = dict()
phrases['landing'] = 'landing'

def startup_drone(device, baud):
    #устанавливаем соединение с дроном
    vehicle = drone_lib.connect(device, baud)
    return vehicle

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

def main():

    #лог для записи событий
    log = open('log.txt', 'a')
    startup_time = time()

    #начальное задание параметров для работы цикла
    vehicle_parameters = dict()
    vehicle_parameters['channel']      = 6
    vehicle_parameters['channel_val']  = 900
    vehicle_parameters['landing_mode'] = 'LAND'
    vehicle_parameters['landing_alt']  = 0.2
    landing_eps = 50

    parser = arg_lib.create_arg_parser_main()
    args = parser.parse_args()

    #установка соединения с дроном
    vehicle = startup_drone(args.device, args.baud)
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

    #основной цикл, бесконечный, прерывание внутри
    while run_flag:
        #проверка на то, нужно ли сейчас авоматическое управление
        if (vehicle.channels[vehicle_parameters['channel']] > vehicle_parameters['channel_val']):
            #если да - считываем изображение
            control_flag = True
            print("Controlling drone")
            write_log(log, startup_time, "Controlling drone")
            flag, img = cap.read()
            orig_video.write(img)
            img_shape = img.shape[1::-1]

            if (flag):
                print("Image read")
                write_log(log, startup_time, "Image read")
                #если изображение было считано - выполняется поиск qr-кодов
                qr_codes = detect_qr.get_qr_codes(img)

                if (qr_codes):
                    print("QR codes detected")
                    write_log(log, startup_time, "Qr codes detected")
                    #если найдены qr-коды: получение из параметро - геометрия, расположение, данные
                    centers, polygons, texts = detect_qr.parse_decoded_qrs(qr_codes)
                    #запись значений на изображение
                    result_image = detect_qr.draw_qr_contour(img, centers, polygons, texts, landing_comparisson)
                    result_video.write(result_image)

                    #проверка по всем qr-кодам
                    for qr_num in range(len(texts)):
                        if (landing_comparisson(texts[qr_num][0])):
                            print("Landing qr code founded")
                            write_log(log, startup_time, "Landig qr code founded")
                            # если среди qr-кодов найден нужный (с надписью 'landing')
                            distance = ((centers[qr_num][0] - img_shape[0]/2)**2 + (centers[qr_num][1] - img_shape[1]/2)**2)**0.5
                            if (distance > landing_eps):
                                #если дистанция до qr больше, чем landign_eps
                                print("Moving to qr")
                                write_log(log, startup_time, "Moving to qr")
                                angles = physics_lib.calculate_angles(vehicle.location.global_relative_frame.alt,
                                                                      centers[qr_num],
                                                                      camera_angle,
                                                                      img_shape)

                                print("Calculated angles = " + str(angles))
                                write_log(log, startup_time, "Calculated angles = " + str(angles))
                                angles[0] += vehicle.attitude.pitch
                                angles[1] += vehicle.attitude.roll
                                angles[2] += vehicle.attitude.yaw
                                drone_lib.set_vehicle_attitude(vehicle, angles=angles, thrust=0.5)
                            else:
                                print("Landing")
                                write_log(log, startup_time, "Landing")
                                if (vehicle.mode.name != vehicle_parameters['landing_mode']):
                                    vehicle.mode = dronekit.VehicleMode('LAND')
                                    while (vehicle.location.global_relative_frame.alt > vehicle_parameters['landing_alt']):
                                        sleep(3)
                            break
                        else:
                            print("Landing qr not found. Waiting")
                            write_log(log, startup_time, "Landing qr not found. Waiting")
                            drone_lib.set_vehicle_attitude(vehicle, angles=[0, 0, 0], thrust=0.5)
                else:
                    #если qr-коды не найдены
                    print("Qr codes not found")
                    write_log(log, startup_time, "QR codes not found")
                    result_video.write(img)
                    if (vehicle.location.global_relative_frame.alt < vehicle_parameters['landing_alt']):
                        print("Landing successful.")
                        write_log(log, startup_time, "LANDING SUCCESSFUL! Exiting...")
                        #если дрон ниже определнной высоты посадки - завершение цикла
                        run_flag = False
                        break
                    else:
                        print("No qr detected. Waiting")
                        write_log(log, startup_time, "No qr detected. Waiting")
                        drone_lib.set_vehicle_attitude(vehicle, angles=[0, 0, 0], thrust=0.5)
        else:
            #если не выполняется условие по каналам
            print("Not controlling drone")
            write_log(log, startup_time, "Not controlling drone")
            if (control_flag):
                #останавливаем управление, если до этого управляли квадрокоптером, будет произведен выход из цикла
                print("Changed to manual control. Exiting...")
                write_log(log, startup_time, "Changed to manual control. Exiting...")
                run_flag = False


    finish_all(cap, orig_video, result_video, vehicle, log)

if (__name__ == '__main__'):
    main()