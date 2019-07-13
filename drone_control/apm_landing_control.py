# coding=utf-8

import cv2
import dronekit
from time import time, sleep
import sys
sys.path.append('../lib/')

import drone_lib as dl
import detect_qr as dq
import arg_lib as al
import physics_lib as pl

phrases = dict()
phrases['landing'] = 'landing'
command_types = ['land', 'att']

def startup_drone(device, baud):
    vehicle = dl.connect(device, baud)
    return vehicle

def startup_videos(angle, source, orig, result, fps):

    camera_angle = angle

    cap = cv2.VideoCapture(source)
    flag, img = cap.read()

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    orig_video = cv2.VideoWriter(orig, fourcc, fps, (img.shape[1::-1]))
    result_video = cv2.VideoWriter(result, fourcc, fps, (img.shape[1::-1]))

    return [cap, orig_video, result_video, camera_angle]

def finish_all(cap, orig_video, result_video, vehicle, log):

    vehicle.close()
    cap.release()
    orig_video.release()
    result_video.release()
    log.close()

def landing_comparisson(text):

    if (str(text) == phrases['landing']):
        return True
    else:
        return False

def detect_qr_codes(qr_codes):
    for qr in qr_codes:
        if (landing_comparisson(qr.data)):
            return qr
    return None

def write_log(file, start_time, text):
    file.write('%f : %s' % (time() - start_time, text) + '\n')

def command_control(vehicle, commands):

    for command in commands:
        if (command[0] == command_types[0]):
            vehicle.mode = dronekit.VehicleMode('LAND')
            break
        elif (command[0] == command_types[1]):
            dl.set_attitude(vehicle, command[1:4], command[4])
        elif (command[0] == command_types[2]):
            dl.take_off(vehicle, command[1])

def main():

    log = open('log.txt', 'a')
    startup_time = time()

    vehicle_parameters = dict()
    vehicle_parameters['guided_mode'] = dronekit.VehicleMode('GUIDED')
    vehicle_parameters['land_mode'] = dronekit.VehicleMode('LAND')
    vehicle_parameters['landing_delta'] = 0.5

    parser = al.create_arg_parser_main()
    args = parser.parse_args()

    vehicle = startup_drone(args.device, args.baud)
    if (vehicle):
        print("Drone connected")
        write_log(log, startup_time, "Drone connection successul")
    else:
        print("Cannot connect to drone")
        write_log(log, startup_time, "Drone connection failed. Exiting.")
        vehicle.close()
        return

    cap, orig_video, result_video, camera_angle = startup_videos(args.angle, args.source, args.orig, args.result, args.fps)
    if (cap and orig_video and result_video):
        print("Video components created")
        write_log(log, startup_time, "Video componenets created")
    else:
        print("Cannot create video components")
        write_log(log, startup_time, "Cannot create video components. Exiting.")
        finish_all(cap, orig_video, result_video, vehicle, log)

    img_shape = None
    run_flag = True
    control_flag = False
    vehicle_location = vehicle.location.global_relative_frame
    v_att = vehicle.attitude
    distance = 0;
    prev_time = time() - startup_time

    while run_flag:

        if (vehicle.mode == vehicle_parameters['guided_mode']):

            control_flag = True
            print("Controlling drone")
            write_log(log, startup_time, "Controlling drone")
            flag, img  = cap.read()
            orig_video.write(img)
            img_resolution = img.shape[1::-1]

            if (flag):

                print("Image readed")
                write_log(log, startup_time, "Image readed")
                qr_codes = dq.get_qr_codes(img)

                if (qr_codes):

                    print("QR codes detected")
                    write_log(log, startup_time, "QR codes detected")
                    centers, polygons, texts = dq.parse_decoded_qrs(qr_codes)
                    result_image = dq.draw_qr_contour(img, centers, polygons, texts, landing_comparisson)
                    result_video.write(result_image)

                    for qr_num in range(len(texts)):

                        if (landing_comparisson(texts[qr_num][0])):

                            print("Landing QR founded")
                            write_log(log, startup_time, "Landing QR founded")
                            distance, angles = pl.calculate_dist_angles(vehicle.location.global_relative_frame.alt,
                                                                        centers[qr_num],
                                                                        camera_angle,
                                                                        img_resolution,
                                                                        distance,
                                                                        time() - prev_time)
                            prev_time = time()
                            print("Calculated distance: %f" % (distance))
                            write_log(log, startup_time, "Calculated distance: %f" % (distance))

                            if (distance < vehicle_parameters['landing_delta']):
                                print("Landing can be performed")
                                write_log(log, startup_time, "Landing can be performed")
                                if (vehicle.mode != vehicle_parameters['land_mode']):
                                    vehicle.mode = vehicle_parameters['land_mode']
                                while (vehicle.armed):
                                    print("Waiting for landing")
                                    write_log(log, startup_time, "Waiting for landing")
                                    sleep(1)
                            else:
                                print("Calculated angles: " + str(angles))
                                write_log("Calculated andgles: " + str(angles))
                                dl.set_attitude(vehicle, angles=angles)

                            break
                    if (not detect_qr_codes(qr_codes)):
                        print("No landing QR founded. Waiting.")
                        write_log(log, startup_time, "No landign QR founded. Waiting.")
                        angles = [v_att.roll, v_att.pitch, v_att.yaw]
                        dl.set_attitude(vehicle, angles=angles)

                else:

                    print("Waiting for QR codes. Setting angles: 0; 0; 0")
                    result_video.write(img)
                    write_log(log, startup_time, "Waiting for QR codes. Setting angles: 0; 0; 0")
                    dl.set_attitude(vehicle, angles = [0, 0, 0])

            else:
                print("Cannot get image. LANDING")
                write_log(log, startup_time, "Cannot get image. LANDING")
                vehicle.mode = vehicle_parameters['land_mode']
                while (vehicle.armed):
                    print("Waiting for landing")
                    log.write("Waiting for landing")
                    sleep(1)

        else:
            continue

    finish_all(cap, orig_video, result_video, vehicle, log)

if __name__ == '__main__':
    main()