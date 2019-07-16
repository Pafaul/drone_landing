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
landing_flag = False
prev_time = time()
distance = 0
x_dist = 0
y_dist = 0

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
    vehicle_parameters['guided_mode'] = dronekit.VehicleMode('GUIDED_NOGPS')
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

    print("Arming")
    write_log(log, startup_time, "Arming")
    dl.vehicle_arm(vehicle)

    print("Taking off to 1m")
    write_log(log, startup_time, "Taking off to 1m")
    dl.take_off(vehicle, 1.3)

    img_shape = None
    run_flag = True
    global landing_flag
    landing_flag = False
    vehicle_location = vehicle.location.global_relative_frame
    v_att = vehicle.attitude
    distance = 0;
    prev_time = time() - startup_time

    while not landing_flag:
        cycle_time = time()

        if (vehicle.mode == vehicle_parameters['guided_mode']):

            control_flag = True
            print("Controlling drone")
            write_log(log, startup_time, "Controlling drone")
            print("drone attitude: " + str(v_att))
            write_log(log, startup_time, "drone attitude" + str(v_att))
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
                            global prev_time
                            global distance
                            global x_dist
                            global y_dist
                            x_dist, y_dist, angles = pl.calculate_dist_angles(vehicle.location.global_relative_frame.alt,
                                                                            centers[qr_num],
                                                                            camera_angle,
                                                                            img_resolution,
                                                                            x_dist,
                                                                            y_dist,
                                                                            time() - prev_time)
                            prev_time = time()
                            distance = (float(x_dist)**2 + float(y_dist)**2)**0.5
                            print("Calculated deltas: x = %.5f; y = %.5f" % (x_dist, y_dist))
                            write_log(log, startup_time, "Calculated deltas: x = %.5f; y = %.5f" % (x_dist, y_dist))
                            print("Calculated distance: %.5f" % (distance))
                            write_log(log, startup_time, "Calculated distance: %.5f" % (distance))

                            if (distance < vehicle_parameters['landing_delta']):
                                print("Landing can be performed")
                                write_log(log, startup_time, "Landing can be performed")
                                #if (vehicle.mode != vehicle_parameters['land_mode']):
                                #    vehicle.mode = vehicle_parameters['land_mode']
                                global landing_flag
                                landing_flag = True

                            else:
                                angles[0] = angles[0]
                                angles[1] = angles[1]
                                angles[2] = v_att.yaw
                                print("Calculated angles: " + str(angles))
                                write_log(log, startup_time, "Calculated angles: " + str(angles))
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
                #vehicle.mode = vehicle_parameters['land_mode']
                global landing_flag
                landing_flag = True

        else:
            print("NOT controlling drone")
            write_log(log, startup_time, "NOT controlling drone")
            flag, img  = cap.read()
            orig_video.write(img)
            result_video.write(img)

        print("Cycle time: %.4f" % (time() - cycle_time))

    if (landing_flag):
        print("Waiting for landing")
        write_log(log, startup_time, "Waiting for landing")
        flag, img = cap.read()
        orig_video.write(img)
        result_video.write(img)
        dl.landing(vehicle)
    #while (vehicle.armed):
    #    print("Waiting for landing")
    #    write_log(log, startup_time, "Waiting for landing")
    #    flag, img = cap.read()
    #    orig_video.write(img)
    #    result_video.write(img)
    #    sleep(1)

    finish_all(cap, orig_video, result_video, vehicle, log)

if __name__ == '__main__':
    main()
