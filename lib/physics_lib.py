# coding=utf-8
from math import sin, cos, tan, atan, acos, pi

def norm(v):
    s = 0
    for i in v:
        s += i**2
    return s**0.5

def calculate_angle(v1, v2):
    scal = v1[0]*v2[0] + v1[1]*v2[1]
    return acos(scal/(norm(v1)*norm(v2)))

def calculate_yaw(x, y):
    #расчёт угла рысканья относительно строительной оси
    angle = 0
    if (y > 0 and x == 0): angle = 0
    elif (y < 0 and x == 0): angle = pi
    elif (y == 0 and x > 0): angle = pi/2
    elif (y == 0 and x < 0): angle = -pi/2
    elif (y > 0 and x > 0): angle = atan(x/y)
    elif (y > 0 and x < 0): angle = -atan(abs(x)/y)
    elif (y < 0 and x > 0): angle = atan(x/abs(y)) + pi/2
    elif (y < 0 and x < 0): angle = -atan(abs(x)/abs(y)) - pi/2

    return angle

def calculate_dist_angles(height, point, camera_angle, resolution, prev_x_dist, prev_y_dist, time):
    #возвращаемые углы
    angles = [0, 0, 0]

    #коэффициент наклона по тангажу в зависимости от расстояния(k1)/скорости(k2)
    k1 = 0.01; k2 = 0.001; k3 = -1;
    #дополнительные коэффициенты для расчёта скоростей
    k4 = 2; k5 = 0.6

    #максимальный угол наклона по тангажу
    max_angle = 10*pi/180

    #расчёт углов обзора по осям и расчёт видимого пространства
    delta_angle = (camera_angle**2/float(resolution[0]**2 + resolution[1]**2))**0.5
    x_angle = delta_angle * resolution[0]; x_range = tan(x_angle/2) * height;
    y_angle = delta_angle * resolution[1]; y_range = tan(y_angle/2) * height;

    #расчёт дистанции до цели
    decart_pix = calculate_img_decart_pixels(point, resolution)
    x_dist = float(decart_pix[0]) * x_range / float(resolution[0])*2
    y_dist = float(decart_pix[1]) * y_range / float(resolution[1])*2
    distance = ((x_dist)**2 + (y_dist)**2) ** 0.5
    angles[0] = (k1 * x_range + k2 * (x_dist - prev_x_dist) / time / 2) * k4
    angles[1] = (k1 * y_range + k2 * (y_dist - prev_y_dist) / time / 2) * k5
    angles[0] = abs(angles[0]) * k3 if (x_dist < 0) else abs(angles[0])
    angles[1] = abs(angles[1]) * k3 if (y_dist > 0) else abs(angles[1])

    if (abs(angles[0]) > max_angle):
        angles[0] = max_angle if angles[1] > 0 else max_angle * (-1)
    if (abs(angles[1]) > max_angle):
        angles[1] = max_angle if angles[1] > 0 else max_angle*(-1)

    #расчёт угла на который надо повернуться, чтобы быть на одном направлении с вектором цели
    angles[2] = calculate_yaw(decart_pix[0], decart_pix[1])

    return [x_dist, y_dist, angles]

def calculate_img_decart_pixels(point, resolution):
    res = [0, 0]
    res[0] = point[0] - resolution[0]/2
    res[1] = point[1] * (-1) + resolution[1]/2
    return res

def calculate_distance_NEU_meters(distance, location):
    #TODO: сделать перевод из пиксельных координат в NED/NEU
    return distance





