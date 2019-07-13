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

def calculate_dist_angles(height, point, camera_angle, resolution, prev_distance, time):
    #возвращаемые углы
    angles = [0, 0, 0]

    #коэффициент наклона по тангажу в зависимости от расстояния(k1)/скорости(k2)
    k1 = 0.01; k2 = 0.01

    #максимальный угол наклона по тангажу
    max_angle = 10*pi/180

    #расчёт углов обзора по осям и расчёт видимого пространства
    delta_angle = (camera_angle**2/(resolution[0]**2 + resolution[1]**2))**0.5
    x_angle = delta_angle * resolution[0]; x_range = tan(x_angle/2) * height;
    y_angle = delta_angle * resolution[1]; y_range = tan(y_angle/2) * height;

    #расчёт дистанции до цели
    decart_pix = calculate_img_decart_pixels(point, resolution)
    distance = ((decart_pix[0]*x_range/resolution[0]*2)**2 + (decart_pix[1]*y_range/resolution[1]*2)**2) ** 0.5
    angles[1] = k1 * distance + k2 * (distance - prev_distance)/time
    if (abs(angles[1]) > max_angle):
        angles[1] = max_angle if angles[1] > 0 else max_angle*(-1)

    #расчёт угла на который надо повернуться, чтобы быть на одном направлении с вектором цели
    angles[2] = calculate_yaw(decart_pix[0], decart_pix[1])

    return [distance, angles]

def calculate_img_decart_pixels(point, resolution):
    res = [0, 0]
    res[0] = point[0] - resolution[0]/2
    res[1] = point[1] * (-1) + resolution[1]/2
    return res

def calculate_distance_NEU_meters(distance, location):
    #TODO: сделать перевод из пиксельных координат в NED/NEU
    return distance





