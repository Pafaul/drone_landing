# coding=utf-8
from math import sin, cos, tan, atan2, acos, pi

def norm(v):
    s = 0
    for i in v:
        s += i**2
    return s**0.5

def calculate_angle(v1, v2):
    scal = v1[0]*v2[0] + v1[1]*v2[1]
    return acos(scal/(norm(v1)*norm(v2)))

def calculate_angles(height, point, camera_angle, resolution):
    #возвращаемые углы
    angles = [0, 0, 0]
    #коэффициент наклона по тангажу в зависимости от расстояния
    k = 0.01
    #максимальный угол наклона по тангажу
    max_angle = 10*pi/180
    #расчёт углов обзора по осям и расчёт видимого пространства
    delta_angle = (camera_angle**2/(resolution[0]**2 + resolution[1]**2))**0.5
    x_angle = delta_angle * resolution[0]; x_range = tan(x_angle) * height;
    y_angle = delta_angle * resolution[1]; y_range = tan(y_angle) * height;
    #расчёт дистанции до цели
    distance = (((point[0] - resolution[0]/2)*x_range)**2 + ((point[1] - resolution[1]/2)*y_range)**2) ** 0.5
    angles[0] = distance * k
    if (abs(angles[0]) > max_angle):
        if (angles[0] > 0): angles[0] = max_angle
        else: angles[0] = -max_angle

    #расчёт угла на который надо повернуться, чтобы быть на одном направлении с вектором цели
    target_vector = [ point[0] - resolution[0]/2, point[1] - resolution[1]/2  ]
    main_vector = [ 0, resolution[1]/2 ]
    angles[2] = calculate_angle(target_vector, main_vector)
    angles[2] = atan2(point[1], point[0]) - pi

    return angles





