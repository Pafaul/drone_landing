# -*- coding=utf-8 -*-

from __future__ import print_function

import time

import cv2

import detect_qr


def landing_comparisson(text):
    phrases = dict()
    phrases['landing'] = 'landing'

    if (text == phrases['landing']):
        return True
    else:
        return False

def main():

    test_time = 120

    cap = cv2.VideoCapture(0)
    time.wait(2)
    flag, img = cap.read()

    fourcc = cv2.VideoWriter_fourcc(*'XVID')

    result_video = cv2.VideoWriter('result.avi', fourcc, 12, (img.shape[1::-1]))
    orig_video = cv2.VideoWriter('orig.avi', fourcc, 12, (img.shape[1::-1]))

    start_time = time.time()

    while ( time.time() - start_time < test_time ) and (cap.isOpened()):
        flag, img = cap.read()
        orig_video.write(img)
        if (flag):
            qr_codes = detect_qr.get_qr_codes(img)
            centers, polygons, texts = detect_qr.parse_decoded_qrs(qr_codes)
            result_image = detect_qr.draw_qr_contour(img, centers, polygons, texts, landing_comparisson)
            result_video.write(result_image)
        else:
            result_video.write(img)

    orig_video.release()
    result_video.release()
    cap.release()

if ( __name__ == '__main__' ):
    main()