# -*- coding=utf-8 -*-

from __future__ import print_function

import time
import cv2
import sys
sys.path.append('../lib/')
import detect_qr
import arg_lib


def landing_comparisson(text):
    phrases = dict()
    phrases['landing'] = 'landing'

    if (text == phrases['landing']):
        return True
    else:
        return False

def main():
    
    print("QR VIDEO TEST")
    print("Parsing arguments...")
    
    parser = arg_lib.create_arg_parser_video()
    
    args = parser.parse_args()

    test_time = args.time

    cap       = cv2.VideoCapture(args.source)
    time.sleep(2)
    
    flag, img = cap.read()

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    
    print("Video writers created")

    result_video = cv2.VideoWriter(args.result, fourcc, args.fps, (img.shape[1::-1]))
    orig_video = cv2.VideoWriter(args.orig, fourcc, args.fps, (img.shape[1::-1]))

    start_time = time.time()
    
    print("Starting qr test")

    while ( time.time() - start_time < test_time ) and (cap.isOpened()):
        flag, img = cap.read()
        orig_video.write(img)
        if (flag):
            qr_codes = detect_qr.get_qr_codes(img)
            if (qr_codes):
                centers, polygons, texts = detect_qr.parse_decoded_qrs(qr_codes)
                result_image = detect_qr.draw_qr_contour(img, centers, polygons, texts, landing_comparisson)
                result_video.write(result_image)
            else:
                result_video.write(img)
    
    print("Finishing QR test...")
    print("Writing videos to SD")
    orig_video.release()
    result_video.release()
    cap.release()
    print("QR test finished")
if ( __name__ == '__main__' ):
    main()