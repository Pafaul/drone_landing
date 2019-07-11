# -*- coding: utf-8 -*-

from __future__ import print_function

import copy

import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar

landing_phrase = 'landing'

def get_qr_codes(img):
	'''
	получение qr кодов с изображения
	если таких не найдено, возвращается None
	'''

	decoded_qr = pyzbar.decode(img)
	if (decoded_qr):
		return decoded_qr
	else:
		return None

def parse_decoded_qrs(decoded_qr):
	"""
	обработка полученных qr кодов (если такие имеются)
	для каждого подходящего qr кода расчитывается его центр
	в качестве возвращаемых данных:
	(центры, крайние точки qr-кода на изображении)
	:return texts: Возвращает текст qr, а также левую и верхнюю границу qr
	"""

	centers = list()
	polygons = list()
	texts = list()
	for qr in decoded_qr:

		center = [0, 0]
		points = qr.polygon
		for point in qr.polygon:
			center[0] += point.x
			center[1] += point.y

		for p in range(len(center)):
			center[p] = int(center[p]/len(qr.polygon))

		centers.append(center)
		polygons.append(qr.polygon)
		texts.append([qr.data, qr.rect.left, qr.rect.top-3])

	return (centers, polygons, texts)

def draw_qr_contour(img, centers, polygons, texts, test_qr_text):
	'''
	cv2 работает с цветами в кодировке BGR!!!! (0, 0, 255) это красный
	image.shape выдает высоту, ширину изображения и каналы,
	именно в такой последовательности
	'''
	color_red = (0, 0, 255)
	color_blue = (255, 0, 0)
	font = cv2.FONT_HERSHEY_SIMPLEX

	result_image = copy.deepcopy(img)

	if (centers):
		height, width, channels = img.shape
		img_center = (width / 2, height / 2)

		for num in range(len(centers)):
			cv2.circle(result_image, tuple(centers[num]), 5, color_red, -1)
			text = str(texts[num][0]) + " ; location: %f , %f" % (centers[num][0], centers[num][1])
			print(text)
			cv2.putText(result_image, text, tuple(texts[num][1:3]), font, 1, color_blue, 2, cv2.LINE_AA)
			if (test_qr_text(texts[num][0])):
				cv2.line(result_image, img_center, tuple(centers[num]), color_red, 2)

		for polygon in polygons:
			pts = np.array(polygon, np.int32)
			cv2.polylines(result_image, [pts], True, color_red, thickness=3)

	return result_image

def test_qr_text(text):
	if (text == 'landing'):
		return True

def main():

	image_name = 'qr_full_test.png'
	result_image_name = 'qr_result.png'

	img = cv2.imread(image_name);
	decoded_qr = get_qr_codes(img);

	if (decoded_qr):
		print('qr detected')
		centers, polygons, texts = parse_decoded_qrs(decoded_qr)

		result_image = draw_qr_contour(img, centers, polygons, texts, test_qr_text)

		cv2.imwrite(result_image_name, result_image)


	else :
		print('qr not detected')

if (__name__ == '__main__'):
	main()