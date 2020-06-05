# -*- coding: utf-8 -*-
import time
import math
import numpy as np
import cv2
from pyzbar import pyzbar

cam = cv2.VideoCapture(0)
while True:
    ret, frame = cam.read() # Считывает картинку   
    barcodes  = pyzbar.decode(frame)    # Распознование QR-кодов
    if barcodes:    # Если они на картинке есть
        texts = []
        for bar in barcodes:       # Проходит по всем QR кодам, которые он нашел

            (x, y, w, h) = bar.rect     # координаты QR кода

            barcodeData = bar.data.decode("utf-8") # Записывает в переменную, что за информация там находится

            texts.append([barcodeData, (x,y,w,h)])  # Сохраняет в список что в QR коде написано, и его координаты

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 4)    # Выделение по контуру на изоображении
            cv2.putText(frame, str(barcodeData), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 255), 1) # Делает надпись над QR кодом, что там написано, на изоображении
    cv2.imshow("QR", frame) # Вывод финального изображения на дисплей
    if cv2.waitKey(10) == 27:  # Вывод из программы на кнопку ESC
        break                                                                       
cap.release()
cv2.destroyAllWindows() 
