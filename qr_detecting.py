# -*- coding: utf-8 -*-
import numpy as np
import time
import roslib
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from pyzbar import pyzbar

class Qr_Code():                                                                                               # Класс для распознавание цветов - желтый, синий, красный
    def __init__(self): 
        rospy.init_node('Qr_detect', anonymous=True)                                                           # Создание ноды
        self.image_pub = rospy.Publisher("QR",Image,queue_size=10)
        
        self.bridge = CvBridge()                                                                               # Переменная необходимая для конвертация изображения из типа msg в обычный вид и обратно
        self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)                                   # Подписание на топик с изображением (Вставьте свой вместо image_topic)

    def callback(self,data):                                                                                   # Основная функция (data- изображения из типа msg)  
        try:                                                                                                   # Считывание и конвертация изображения в вид пригодный для дальнейшей работы (try- для отсутствия ошибки если топик будет пустой)
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")                                                                      
        except CvBridgeError as e:
            print(e)
            
        barcodes  = pyzbar.decode(frame)                                                                       # Распознование кодов
        if barcodes:                                                                                           # Если они на картинке есть
            texts = []
            for bar in barcodes:                                                                               # Проходит по всем QR кодам, которые он нашел
                 
                (x, y, w, h) = bar.rect                                                                        # координаты QR кода
                
                barcodeData = bar.data.decode("utf-8")                                                         # Записывает в переменную, что за информация там находится
                
                texts.append([barcodeData, (x,y,w,h)])                                                         # Сохраняет в список что в QR коде написано, и его координаты

                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 4)                                   # Выделение по контуру на изоображении
                cv2.putText(frame, str(barcodeData), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 255), 1)# Делает надпись над QR кодом, что там написано, на изоображении
            print(*texts, sep = '\n')                                                                          # Вывод списка
            cv2.imshow('QR', frame)                                                                            # Вывод в отдельное окно (Нужно закоментить в случае отсутствия дисплея)
            
            try:                                                                                               # Вывод в отдельный топик
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            except CvBridgeError as e:
                print(e)
                
def main(args):                                                                                                # Начальная функция
  qr = Qr_Code()                                                                                               # Обращение к классу Qr_Code
  try:
    rospy.spin()                                                                                               # Обезательная функция для работы с топиками
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
