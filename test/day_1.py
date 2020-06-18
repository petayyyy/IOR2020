# -*- coding: utf-8 -*-
import numpy as np
import math
import cv2
import time
import roslib
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ColorDetecting():                                                                                              # Класс для распознавание цветов - желтый, синий, красный
    def __init__(self):                                                                                              # Функция init содежит:
        rospy.init_node('Color_detect', anonymous=True)                                                              # Создание ноды
        self.image_pub_potato = rospy.Publisher("Potato",Image,queue_size=10)                                              # Создание топиков с масками цветов
        self.image_pub_water = rospy.Publisher("Water",Image,queue_size=10)
        self.image_pub_seed = rospy.Publisher("Seed",Image,queue_size=10)
        self.image_pub_pastures = rospy.Publisher("Pastures",Image,queue_size=10)
        self.image_pub_soil = rospy.Publisher("Soil",Image,queue_size=10)
        
        self.image_pub = rospy.Publisher("Final",Image,queue_size=10)                                                # И топика для вывода финального изображения

        self.potato_low = np.array([0,140,170])                                                                         # Параметры необходимые для определения облака точек каждого цвета:
        self.potato_high = np.array([20, 255, 255])                                                                     # Красного

        self._potato_low = np.array([10,140,255])                                                                           # Доп фильтр для красного цвета 
        self._potato_high = np.array([20,255,255])

        self.water_low = np.array([93,76,153])                                                                       # Синего
        self.water_high = np.array([153,255,255])

        self.seed_low = np.array([19,134,202])                                                                      # И желтого
        self.seed_high = np.array([35,255,255])
        
        self.pastures_low = np.array([70,153,104])                                                                      # И желтого
        self.pastures_high = np.array([95,255,255])
        
        self.soil_low = np.array([12,223,128])                                                                      # И желтого
        self.soil_high = np.array([111,243,138])
        
        
        self.pix_x = 320
        self.pix_y = 240
        self.yaw_x = 160
        self.yaw_y = 120
        
        self.Qr = True
        self.Color = True
        self.Land = False
        self.Pole = 'A'
        self.bridge = CvBridge()                                                                                     # Переменная необходимая для конвертации изображения из типа msg в обычный вид и обратно
        self.image_sub = rospy.Subscriber("main_camera/image_raw",Image,self.callback)                               # Подписание на топик с изображением
    def distance_x(self,x,z):
        if x >= pix_x //2:
            tg_yaw_x = ((x - self.pix_x)*math.tan(math.radians(self.yaw_x // 2)))/(self.pix_x)
            return int(tg_yaw_x * z)
        else:
            tg_yaw_x = ((self.pix_x - x)*math.tan(math.radians(self.yaw_x // 2)))/(self.pix_x)
            return -int(tg_yaw_x * z)
    def distance_y(self,y,z):
        if y >= pix_y //2:
            tg_yaw_y = ((y - self.pix_y)*math.tan(math.radians(self.yaw_y // 2)))/(self.pix_y)
            return -int(tg_yaw_y * z)
        else:
            tg_yaw_y = ((self.pix_y - y)*math.tan(math.radians(self.yaw_y // 2)))/(self.pix_y)
            return int(tg_yaw_y * z)        
    def callback(self,data):                                                                                         # Основная функция (data- изображения из типа msg)
        if self.Color == True or self.Qr == True:
            try:                                                                                                         # Считывание и конвертация изображения в вид пригодный для дальнейшей работы (try- для отсутствия ошибки если топик будет пустой)
                img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)

            imgGrey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)                                                              # Наложения серого фильтра на изображение
            Grey = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            
            if self.Qr == True:
                barcodes  = pyzbar.decode(img)    # Распознование QR-кодов
                if barcodes:    # Если они на картинке есть
                    for bar in barcodes:       # Проходит по всем QR кодам, которые он нашел
                        self.land = bar.data.decode("utf-8") # Записывает в переменную информацию, находящуюся в данном коде
                    self.Qr = False

            elif self.Color == True:

                mask1_1 = cv2.inRange(Grey, self.potato_low, self.potato_high)                                                          # Создание облак точек для каждого цвета
                mask1_2 = cv2.inRange(Grey, self._potato_low, self._potato_high)
                mask2 = cv2.inRange(Grey, self.water_low, self.water_high)
                mask3 = cv2.inRange(Grey, self.seed_low, self.seed_high)
                mask4 = cv2.inRange(Grey, self.pastures_low, self.pastures_high)
                mask5 = cv2.inRange(Grey, self.soil_low, self.soil_high)
                
                res1 = cv2.bitwise_and(img, img, mask= mask1_1|mask1_2)                                                          # Метод для отображения облаков точек в цвете
                try:
                    self.image_pub_red.publish(self.bridge.cv2_to_imgmsg(res1, "bgr8"))                                      # Вывод в отдельный топик
                except CvBridgeError as e:
                    print(e)

                res2 = cv2.bitwise_and(img, img, mask= mask2)                                                                # И так с каждым цветом
                try:
                    self.image_pub_blue.publish(self.bridge.cv2_to_imgmsg(res2, "bgr8"))
                except CvBridgeError as e:
                    print(e)

                res3 = cv2.bitwise_and(img, img, mask= mask3)
                try:
                    self.image_pub_yellow.publish(self.bridge.cv2_to_imgmsg(res3, "bgr8"))
                except CvBridgeError as e:
                    print(e)
                
                res4 = cv2.bitwise_and(img, img, mask= mask4)                                                                # И так с каждым цветом
                try:
                    self.image_pub_blue.publish(self.bridge.cv2_to_imgmsg(res4, "bgr8"))
                except CvBridgeError as e:
                    print(e)

                res5 = cv2.bitwise_and(img, img, mask= mask5)
                try:
                    self.image_pub_yellow.publish(self.bridge.cv2_to_imgmsg(res5, "bgr8"))
                except CvBridgeError as e:
                    print(e)

                _, potato, hier = cv2.findContours(mask1_1|mask1_2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)                     # Поиск контуров в облаке точек (Красном)
                for c in potato:                                                                                                # Перебор каждого контура
                    try:    
                        #print(len(c))
                        y,x = 0,0
                        moments = cv2.moments(c, 1)                                                                   # Метод создающий матрицу объекта
                        sum_y = moments['m01']
                        sum_x = moments['m10']
                        sum_pixel = moments['m00']
                        approx = cv2.approxPolyDP(c, 0.01* cv2.arcLength(contour, True), True)
                        #cv2.drawContours(res, [approx], 0, (255, 255, 255), 2)
                        if sum_pixel > 20:
                            x = int(sum_x / sum_pixel)
                            y = int(sum_y / sum_pixel)
                            x_d = self.distance_x(x)/10
                            y_d = self.distance_y(y)/10
                            if len(approx) < 10:
                                cv2.putText(img, 'N3_Potato', (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))
                                self.ploh{self.Pole:[x_d,y_d]}
                                cv2.drawContours(img, [c], 0, (0, 0, 0), 2)
                            else:
                                self.lan{'Potato':[x_d,y_d]}
                    except:pass
                
                _, water, hier = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)                          # Тоже самое для синего 
                for c in water:
                    try:
                        approx = cv2.approxPolyDP(c, 0.01* cv2.arcLength(contour, True), True)
                        moments = cv2.moments(c, 1)
                        sum_y = moments['m01']
                        sum_x = moments['m10']
                        sum_pixel = moments['m00']
                        if sum_pixel > 20:
                            x = int(sum_x / sum_pixel)
                            y = int(sum_y / sum_pixel)
                            x_d = self.distance_x(x)/10
                            y_d = self.distance_y(y)/10
                            if len(approx) < 10:
                                cv2.putText(img, 'N3_Water', (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))
                                self.ploh{self.Pole:[x_d,y_d]}
                                cv2.drawContours(img, [c], 0, (0, 0, 0), 2)
                            else:
                                self.lan{'Water':[x_d,y_d]}
                    except:pass

                _, seed, hier = cv2.findContours(mask3, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)                        # И желтого
                for c in seed:
                    try:
                        approx = cv2.approxPolyDP(c, 0.01* cv2.arcLength(contour, True), True)
                        moments = cv2.moments(c, 1)
                        sum_y = moments['m01']
                        sum_x = moments['m10']
                        sum_pixel = moments['m00']
                        if sum_pixel > 20:
                            x = int(sum_x / sum_pixel)
                            y = int(sum_y / sum_pixel)
                            x_d = self.distance_x(x)/10
                            y_d = self.distance_y(y)/10
                            if len(approx) < 10:
                                cv2.putText(img, 'N3_Seed', (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))
                                self.ploh{self.Pole:[x_d,y_d]}
                                cv2.drawContours(img, [c], 0, (0, 0, 0), 2)
                            else:
                                self.lan{'Seed':[x_d,y_d]}
                    except:pass
                
                _, pastures, hier = cv2.findContours(mask4, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)                        # И желтого
                for c in pastures:
                    try:
                        approx = cv2.approxPolyDP(c, 0.01* cv2.arcLength(contour, True), True)
                        moments = cv2.moments(c, 1)
                        sum_y = moments['m01']
                        sum_x = moments['m10']
                        sum_pixel = moments['m00']
                        if sum_pixel > 20:
                            x = int(sum_x / sum_pixel)
                            y = int(sum_y / sum_pixel)
                            x_d = self.distance_x(x)/10
                            y_d = self.distance_y(y)/10
                            if len(approx) < 10:
                                cv2.putText(img, 'N3_Pastures', (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))
                                self.ploh{self.Pole:[x_d,y_d]}
                                cv2.drawContours(img, [c], 0, (0, 0, 0), 2)
                            else:
                                self.lan{'Pastures':[x_d,y_d]}
                    except:pass
                
                _, soil, hier = cv2.findContours(mask5, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)                        # И желтого
                for c in soil:
                    try:
                        approx = cv2.approxPolyDP(c, 0.01* cv2.arcLength(contour, True), True)
                        moments = cv2.moments(c, 1)
                        sum_y = moments['m01']
                        sum_x = moments['m10']
                        sum_pixel = moments['m00']
                        if sum_pixel > 20:
                            x = int(sum_x / sum_pixel)
                            y = int(sum_y / sum_pixel)
                            x_d = self.distance_x(x)/10
                            y_d = self.distance_y(y)/10
                            if len(approx) < 10:
                                cv2.putText(img, 'N3_Soil', (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))
                                self.ploh{self.Pole:[x_d,y_d]}
                                cv2.drawContours(img, [c], 0, (0, 0, 0), 2)
                            else:
                                self.lan{'Soil':[x_d,y_d]}
                    except:pass
                
                try:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))                                           # Вывод конвертипованного изображения
                except CvBridgeError as e:
                    print(e)
        print(self.lan)
        print(self.ploh)

def main(args):                                                                                                      # Начальная функция
  col_det = ColorDetecting()                                                                                         # Обращение к классу Color_detect
  try:
    rospy.spin()                                                                                                     # Обязательная функция для работы с топиками
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
