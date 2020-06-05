# -*- coding: utf-8 -*-
import numpy as np
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
        self.image_pub_red = rospy.Publisher("Red",Image,queue_size=10)                                              # Создание топиков с масками цветов
        self.image_pub_blue = rospy.Publisher("Blue",Image,queue_size=10)
        self.image_pub_yellow = rospy.Publisher("Yellow",Image,queue_size=10)
        self.image_pub = rospy.Publisher("Final",Image,queue_size=10)                                                # И топика для вывода финального изображения

        self.red_low = np.array([170,70,50])                                                                         # Параметры необходимые для определения облак точек каждого цвета:
        self.red_high = np.array([180, 255, 255])                                                                    # Красного

        self._red_low = np.array([0,70,50])                                                                          # Доп фильтр для красного цвета 
        self._red_high = np.array([10, 255,255])

        self.blue_low = np.array([93,76,153])                                                                        # Синего
        self.blue_high = np.array([153,255,255])

        self.yellow_low = np.array([19,69,157])                                                                     # И желтого
        self.yellow_high = np.array([51,255,229])

        self.bridge = CvBridge()                                                                                     # Переменная необходимая для конвертация изображения из типа msg в обычный вид и обратно
        self.image_sub = rospy.Subscriber("main_camera/image_raw",Image,self.callback)                               # Подписание на топик с изображением

    def callback(self,data):                                                                                         # Основная функция (data- изображения из типа msg)
        try:                                                                                                         # Считывание и конвертация изображения в вид пригодный для дальнейшей работы (try- для отсутствия ошибки если топик будет пустой)
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        imgGrey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)                                                              # Наложения серого фильтра на изображение
        Grey = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(Grey, self.red_low, self.red_high)                                                       # Создание облак точек для каждого цвета
        mask3 = cv2.inRange(Grey, self.yellow_low, self.yellow_high)
        mask2 = cv2.inRange(Grey, self.blue_low, self.blue_high)
        mask4 = cv2.inRange(Grey, self._red_low, self._red_high)
        
        res1 = cv2.bitwise_and(img, img, mask= mask1|mask4)                                                          # Метод для отображения облака точек в цвете
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

        _, red, hier = cv2.findContours(mask1|mask4, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)                     # Поиск контуров в облаке точек (Красном)
        for c in red:                                                                                                # Перебор каждого контура
                y,x = 0,0
                moments = cv2.moments(c, 1)                                                                          # Метод создающий матрицу объекта
                sum_y = moments['m01']
                sum_x = moments['m10']
                sum_pixel = moments['m00']
                if sum_pixel > 20:                                                                                   # Отсеивание помех
                    x = int(sum_x / sum_pixel)                                                                       # Определение центра объекта
                    y = int(sum_y / sum_pixel)
                    cv2.drawContours(img, [c], 0, (0, 0, 0), 2)                                                      # Обвод контуров и вывод на изображение его цвет
                    cv2.putText(img, 'red', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0))


        _, blue, hier = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)                          # Тоже самое для синего 
        mas2,m = [],0
        for c in blue:
         try:
                moments = cv2.moments(c, 1)
                sum_y = moments['m01']
                sum_x = moments['m10']
                sum_pixel = moments['m00']
                try:
                    m += sum_pixel                                                                                   # Только добавлен подсчет площади (+= так как я нахожу площадь каждой фигуры по отдельности и потом суммирую их)
                    x = int(sum_x / sum_pixel)
                    y = int(sum_y / sum_pixel)
                    cv2.drawContours(img, [c], 0, (0, 0, 0), 2)
                    mas2.append([x,y])                                                                               # Добавление каждой фигуры для подсчета колличества (С сохранением координат объектов)
                    cv2.putText(img, 'blue', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0))
                except:pass
         except:pass
        print('Blue elements: ', len(mas2))                                                                          # Вывод колличества и площади для синих фигур в консоль
        print('Blue area: ',m,' pix')
        cv2.putText(img, ('Blue elements: '+str(len(mas2))), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255)) # И в финальное изображение (В левом верхнем угле)
        cv2.putText(img, ('Blue area: '+str(int(m))+'pix'), (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255))


        _, yellow, hier = cv2.findContours(mask3, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)                        # И желтого
        for c in yellow:
            try:
                moments = cv2.moments(c, 1)
                sum_y = moments['m01']
                sum_x = moments['m10']
                sum_pixel = moments['m00']
                if sum_pixel > 20:
                    x = int(sum_x / sum_pixel)
                    y = int(sum_y / sum_pixel)
                    cv2.drawContours(img, [c], 0, (0, 0, 0), 2)
                    cv2.putText(img, 'yellow', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0))
            except:pass

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))                                           # Вывод конвертипованного изображения
        except CvBridgeError as e:
            print(e)

def main(args):                                                                                                      # Начальная функция
  col_det = ColorDetecting()                                                                                         # Обращение к классу Color_detect
  try:
    rospy.spin()                                                                                                     # Обезательная функция для работы с топиками
  except KeyboardInterrupt:
    print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
