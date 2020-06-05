# -*- coding: utf-8 -*-
import numpy as np
import cv2
import time
import roslib
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ColorDetecting():                                                                                       # Класс для распознавание цветов - желтый, синий, красный
    def __init__(self):                                                                                       # Функция init содежит: 
        rospy.init_node('Color_detect', anonymous=True)                                                       # Создание ноды
        self.image_pub_red = rospy.Publisher("Red",Image,queue_size=10)                                       # Создание топиков с масками цветов
        self.image_pub_blue = rospy.Publisher("Blue",Image,queue_size=10)
        self.image_pub_yellow = rospy.Publisher("Yellow",Image,queue_size=10)
        self.image_pub = rospy.Publisher("Final",Image,queue_size=10)                                         # И топика для вывода финального изображения
        
        self.red_low = np.array([0,192,222])                                                                  # Параметры необходимые для определения облак точек каждого цвета:
        self.red_high = np.array([23,255,255])                                                                # Красного

        self.blue_low = np.array([90,51,79])                                                                  # Синего
        self.blue_high = np.array([149,255,255])

        self.yellow_low = np.array([19,151,186])                                                              # И желтого
        self.yellow_high = np.array([35,255,255])
        
        self.bridge = CvBridge()                                                                              # Переменная необходимая для конвертация изображения из типа msg в обычный вид и обратно
        self.image_sub = rospy.Subscriber("main_camera/image_raw",Image,self.callback)                        # Подписание на топик с изображением (Вставьте свой вместо main_camera/image_raw если у вас топик с изображением отличается)

    def callback(self,data):                                                                                  # Основная функция (data- изображения из типа msg)  
        try:                                                                                                  # Считывание и конвертация изображения в вид пригодный для дальнейшей работы (try- для отсутствия ошибки если топик будет пустой)
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")                      
        except CvBridgeError as e:
            print(e)
        
        imgGrey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)                                                       # Наложения серого фильтра на изображение
        Grey = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(Grey, self.red_low, self.red_high)                                                # Создание облак точек для каждого цвета
        mask2 = cv2.inRange(Grey, self.blue_low, self.blue_high)
        mask3 = cv2.inRange(Grey, self.yellow_low, self.yellow_high)

        res1 = cv2.bitwise_and(img, img, mask= mask1)                                                         # Метод для отображения облака точек в цвете
        cv2.imshow('Red',res1)                                                                                # Вывод в отдельное окно (Нужно закоментить в случае отсутствия дисплея)
        try:
            self.image_pub_red.publish(self.bridge.cv2_to_imgmsg(res1, "bgr8"))                               # Вывод в отдельный топик
        except CvBridgeError as e:
            print(e)

        res2 = cv2.bitwise_and(img, img, mask= mask2)                                                         # И так с каждым цветом
        cv2.imshow('Blue',res2)
        try:
            self.image_pub_blue.publish(self.bridge.cv2_to_imgmsg(res2, "bgr8"))
        except CvBridgeError as e:
            print(e)

        res3 = cv2.bitwise_and(img, img, mask= mask3)
        cv2.imshow('Yellow',res3)
        try:
            self.image_pub_yellow.publish(self.bridge.cv2_to_imgmsg(res3, "bgr8"))
        except CvBridgeError as e:
            print(e)

        red = cv2.findContours(mask1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)                                   # Поиск контуров в облаке точек (Красном)
        for c in red[0]:                                                                                      # Перебор каждого контура
            try:
                y,x = 0,0
                moments = cv2.moments(c, 1)                                                                   # Метод создающий матрицу объекта
                sum_y = moments['m01']
                sum_x = moments['m10']
                sum_pixel = moments['m00']
                if sum_pixel > 100:                                                                           # Отсеивание помех
                    x = int(sum_x / sum_pixel)                                                                # Определение центра объекта
                    y = int(sum_y / sum_pixel)
                    cv2.drawContours(img, [c], 0, (0, 0, 0), 5)                                               # Обвод контуров и вывод на изображение его цвет
                    cv2.putText(img, 'red', (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))
            except:pass
            
        blue = cv2.findContours(mask2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)                                  # Тоже самое для синего   
        mas2,m = [],0
        for c in blue[0]:
            try:
                moments = cv2.moments(c, 1)
                sum_y = moments['m01']
                sum_x = moments['m10']
                sum_pixel = moments['m00']
                
                if sum_pixel > 100:
                    m += sum_pixel                                                                            # Только добавлен подсчет площади (+= так как я нахожу площадь каждой фигуры по отдельности и потом суммирую их)
                    x = int(sum_x / sum_pixel)
                    y = int(sum_y / sum_pixel)
                    cv2.drawContours(img, [c], 0, (0, 0, 0), 5)    
                    mas2.append([x,y])                                                                        # Добавление каждой фигуры для подсчета колличества (С сохранением координат объектов)
                    cv2.putText(img, 'blue', (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))
            except:pass
        print('Blue elements: ', len(mas2))                                                                   # Вывод колличества и площади для синих в консоль
        print('Blue area: ',m,' pix')
        cv2.putText(img, ('Blue elements: '+str(len(mas2))), (60, 60), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))# И в финальное изображение (В левом верхнем угле) 
        cv2.putText(img, ('Blue area: '+str(int(m))+'pix'), (60, 90), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))


        yellow = cv2.findContours(mask3,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) # И желтого
        for c in yellow[0]:
            try:
                moments = cv2.moments(c, 1)
                sum_y = moments['m01']
                sum_x = moments['m10']
                sum_pixel = moments['m00']
                if sum_pixel > 100:
                    x = int(sum_x / sum_pixel)
                    y = int(sum_y / sum_pixel)
                    cv2.drawContours(img, [c], 0, (0, 0, 0), 5)    
                    cv2.putText(img, 'yellow', (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))
            except:pass
        
        cv2.imshow("Final", img)                                                                              # Вывод финального изображения на дисплей (Так же закоментить есди отсутствует дисплей) 
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))                                    # Вывод конвертипованного изображения
        except CvBridgeError as e:
            print(e)        
        
def main(args):                                                                                               # Начальная функция
  col_det = ColorDetecting()                                                                                  # Обращение к классу Color_detect
  try:
    rospy.spin()                                                                                              # Обезательная функция для работы с топиками
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
