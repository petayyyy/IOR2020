# -*- coding: utf-8 -*-
import numpy as np
import math
import cv2
import time
import roslib
import sys
import rospy
from pyzbar import pyzbar
from sensor_msgs.msg import Image
import threading
from cv_bridge import CvBridge, CvBridgeError

from clever import srv
from std_srvs.srv import Trigger

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)


def point(mas, text):
    global mark,b
    for i in range (len(mas)):
        for j in (i+1,len(mas)):
            if j+1 != len(mas):
                break
            if math.sqrt(abs(mas[i][0] - mas[j][0])**2 + abs(mas[i][1] - mas[j][1])**2) <= b:
                mas[i][0] = (mas[i][0] + mas[j][0])/2
                mas[i][1] = (mas[i][1] + mas[j][1])/2
                del mas[j]
            if j+1 != len(mas):
                break
        if mas[i][0] <= 2 and mas[i][1] <= 2:
            mark['C'].append([text,mas[i][0],mas[i][1]])
        elif mas[i][0] > 2 and mas[i][1] < 2:
            mark['D'].append([text,mas[i][0],mas[i][1]])
        elif mas[i][0] > 2 and mas[i][1] > 2:
            mark['B'].append([text,mas[i][0],mas[i][1]])
        elif mas[i][0] < 2 and mas[i][1] > 2:
            mark['A'].append([text,mas[i][0],mas[i][1]])
        if i+1 != len(mas):
            break

class ColorDetecting():                                                                                              # Класс для распознавание цветов - желтый, синий, красный
    def __init__(self):                                                                                              # Функция init содежит:
        rospy.init_node('Color_detect', anonymous=True)                                                              # Создание ноды
        self.image_pub = rospy.Publisher("Final",Image,queue_size=10)                                                # И топика для вывода финального изображения

        self.potato_low = np.array([0,140,170])                                                                         # Параметры необходимые для определения облака точек каждого цвета:
        self.potato_high = np.array([20, 255, 255])                                                                     # Красного

        self._potato_low = np.array([10,140,255])                                                                           # Доп фильтр для красного цвета
        self._potato_high = np.array([20,255,255])

        
        self.water_low = np.array([111, 79, 132])                                                                       # Синего
        self.water_high = np.array([120, 171, 241])

        self.seed_low = np.array([0,49,130])                                                                      # И желтого
        self.seed_high = np.array([63,255,255])

        self.pastures_low = np.array([42,146,32])                                                                      # И желтого
        self.pastures_high = np.array([100,255,79])

        self.soil_low = np.array([0,30,130])                                                                      # И желтого
        self.soil_high = np.array([9,62,150])
        
        self._soil_low = np.array([153,12,130])                                                                      # И желтого
        self._soil_high = np.array([185,35,140])
        
        self.Qr = False
        self.Color = True
        self.land = ''
        self.ploh = {'Water':[],'Seed':[],'Pastures':[],'Soil':[],'Potato':[]}
        self.lan = {'Water':[],'Seed':[],'Pastures':[]}
        self.bridge = CvBridge()                                                                                     # Переменная необходимая для конвертации изображения из типа msg в обычный вид и обратно
        self.image_sub = rospy.Subscriber("main_camera/image_raw",Image,self.callback)                               # Подписание на топик с изображением
    def distance_x(self,x,z):
        if x >= 160:
            return int((x - 160)*0.035445511372610664 * z)/10
        else:
            return -int((160 - x)*0.035445511372610664 * z)/10
    def distance_y(self,y,z):
        if y >= 120:
            return -int((y - 120)*0.01443375672974064 * z)/10
        else:
            return int((120 - y)*0.01443375672974064 * z)/10
    def callback(self,data):                                                                                         # Основная функция (data- изображения из типа msg)
        if self.Color == True or self.Qr == True:
            try:                                                                                                         # Считывание и конвертация изображения в вид пригодный для дальнейшей работы (try- для отсутствия ошибки если топик будет пустой)
                img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except:pass
            
            start = get_telemetry(frame_id='aruco_map')
            Grey = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            if self.Qr == True:
                barcodes  = pyzbar.decode(img)    # Распознование QR-кодов
                if barcodes:    # Если они на картинке есть
                    for bar in barcodes:       # Проходит по всем QR кодам, которые он нашел
                        self.land = (bar.data.decode("utf-8")).lower() # Записывает в переменную информацию, находящуюся в данном коде
                    self.Qr = False
            if self.Color == True:
                mask1_1 = cv2.inRange(Grey, self.potato_low, self.potato_high)                                                          # Создание облак точек для каждого цвета
                mask1_2 = cv2.inRange(Grey, self._potato_low, self._potato_high)
                mask2 = cv2.inRange(Grey, self.water_low, self.water_high)
                mask3 = cv2.inRange(Grey, self.seed_low, self.seed_high)
                mask4 = cv2.inRange(Grey, self.pastures_low, self.pastures_high)
                mask5 = cv2.inRange(Grey, self.soil_low, self.soil_high)
                mask6 = cv2.inRange(Grey, self._soil_low, self._soil_high)
                _, potato, hier = cv2.findContours(mask1_1|mask1_2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)                     # Поиск контуров в облаке точек (Красном)
                for c in potato:                                                                                                # Перебор каждого контура
                    try:
                        #print(len(c))
                        y,x = 0,0
                        moments = cv2.moments(c, 1)                                                                   # Метод создающий матрицу объекта
                        sum_y = moments['m01']
                        sum_x = moments['m10']
                        sum_pixel = moments['m00']
                        if sum_pixel > 20:
                            x = int(sum_x / sum_pixel)
                            y = int(sum_y / sum_pixel)
                            x_d = self.distance_x(x,start.z)/10
                            y_d = self.distance_y(y,start.z)/10
                            cv2.putText(img, 'N3_Potato', (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))
                            self.ploh['Potato'].append([start.x+x_d,start.y+y_d])
                            cv2.drawContours(img, [c], 0, (0, 0, 0), 2)
                    except:pass

                _, water, hier = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)                          # Тоже самое для синего                
                for c in water:
                    try:
                        approx = cv2.approxPolyDP(c, 0.01* cv2.arcLength(c, True), True)
                        moments = cv2.moments(c, 1)
                        sum_y = moments['m01']
                        sum_x = moments['m10']
                        sum_pixel = moments['m00']
                        if sum_pixel > 10:
                            x = int(sum_x / sum_pixel)
                            y = int(sum_y / sum_pixel)
                            x_d = self.distance_x(x,start.z)
                            y_d = self.distance_y(y,start.z)
                            print(x_d, y_d)
                            print(len(approx))
                            cv2.drawContours(img, [c], 0, (0, 0, 0), 1)
                            #if len(approx) < 10:
                            cv2.putText(img, 'N3_Water', (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))
                            self.ploh['Water'].append([start.x+x_d,start.y+y_d])
                            #else:
                            #    self.lan['water'].append([start.x+x_d,start.y+y_d])
                    except:pass

                _, seed, hier = cv2.findContours(mask3, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)                        # И желтого
                for c in seed:
                    try:
                        approx = cv2.approxPolyDP(c, 0.01* cv2.arcLength(c, True), True)
                        moments = cv2.moments(c, 1)
                        sum_y = moments['m01']
                        sum_x = moments['m10']
                        sum_pixel = moments['m00']
                        if sum_pixel > 10:
                            x = int(sum_x / sum_pixel)
                            y = int(sum_y / sum_pixel)
                            x_d = self.distance_x(x,start.z)/10
                            y_d = self.distance_y(y,start.z)/10
                            print(x_d,y_d)
                            print(len(approx))
                            #if len(approx) < 10:
                            cv2.drawContours(img, [c], 0, (0, 0, 0), 2)
                            cv2.putText(img, 'N3_Seed', (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))
                            self.ploh['Seed'].append([start.x+x_d,start.y+y_d])
                            #else:
                            #    self.lan['seed'].append([start.x+x_d,start.y+y_d])
                    except:pass

                _, pastures, hier = cv2.findContours(mask4, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)                        # И желтого
                for c in pastures:
                    try:
                        approx = cv2.approxPolyDP(c, 0.01* cv2.arcLength(c, True), True)
                        moments = cv2.moments(c, 1)
                        sum_y = moments['m01']
                        sum_x = moments['m10']
                        sum_pixel = moments['m00']
                        if sum_pixel > 10:
                            x = int(sum_x / sum_pixel)
                            y = int(sum_y / sum_pixel)
                            x_d = self.distance_x(x,start.z)/10
                            y_d = self.distance_y(y,start.z)/10
                            if len(approx) < 10:
                                cv2.putText(img, 'N3_Pastures', (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))
                                self.ploh['Pastures'].append([start.x+x_d,start.y+y_d])
                                cv2.drawContours(img, [c], 0, (0, 0, 0), 2)
                            else:
                                self.lan['pastures'].append([start.x+x_d,start.y+y_d])
                    except:pass

                _, soil, hier = cv2.findContours(mask5|mask6, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)                        # И желтого
                for c in soil:
                    try:
                        moments = cv2.moments(c, 1)
                        sum_y = moments['m01']
                        sum_x = moments['m10']
                        sum_pixel = moments['m00']
                        if sum_pixel > 10:
                            x = int(sum_x / sum_pixel)
                            y = int(sum_y / sum_pixel)
                            x_d = self.distance_x(x,start.z)/10
                            y_d = self.distance_y(y,start.z)/10
                            cv2.putText(img, 'N3_Soil', (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))
                            self.ploh['Soil'].append([start.x+x_d,start.y+y_d])
                            cv2.drawContours(img, [c], 0, (0, 0, 0), 2)
                    except:pass

                try:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))                                           # Вывод конвертипованного изображения
                except CvBridgeError as e:
                    print(e)
def main():                                                                                                      # Начальная функция
  global col_det
  col_det = ColorDetecting()                                                                                         # Обращение к классу Color_detect

main()
print('sleep')
#time.sleep(10)
print('ready')
print navigate(x=0, y=0, z=1, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(3)
print navigate(x=1.5, y=1, z=1.5, speed=0.5, yaw=math.radians(90),frame_id='aruco_map')
col_det.Qr = True
print('Захар делай скрин')
rospy.sleep(6)
print('Qr detect:' + col_det.land)
print navigate(x=1.5, y=1.5, z=1.5, speed=0.5, yaw=math.radians(90),frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(6)
print navigate(x=1.5, y=2, z=1.5, speed=0.5, yaw=math.radians(90),frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(6)
print navigate(x=1.5, y=3, z=1.5, speed=0.5, yaw=math.radians(90),frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(6)
print navigate(x=2, y=3, z=1.5, speed=0.5, yaw=math.radians(90),frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(6)
print navigate(x=2, y=2, z=1.5, speed=0.5, yaw=math.radians(90),frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(6)
print navigate(x=2, y=1, z=1.5, speed=0.5, yaw=math.radians(90),frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(6)
print navigate(x=2, y=0.3, z=1.5, speed=0.5, yaw=math.radians(90),frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(6)
print navigate(x=1, y=0, z=1.5, speed=0.5, yaw=math.radians(90),frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(6)

print('Qr detect:' + col_det.land)
if col_det.land in col_det.lan:
    x1 = col_det.lan[col_det.Qr][0]
    y1 = col_det.lan[col_det.Qr][1]
    print navigate(x=x1, y=y1, z=1, speed=0.5, yaw=math.radians(90),frame_id='aruco_map')
else:
    print navigate(x=1, y=1, z=1, speed=0.5, yaw=math.radians(90),frame_id='aruco_map')
rospy.sleep(10)

land()
mas = col_det.ploh['Water']
b = 1
mark = {'A':[],'B':[],'C':[],'D':[]}
   
point(col_det.ploh['Water'],'Water')
point(col_det.ploh['Potato'],'Potato')
point(col_det.ploh['Pastures'],'Pastures')
point(col_det.ploh['Soil'],'Soil')
point(col_det.ploh['Seed'],'Seed')

print('Сектор       Тип территории      Координаты (см) от центра')
print('                                        x           y')
for i in mark:
    if len(mark[i])>0:
        for j in mark[i]:
            print("{}             {}                    {}        {}".format(i, j[0], j[1], j[2]))
