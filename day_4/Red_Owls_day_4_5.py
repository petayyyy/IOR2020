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


def intNumber(x):
    return int(x) != x

def point(mas, text):
    global mark,b
    i = 0
    while i < len(mas):
        j = i+1
        while j < len(mas):
            if math.sqrt(abs(mas[i][0] - mas[j][0])**2 + abs(mas[i][1] - mas[j][1])**2) <= b:
                mas[i][0] = (mas[i][0] + mas[j][0])//2
                mas[i][1] = (mas[i][1] + mas[j][1])//2
                del mas[j]
            j += 1
        if mas[i][0] <= 200 and mas[i][1] <= 200 and intNumber(mas[i][0]) and intNumber(mas[i][1]) and mas[i][0] > 0 and mas[i][1] > 0:
            mark['C'].append([text,mas[i][0],mas[i][1]])
        elif mas[i][0] > 200 and mas[i][1] < 200 and intNumber(mas[i][0]) and intNumber(mas[i][1]) and mas[i][0] < 300 and mas[i][1] > 0:
            mark['D'].append([text,mas[i][0],mas[i][1]])
        elif mas[i][0] > 200 and mas[i][1] > 200 and intNumber(mas[i][0]) and intNumber(mas[i][1]) and mas[i][0] < 300 and mas[i][1] < 300:
            mark['B'].append([text,mas[i][0],mas[i][1]])
        elif mas[i][0] < 200 and mas[i][1] > 200 and intNumber(mas[i][0]) and intNumber(mas[i][1]) and mas[i][0] > 0 and mas[i][1] < 300:
            mark['A'].append([text,mas[i][0],mas[i][1]])
        i += 1
def point_pos(mas):
    global mark_pos
    i = 0
    while i < len(mas):
        j = i+1
        while j < len(mas):
            if math.sqrt(abs(mas[i][0] - mas[j][0])**2 + abs(mas[i][1] - mas[j][1])**2) <= 100:
                mas[i][0] = (mas[i][0] + mas[j][0])//2
                mas[i][1] = (mas[i][1] + mas[j][1])//2
                del mas[j]
            j += 1
        if mas[i][0] <= 300 and mas[i][1] <= 300 and intNumber(mas[i][0]) and intNumber(mas[i][1]) and mas[i][0] > 0 and mas[i][1] > 0:
            mark_pos.append([mas[i][0]/100,mas[i][1]]/100)
        i += 1


class ColorDetecting():                                                                                              # Класс для распознавание цветов - желтый, синий, красный
    def __init__(self):                                                                                              # Функция init содежит:
        rospy.init_node('Color_detect', anonymous=True)                                                              # Создание ноды
        self.image_pub = rospy.Publisher("Final",Image,queue_size=10)                                                # И топика для вывода финального изображения

        self.potato_low = np.array([160, 165, 80])                                                                       # Параметры необходимые для определения облака точек каждого цвета:
        self.potato_high =  np.array([180, 210, 185])                                                                    # Красного
      
        self.water_low = np.array([106,65,62])                                                                       # Синего
        self.water_high = np.array([130,255,255])

        self.seed_low = np.array([15, 65, 90])                                                                     # И желтого
        self.seed_high = np.array([41, 220, 240])

        self.pastures_low = np.array([65,86,42])                                                                     # И желтого
        self.pastures_high = np.array([95,255,99])

        self.soil_low = np.array([0,90,55])                                                                   # И желтого
        self.soil_high =np.array([15,165,150])
        
        self.out = cv2.VideoWriter('Scinti_pogalyista.avi',cv2.VideoWriter_fourcc('X','V','I','D'), 20, (320,240))
        self.Qr = True
        self.Color = False
        self.land = ''
        self.koord = []
        self.ploh = {'Water':[],'Seed':[],'Pastures':[],'Soil':[],'Potato':[]}
        self.lan = {'water':[],'seed':[],'pastures':[]}
        self.bridge = CvBridge()                                                                                     # Переменная необходимая для конвертации изображения из типа msg в обычный вид и обратно
        self.image_sub = rospy.Subscriber("main_camera/image_raw_throttled",Image,self.callback)                               # Подписание на топик с изображением
    def distance_x(self,x,z):
        if x >= 160:
            #return ((x - 160)*0.00625 * z) # 45
            return ((x - 160)*0.00524437269 * z) # 40
        else:
            #return -((160 - x)*0.00625 * z)
            return -((160 - x)*0.00524437269 * z)
    def distance_y(self,y,z):
        if y >= 120:
            #return -((y - 120)*0.00583506281 * z) # 35
            return -((y - 120)*0.00481125224 * z) # 30
        else:
            #return ((120 - y)*0.00583506281 * z)
            return ((120 - y)*0.00481125224 * z)
    def callback(self,data):                                                                                         # Основная функция (data- изображения из типа msg)
        if self.Color == True or self.Qr == True:
            try:                                                                                                         # Считывание и конвертация изображения в вид пригодный для дальнейшей работы (try- для отсутствия ошибки если топик будет пустой)
                img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except:pass
            start = get_telemetry(frame_id='aruco_map')
            img = cv2.undistort( img,np.array([[166.23942373073172,0,162.19011246829268],[0,166.5880923974026,109.82227735714285],[0,0,1]]), np.array([ 2.15356885e-01,  -1.17472846e-01,  -3.06197672e-04,-1.09444025e-04,  -4.53657258e-03,   5.73090623e-01,-1.27574577e-01,  -2.86125589e-02,   0.00000000e+00,0.00000000e+00,   0.00000000e+00,   0.00000000e+00,0.00000000e+00,   0.00000000e+00]),np.array([[166.23942373073172,0,162.19011246829268],[0,166.5880923974026,109.82227735714285],[0,0,1]]))
            Grey = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            self.out.write(img)
            
            if self.Qr == True:
                barcodes  = pyzbar.decode(img)    # Распознование QR-кодов
                if barcodes:    # Если они на картинке есть
                    for bar in barcodes:       # Проходит по всем QR кодам, которые он нашел
                        self.land = (bar.data.decode("utf-8")).lower() # Записывает в переменную информацию, находящуюся в данном коде
                        print('Qr detect:' + self.land)
                    self.Qr = False
            if self.Color == True:
                mask1_1 = cv2.inRange(Grey, self.potato_low, self.potato_high)                                                          # Создание облак точек для каждого цвета
                mask2 = cv2.inRange(Grey, self.water_low, self.water_high)
                mask3 = cv2.inRange(Grey, self.seed_low, self.seed_high)
                mask4 = cv2.inRange(Grey, self.pastures_low, self.pastures_high)
                mask5 = cv2.inRange(Grey, self.soil_low, self.soil_high)
                
                st1 = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 21), (10, 10))
                st2 = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11), (5, 5))
                
                thresh = cv2.morphologyEx(mask1_1, cv2.MORPH_CLOSE, st1)
                thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, st2)
                _, potato, hier = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)                     # Поиск контуров в облаке точек (Красном)
                for c in potato:                                                                                                # Перебор каждого контура
                    try:
                        #print(len(c))
                        y,x = 0,0
                        moments = cv2.moments(c, 1)                                                                   # Метод создающий матрицу объекта
                        sum_y = moments['m01']
                        sum_x = moments['m10']
                        sum_pixel = moments['m00']
                        if sum_pixel > 1000:
                            x = int(sum_x / sum_pixel)
                            y = int(sum_y / sum_pixel)
                            x_d = self.distance_x(x,start.z*100)
                            y_d = self.distance_y(y,start.z*100)
                            cv2.putText(img, 'N3_Potato', (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                            self.ploh['Potato'].append([round(start.x*100+x_d,2),round(start.y*100+y_d,2)])
                            cv2.drawContours(img, [c], 0, (193,91,154), 2)
                    except:pass
                
                thresh = cv2.morphologyEx(mask2, cv2.MORPH_CLOSE, st1)
                thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, st2)
                _, water, hier = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)                          # Тоже самое для синего                
                for c in water:
                    try:
                        approx = cv2.approxPolyDP(c, 0.01* cv2.arcLength(c, True), True)
                        moments = cv2.moments(c, 1)
                        sum_y = moments['m01']
                        sum_x = moments['m10']
                        sum_pixel = moments['m00']
                        if sum_pixel > 1000:
                            x = int(sum_x / sum_pixel)
                            y = int(sum_y / sum_pixel)
                            x_d = self.distance_x(x,start.z*100)
                            y_d = self.distance_y(y,start.z*100)
                            if math.sqrt(x_d**2+y_d**2) < 150:
                                if len(approx) < 5:
                                    cv2.drawContours(img, [approx], 0, (193,91,154), 2)
                                    cv2.putText(img, 'N3_Water', (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                                    self.ploh['Water'].append([round(start.x*100+x_d,2),round(start.y*100+y_d,2)])
                                    self.koord = []
                                else:
                                    if self.land == 'water':
                                        self.koord = [round(start.x*100+x_d,2)/100,round(start.y*100+y_d,2)/100]
                                    self.lan['water'].append([round(start.x*100+x_d,2),round(start.y*100+y_d,2)])
                                    #cv2.putText(img, str(len(approx)), (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                                #cv2.putText(img, str(start.x+x_d)+' '+str(start.y+y_d), (x, y+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
                    except:pass
                
                thresh = cv2.morphologyEx(mask3, cv2.MORPH_CLOSE, st1)
                thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, st2)
                _, seed, hier = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)                        # И желтого
                for c in seed:
                    try:
                        approx = cv2.approxPolyDP(c, 0.01* cv2.arcLength(c, True), True)
                        moments = cv2.moments(c, 1)
                        sum_y = moments['m01']
                        sum_x = moments['m10']
                        sum_pixel = moments['m00']
                        if sum_pixel > 1000:
                            x = int(sum_x / sum_pixel)
                            y = int(sum_y / sum_pixel)
                            x_d = self.distance_x(x,start.z*100)
                            y_d = self.distance_y(y,start.z*100)
                            if math.sqrt(x_d**2+y_d**2) < 150:
                                if len(approx) < 7:
                                    cv2.drawContours(img, [approx], 0, (193,91,154), 2)
                                    cv2.putText(img, 'N3_Seed', (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                                    self.ploh['Seed'].append([round(start.x*100+x_d,2),round(start.y*100+y_d,2)])
                                    self.koord = []
                                else:
                                    if self.land == 'seed':
                                        self.koord = [round(start.x*100+x_d,2)/100,round(start.y*100+y_d,2)/100]
                                    self.lan['seed'].append([round(start.x*100+x_d,2),round(start.y*100+y_d,2)])
                                    #cv2.putText(img, str(len(approx)), (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                                #cv2.putText(img, str(start.x+x_d)+' '+str(start.y+y_d), (x, y+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
                    except:pass
                
                thresh = cv2.morphologyEx(mask4, cv2.MORPH_CLOSE, st1)
                thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, st2)
                _, pastures, hier = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)                        # И желтого
                for c in pastures:
                    try:
                        approx = cv2.approxPolyDP(c, 0.01* cv2.arcLength(c, True), True)
                        moments = cv2.moments(c, 1)
                        sum_y = moments['m01']
                        sum_x = moments['m10']
                        sum_pixel = moments['m00']
                        if sum_pixel > 1000:
                            x = int(sum_x / sum_pixel)
                            y = int(sum_y / sum_pixel)
                            x_d = self.distance_x(x,start.z*100)
                            y_d = self.distance_y(y,start.z*100)
                            if math.sqrt(x_d**2+y_d**2) < 150:
                                if len(approx) < 8:
                                    cv2.drawContours(img, [approx], 0, (193,91,154), 2)
                                    cv2.putText(img, 'N3_Pastures', (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                                    self.ploh['Pastures'].append([round(start.x*100+x_d,2),round(start.y*100+y_d,2)])
                                    self.koord = []
                                else:
                                    if self.land == 'pastures':
                                        self.koord = [round(start.x*100+x_d,2)/100,round(start.y*100+y_d,2)/100]
                                    self.lan['pastures'].append([round(start.x*100+x_d,2),round(start.y*100+y_d,2)])
                                    #cv2.putText(img, str(len(approx)), (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                                #cv2.putText(img, str(start.x+x_d)+' '+str(start.y+y_d), (x, y+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))

                    except:pass
                
                thresh = cv2.morphologyEx(mask5, cv2.MORPH_CLOSE, st1)
                thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, st2)
                _, soil, hier = cv2.findContours(thresh, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)                        # И желтого
                for c in soil:
                    try:
                        moments = cv2.moments(c, 1)
                        sum_y = moments['m01']
                        sum_x = moments['m10']
                        sum_pixel = moments['m00']
                        if sum_pixel > 1000:
                            x = int(sum_x / sum_pixel)
                            y = int(sum_y / sum_pixel)
                            x_d = self.distance_x(x,start.z*100)
                            y_d = self.distance_y(y,start.z*100)
                            cv2.putText(img, 'N3_Soil', (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                            self.ploh['Soil'].append([round(start.x*100+x_d,2),round(start.y*100+y_d,2)])
                            cv2.drawContours(img, [c], 0, (193,91,154), 2)
                    except:pass

                try:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))                                           # Вывод конвертипованного изображения
                except CvBridgeError as e:
                    print(e)
                #self.Color = False
def main():                                                                                                      # Начальная функция
  global col_det
  col_det = ColorDetecting()                                                                                         # Обращение к классу Color_detect

main()
print('ready')

print navigate(x=0, y=0, z=1, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(3)

print navigate(x=0.5, y=0.3, z=1.2, speed=0.25, yaw=math.radians(90), frame_id='aruco_map')
rospy.sleep(5)
col_det.Qr = True
print('Qr detect:' + col_det.land)
print('Захар делай скрин')
col_det.Color = True

print navigate(x=0.5, y=2.8, z=1.2, speed=0.25, yaw=math.radians(90), frame_id='aruco_map')
rospy.sleep(14)
col_det.Color = False

print navigate(x=1, y=2.8, z=1.2, speed=0.25, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
col_det.Color = True

print navigate(x=1, y=0.3, z=1.2, speed=0.25, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(14)
col_det.Color = False

print navigate(x=1.5, y=0.3, z=1.2, speed=0.25, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
col_det.Color = True

print navigate(x=1.5, y=2.8, z=1.2, speed=0.25, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(14)
col_det.Color = False

print navigate(x=2, y=2.8, z=1.2, speed=0.25, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
col_det.Color = True

print navigate(x=2, y=0.3, z=1.2, speed=0.25, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(14)
col_det.Color = False

print navigate(x=2.5, y=0.3, z=1.2, speed=0.25, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
col_det.Color = True

print navigate(x=2.5, y=2.8, z=1.2, speed=0.25, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(14)
col_det.Color = False

print('Qr detect:' + col_det.land)
mark_pos = []
print(col_det.lan,'col_det.lan')

if col_det.land == '':
    print navigate(x=0.5, y=0.3, z=1.2, speed=0.25, yaw=math.radians(90), frame_id='aruco_map')
    rospy.sleep(10)
    col_det.Qr = True 
    rospy.sleep(3)
    print('Qr detect:' + col_det.land)  
if col_det.land in col_det.lan:
    point_pos(col_det.lan[col_det.land])
    print(mark_pos,'mark_pos')
    for i in range(len(mark_pos)):
        x1 = mark_pos[i][0]
        y1 = mark_pos[i][1]
        print navigate(x=x1, y=y1, z=1, speed=0.5, yaw=math.radians(90),frame_id='aruco_map')
        rospy.sleep(10)
        col_det.Color = True
        rospy.sleep(1)
        if len(col_det.koord) > 1:
            print navigate(x=col_det.koord[0], y=col_det.koord[1], z=1, speed=0.5, yaw=math.radians(90),frame_id='aruco_map')
            rospy.sleep(5)
            col_det.Color = False
            break
    
else:
    print navigate(x=1, y=1, z=1, speed=0.5, yaw=math.radians(90),frame_id='aruco_map')
    rospy.sleep(13)

land()
b = 40
mark = {'A':[],'B':[],'C':[],'D':[]}
print('Подождите обработку данных') 

point(col_det.ploh['Water'],'Water')
point(col_det.ploh['Potato'],'Potato')
point(col_det.ploh['Pastures'],'Pastures')
point(col_det.ploh['Soil'],'Soil')
point(col_det.ploh['Seed'],'Seed')

f = open('Red_Owls_avt_othet.txt', 'w')
print('Сектор       Тип территории      Координаты (см) от центра')
print('                                        x           y')
f.write('Сектор       Тип территории      Координаты (см) от центра\n')
f.write('                                        x           y\n')
for i in ['A', 'B', 'C', 'D']:
    if len(mark[i])>0:
        for j in mark[i]:
            print("{}             {}                    {}        {}".format(i, j[0], j[1], j[2]))
            f.write("{}             {}                    {}        {}\n".format(i, j[0], j[1], j[2]) )

 
f.close()
