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

f = open('coordin.txt','r')
# работа с файлом


class ColorDetecting():                                                                                              # Класс для распознавание цветов - желтый, синий, красный
    def __init__(self):                                                                                              # Функция init содежит:
        rospy.init_node('Color_detect', anonymous=True)                                                              # Создание ноды
        self.out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('X','V','I','D'), 20, (320,240))
        self.bridge = CvBridge()                                                                                     # Переменная необходимая для конвертации изображения из типа msg в обычный вид и обратно
        self.image_sub = rospy.Subscriber("main_camera/image_raw",Image,self.callback)                               # Подписание на топик с изображением
    def callback(self,data):                                                                                         # Основная функция (data- изображения из типа msg)
        try:                                                                                                         # Считывание и конвертация изображения в вид пригодный для дальнейшей работы (try- для отсутствия ошибки если топик будет пустой)
          img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except:pass
        self.out.write(img)

def main():                                                                                                      # Начальная функция
  global col_det
  col_det = ColorDetecting()                                                                                         # Обращение к классу Color_detect

main()

print navigate(x=0, y=0, z=1, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(3)

print navigate(x=0.5, y=0.3, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
f.write(str(start.x,start.y,start.z)+'\n')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=0.5, y=0.8, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=0.5, y=1.3, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=0.5, y=1.8, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=0.5, y=2.3, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=0.5, y=2.8, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=1, y=2.8, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=1, y=2.3, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=1, y=1.8, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=1, y=1.3, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=1, y=0.8, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=1, y=0.3, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=1.5, y=0.3, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=1.5, y=0.8, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=1.5, y=1.3, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=1.5, y=1.8, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=1.5, y=2.3, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=1.5, y=2.8, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=1.5, y=2.8, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=2, y=2.8, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=2, y=2.3, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=2, y=2.3, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=2, y=1.8, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=2, y=1.3, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=2, y=0.8, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=2, y=0.3, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=2.5, y=0.3, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=2.5, y=0.8, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=2.5, y=1.3, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=2.5, y=0.3, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=2.5, y=0.8, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=2.5, y=1.3, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=2.5, y=1.8, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=2.5, y=2.3, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

print navigate(x=2.5, y=2.8, z=1.2, speed=0.5, yaw=math.radians(90), frame_id='aruco_map')
print('Захар делай скрин')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')
rospy.sleep(3)
start = get_telemetry(frame_id='aruco_map')
print(start.x,start.y,start.z)
f.write(str(start.x,start.y,start.z)+'\n')

land()

f.close()
