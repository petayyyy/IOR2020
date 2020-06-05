# -*- coding: utf-8 -*-
import numpy as np
import cv2
import time

cam = cv2.VideoCapture(0)
red_low = np.array([125,153,0])                                                                               # Параметры необходимые для определения облак точек каждого цвета:
red_high = np.array([255,255,255])                                                                            # Красного

blue_low = np.array([91,134,255])                                                                             # Синего
blue_high = np.array([137,255,255])

yellow_low = np.array([11,148,204])                                                                           # И желтого
yellow_high = np.array([35,255,255])

while True:
    try:	
        ret, img = cam.read()                                                                                 # Считывание изображения
        imgGrey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)                                                       # Наложения серого фильтра на изображение
        Grey = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(Grey, red_low, red_high)                                                          # Создание облак точек для каждого цвета
        mask2 = cv2.inRange(Grey, blue_low, blue_high)
        mask3 = cv2.inRange(Grey, yellow_low, yellow_high)

        res1 = cv2.bitwise_and(img, img, mask= mask1)                                                         # Метод для отображения облака точек в цвете
        cv2.imshow('Red',res1)                                                                                # Вывод в отдельное окно

        res2 = cv2.bitwise_and(img, img, mask= mask2)                                                         # И так с каждым цветом
        cv2.imshow('Blue',res2)
            
        res3 = cv2.bitwise_and(img, img, mask= mask3)
        cv2.imshow('Yellow',res3)
        
        red = cv2.findContours(mask1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)                                   # Поиск контуров в облаке точек (Красном)
        for c in red[0]:                                                                                      # Перебор каждого контура
            try:
                y,x = 0,0
                moments = cv2.moments(c, 1)                                                                   # Метод создающий матрицу объекта
                sum_y = moments['m01']
                sum_x = moments['m10']
                sum_pixel = moments['m00']
                if sum_pixel > 100:                                                                           # Отсеивание помех(нужно подстроить под ваше разрешение камеры)
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
        cv2.putText(img, ('Blue elements: '+str(len(mas2))), (60, 90), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))# И в финальное изображение (В левом верхнем угле) 
        cv2.putText(img, ('Blue area: '+str(int(m))+'pix'), (60, 110), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))


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
        cv2.imshow("camera", img)                                                                             # Вывод финального изображения на дисплей
        if cv2.waitKey(10) == 27:                                                                             # Вывод из программы на кнопку ESC
            break                                                                       
    except:pass
cap.release()
cv2.destroyAllWindows() 
