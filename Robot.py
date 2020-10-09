#!/usr/bin/python
# -*- coding: UTF-8 -*-

"""
Authors: Carla Pascual Real, Pablo Noel Carreras Aguerri, Isaac Valdivia Lopez
"""
from __future__ import print_function
from __future__ import division          

import brickpi3
import picamera
from matching import match_images
from picamera.array import PiRGBArray
import time 
import sys
import numpy as np
import cv2
from math import sqrt, atan2

from multiprocessing import Process, Value, Lock

class Robot:
    def __init__(self, cell_size, option, init_position=[0.0, 0.0, 0.0]):
        """
        Inicializa los parametros basicos del robot y lo configura en funcion
        de los puertos a los cuales estan conectados los sensores y motores
        """

        # Robot construction parameters

        self.r = 0.027  # Radio de las ruedas, en metros
        self.L = 0.135  # Distancia entre centros de ruedas, en metros
        self.thd = 0.0
        self.thi = 0.0
        self.logFile = open("log.txt", "w+")    # Fichero de log de odometria
        self.img_count = 0
        ##################################################
        # Motors and sensors setup

        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        self.BP = brickpi3.BrickPi3()

        # Configure sensors
        self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.EV3_ULTRASONIC_CM)
        time.sleep(5)

        self.distance_from_obstacles = 20.0 # In centimeters
        self.security_distance = 10.0 # In centimeters
        self.cell_size = cell_size # In meters

        # reset encoder B and C (or all the motors you are using)
        self.BP.offset_motor_encoder(self.BP.PORT_B, self.BP.get_motor_encoder(self.BP.PORT_B))
        self.BP.offset_motor_encoder(self.BP.PORT_C, self.BP.get_motor_encoder(self.BP.PORT_C))
        self.BP.offset_motor_encoder(self.BP.PORT_A, self.BP.get_motor_encoder(self.BP.PORT_A))

        # CAMERA SETTINGS
        self.CAM_CENTER = 320 / 2.0
        self.cam = picamera.PiCamera()
        self.cam.resolution = (320, 240)
        self.cam.framerate = 32
        self.cam_refresh = 1.0 / self.cam.framerate
        
        # EXIT SETTINGS

        if option == 'A':
            self.cmp_img = cv2.imread("img1.png", cv2.IMREAD_COLOR)
        else:
            self.cmp_img = cv2.imread("img2.png", cv2.IMREAD_COLOR)

        ##################################################
        # odometry shared memory values
        self.x = Value('d',init_position[0])
        self.y = Value('d',init_position[1])
        self.th = Value('d',init_position[2]) 
        self.finished = Value('b',1) # boolean to show if odometry updates are finished
        
        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()

        self.P = 1.0 / 50   # Frecuencia de actualizacion de la odometria, en acts/segundo

    def setSpeed(self, v,w):
        """
        Asigna una velocidad lineal v, y velocidad angular w al robot
        Unidades: v(m/s), w(rad/s)
        """
        print("setting speed to %.2f %.2f" % (v, w))

        # speedPower = 100
        # self.BP.set_motor_power(self.BP.PORT_B + self.BP.PORT_C, speedPower)
        wdwi = (np.linalg.inv([[self.r/2, self.r/2], [self.r/self.L, -self.r/self.L]])).dot([[v],[w]])

        speedDPS_left = np.degrees(wdwi[1])
        speedDPS_right = np.degrees(wdwi[0])

        self.BP.set_motor_dps(self.BP.PORT_B, -speedDPS_right)
        self.BP.set_motor_dps(self.BP.PORT_C, -speedDPS_left)
        # ! Los signos negativos se deben a que los motores de ruedas giran
        # en la direccion contraria a la esperada, debido al montaje

    def normalize(angulo):
        vueltas=0
        while (angulo > np.pi):
            vueltas = vueltas + 1
            angulo = angulo - np.pi
        if (vueltas % 2 == 1):
            angulo = np.pi*(-1)**vueltas + angulo
        return angulo
        
    #x = número de giros de 90 grados
    #dir == r: gira hacia la derecha (por defecto)
    #dir == l: gira hacia la izquierda
    def turn90(self, x = 1, dir = 'r'):
        w = np.pi/4
        if dir == 'l':
            self.setSpeed(0,w)
        else:
            self.setSpeed(0,-w)
        time.sleep(2)
        self.setSpeed(0,0)

    def advance40_t(self):
        v = 0.2
        self.setSpeed(v,0)
        time.sleep(2)
        self.setSpeed(0,0)

    def advance40_o(self):
        origin_x, origin_y, _ = self.readOdometry()
        x = origin_x
        y = origin_y
        v = 0.1
        self.setSpeed(v,0)
        while sqrt(((origin_x - x)**2) + ((origin_y - y)**2)) < 0.4:
            x, y, th = self.readOdometry()
            time.sleep(1/20)
        self.setSpeed(0,0)
        

    def readOdometry(self):
        """ 
        Devuelve los valores actuales de odometria.
        Unidades: x(m), y(m), th(rad)
        """
        return self.x.value, self.y.value, self.th.value

    def startOdometry(self):
        """ 
        Inicia el proceso de actualizacion periodica de odometria
        """
        self.finished.value = False
        self.p = Process(target=self.updateOdometry, args=()) #additional_params?))
        self.p.start()

        self.thd = np.radians(self.BP.get_motor_encoder(self.BP.PORT_B))
        self.thi = np.radians(self.BP.get_motor_encoder(self.BP.PORT_C))

        print("PID: ", self.p.pid)

    def updateOdometry(self):
        """ 
        Funcion periodica ejecutada en un proceso aparte, iniciado en startOdometry()

        """

        while not self.finished.value:
            tIni = time.clock()

            # Radianes girados desde el inicio de los tiempos por cada una de las ruedas, d(erecha) e i(zquierda)
            now_thd = -np.radians(self.BP.get_motor_encoder(self.BP.PORT_B))
            now_thi = -np.radians(self.BP.get_motor_encoder(self.BP.PORT_C))

            # Radianes girados desde la ultima invocacion a updateOdometry()
            delta_thd = now_thd - self.thd
            delta_thi = now_thi - self.thi

            self.thd = now_thd
            self.thi = now_thi
            
            # Metros avanzados por cada unos de las ruedas
            delta_sd = self.r * delta_thd
            delta_si = self.r * delta_thi

            # Metros generales avanzados por el robot desde la ultima invocacion
            delta_s = (delta_sd + delta_si) / 2.0

            # Radianes girados por el robot desde la ultima invocacion
            delta_th = (delta_sd - delta_si) / self.L

            # Metros avanzados en coordenadas x e y desde la ultima invocacion
            delta_x = delta_s * np.cos(self.th.value + (delta_th / 2.0))
            delta_y = delta_s * np.sin(self.th.value + (delta_th / 2.0))

            self.lock_odometry.acquire()

            # Inicio de la seccion critica, actualizacion de valores de odometria (y normalizacion de angulos)
            self.x.value += delta_x
            self.y.value += delta_y
            self.th.value += delta_th
            # self.th.value %= 2*np.pi
            if self.th.value > np.pi:
                self.th.value %= -np.pi
            elif self.th.value < -np.pi:
                self.th.value %= np.pi

            self.lock_odometry.release()

            self.logFile.write(str(self.x.value) + "," + str(self.y.value) 
                + "," + str(self.th.value) + "\n")

            tEnd = time.clock()
            time.sleep(self.P - (tEnd-tIni))

        #print("Stopping odometry ... X= %d" %(self.x.value))
        sys.stdout.write("Stopping odometry ... X=  %.2f, \
                Y=  %.2f, th=  %.2f \n" %(self.x.value, self.y.value, self.th.value))




    # Stop the odometry thread.
    def stopOdometry(self):
        self.logFile.close()
        self.finished.value = True
        self.BP.reset_all()

    def trackObject(self, colorRangeMin=[0,0,0], colorRangeMax=[255,255,255]):
        time.sleep(0.1) # Dar margen de tiempo a la camara
        rawCapture = PiRGBArray(self.cam, size=(320,240))

        finished = False
        targetPositionReached = False

        # params = PARAMETROS DEL BLOB DETECTOR
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 10
        params.maxThreshold = 200

        # Filter by Area
        params.filterByArea = True
        params.minArea = 100
        params.maxArea = 70000

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1
        params.maxCircularity = 1

        # Filter by Color
        params.filterByColor = False
        # not directly color, but intensity on the channel input
        params.filterByConvexity = False
        params.minConvexity = 0.5
        params.maxConvexity = 1
        params.filterByInertia = False

        # detector = CREAR BLOB DETECTOR
        detector = cv2.SimpleBlobDetector(params)

        # redMin, redMax = CREAR RANGO DE ROJOS
        redMin = np.array([0, 0, 50])
        redMax = np.array([0, 0, 172])

        def get_most_promising_blob():
            '''
            Toma una foto y le aplica una mascara de rojo, extrayendo blobs rojos.
            Si hay alguno, devuelve el blob rojo de mayor tamaño (mas prometedor)
            '''
            # photo = TOMAR FOTO
            photo = None
            for img in self.cam.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                photo = img
                break

            # mask_red = OBTENER MASCARA DE ROJO SOBRE IMAGEN ORIGINAL
            mask_red = cv2.inRange(photo.array, redMin, redMax)

            # keypoints_red = OBTENER BLOBS ROJOS, PROMETEDORES
            keypoints_red = detector.detect(255 - mask_red)

            # target_blob = QUEDARSE CON keypoint/blob de mayor size
            rawCapture.truncate(0)

            target_blob = None
            for this_blob in keypoints_red:
                if target_blob == None or this_blob.size > target_blob.size:
                    ''' 
                    DESCOMENTAR PARA ALMACENAR FOTOS MIENTRAS EL ROBOT HACE EL SEGUIMIENTO
                    im_with_keypoints = cv2.drawKeypoints(photo.array, keypoints_red, np.array([]),
                            (255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                    name = "image" + str(self.img_count) + ".png"
                    cv2.imwrite(name, im_with_keypoints)
                    self.img_count += 1
                    '''
                    target_blob = this_blob
            
            return target_blob

        ###################
        rotate_w = np.pi / 5.0
        self.setSpeed(0, rotate_w)

        while not finished:
            #### 1. search the most promising blob:

            target_blob = None
            target_blob = get_most_promising_blob()
       
            if target_blob != None:
                ## Se ha encontrado un objetivo (pelota)

                # cam_diam = DIAMETRO DEL FOCO DE LA CAMARA
                cam_diam = 67.0

                # cam_center = OBTENER CENTRO DE LA CAMARA
                cam_center = 320 / 2.0

                ################################
                while not targetPositionReached:
                    #### 2. decide v and w for the robot to get closer to target position:

                    # blob_diam = OBTENER DIAMETRO DEL BLOB
                    blob_diam = target_blob.size

                    # distancia_diams = OBTENER DISTANCIA
                    distancia_diams = cam_diam - blob_diam
                    print("Distancia diametros:", distancia_diams)

                    # v = DECIDIR SI W POSITIVA O NEGATIVA
                    if abs(distancia_diams) > 50:
                        # Hay un una distancia en linea recta considerable
                        if distancia_diams > 0:
                            v = 0.2 # Hay que acercarse
                        else:
                            v = -0.1 # Hay que alejarse
                    else:
                        # Hay un una distancia en linea recta insignificante
                        v = 0.0
                    
                    # blob_center = OBTENER CENTRO DEL BLOB
                    blob_center = target_blob.pt[0] + target_blob.size / 2.0

                    # distancia_centros = OBTENER DISTANCIA ENTRE CENTROS
                    distancia_centros = cam_center - blob_center

                    print("Distancia centros:", distancia_centros)
                    # w = DECIDIR SI W POSITIVA O NEGATIVA
                    if abs(distancia_centros) > 15:
                        # Hay un desvio considerable entre robot y pelota
                        if distancia_centros > 0:
                            w = np.radians(12) # Giro a la izquierda
                        else:
                            w = -np.radians(12) # Giro a la derecha
                    else:
                        # Hay un desvio insignificante entre robot y pelota
                        w = 0.0
            
                    # INSPECCIONAR VELOCIDADES
                    if v == 0 and w == 0:
                        # El robot tiene la pelota delante
                        targetPositionReached = True
                        finished = True
                        self.setSpeed(0, 0)
                    else:
                        # El robot tiene que seguir moviendose hacia la pelota
                        self.setSpeed(v,w)
                        time.sleep(self.cam_refresh)
                        
                        target_blob = get_most_promising_blob()
                        if target_blob == None:
                            w_aux = 0
                            if w > 0:
                                w_aux = rotate_w
                            else:
                                w_aux = -rotate_w
                            self.setSpeed(0,w_aux)
                            break

            else:
                ## No se ha encontrado ningun objetivo (pelota)
                time.sleep(self.cam_refresh)

        return finished
    
    def catch(self):
        '''
        Bajar cesta
        '''
        self.setSpeed(0.26,0)
        self.BP.set_motor_dps(self.BP.PORT_A, 110)
        
        time.sleep(1.0)

        self.setSpeed(0,0)
        self.BP.set_motor_dps(self.BP.PORT_A, 0)

    def findExit(self):
        """ Busca la caratula por la que debe salir y devuelve """
        time.sleep(0.1) # Dar margen de tiempo a la camara
        rawCapture = PiRGBArray(self.cam, size=(320,240))

        fail_counter = 0
        fail_limit = 5
        found = False
        photo = None

        while not found:
            if fail_counter >= fail_limit:
                fail_counter = 0
                self.setSpeed(0.06, 0)
                time.sleep(0.3)
                self.setSpeed(0, 0)

            time.sleep(self.cam_refresh)

            for img in self.cam.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                photo = img
                break
            rawCapture.truncate(0)

            found, keypoints = match_images(self.cmp_img, photo.array)

            if not found:
                fail_counter += 1
        
        # Hemos encontrado la salida
        self.setSpeed(0, 0)

        print("CAM_CENTER:", self.CAM_CENTER)

        acum_coords = 0
        for kp in keypoints:
            acum_coords += (kp.pt[0] - self.CAM_CENTER)
            #print("kp.pt:", kp.pt[0])
        
        print("acum_cords:", acum_coords)
        if acum_coords > 0:
            # Los keypoints se encuentran mayoritariamente a la derecha
            return (0, 6)
        else:
            # Los keypoints se encuentran mayoritariamente a la izquierda
            return (0, 4)

    def odometria_to_goal(self, x_odom, y_odom):
        """
        Devuelve el equivalente en casilla (x_goal, y_goal)
        Las celdas tienen tamaño cell_size x cell_size en metros
        """
        x_goal = (x_odom) / self.cell_size
        y_goal = (y_odom) / self.cell_size
        print("start pre truc ", [x_goal, y_goal])
        return (int(x_goal), int(y_goal))
    
    def goal_to_odometria(self, x_goal, y_goal):
        """ 
        Devuelve el equivalente en posicion en metros de la celda (x_goal, y_goal)
        Las celdas tienen tamaño cell_size x cell_size en metros
        """
        x_odom = x_goal * self.cell_size + self.cell_size / 2.0
        y_odom = y_goal * self.cell_size + self.cell_size / 2.0
        return x_odom, y_odom

    def girar_hasta2(self, grados):
        margin = 0.6
        w = np.pi/6.0

        rth = self.readOdometry()[2]
        dth = np.degrees(rth)
        #print("girar_hasta2: ", dth, grados)
        
        m = grados - dth
        if m > 0:
            self.setSpeed(0, w)
        else:
            self.setSpeed(0, -w)

        turn = True
        while turn:
            time.sleep(self.P)
            dth = np.degrees(self.readOdometry()[2])
            if (dth > (grados - margin) and dth < (grados + margin)):
                turn = False
            #print("girar_hasta2: ", dth, grados)
        
        self.setSpeed(0, 0)

    def girar_hasta(self, grados):
        """ Gira hasta orientarse a grados con respecto al mundo """
        margin = 0.60
        w = np.pi/7.0

        rth = self.readOdometry()[2]
        dth = np.degrees(rth)

        if (grados == 0.0 or grados == 90.0 or grados == -90.0):
            if (-90.0 < dth < 90.0 and grados == -90.0) or (grados == 0.0 and dth >= 0.0) or (grados == 90.0 and (-90.0 > dth > -180.0 or 90.0 < dth < 180.0)):
                self.setSpeed(0, -w)
            else:
                self.setSpeed(0, w)
            # bucle mientras no tenga la tita los grados deseados
            while (dth > grados + margin) or (dth < grados - margin):
                time.sleep(self.P)
                _, _, rth = self.readOdometry()
                dth = np.degrees(rth)

        elif grados == 180:

            # [-179.65 , 179.65]
            if dth > 0.0:
                self.setSpeed(0, w)
            else:
                self.setSpeed(0, -w)
            
            while abs(dth) <  180 - margin:
                time.sleep(self.P)
                _, _, rth = self.readOdometry()
                dth = np.degrees(rth)
        
        self.setSpeed(0, 0)

    def go_cell(self, x_now, y_now, x_goal, y_goal):
        """ 
        Avanza desde la celda (x_now, y_now) a la celda (x_goal, y_goal)
        Las celdas deben tener cell_size x cell_size de tamanyo, en metros
        """
        if x_now == x_goal and y_now == y_goal:
            # No hay que moverse
            return True
        
        # si: hay que aumentar en x
        if x_now < x_goal:
            # girar hasta 0 grados
            degrees = 0.0
        
        # si: hay que reducir en x
        elif x_now > x_goal:
            # girar hasta theta ~= 180 grados
            degrees = 180.0
        
        # si: hay que aumentar en y
        elif y_now < y_goal:
            # girar hasta theta ~= 90 grados
            degrees = 90.0
        
        # si: hay que reducir en y
        else:
            # girar hasta theta ~= -90 grados
            degrees = -90.0

        return self.go(degrees, x_goal, y_goal)
    

    def go_center(self):
        x_odom, y_odom, _ = self.readOdometry()
        goal = self.odometria_to_goal(x_odom, y_odom)
        goal_x_odom, goal_y_odom = self.goal_to_odometria(goal[0], goal[1])

        th = np.degrees(atan2(goal_y_odom - y_odom, goal_x_odom - x_odom))

        self.girar_hasta2(th)

        # Avanzar hasta que la posicion sea la del centro de la celda objetivo
        #print("Goal:", goal_x_odom, goal_y_odom)
        x, y, _ = self.readOdometry()

        self.setSpeed(0.1, 0)

        i = 0
        dist = sqrt(((goal_x_odom - x)**2) + ((goal_y_odom - y)**2))
        while dist > 0.01:
            x, y, _ = self.readOdometry()
            dist_new = sqrt(((goal_x_odom - x)**2) + ((goal_y_odom - y)**2))
            #print("current: ",x, y, dist_new)
            if dist_new > dist and i > 10:
                break
            
            wall_dist = self.BP.get_sensor(self.BP.PORT_1)
            if wall_dist <= self.security_distance:
                break 
            
            i += 1
            dist = dist_new
            time.sleep(self.P)
    
        self.setSpeed(0, 0)
        time.sleep(0.5)

        return True

        #return self.go(th, goal[0], goal[1])
    
    def center(self):
        rotate_w = np.pi / 8.0
        margin = -1.0

        dth = np.degrees(self.readOdometry()[2])

        if dth > margin:
            self.setSpeed(0, -rotate_w)
        elif dth < -margin:
            self.setSpeed(0, rotate_w)
        
        while dth > margin or dth < -margin:
            time.sleep(self.P)
            dth = np.degrees(self.readOdometry()[2])
        
        self.setSpeed(0, 0)

    def go(self, degrees, x_goal, y_goal):
        """ 
        Avanza desde la celda (x_now, y_now) a la celda (x_goal, y_goal)
        Las celdas deben tener cell_size x cell_size de tamanyo, en metros
        """

        # goal_x_odom, goal_y_odom = self.goal_to_odometria(x_goal, y_goal)
        # x_odom, y_odom, _ = self.readOdometry()

        # th = np.degrees(atan2(goal_y_odom - y_odom, goal_x_odom - x_odom))

        # self.girar_hasta2(th)

        self.girar_hasta(degrees)

        # Ha alcanzado orientacion deseada, hace una pausa
       
        time.sleep(0.5)

        if self.detectObstacle():
            return False
            
        # self.setSpeed(0.32, 0)
        # time.sleep(1.25)
        
        self.setSpeed(0.22, 0)

        # Avanzar hasta que la posicion sea la del centro de la celda objetivo
        x_goalod, y_goalod = self.goal_to_odometria(x_goal, y_goal)
        print("Goal:", x_goalod, y_goalod)
        x, y, _ = self.readOdometry()
        print("From:", x, y)
        i = 0

        dist = sqrt(((x_goalod - x)**2) + ((y_goalod - y)**2))
        while dist > 0.01:
            x, y, _ = self.readOdometry()
            dist_new = sqrt(((x_goalod - x)**2) + ((y_goalod - y)**2))
            #print("current: ",x, y, dist_new)
            if dist_new > dist and i > 10:
                break
            
            wall_dist = self.BP.get_sensor(self.BP.PORT_1)
            if wall_dist <= self.security_distance:
                break 
            
            i += 1
            dist = dist_new
            time.sleep(self.P)
    
        if (x_goal == 6 and y_goal == 4):
            time.sleep(0.4)
        self.setSpeed(0, 0)
        time.sleep(0.5)

        self.girar_hasta(degrees)

        return True

    def advance_slightly(self, v=0.15):
        self.setSpeed(0.15, 0)
        time.sleep(0.4)
        self.setSpeed(0, 0)
    
    def detectObstacle(self):
        """ Comprueba si hay un obstaculo delante y devuelve true en tal caso """
        value = self.BP.get_sensor(self.BP.PORT_1)
        return value <= self.distance_from_obstacles
