#!/usr/bin/python
# -*- coding: UTF-8 -*-

"""
Authors: Carla Pascual Real, Pablo Noel Carreras Aguerri, Isaac Valdivia Lopez
"""

import argparse
import os
import numpy as np
import time

import matplotlib
matplotlib.use("TkAgg")
# sudo apt-get install tcl-dev tk-dev python-tk python3-tk if TkAgg is not available

from Robot import Robot
from MapLib import Map2D

def reachGoal(robot, myMap, start, goal, path):
    cur_cell = start
    for step, tipo_vecino in path:
        possible = robot.go_cell(cur_cell[0], cur_cell[1], step[0], step[1])
        if not possible:
            # replan path
            myMap.replanPath(tipo_vecino, cur_cell[0], cur_cell[1], goal[0], goal[1])
            return False, cur_cell
        else:
            cur_cell = step
    
    return True, cur_cell

def zona_incertidumbre(myMap, robot):
    start = (5, 1)
    goal = (2, 4)

    myMap.planPath(start[0], start[1], goal[0], goal[1])

    
    goalReached = False
    cur_cell = start
    while not goalReached:
        goalReached, cur_cell = reachGoal(robot, myMap, cur_cell, goal, myMap.currentPath)

def do_A(robot):
    """
    Realiza una S en el sentido de la trayectoria A
    """
    R = robot.cell_size / 2.0
    w = np.pi/(3 * 1.5)
    v = R * w

    # Initial rotation
    robot.girar_hasta(90)

    ############## CIRCULO 1
    # First semicircle
    robot.setSpeed(v, -w)
    _, _, rth = robot.readOdometry()
    dth = np.degrees(rth)

    while dth > -90.0:
        time.sleep(robot.P)
        _, _, rth = robot.readOdometry()
        dth = np.degrees(rth)

    print("PRIMER SEMICIRCULO HECHO")

    ##
    robot.setSpeed(v, w)

    # half upper
    while (dth < 90.0):
        time.sleep(robot.P)
        _, _, rth = robot.readOdometry()
        dth = np.degrees(rth)
    
    print("SEGUNDO SEMICIRCULO HECHO")

def do_B(robot):
    """
    Realiza una S en el sentido de la trayectoria B
    """
    R = robot.cell_size / 2.0
    w = np.pi/(3 * 1.5)
    v = R * w

    # Initial rotation
    robot.girar_hasta(-90)

    ############## CIRCULO 1
    # First semicircle
    robot.setSpeed(v, w)
    _, _, rth = robot.readOdometry()
    dth = np.degrees(rth)

    while dth < 90.0:
        time.sleep(robot.P)
        _, _, rth = robot.readOdometry()
        dth = np.degrees(rth)

    ##
    robot.setSpeed(v, -w)

    # half upper
    while (dth > -90.0):
        time.sleep(robot.P)
        _, _, rth = robot.readOdometry()
        dth = np.degrees(rth)
    

def main(args):
    try:
        MAP_FILE = "mapa.txt"
        if not os.path.isfile(MAP_FILE):
            print('Map file %s does not exist' % args.mapfile)
            exit(1)

        myMap = Map2D(MAP_FILE)
        if args.option == "B":
            robot = Robot(myMap.sizeCell / 1000.0, option='B', init_position=[1.8, 0.4, 0.0])
        else:
            robot = Robot(myMap.sizeCell / 1000.0, option='A', init_position=[1.8, 0.4, 0.0])
        
        robot.startOdometry()

        DO_S    = True
        DO_NAV  = True
        DO_BALL = True
        DO_EXIT = True

        ########## 1. perform S ##########
        if DO_S:
            print("Empiezo S")
            if args.option == "B":
                do_B(robot)
            else:
                do_A(robot)

            robot.advance_slightly()
            
            robot.go_center()
            pregoal = (6, 1)
            x_odom, y_odom, _ = robot.readOdometry()
            x_start, y_start = robot.odometria_to_goal(x_odom, y_odom)
            robot.go_cell(x_start, y_start, pregoal[0], pregoal[1])

            # Centrarse
            robot.girar_hasta(0)


        # ########## 2. navigate ##########
        if DO_NAV:
            print("Empiezo navegacion")
            start = pregoal
            print("start", start)
            goal = (2, 5)

            myMap.planPath(start[0], start[1], goal[0], goal[1])

            goalReached = False
            cur_cell = start
            while not goalReached:
                goalReached, cur_cell = reachGoal(robot, myMap, cur_cell, goal, myMap.currentPath)

        ########## 3. search for exit ##########
        if DO_EXIT:
            print("Empiezo busqueda y salida")

            # Girar a -90º para mirar hacia salidas
            robot.girar_hasta(180)

            ## Buscar salida
            start = goal
            final_exit = robot.findExit()
 
        ########## 4. search for ball and capture it ##########
        if DO_BALL:
            robot.advance_slightly(-0.1)

            print("Empiezo busqueda y captura")
            ball_reached = robot.trackObject()

            if ball_reached:
                robot.catch()
        
        ########## 5. exit ##########
        if DO_EXIT:
            robot.go_center()
            print("Me he centrado")
            
            x_odom, y_odom, _ = robot.readOdometry()
            start = robot.odometria_to_goal(x_odom, y_odom)

            myMap.planPath(start[0], start[1], final_exit[0], final_exit[1])

            goalReached = False
            cur_cell = start
            while not goalReached:
                goalReached, cur_cell = reachGoal(robot, myMap, cur_cell, final_exit, myMap.currentPath)

        robot.stopOdometry()

    except KeyboardInterrupt:
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()


if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--option", default="A", help="path option: A or B")

    args = parser.parse_args()
    main(args)
