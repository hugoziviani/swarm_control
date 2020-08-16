#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2020 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from genericworker import *
import time, traceback, random, os, math
from scipy.spatial.transform import Rotation as R
import numpy as np
import sys
import matplotlib.pyplot as plt

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
#sys.path.append('./opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        #robot
        self.robot_vector = [0,0]
        self.unity_vector = [0,1]
        self.robot_angle = 0.0
        self.robot_id = int()
        self.Period = 2000
        self.T0_befor_signal = 0
        self.T1_got_signal = 0
        self.T2_befor_acting = 0
        self.T3_after_execution = 0
        
        self.walk_distance = 0
        self.front_robot_angle = 0.0
        
        #destiny
        self.angle_destiny = 0.0
        self.destiny = [800, 800]
        
        self.last_angle_measure = 0
        self.last_distance_measure = 0
        self.last_turn = 0
        self.last_x = 0
        self.last_z = 0

        self.newfile = "/home/hz/robocomp/components/autonomous/log_outputs/"
        self.first_exec = True
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        print('SpecificWorker destructor')

    def clean_outputs(self):
        if os.path.exists(self.newfile+str(self.robot_id)):
            os.remove(self.newfile+str(self.robot_id))

    def setParams(self, params):
        #try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        #except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True

    def set_robot_id(self, id):
        self.robot_id = id

    def __get_separete_values(self, base_state):
        out_put = str(base_state).split('\n')
        state = {
            "x" : float(out_put[2].split("=")[1]),
            "correctedX" : float(out_put[3].split("=")[1]),
            "z" : float(out_put[4].split("=")[1]),
            "correctedZ" : float(out_put[5].split("=")[1]),
            "alpha" : float(out_put[6].split("=")[1]),
            "correctedAlpha" : float(out_put[7].split("=")[1]),
            "advVx" : float(out_put[8].split("=")[1]),
            "advVz" : float(out_put[9].split("=")[1]),
            "rotV" : float(out_put[10].split("=")[1]),
            "isMoving" : bool(out_put[11].split("=")[1])
        }
        return state

    def __distance(self, p0, p1):
        x1, z1 = float(p0[0]), float(p0[1])
        x2, z2 = float(p1[0]), float(p1[1])
        return math.sqrt((x2 - x1)**2 + (z2 - z1)**2)

    def __to_target(self):
        vector_distancy = math.sqrt(self.robot_vector[0]**2 + self.robot_vector[1]**2) + math.sqrt(self.destiny[0]**2+self.destiny[1]**2)
        return round(vector_distancy, 2)

    def __vel_robot(self, displacement):
        delta_time = self.T0_befor_signal - self.T3_after_execution
        return (displacement/delta_time)

    def __angle_to_destiny(self, p0, p1):
        (x1, z1) = float(p0[0]), float(p0[1])
        (x2, z2) = float(p1[0]), float(p1[1])
        try:
            return math.acos((z2-z1)/self.__distance((x1, z1), (x2, z2)))
        except Exception as e:
            print("ERROR: ",e)
            return 0
    
    def __angle_between(self, p0, p1):
        (x1, z1) = float(p0[0]), float(p0[1])
        (x2, z2) = float(p1[0]), float(p1[1])
        a = (x1*x2 + z1*z2)
        if a == 0:
            return 0
            # vai pra frente conforme a distancia do objetivo
        else:
            return(math.acos((round((x1*x2 + z1*z2),2)/round(math.sqrt(x1**2 + z1**2) * math.sqrt(x2**2 + z2**2), 2))))
        
    def __vel_angle(self, vel_angle):
        delta_time = self.T0_befor_signal - self.T3_after_execution
        return vel_angle * delta_time
    
    def __angle_between_origin(self, position):
        xO, zO = self.origin[0], self.origin[2]
        return (self.__angle_between([xO,zO], position))

    def __rotate_vector(self, vector, angle, inverse=False): # se radianos, o resultado é em radianos
        x1, z1 = float(vector[0]), float(vector[1])
        if inverse:
            x = round(math.cos(angle),2) * x1 + round(math.sin(angle)*-1, 2) * z1
            z = round(math.sin(angle),2) * x1 + round(math.cos(angle), 2) * z1
            return [x, z]

        else:
            x = round(math.cos(angle),2) * x1 + round(math.sin(angle), 2) * z1
            z = round(math.sin(angle)*-1,2) * x1 + round(math.cos(angle), 2) * z1
            return [x, z]

    @QtCore.Slot()
    # T0 - antes de adquirir o dado do sensor
    # T1 - depois de adquirir o dado do sensor e antes de enviar.
    # T2 - Antes de começar a atuar a partir da resposta, depois de enviar e receber a resposta,
    # T3 - depois de atuar
    
    def compute(self):
        self.T0_befor_signal = time.perf_counter()
        try:
            x, z, alpha = self.differentialrobot_proxy.getBasePose()
            base_state_raw = self.differentialrobot_proxy.getBaseState()
            base_state = self.__get_separete_values(base_state_raw)
            self.robot_vector = [self.robot_vector[0] + float(x), self.robot_vector[1] + float(z)]

            if not self.first_exec:
                self.angle_destiny = round(self.__angle_between(self.unity_vector, self.destiny) * 180/math.pi, 2)
                self.front_robot_angle = round(alpha * 180/math.pi % 360.0, 2)
                
                self.last_turn = self.last_turn - self.front_robot_angle
                print("Angulo para o destino:", self.angle_destiny)
                print("ANGULO ROBO:", self.front_robot_angle)

                ldata = []
                d = []
                ldata = self.laser_proxy.getLaserData()
                for i in range(0,len(ldata)):
                    dis = ldata[i]
                    y = dis.dist
                    d.append(y)
                d.sort()
                limiar = d[0]

                turn = 0.05
                distance = round(self.__distance([x, z], self.destiny), 2)
                self.angle_destiny = self.__angle_between(self.unity_vector, self.destiny)
                
                if distance < 200:
                    self.differentialrobot_proxy.setSpeedBase(0, 0)
                    print("Tá perto, Distance:", distance)

                
                if abs(self.angle_destiny - self.robot_angle) > 30 and limiar > 300:
                    self.differentialrobot_proxy.setSpeedBase(0, turn)
                    #inverse = True if self.last_turn < 0 else False
                    vector_result = self.__rotate_vector(self.unity_vector, abs(self.last_turn), False)
                    self.unity_vector = vector_result
                
                    
                else:
                    self.differentialrobot_proxy.setSpeedBase(100, 0)
                    
                    print("OBJETIVO: ", self.destiny)
                    print("Minha Posicao: ", self.robot_vector)

                    

                    
                # self.differentialrobot_proxy.setSpeedBase(0, -turn)

                ldata = []
                d = []
                ldata = self.laser_proxy.getLaserData()
                for i in range(0,len(ldata)):
                    dis = ldata[i]
                    y = dis.dist
                    d.append(y)
                d.sort()
                limiar = d[0]
                if limiar < 500:
                    # virar para o primeiro lado maior que 400 de distancia
                    self.differentialrobot_proxy.setSpeedBase(0, turn)
                    time.sleep(0.1)
                else:
                    self.differentialrobot_proxy.setSpeedBase(100, 0)
                    time.sleep(0.1)

                # displacement = self.__distance([self.last_x, self.last_z],[base_state.get("x"), base_state.get("z")])
                
                # delta_time = self.T0_befor_signal - self.T3_after_execution
                
                # inst_vel = self.__vel_robot(displacement)
                
                # Vx = inst_vel * math.cos(alpha)
                # Vz = inst_vel * math.sin(alpha)
                
                # X = x + Vx * delta_time
                # Z = z + Vz * delta_time
                # distance = round(self.__distance([x,z], self.destiny), 2)

# C = B + k*u
# k vai ser o quanto o robo já andou x ou z
# u é o vetor unitário
# C é o destino
# B posição atual

# OBS: se eu souber quantos graus/segundo, sei a vel angular.
# pra poder virar o robo, dou o comando durante alguns segundos.

                # #if ang < 30.0:
                # if True:
                #     # se for virar pra direita, calcular o deslocamento de angulo
                #     # calcular o angulo do destino a partir do zero
                #     self.differentialrobot_proxy.setSpeedBase(0, turn)
                # else:
                #     self.differentialrobot_proxy.setSpeedBase(0, -turn)
                    # # ver angulo
                    # # print("ang: ", ang, "vetor", round(self.unity_vector[0],2), round(self.unity_vector[1],2))
                    # print("Alpha: ", alpha * 180/math.pi)
                    # # 2Pi*r = 360
                    # self.differentialrobot_proxy.setSpeedBase(0, turn)
                    # v = self.__rotate_vector(self.unity_vector, alpha - self.last_turn, inverse = False)
                    # self.unity_vector = v
                    

                    #if ang > 15: # tem que andar pra trás, ou dar o giro até ficar de frente pro angulo
                        # if self.destiny[0] < self.robot_vector[0] and self.destiny[1] < self.robot_vector[1]:
                        #     # virar pra esquerda e calcula o deslocamento do vetor_robo p esquerda
                        #     #print("<,<")
                        #     self.differentialrobot_proxy.setSpeedBase(0, -turn)
                        #     vector_result = self.__rotate_vector(self.robot_vector, alpha - self.last_turn, inverse = True)
                        #     time.sleep(0.1)
                        
                        # elif self.destiny[0] > self.robot_vector[0] and self.destiny[1] > self.robot_vector[1]:
                        #     # virar pra direita e calcula o deslocamnto do vetor_robo p direita
                        #     #print(">,>")
                        #     self.differentialrobot_proxy.setSpeedBase(0, turn)
                        #     vector_result = self.__rotate_vector(self.robot_vector, alpha - self.last_turn, inverse = False)
                        #     time.sleep(0.1)
                        
                        # elif self.destiny[0] < self.robot_vector[0] and self.destiny[1] > self.robot_vector[1]:
                        #     # virar pra esquerda e calcula o deslocamento do vetor_robo p esquerda
                        #     #print("<,>")
                        #     self.differentialrobot_proxy.setSpeedBase(0, -turn)
                        #     vector_result = self.__rotate_vector(self.robot_vector, alpha - self.last_turn, inverse = True)
                        #     time.sleep(0.1)

                        # elif self.destiny[0] > self.robot_vector[0] and self.destiny[1] < self.robot_vector[1]:
                        #     # virar pra direita e calcula o deslocamnto do vetor_robo p direita
                        #     #print(">,<")
                        #     self.differentialrobot_proxy.setSpeedBase(0, turn)
                        #     vector_result = self.__rotate_vector(self.robot_vector, alpha - self.last_turn, inverse = False)
                        #     time.sleep(0.1)

                    
                # elif distance > 200 and 20 < ang < 40:
                #     self.differentialrobot_proxy.setSpeedBase(70, 0)
                
                # else:
                #     self.differentialrobot_proxy.setSpeedBase(0, 0)
                
                # self.decreasing = True if distance <= self.last_distance_measure else False
                # self.last_distance_measure = distance
                # self.robot_vector = vector_result
                    
                    #    print("parou")

                    # if self.robot_vector[0] < reference_to_move_base[0]:
                    #     self.differentialrobot_proxy.setSpeedBase(0, 0)
                    #     print("robot_vector", self.robot_vector[0], "Ref:" ,reference_to_move_base[0])
                    
                        #self.differentialrobot_proxy.setSpeedBase(70, 0)
                    
                #print("ALPHA:",alpha)
                #self.robot_vector = [x + self.robot_vector[0], z + self.robot_vector[1]]
                #print(x, z, alpha)
                # base_state_raw = self.differentialrobot_proxy.getBaseState()
                # base_state = self.__get_separete_values(base_state_raw)
                
                # displacement = self.__distance([self.last_x, self.last_z],[base_state.get("x"), base_state.get("z")])
                # inst_vel = self.__vel_robot(displacement)
                # delta_time = self.T0_befor_signal - self.T3_after_execution
                
                # ang = alpha
                # Vx = inst_vel * math.cos(ang)
                # Vz = inst_vel * math.sin(ang)

                # X = x + Vx * delta_time
                # Z = z + Vz * delta_time
                
                #print("Base Pose:", self.differentialrobot_proxy.getBasePose())
                #print("Base State:", self.differentialrobot_proxy.getBaseState())
                #print("stop",self.differentialrobot_proxy.stopBase())
                
                
                # turn = self.__angle_to_destiny(origin, destiny)
                # tork = math.sqrt(destiny[0]**2 + destiny[1]**2)
                # distance = self.__distance(origin, destiny)
                # timeTo = distance * inst_vel
                # print("vel. inst.: ", inst_vel)
                # print("Distance: ", distance)
                # print("Turn: ", turn)
                # print("Tork: ", tork)
                # print("Time: ", timeTo)
                # ang = self.__angle_between(origin, destiny)
                # print("ANG: ", ang)
                # print("Alfa", alpha)
                # print("rot", base_state_raw)
                
                #print("Antes: ", self.__angle_between(self.robot_vector, destiny))
                #self.__angle_between(origin, destiny)
                #print(alpha * 180/math.pi)
                #self.differentialrobot_proxy.setSpeedBase(0, 0.1)
                
                #angle_delocate = self.robot_angle + delta_time * base_state.get("rotV")
                #print("desloc:",angle_delocate)
                #self.robot_angle = angle_delocate
                #r = R.from_rotvec( angle_delocate * np.array([self.robot_vector[0], 0.0, self.robot_vector[1]]))
                #angle_result = r.as_rotvec()
                #print("Ang_result",angle_result)
                
                #self.differentialrobot_proxy.setSpeedBase(0, 0.1)
                #self.robot_vector = [angle_result[0], angle_result[2]]
                #self.robot_vector = [angle_result[0], angle_result[2]]
            
                #print("Depois: ",self.__angle_between(self.robot_vector, destiny))

                #print("Betewwn origin:",self.__angle_between_origin(origin))
                
                # r = R.from_rotvec((2*np.pi/angle_delocate) * np.array([10, 0, 10]))
                # angle_result = r.as_rotvec()
                # print("Anle_result ", angle_result)
                
                #if ang > 0.001:
                
                #self.differentialrobot_proxy.setSpeedBase(0.1, 0)

                

                self.T2_befor_acting = time.perf_counter()
            #     # self.last_x = base_state.get("x")
            #     # self.last_z = base_state.get("z")
            else:
                self.differentialrobot_proxy.getBaseState()
                self.first_exec = False

        except Ice.Exception as e:
            traceback.print_exc()
            print(e)
        self.T3_after_execution = time.perf_counter()
        # print(self.robot_id)
        # print("T0",self.T0_befor_signal)
        # print("T1",self.T1_got_signal)
        # print("T2",self.T2_befor_acting)
        # print("T3",self.T3_after_execution)
        
        content = [
            "\nT0 " + str(self.T0_befor_signal),
            "\nT1 " + str(self.T1_got_signal),
            "\nT2 " + str(self.T2_befor_acting),
            "\nT3 " + str(self.T3_after_execution)
        ]
        
        with open(self.newfile+str(self.robot_id), 'a') as outfile:
            outfile.writelines(content)
        
        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    ######################
    # From the RoboCompDifferentialRobot you can call this methods:
    # self.differentialrobot_proxy.correctOdometer(...)
    # self.differentialrobot_proxy.getBasePose(...)
    # self.differentialrobot_proxy.getBaseState(...)
    # self.differentialrobot_proxy.resetOdometer(...)
    # self.differentialrobot_proxy.setOdometer(...)
    # self.differentialrobot_proxy.setOdometerPose(...)
    # self.differentialrobot_proxy.setSpeedBase(...)
    # self.differentialrobot_proxy.stopBase(...)

    ######################
    # From the RoboCompDifferentialRobot you can use this types:
    # RoboCompDifferentialRobot.TMechParams

    ######################
    # From the RoboCompLaser you can call this methods:
    # self.laser_proxy.getLaserAndBStateData(...)
    # self.laser_proxy.getLaserConfData(...)
    # self.laser_proxy.getLaserData(...)

    ######################
    # From the RoboCompLaser you can use this types:
    # RoboCompLaser.LaserConfData
    # RoboCompLaser.TData

