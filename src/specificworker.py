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

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.robot_id = int()
        self.Period = 2000
        self.T0_befor_signal = 0
        self.T1_got_signal = 0
        self.T2_befor_acting = 0
        self.T3_after_execution = 0
        self.walk_distance = 0

        self.last_x = 0
        self.last_z = 0

        self.newfile = "/home/hz/robocomp/components/autonomous/log_outputs/"

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
    def __angle_to_destiny2(self, p0, p1):
        (x1, z1) = float(p0[0]), float(p0[1])
        (x2, z2) = float(p1[0]), float(p1[1])
        # norma:
        x1 = 10 if (x1 and z1) == 0 else  x1
        z1 = 10 if (x1 and z1) == 0 else  z1
        x2 = 10 if (x1 and z1) == 0 else  x2
        z2 = 10 if (x1 and z1) == 0 else  z2
        #n1 = 1 if math.sqrt(x1**2 + z1**2) == 0 else math.sqrt(x1**2 + z1**2)
        n1 = math.sqrt(x1**2 + z1**2)
        #n2 = 1 if math.sqrt(x2**2 + z2**2) == 0 else math.sqrt(x2**2 + z2**2)
        n2 = math.sqrt(x2**2 + z2**2)
        # produto interno:
        pI = x1*x2 + z1*z2
        ang = (pI/n1*n2)
        
        return math.cos(ang)

    @QtCore.Slot()
    # T0 - antes de adquirir o dado do sensor
    # T1 - depois de adquirir o dado do sensor e antes de enviar.
    # T2 - Antes de começar a atuar a partir da resposta, depois de enviar e receber a resposta,
    # T3 - depois de atuar
    def compute(self):
        self.T0_befor_signal = time.perf_counter()
        
        #rot = random.uniform(-1,1)
        #print(rot)
        try:
            x, z, alpha = self.differentialrobot_proxy.getBasePose()
            base_state_raw = self.differentialrobot_proxy.getBaseState()
            base_state = self.__get_separete_values(base_state_raw)
            
            displacement = self.__distance([self.last_x, self.last_z],[base_state.get("x"), base_state.get("z")])
            inst_vel = self.__vel_robot(displacement)
            delta_time = self.T0_befor_signal - self.T3_after_execution
            
            ang = alpha
            Vx = inst_vel * math.cos(ang)
            Vz = inst_vel * math.sin(ang)

            X = x + Vx * delta_time
            Z = z + Vz * delta_time
            
            #print("Base Pose:", self.differentialrobot_proxy.getBasePose())
            #print("Base State:", self.differentialrobot_proxy.getBaseState())
            #print("stop",self.differentialrobot_proxy.stopBase())
            destiny = (100, 100)
            origin = (x, z)
            turn = self.__angle_to_destiny(origin, destiny)
            tork = math.sqrt(destiny[0]**2 + destiny[1]**2)
            distance = self.__distance(origin, destiny)
            timeTo = distance * inst_vel
            print("vel. inst.: ", inst_vel)
            print("Distance: ", distance)
            print("Turn: ", turn)
            print("Tork: ", tork)
            print("Time: ", timeTo)
            ang = self.__angle_to_destiny2(origin, destiny)
            print("ANG: ",ang)
            if ang > 0.001:
                self.differentialrobot_proxy.setSpeedBase(0, 0.1)
            else:
                self.differentialrobot_proxy.setSpeedBase(0, 0)

            ldata = []
            d = []
            ldata = self.laser_proxy.getLaserData()
            for i in range(0,len(ldata)):
                dis = ldata[i]
                y = dis.dist
                d.append(y)
            d.sort()
            limiar = d[0]
            # if limiar < 500:
            #     # virar para o primeiro lado maior que 400 de distancia
            #     self.differentialrobot_proxy.setSpeedBase(0, 0.5)
            #     time.sleep(0.001)
            # else:
            #     self.differentialrobot_proxy.setSpeedBase(100, 0)
            #     #time.sleep(0.3)

            self.T2_befor_acting = time.perf_counter()
            self.last_x = base_state.get("x")
            self.last_z = base_state.get("z")

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


