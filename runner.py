#!/usr/bin/env python


from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
import json
import argparse
from tkinter import NS

import numpy as np

import matplotlib.pyplot as plt
from pandas import Int8Dtype
import seaborn as sns

# we need to import python modules from the $SUMO_HOME/tools directory
# the install instructions for SUMO should guide you through all of 
# getting this set up
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary 
import traci
import traci.constants as tc

##########################
# function to generate a routefile with a random traffic flow
# input
#  - probs : a dictionary of the probabilities of a car taking a given 
#           route appearing at each time step
#  - speed : the initial speed of cars when they spawn
#  - N     : the number of times steps for which to generate the routefile
#  - accel : the max acceleration of the cars
#  - decdel: the max deceleration of the cars
#
# output
#  - the routefile written to `routes.rou.xml`


def generate_routefile(probs, speed, N, accel, deccel):
    # make tests reproducible by uncommenting the below
    # you can pick whatever seed you want 
    random.seed(42)

    # unpack demand per second for different routes
    pWE = probs["WE"]
    pWN = probs["WN"]
    pWS = probs["WS"]
    pEW = probs["EW"]
    pEN = probs["EN"]
    pES = probs["ES"]
    pNS = probs["NS"]
    pNE = probs["NE"]
    pNW = probs["NW"]
    pSN = probs["SN"]
    pSE = probs["SE"]
    pSW = probs["SW"]
    last_chr="W"
    with open("routes.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="typeCar" accel="%f" decel="%f" sigma="0.0" length="5" minGap="1" speedDev="0" maxSpeed="%f" guiShape="passenger"/>
        <route id="WE" edges="wc ce" />
        <route id="WN" edges="wc cn" />
        <route id="WS" edges="wc cs" />
        <route id="EW" edges="ec cw" />
        <route id="EN" edges="ec cn" />
        <route id="ES" edges="ec cs" />
        <route id="NS" edges="nc cs" />
        <route id="NE" edges="nc ce" />
        <route id="NW" edges="nc cw" />
        <route id="SN" edges="sc cn" />
        <route id="SE" edges="sc ce" />
        <route id="SW" edges="sc cw" />
        """ % (accel, deccel, speed),file=routes)
        vehNr = 0
        sc_pre=chr(65+int(vehNr/10))
        # loop through the desired number of timesteps
        for i in range(N):
            if random.uniform(0, 1) < pWE:
                print('    <vehicle id="%c%i" type="typeCar" route="WE" depart="%f" departSpeed="last"  />' % (
                    sc_pre, vehNr, 2*i), file=routes)
                vehNr += 1
                sc_pre=chr(65+int(vehNr/10))
            if random.uniform(0, 1) <pSN:
                print('    <vehicle id="%c%i" type="typeCar" route="SN" depart="%f" departSpeed="last"  color="1,0,0"/>' % (
                    sc_pre,vehNr, 2*i), file=routes)
                vehNr += 1
                sc_pre=chr(65+int(vehNr/10))

            # Choose random directions
            '''
            directions = ["W", "E", "S", "N"]
            directions.remove(last_chr)
            first_chr = random.choice(directions)
            directions = ["W", "E", "S", "N"]
            directions.remove(first_chr)
            second_char = random.choice(directions)
            route_dir = first_chr + second_char
            last_chr=first_chr
             '''

            '''
            directions = ["W",  "S"]
            first_chr = random.choice(directions)
            if first_chr=="W":
                second_char="E"
            else:
                second_char="N"
            route_dir=first_chr+second_char
           # totalGenTime = generate_time_array[i]
            # Generate one vehicle in one direction one by one.
            # randomly sample each route probability to see
            # if a car appears and if so write it to the xml file
            if route_dir[0] == "S" or route_dir[0] == "N":
                print('    <vehicle id="%c%i" type="typeCar" route="%s" depart="%f" departSpeed="%f" color="1,0,0"/>' % (
                    sc_pre, vehNr, route_dir, i, speed), file=routes)
            else:
                print('    <vehicle id="%c%i" type="typeCar" route="%s" depart="%f" departSpeed="%f"/>' % (
                    sc_pre, vehNr, route_dir, i, speed), file=routes)
            vehNr += 1
            '''
            # totalGenTime += currTime
        print("</routes>", file=routes)
        return vehNr


########################
# a class that implements the latency-resilient tracking
# based on the first come first serve intersection control 
# algorithm
class FCFS(object):

    def __init__(self, radius, pad, topSpeed):

        # the radius of traffic that the algorithm controls
        self.radius = radius

        # the desired time between cars passing through the same point
        self.pad = pad
        # top speed of cars on the road
        self.topSpeed = topSpeed

        self.time={}
        # dictionary of time cars are driven
        self.driveTime = {}
        # dictionary of delay time for the cars
        self.delayTime = {} 
        # reference speed of the car
        self.refSpeed = self.topSpeed*0.8
        self.nowtime=0
        # dictionary of direction pair stored in Int.
        self.intDir = {}
        #REF_DRIVE TIME 

        self.f=open("car_log.txt",'wt')
        self.FCFS_queue=[]
        self.generated_car_num=0
        self.frontcar={}
        self.cur_NS_car=0
        self.cur_WE_car=0





    ##########################
    # function to control the cars for the next time step
    # input
    #  - cars: the dictionary of cars from the junction subscription
    #          includes the lane position, road id, route id, and speed for each car.
    #  - step: the current simulation time step
    #
    # output
    #  - speeds: a dictionary of cars and the speeds they should be set to
    #  - modes : a dictionary of cars and the driving function speed modes they should be set to
    def controlNextStep(self, cars, step,vehnum):
        if not cars:
            init_steps = {}  ## used for recording the initial time step when the car is generated.
            return {}, {}
        speeds = {}
        modes = {}
        length_of_road=190 #idk what is this used for
        current_default_interval = float(params["interval"])
        self.nowtime+=STEP_SIZE
        
        # loop through the cars we are currently controlling 
        for car in cars:
            # Transfer the driving direction from string to int.
            # car_index = int(car[1:]) and traci.vehicle.getDistance(car)>30 and self.generated_car_num==len(self.FCFS_queue)
            if car not in self.FCFS_queue:
                strDir = cars[car][tc.VAR_ROUTE_ID]
                dirFrom = dirTo = 0
                if strDir[0] == "N": dirFrom = 0
                if strDir[0] == "E": dirFrom = 1
                if strDir[0] == "S": dirFrom = 2
                if strDir[0] == "W": dirFrom = 3
                if strDir[1] == "N": dirTo = 0
                if strDir[1] == "E": dirTo = 1
                if strDir[1] == "S": dirTo = 2
                if strDir[1] == "W": dirTo = 3
                self.intDir[len(self.FCFS_queue)] = (dirFrom, dirTo)
                self.FCFS_queue.append(car)
                if dirFrom%2==0:
                    self.frontcar[car]=self.cur_NS_car
                    self.cur_NS_car=car
                else:
                    self.frontcar[car]=self.cur_WE_car
                    self.cur_WE_car=car
        index=0
        for car in self.FCFS_queue:
            if car not in cars:
                index+=1
                continue
            if car == list(cars.keys())[0]:
                pass
            curr_speed = traci.vehicle.getSpeed(car)
            newDelay=0
            # Define the reference speed.
            # Total time spent should be STEP_SIZE * STEP
            # refPos = self.refSpeed * (self.driveTime[car] - self.delayTime[car])
            curr_pos=traci.vehicle.getDistance(car)
            # use physic equation to calculate current distance, time and enter time 
            rt_dis=(self.refSpeed*self.refSpeed-curr_speed*curr_speed)/2/ACCEL
            rt_time=(self.refSpeed-curr_speed)/ACCEL
            now_enter_time=(length_of_road-curr_pos-rt_dis)/self.refSpeed+self.nowtime+rt_time
            if curr_pos>length_of_road: # did not understand the compair 
                index+=1
                refSpeed=curr_speed+ACCEL*STEP_SIZE
                if refSpeed>self.topSpeed*0.8:
                    refSpeed=self.topSpeed*0.8
                speeds[car] = refSpeed
                continue
            elif index!=0:
                # calculate the new delay according to the direction
                if 0:
                    newDelay=current_default_interval
                elif 1:
                    if self.intDir[index-1][0]==self.intDir[index][0]:
                        newDelay=current_default_interval/2
                    elif self.intDir[index-1][1]==self.intDir[index][1]:
                        newDelay=current_default_interval
                    elif self.intDir[index-1][0]-self.intDir[index-1][1]==1 or self.intDir[index-1][0]-self.intDir[index-1][1]==-3:
                        newDelay=current_default_interval * 2 / 3
                    elif self.intDir[index-1][0]-self.intDir[index-1][1]==-1 or self.intDir[index-1][0]-self.intDir[index-1][1]==3:
                        newDelay=current_default_interval
                    elif self.intDir[index-1][0]==self.intDir[index][1] and self.intDir[index-1][1]==self.intDir[index][0]:
                        if (self.intDir[index-1][1]-self.intDir[index-1][0])%2==0:
                            newDelay=current_default_interval / 5
                        else:
                            newDelay=current_default_interval*2/3
                    else :
                        newDelay=current_default_interval
                    self.time[index]=self.time[index-1]+newDelay
            else:
                self.time[index]=now_enter_time

            if curr_pos<length_of_road:
                dis_to_int=length_of_road-curr_pos
                min_time=now_enter_time
                # ref_to_int_time=(length_of_road-refPos)/self.refSpeed
                if self.time[index]<min_time:
                    self.time[index]=min_time
            # refPos = self.refSpeed * self.ref_drive_time[index]
            #refAccl = -(traci.vehicle.getDistance(car) - refPos) / (STEP_SIZE ** 2) - 2 * (curr_speed - self.refSpeed) / STEP_SIZE

            # refAccel should not exceed the maximum accleration.
            if self.time[index] > now_enter_time:
                refAccl = -DECCEL
            else:
                refAccl = ACCEL
            refSpeed = curr_speed + refAccl * STEP_SIZE
            if refSpeed>self.topSpeed*0.8:
                refSpeed=self.topSpeed*0.8
            if refSpeed<self.refSpeed*(dis_to_int)/length_of_road:
               refSpeed=self.refSpeed*dis_to_int/length_of_road
            if self.frontcar[car] in cars:
                if traci.vehicle.getDistance(self.frontcar[car])-curr_pos<7+curr_speed*curr_speed/2/DECCEL:
                    refSpeed=curr_speed-DECCEL*STEP_SIZE
            f=self.f
            
            # ??? this will not happen?
            
            if  index<0:
                print(index,file=f)
                print("accl: "+ str(refAccl),file=f)
                print("now speed:" + str(curr_speed),file=f)
                print("new speed:" + str(refSpeed),file=f)
                if self.frontcar[car] in cars:
                    print("front distance:" + str(traci.vehicle.getDistance(self.frontcar[car])),file=f)
                print("now distance: " + str(traci.vehicle.getDistance(car)),file=f)
                print("number of cars: " + str(len(cars)),file=f)
                print(" ",file=f)

            index+=1
            speeds[car] = refSpeed
            # Add random acceleration to the car.
            #speeds[car] += random.uniform(-float(params["maxRandAcc"]), float(params["maxRandAcc"]))
        return speeds, modes

    ####################
    # method to reset the instance after a run of the simulation
    def reset(self):
        self.FCFS_queue = []
        self.intDir = {}
        self.frontcar={}
        self.cur_NS_car=0
        self.cur_WE_car=0
        self.time={}
        self.nowtime=0



########################
# a class that implements queuing intersection control algorithm
# NB: Does NOT work with turning traffic
class MSO(object):

    def __init__(self, radius, topSpeed):

        # the radius of traffic that the algorithm controls
        self.radius = radius

        # top speed of cars on the road
        self.topSpeed = topSpeed

        # reference speed of the car
        self.refSpeed = self.topSpeed*0.8
        
        # dictionary of direction pair stored in Int.
        self.intDir = {}

        self.intDirQ={}

        self.EWQueue = []
        # the queue of cars approaching the intersection from the north and south
        self.NSQueue = []
        # the direction of last queue


        self.clearQueue = []
        # the queue of cars approaching the intersection from the east and west
        self.f=open("car_log.txt",'wt')

        self.now_direction=0
        self.time={}
        self.EWindex=0
        self.NSindex=0
        self.nowtime=0
        self.begin_direction=0
        self.frontcar={}
        self.cur_NS_car=0
        self.cur_WE_car=0

        self.resort=0




    ##########################
    # function to control the cars for the next time step
    # input
    #  - cars: the dictionary of cars from the junction subscription
    #          includes the lane position, road id, route id, and speed for each car.
    #  - step: the current simulation time step
    #
    # output
    #  - speeds: a dictionary of cars and the speeds they should be set to
    #  - modes : a dictionary of cars and the driving function speed modes they should be set to
    def controlNextStep(self, cars, step,vehnum):
        if not cars:
            init_steps = {}  ## used for recording the initial time step when the car is generated.
            return {}, {}

        speeds = {}
        modes = {}
        index=0
        length_of_road=190
        current_default_interval = float(params["interval"])
        existing_car_index=0
        self.nowtime+=STEP_SIZE
        # distribute direction to each car

        self.EWindex=0
        self.NSindex=0
        if self.NSindex<len(self.NSQueue):
            while self.NSQueue[self.NSindex] not in cars or traci.vehicle.getDistance(self.NSQueue[self.NSindex])>length_of_road:
                self.NSindex+=1
                if self.NSindex==len(self.NSQueue):
                    break
        if self.EWindex<len(self.EWQueue):
            while self.EWQueue[self.EWindex] not in cars or traci.vehicle.getDistance(self.EWQueue[self.EWindex])>length_of_road:
                self.EWindex+=1
                if self.EWindex==len(self.EWQueue):
                    break
        # If no car, remain the direction 
        if self.NSindex==len(self.NSQueue):
            self.now_direction=1
        elif self.EWindex==len(self.EWQueue):
            self.now_direction=0
        else:
            # print(self.NSQueue[self.NSindex])
            # print(self.time[self.NSQueue[self.NSindex]])
            # print(self.EWQueue[self.EWindex])
            # print(self.time[self.EWQueue[self.EWindex]])
            if self.time[self.NSQueue[self.NSindex]]<self.time[self.EWQueue[self.EWindex]]:
                self.now_direction=0
            else:
                self.now_direction=1
                    
        for car in cars:
            if car not in self.intDirQ.keys():
                strDir = cars[car][tc.VAR_ROUTE_ID]
                dirFrom = dirTo = 0
                if strDir[0] == "N": dirFrom = 0
                if strDir[0] == "E": dirFrom = 1
                if strDir[0] == "S": dirFrom = 2
                if strDir[0] == "W": dirFrom = 3
                if strDir[1] == "N": dirTo = 0
                if strDir[1] == "E": dirTo = 1
                if strDir[1] == "S": dirTo = 2
                if strDir[1] == "W": dirTo = 3
                self.intDirQ[car] = (dirFrom, dirTo)
                if existing_car_index==0:
                    self.begin_direction=self.intDirQ[car][0]
            existing_car_index+=1
        existing_car_index=0

        for car in cars:
            if car not in self.NSQueue and car not in self. EWQueue:
                self.resort=1
                curr_speed=traci.vehicle.getSpeed(car)
                curr_pos=traci.vehicle.getDistance(car)
                # use physic equation to calculate current distance, time and enter time 
                rt_dis=(self.refSpeed * self.refSpeed - curr_speed * curr_speed)/2/ACCEL
                rt_time=(self.refSpeed-curr_speed)/ACCEL
                now_enter_time=(length_of_road-curr_pos-rt_dis)/self.refSpeed+self.nowtime+rt_time
                # same as pesudocode
                if self.intDirQ[car][0]%2==0:
                    self.frontcar[car]=self.cur_NS_car
                    self.cur_NS_car=car
                    self.NSQueue.append(car)
                    if len(self.NSQueue)==1:
                        self.time[car]=now_enter_time
                    else:
                        # time is a dictionary to record how many time the car need to cross the intersection in the queue given by the value of the interval, which is a input parameter, and the value of the last two car in the queue
                        self.time[car]=self.time[self.NSQueue[-2]]+current_default_interval/5
                        min_time=now_enter_time
                        self.time[car]=max(self.time[car],min_time)
                else:
                    self.EWQueue.append(car)
                    self.frontcar[car]=self.cur_WE_car
                    self.cur_WE_car=car
                    if len(self.EWQueue)==1:
                        self.time[car]=now_enter_time
                    else:
                        self.time[car]=self.time[self.EWQueue[len(self.EWQueue)-2]]+current_default_interval/5
                        min_time=now_enter_time
                        self.time[car]=max(self.time[car],min_time)
                # print(car, self.intDirQ[car])
                # print(self.NSQueue)
                # print(self.EWQueue)

        if self.resort==1:
            self.resort=0
            self.clearQueue=[]
            self.EWindex=0
            self.NSindex=0
            while self.EWindex+self.NSindex<len(self.NSQueue)+len(self.EWQueue):
                if self.NSindex<len(self.NSQueue):
                    while self.NSQueue[self.NSindex] not in cars or traci.vehicle.getDistance(self.NSQueue[self.NSindex])>length_of_road:
                        self.clearQueue.append(self.NSQueue[self.NSindex])
                        self.NSindex+=1
                        if self.NSindex==len(self.NSQueue):
                            break
                if self.EWindex<len(self.EWQueue):
                    while self.EWQueue[self.EWindex] not in cars or traci.vehicle.getDistance(self.EWQueue[self.EWindex])>length_of_road:
                        self.clearQueue.append(self.EWQueue[self.EWindex])
                        self.EWindex+=1
                        if self.EWindex==len(self.EWQueue):
                            break
                if self.now_direction%2==0 or self.EWindex>=len(self.EWQueue):
                    # print(self.NSQueue)
                    # print(self.NSindex)
                    # print(self.EWindex)
                    # print(self.EWQueue)
                    # print(self.clearQueue)
                    self.intDir[len(self.clearQueue)]=self.intDirQ[self.NSQueue[self.NSindex]]
                    self.clearQueue.append(self.NSQueue[self.NSindex])
                    self.NSindex+=1
                    if self.NSindex==len(self.NSQueue):
                        self.now_direction=1
                        continue
                    if self.NSindex<len(self.NSQueue):
                        if self.time[self.NSQueue[self.NSindex]]-self.time[self.NSQueue[self.NSindex-1]]>current_default_interval*2 and self.EWindex<len(self.EWQueue):
                            temp_curr_pos=traci.vehicle.getDistance(self.EWQueue[self.EWindex])
                            temp_curr_speed=traci.vehicle.getSpeed(self.EWQueue[self.EWindex])
                            rt_dis=(self.refSpeed*self.refSpeed-temp_curr_speed*temp_curr_speed)/2/ACCEL
                            rt_time=(self.refSpeed-temp_curr_speed)/ACCEL
                            now_enter_time=(length_of_road-temp_curr_pos-rt_dis)/self.refSpeed+self.nowtime+rt_time
                            min_time=now_enter_time
                            # print(self.NSQueue[self.NSindex])
                            # print(min_time)
                            # print(self.time[self.NSQueue[self.NSindex]])
                            # print(self.time[self.NSQueue[self.NSindex-1]])
                            if min_time<self.time[self.NSQueue[self.NSindex]]-current_default_interval:
                                #print("change")
                                self.now_direction=1
                elif self.now_direction%2==1 or self.NSindex>=len(self.NSQueue):
                    self.intDir[len(self.clearQueue)]=self.intDirQ[self.EWQueue[self.EWindex]]
                    self.clearQueue.append(self.EWQueue[self.EWindex])
                    self.EWindex+=1
                    if self.EWindex==len(self.EWQueue):
                        self.now_direction=0
                        continue
                    if self.EWindex<len(self.EWQueue):
                        if self.time[self.EWQueue[self.EWindex]]-self.time[self.EWQueue[self.EWindex-1]]>current_default_interval*2 and self.NSindex<len(self.NSQueue):
                            temp_curr_pos=traci.vehicle.getDistance(self.NSQueue[self.NSindex])
                            temp_curr_speed=traci.vehicle.getSpeed(self.NSQueue[self.NSindex])
                            rt_dis=(self.refSpeed*self.refSpeed-temp_curr_speed*temp_curr_speed)/2/ACCEL
                            rt_time=(self.refSpeed-temp_curr_speed)/ACCEL
                            now_enter_time=(length_of_road-temp_curr_pos-rt_dis)/self.refSpeed+self.nowtime+rt_time
                            min_time=now_enter_time
                            if min_time<self.time[self.EWQueue[self.EWindex]]-current_default_interval:
                                print("change")
                                self.now_direction=0
                    
            
        # loop through the cars we are currently controlling 
        for car in self.clearQueue:
            if car not in cars:
                index+=1
                continue
            if car == list(cars.keys())[0]:
                pass
            self.intDir[index]=self.intDirQ[car]
            # Always consider the delaySpeed of the cars at all timeSteps.
            curr_speed = traci.vehicle.getSpeed(car)
            newDelay=0
            curr_pos=traci.vehicle.getDistance(car)
            rt_dis=(self.refSpeed*self.refSpeed-curr_speed*curr_speed)/2/ACCEL
            rt_time=(self.refSpeed-curr_speed)/ACCEL
            now_enter_time=(length_of_road-curr_pos-rt_dis)/self.refSpeed+self.nowtime+rt_time
            if curr_pos>length_of_road:
                index+=1 #为什么这里加1啊
                refSpeed=curr_speed+ACCEL*STEP_SIZE
                if refSpeed>self.refSpeed:
                    refSpeed=self.refSpeed
                speeds[car] = refSpeed
                continue
            elif index!=0 :
                if 0:
                    newDelay=current_default_interval
                elif 1:
                    if self.intDir[index-1][0]==self.intDir[index][0]:
                        newDelay=current_default_interval/2
                    elif self.intDir[index-1][1]==self.intDir[index][1]:
                        newDelay=current_default_interval
                    elif self.intDir[index-1][0]-self.intDir[index-1][1]==1 or self.intDir[index-1][0]-self.intDir[index-1][1]==-3:
                        newDelay=current_default_interval * 2 / 3
                    elif self.intDir[index-1][0]-self.intDir[index-1][1]==-1 or self.intDir[index-1][0]-self.intDir[index-1][1]==3:
                        newDelay=current_default_interval
                    elif self.intDir[index-1][0]==self.intDir[index][1] and self.intDir[index-1][1]==self.intDir[index][0]:
                        if (self.intDir[index-1][1]-self.intDir[index-1][0])%2==0:
                            newDelay=current_default_interval / 5
                        else:
                            newDelay=current_default_interval*2/3
                    else :
                        newDelay=current_default_interval
                self.time[index]=self.time[index-1]+newDelay
            else:
                self.time[index]=now_enter_time

            if curr_pos<length_of_road:
                dis_to_int=length_of_road-curr_pos
                min_time=now_enter_time
                #print(max_time,min_time,ref_to_int_time)
                if self.time[index]<min_time:
                    self.time[index]=min_time
            if self.time[index] > now_enter_time:
                refAccl = -DECCEL
            else:
                refAccl = ACCEL
            refSpeed = curr_speed + refAccl * STEP_SIZE
            if refSpeed>self.topSpeed*0.8:
                refSpeed=self.topSpeed*0.8 #为啥是0.8
            if refSpeed<self.refSpeed*(dis_to_int)/length_of_road:
               refSpeed=self.refSpeed*dis_to_int/length_of_road
            if self.frontcar[car] in cars:
                if traci.vehicle.getDistance(self.frontcar[car])-curr_pos<7+curr_speed*curr_speed/2/DECCEL:
                    refSpeed=curr_speed-DECCEL*STEP_SIZE
            f=self.f
            if index<30:#为啥是10
                print(index,file=f)
                print(car,file=f)
                print("now_enter_time:"+str(now_enter_time),file=f)
                print("accl: "+ str(refAccl),file=f)
                print("now speed:" + str(curr_speed),file=f)
                print("time:" + str(self.time[index]),file=f)
                print("distance: " + str(traci.vehicle.getDistance(car)),file=f)
                print("number of cars: " + str(len(cars)),file=f)
                print(" ",file=f)
            self.time[car]=self.time[index]
            index+=1
            speeds[car] = refSpeed
            # Add random acceleration to the car.
            #speeds[car] += random.uniform(-float(params["maxRandAcc"]), float(params["maxRandAcc"]))
        return speeds, modes

    ####################
    # method to reset the instance after a run of the simulation
    def reset(self):
        self.clearQueue = []
        self.intDir = {}
        self.intDirQ={}
        self.EWQueue = []
        self.NSQueue = []
        self.last_dir=[]
        self.frontcar={}
        self.cur_NS_car=0
        self.cur_WE_car=0


########################
# dummy class for the intersection control algorithm
# NB: Use as a placeholder when testing SUMO defaults
class Unsupervised(object):
    def __init__(self):
        self.radius = 0

    def controlNextStep(self, cars, step):
        return {}, {}

    def reset(self):
        pass


##########################
# function to drive a car based on a custom car following model
# input
#  - car        : the id of the car that is being driven
#  - targetSpeed: the current set speed of the car
#  - mode       : the speed mode of the car following model
#                 NB: these are meant to have similar meanings to
#                     the same numbers in SUMO
#
# output
#  -  the car is driven according to the car following model
def drive(car, targetSpeed, mode):

    # make sure the target speed does not accede the allowed speed for that road
    targetSpeed = min(targetSpeed, traci.vehicle.getAllowedSpeed(car))
    print(targetSpeed, traci.vehicle.getAllowedSpeed(car))
    if mode == 6:
        # depending on if the car is faster or slower than the target speed 
        # either accelerate or decelerate
        currentSpeed = traci.vehicle.getSpeed(car)
        speed = currentSpeed
        if targetSpeed > currentSpeed:
            speed = min(STEP_SIZE * ACCEL + currentSpeed, targetSpeed)
        if targetSpeed < currentSpeed:
            speed = max(currentSpeed - STEP_SIZE * DECCEL, targetSpeed)
        traci.vehicle.setSpeed(car, speed)

    if mode == 31:
        leader = traci.vehicle.getLeader(car)

        # if there is a car in front we use the car following model 
        # use the car following model from Xi Xiong's platooning code
        # NB: this does not take into account acceleration and deceleration 
        # values and can therefore over break and over accelerate
        if leader:
            leaderId = leader[0]
            leaderPos = traci.vehicle.getPosition(leaderId)
            leaderSpeed = traci.vehicle.getSpeed(leaderId)
            carPos = traci.vehicle.getPosition(car)
            carSpeed = traci.vehicle.getSpeed(car)

            posDiff = ((leaderPos[0] - carPos[0])**2 + (leaderPos[1] - carPos[1])**2) ** 0.5

            #car following model
            gapSpeed = leaderSpeed + (posDiff-15)*0.5

            # still make sure that we do not go over the target speed
            speed = min(targetSpeed, gapSpeed)

            traci.vehicle.setSpeed(car, speed)

        # if there is no car in front we just accelerate or decelerate to the target speed
        else:
            currentSpeed = traci.vehicle.getSpeed(car)
            speed = currentSpeed
            if targetSpeed > currentSpeed:
                speed = min(STEP_SIZE * 0.8 + currentSpeed, targetSpeed)
            if targetSpeed < currentSpeed:
                speed = max(currentSpeed - STEP_SIZE * 4.5, targetSpeed)
            traci.vehicle.setSpeed(car, speed)



##########################
# function to run a simulation
# input
#  - algo     : the intersection control algorithm to use in the simulation
#  - dataName : (optional) a prefix to add to the name of the file saved
#  - routefile: (not passed as an argument) the traffic flow will be read from `routes.rou.xml`
#
# output
#  -  the full trip info will be written to `DATA_FOLDER+ / +dataName + tripinfo.xml`
#     and the log data will be written to `log.txt`
def run(vehnum,algo, dataName=""):
    # start SUMO
    traci.start([sumoBinary, "-c", CONF_PATH,
                                         "--tripinfo-output", DATA_FOLDER+ "/"+dataName+"tripinfo.xml", "--duration-log.statistics","--log", "log.txt"])

    step = 0
    # subscribe to get data from the intersection up to the radius of the control algorithm
    traci.junction.subscribeContext("c", tc.CMD_GET_VEHICLE_VARIABLE, algo.radius, [tc.VAR_LANEPOSITION, tc.VAR_ROAD_ID, tc.VAR_ROUTE_ID, tc.VAR_SPEED])
    if params["CUSTOM_FOLLOW"]:
        initCars = {}
    # main loop while there are active cars
    while traci.simulation.getMinExpectedNumber() > 0:

        # if we are using a custom car following model when new cars spawn
        # we turn off the default car following model and add them to the cars we are tracking
        if params["CUSTOM_FOLLOW"]:
            allCars = traci.vehicle.getIDList()

            for car in allCars:
                
                ## Add random speed (move to drive).
                #traci.vehicle.setSpeed(car, traci.vehicle.getSpeed(car) + random.uniform(-0.5, 0.5))

                ## Parameters fpr safety constraint, maybe added to cmd arguments later (move to drive).
                d = 0.5
                beta = 0.05
                traci.vehicle.setMinGap(car, d + beta * traci.vehicle.getSpeed(car))
                ## print("Supposed min gap: " + str(d + beta * traci.vehicle.getSpeed(car)))

                if car not in initCars:
                    traci.vehicle.setSpeedMode(car, 0)
                    initCars[car] = {"targetSpeed": SPEED, "speedMode": 31}

        # get the cars to control from the subscription
        cars = traci.junction.getContextSubscriptionResults("c")

        # run the intersection controller for this time step
        control, modes = algo.controlNextStep(cars, step,vehnum)

        # if we are not using a custom car following model then 
        # use SUMO to update the speeds and speed modes
        if not params["CUSTOM_FOLLOW"]:
            
            for car in modes:
                traci.vehicle.setSpeedMode(car, 31)
            
            for car in control:
                traci.vehicle.setSpeedMode(car, 30)
                control[car]=min(control[car],SPEED)
                d = 0.5
                beta = 0.05
                traci.vehicle.setMinGap(car, d + beta * traci.vehicle.getSpeed(car))
                traci.vehicle.setSpeed(car, control[car])

        # if we are using a custom car following model then update the 
        # target speeds and modes of the cars and then apply our custom 
        # driving function to each of the cars
        if params["CUSTOM_FOLLOW"]:
            for car in modes:
                initCars[car]["speedMode"] = modes[car]
            for car in control:
                initCars[car]["targetSpeed"] = control[car]

            for car in allCars:
                drive(car, initCars[car]["targetSpeed"], initCars[car]["speedMode"])

        # step the simulation forward in time
        traci.simulationStep()
        step += STEP_SIZE

    traci.close()
    sys.stdout.flush()
    algo.reset()

# parser for command line args
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    #optParser.add_option("--algo", type=str, help="control algorithm for the intersectino", default="FCFS")
    optParser.add_option("--algo", type=str, help="control algorithm for the intersectino", default="MSO")
    optParser.add_option('--cf', help="boolean to use the custom car following model", default=False)
    optParser.add_option('--interval', help="the default interval between cars passing the intersections", default=2)
    optParser.add_option('--maxRandAcc', help="the maximum value of the random acceleration", default=0.04)
    optParser.add_option('--genCarNum', help="the totla numbers of cars that need to be generated", default=100)


    options, args = optParser.parse_args()
    return options

##########################
# function to run repeated simulations across a sweep of traffic conditions 
# with all cars going straight
# input
#  - algo     : the intersection control algorithm to use in the simulations
#  - increment: the number of increments into which to divide the sweep 
#               over traffic densities i.e the north/south 
#               and east/west traffic densities simulated will come from a grid
#               of combinations with this many increments on each axis
#  - numRep   : the number of repetitions of the sweep to run
#  - show     : (optional) whether or not to plot a summary of the results
# 
# output
#  -  the results of each simulation will be written to a file 
#     with prefix (north/south increment, east/west increment, repetition)
def repeatedParameterSweep(algo, increment, numRep, N, show = True):

    timesLost = np.zeros((increment, increment, numRep))

    # repeat for number of repetitions
    for l in range(numRep):
        # sweep over the grid of traffic densities
        for i in range(1, increment):
            for j in range(1, increment):

                print("REP %i, params (%0.3f, %0.3f)" % (l, i / increment, j / increment))

                # generate the route file for this simulation with the current density on the sweep
                probs = {
                            "NS" : i / increment,
                            "NE" : 0,
                            "NW" : 0,

                            "SN" : i / increment,
                            "SE" : 0,
                            "SW" : 0,

                            "EW" : j / increment,
                            "EN" : 0,
                            "ES" : 0,

                            "WE" : j / increment,
                            "WN" : 0,
                            "WS" : 0
                        }

                generate_routefile(probs, SPEED, N, ACCEL, DECCEL)


                # run the simulation
                run(algo, dataName="({:d},{:d},{:d})".format(i,j,l))

                # if we are plotting a summary keep track of aggregate 
                # data from the log files
                if show:
                    log = open("log.txt", "r")
                    lines = log.readlines()
                    for line in lines:
                        if 'TimeLoss' in line:
                            break
                    avgTimeLoss = float(line[11:-1])
                    timesLost[i,j,l] = avgTimeLoss
                    for line in lines:
                        if 'DepartDelay' in line:
                            break
                    avgDepartDelay = float(line[14:-1])
                    timesLost[i,j,l] += avgDepartDelay

    # if the option is enabled plot the summary heatmap
    if show:
        for l in range(numRep):
            print(timesLost[:, :, l])

        ticklabels = [i / increment for i in range(increment)]
        timesLost = np.sum(timesLost, axis=2) / numRep
        timesLost = np.flip(timesLost, axis=0)
        ax = sns.heatmap(timesLost, xticklabels = ticklabels, yticklabels = ticklabels[::-1], cbar_kws={'label':'avg time lost per trip (seconds)'})
        plt.xlabel("Prob of car arrival in east/west direction each timestep")
        plt.ylabel("Prob of car arrival in north/south direction each timestep")
        plt.show()

##########################
# function to run repeated simulations across a sweep of traffic conditions 
# with half the cars going straight, 1/4 turning right, and 1/4 turning left
# input
#  - algo     : the intersection control algorithm to use in the simulations
#  - increment: the number of increments into which to divide the sweep 
#               over traffic densities i.e the north/south 
#               and east/west traffic densities simulated will come from a grid
#               of combinations with this many increments on each axis
#  - numRep   : the number of repetitions of the sweep to run
#  - show     : (optional) whether or not to plot a summary of the results
# 
# output
#  -  the results of each simulation will be written to a file 
#     with prefix (north/south increment, east/west increment, repetition)
def repeatedParameterSweepTurning(algo, increment, numRep, numCars, show = True):

    timesLost = np.zeros((increment, increment, numRep))

    for l in range(numRep):

        for i in range(2,increment):
            for j in range(2,increment):

                print("REP %i, params (%0.3f, %0.3f)" % (l, i / increment, j / increment))

                # generate the route file for this simulation with the current density on the sweep
                probs = {
                            "NS" : 0.5 * i / increment,
                            "NE" : 0.25 * i / increment,
                            "NW" : 0.25 * i / increment,

                            "SN" : 0.5 * i / increment,
                            "SE" : 0.25 * i / increment,
                            "SW" : 0.25 * i / increment,

                            "EW" : 0.5 * j / increment,
                            "EN" : 0.25 * j / increment,
                            "ES" : 0.25 * j / increment,

                            "WE" : 0.5 * j / increment,
                            "WN" : 0.25 * j / increment,
                            "WS" : 0.25 * j / increment
                        }

                probs = {
                            "NS" : 0,
                            "NE" : 0,
                            "NW" : 0,

                            "SN" : i / increment,
                            "SE" : 0,
                            "SW" : 0,

                            "EW" : 0 / increment,
                            "EN" : 0,
                            "ES" : 0,

                            "WE" : j / increment,
                            "WN" : 0,
                            "WS" : 0
                        }


                vehnum=generate_routefile(probs, SPEED, numCars, ACCEL, DECCEL)

                # run the simulation
                run(vehnum,algo, dataName="({:d},{:d},{:d})".format(i,j,l))

                # if we are plotting a summary keep track of aggregate 
                # data from the log files
                if show:
                    log = open("log.txt", "r")
                    lines = log.readlines()
                    for line in lines:
                        if 'TimeLoss' in line:
                            break
                    avgTimeLoss = float(line[11:-1])
                    timesLost[i,j,l] = avgTimeLoss-10#50=road_length/refspeed-road_length/topspeed
                    for line in lines:
                        if 'DepartDelay' in line:
                            break
                    avgDepartDelay = float(line[14:-1])
                    timesLost[i,j,l] += avgDepartDelay

    # if the option is enabled plot the summary heatmap
    if show:
        for l in range(numRep):
            print(timesLost[:, :, l])

        ticklabels = [i / increment for i in range(increment)]
        timesLost = np.sum(timesLost, axis=2) / numRep
        timesLost = np.flip(timesLost, axis=0)
        ax = sns.heatmap(timesLost, xticklabels = ticklabels, yticklabels = ticklabels[::-1], annot=True, cbar_kws={'label':'avg time lost per trip (seconds)'})
        plt.xlabel("Prob of car arrival in east/west direction each timestep")
        plt.ylabel("Prob of car arrival in north/south direction each timestep")
        plt.show()

# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    print("START")

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')


# #########################
# simulation parameters
# #########################
    # the radius of control of the intersection control algorithm
    RADIUS = 99999
    # the padding used by the intersection control algorithm
    # (only applies for FCFS)
    PAD = 2.0
    # the max speed for the cars
    SPEED = 10
    # the max acceleration for the cars
    ACCEL = 3
    # the max deceleration for the cars
    DECCEL = 5
    # the increments passed to repeatedParameterSweepTurning or
    # repeatedParameterSweep when running the simulation
    INCREMENTS = 3
    # the repetitions used by repeatedParameterSweepTurning or
    # repeatedParameterSweep when running the simulation
    REPS = 1
    # the number of times steps for which to generate traffic
    # in the routefile
    GENCARS = 100
    # folder to write the complete simulation data
    DATA_FOLDER = "raw_data"
    # timestep size for the simulation
    STEP_SIZE = 0.1
    # the intersection control algorithm to use
    # the options are FCFS for orbit-based first come first serve
    # Q for queuing, and anything else for unsupervised
    # NB: see also CONF_PATH
    ## ALGO = "FCFS"

    # boolean to use repeatedParameterSweepTurning or repeatedParameterSweep
    TURNING = True
    # boolean to use the custom car following model
    # CUSTOM_FOLLOW = False


# #########################
# global values - all must agree with network file
# #########################
    # the distance of the roads
    roadDist = 1000
    # roads entering the intersection
    inRoads = ["nc", "wc", "sc", "ec"]
    # roads entering from the east/west
    EWRoads = ["ec", "wc"]
    # roads entering from the north/south
    NSRoads = ["nc", "sc"]

    # the SUMO configuration file used for the simulations
    # to run a default SUMO stop sign set to `allway.sumocfg`

    # CONF_PATH = "allway.sumocfg"
    CONF_PATH = "config.sumocfg"
# #########################

    # save the parameters to a JSON metadata file

    params =  {
        "RADIUS": RADIUS,
        "PAD": PAD,
        "SPEED": SPEED,
        "ACCEL": ACCEL,
        "DECCEL": DECCEL,
        "INCREMENTS": INCREMENTS,
        "REPS": REPS,
        "GENCARS": options.genCarNum,
        "ALGO": options.algo,
        "TURNING": TURNING,
        "CUSTOM_FOLLOW": options.cf,
        "interval": options.interval,
        "maxRandAcc": options.maxRandAcc,
    }
    json.dump( params, open( DATA_FOLDER+"/params.json", 'w' ) )
    


    # initialize the current algorithm using the parameters
    # and some data about the network (see network info doc)
    # and run the correct set of simulations

    if params["ALGO"] == "MSO":
        algo = MSO(RADIUS, SPEED)

    elif params["ALGO"] == "FCFS":
        algo = FCFS(RADIUS, PAD, SPEED)

    else:
        algo = Unsupervised()


    if not TURNING:
        repeatedParameterSweep(algo, INCREMENTS, REPS, int(params["GENCARS"]))
    if TURNING:
        repeatedParameterSweepTurning(algo, INCREMENTS, REPS, int(params["GENCARS"]))


    