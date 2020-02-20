#!/usr/bin/env python
import os
import sys
import optparse
from math import sqrt,ceil
from random import randrange
import numpy as np
import pandas as pd

# we need to import some python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


from sumolib import checkBinary  # Checks for the binary in environ vars
import traci


def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options

class Vehicle:
    def __init__(self, ID):
        self.ID = ID
    def getSpd(self):
        self.spd = traci.vehicle.getSpeed(self.ID)

    def getRoute(self): #edit not needed
        try:
            self.route = int(traci.vehicle.getRoadID(self.ID)[1])
        except:
            pass
            #print("Warning,current route status: "+traci.vehicle.getRoadID(self.ID) )
    def getPose(self):
        lane_pose = traci.vehicle.getLanePosition(self.ID)
        #edit conisder the agent-emer offset =100
        self.pose = lane_pose + self.route*1000
    def getAcc(self):
        self.accel = traci.vehicle.getAcceleration(self.ID)
    def getL(self):
        self.lane = traci.vehicle.getLaneIndex(self.ID)
    def chL(self,L):
        traci.vehicle.changeLane(self.ID,L, SimTime)
    def acc(self,spd,t):
        traci.vehicle.slowDown(self.ID,spd,t)
    def revertSpd(self):
        traci.vehicle.setSpeed(self.ID,-1)
def getters(vs):
    for v in vs:
        v.getSpd()
        v.getRoute()
        v.getPose()
        v.getAcc()
        v.getL()

def defGlobals():
    #edit
    global SimTime,LH,RB,track_len,fast,slow,Q,speed_range
    speed_range = np.arange(0,30,5)
    fast = 0
    slow = 1
    track_len = 4000
    LH = Vehicle("LH")
    RB = Vehicle("RB")
    SimTime = 1000
    Q = np.zeros((7,4))


def CalcDist(emer,agent):
    #edit
    if emer.route>agent.route:
        dist = track_len-(emer.pose-agent.pose)
    elif emer.route<agent.route:
        dist = agent.pose-emer.pose
    else:
        if emer.pose>agent.pose:
            dist = track_len-(emer.pose-agent.pose)
        else:
            dist = agent.pose-emer.pose
    return dist

#timeOfArrival = quadSolv(0.5*LHacc,LHsp,-dist)

class Anarchia():
    #edit
    def __init__(self,emer,agent):
        self.QActions = ["chL","acc","dec","DoNothing"]
        self.QActionsDict = {"chL":0,"acc":1,"dec":2,"DoNothing":3}
        self.emer = emer
        self.agent = agent
    def pickAction(self):
        self.Action = self.QActions[randrange(len(self.QActions))]
    def takeAction(self):

        if self.Action=="chL":
            self.agent.chL(slow)
        elif self.Action=="kL":
            self.agent.chL(fast)
        elif self.Action=="acc":
            self.agent.acc(40,10)
        elif self.Action=="dec":
            self.agent.acc(4,10)
        elif self.Action=="DoNothing":
            self.agent.chL(fast)
            self.agent.revertSpd()


    def memory(self):
        self.initial_time = getArrivTime(self.emer,self.agent)

    def evaluate(self):

        ArrivTime = getArrivTime(self.emer,self.agent)
        if self.emer.lane == self.agent.lane:
            if ArrivTime<=speed_range[-1]:
                for i in range(1,len(speed_range)):
                    if speed_range[i-1]<=ArrivTime<speed_range[i]:
                        state = i-1
                if ArrivTime>self.initial_time:
                    self.reward = 5
                if ArrivTime<self.initial_time:
                    self.reward = -5
                Q[state,self.QActionsDict[self.Action]] += self.reward
                print("state ",state)
            else:
                self.reward = 1
                Q[5,self.QActionsDict[self.Action]] += self.reward
        else:
            self.reward = -1
            Q[6,self.QActionsDict[self.Action]] += self.reward
def run():
    step = 0

    #print("Hello world, I'm an AV, and I love carrots")
    while traci.simulation.getMinExpectedNumber() > 0:

        traci.simulationStep()

        getters([LH,RB])
        LH.chL(0)
        """
        if step ==10:
            LH.chL(fast)

            Proudhon = Anarchia(LH,RB)
            Proudhon.pickAction()
            Proudhon.takeAction()
            Proudhon.memory()

        elif step%10==0 and step>10:

            Proudhon.evaluate()
            Proudhon.pickAction()
            Proudhon.takeAction()
            Proudhon.memory()


            #print("time ",getArrivTime(LH,RB))
        if step%1000==0 and step>=1000:
            print(step)
            print("Arrive time ", getArrivTime(LH,RB))
            print(Q)
            Q_Table = pd.DataFrame({'Change Lane': Q[:, 0], 'Accelerate': Q[:, 1],'Decelerate': Q[:, 2],'Do no thing': Q[:, 3]})

            Q_Table.to_csv('Results/Q_Table_step='+str(step)+'.csv', index=False)
        if step ==SimTime:
            break
        step += 1

        """
    traci.close()
    sys.stdout.flush()


# main entry point
if __name__ == "__main__":
    options = get_options()

    # check binary
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # traci starts sumo as a subprocess and then this script connects and runs
    traci.start([sumoBinary, "-c", "Anarcho1.2.sumocfg",
                             "--tripinfo-output", "tripinfo.xml"])
    defGlobals()
    run()
