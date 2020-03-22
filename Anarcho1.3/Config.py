import os
import numpy as np

BASE_PATH = os.path.dirname(os.path.abspath(__file__))
Sumocfg_DIR = os.path.join(BASE_PATH, "Sumo_files/Anarcho1.3.sumocfg")


global SimTime,LH,RB,track_len,fast,slow,Q,speed_range


speed_range = np.arange(0,30,5)
fast = 0
slow = 1
track_len = 500
GUI_on = False

SimTime = 1000
#Please, initialize SimTime before importing vehicle
from Utils.Vehicle import Vehicle
LH = Vehicle("LH")
RB = Vehicle("RB")
vehicles_list = [LH, RB]
Q_ = np.zeros((6,3,11,3,58,5))
