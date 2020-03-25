import os
import numpy as np

BASE_PATH = os.path.dirname(os.path.abspath(__file__))
Sumocfg_DIR = os.path.join(BASE_PATH, "Sumo_files/Anarcho1.3.sumocfg")


speed_range = np.arange(0,30,5)
fast = 0
slow = 1
track_len = 500
SimTime = 1000.0
num_episoes = 1000


#Don't forget to initialize SimTime before importing vehicle
from Utils.Vehicle import Vehicle
vehicles_list = [Vehicle("LH"), Vehicle("RB")]
