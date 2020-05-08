import os
import sys
import optparse
import numpy as np
from sumolib import checkBinary
import traci
from Utils.measure_max_window import measure
from Config import *
from Utils.Vehicle import Vehicle
from Utils.helpers import *
from RL.SingleAgent import RLAlgorithm
from Anarcho import *
def are_we_ok( Proudhon=None):
    if (Proudhon is None):
        Proudhon = env(vehicles_list)
    if (vehicles_list[0].getL() != (Proudhon.emer_start_lane  )):

        raise ValueError(
            f"Ambulance Changed lane from {Proudhon.emer_start_lane} to {vehicles_list[0].getL()} on step . "
            f"\nAmbulance Should not change lane. Quitting.")
        # {step}


        '''
        raise ValueError(
            f"Ambulance Changed lane from {Proudhon.emer_start_lane} to {vehicles_list[0].getL()} on step {step} . "
            f"\nAmbulance Should not change lane. Quitting.")
        '''


    return None