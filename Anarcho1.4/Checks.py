from Anarcho import *

def are_we_ok( Proudhon=None):
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