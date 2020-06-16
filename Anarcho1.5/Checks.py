from Anarcho import *

def are_we_ok(Proudhon):
    if (Proudhon.list_of_vehicles[0].getL() != (Proudhon.emer_start_lane  )):

        raise ValueError(
            f"Ambulance Changed lane from {Proudhon.emer_start_lane} to {Proudhon.list_of_vehicles[0].getL()} on step . "
            f"\nAmbulance Should not change lane. Quitting.")
        # {step}

        '''
        raise ValueError(
            f"Ambulance Changed lane from {Proudhon.emer_start_lane} to {Proudhon.list_of_vehicles[0].getL()} on step {step} . "
            f"\nAmbulance Should not change lane. Quitting.")
        '''


    return None