import traci
from Config import SimTime


class Vehicle:

    def __init__(self, ID):
        self.ID = ID
        self.base_route = 1  # Index of base route
        self.length_of_base_route = 100  # Length of base route

        self.type = None
        self.max_speed = None
        self.max_accel = None
        self.max_decel = None

        self.previous_speed = None

    def initialize(self):
        """"
        :return: Initialize function is different from __init__ function  some variables need the SUMO environment to be
        started before being initialized. While the vehicles list itself is being defined before SUMO environment start.
        Note: __init__() function is called once per code run, but initialize() function is called after each traci.load()
        and traci.start() -- i.e. once every environment restart/reset.
        """
        self.type = traci.vehicle.getTypeID(self.ID)
        if(self.type=="Emergency"):
            pass
        else:
            traci.vehicle.setLaneChangeMode(self.ID, 512)  # was 512, should it be 256? # Do we, sometimes, request a change, and it ignores it?
        '''To disable all autonomous changing but still handle safety checks in the simulation,
        either one of the modes 256 (collision avoidance) or 512 (collision avoidance and safety-gap enforcement) may be used.
        ref: https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html#lane_change_mode_0xb6'''

        self.max_speed = traci.vehicle.getMaxSpeed(self.ID)
        self.max_accel = traci.vehicle.getAccel(self.ID)
        self.max_decel = traci.vehicle.getDecel(self.ID)

    def getSpd(self): #ROS
        '''
        :return: vehicle speed in m/sec over last step. This is the speed that will be continued with if no intereference
        occurs from setSpeed or slowDown functions.
        Euler Simulation is used in SUMO :: Constant velocity over time steps.
        '''
        self.spd = traci.vehicle.getSpeed(self.ID)

    def get_previous_spd(self):
        """
        Return previous speed to compute reward , it has same code as getSpeed function , I have written it with
        different name for readability purposes , it is called in the run function before we proceed to next step to get
        previous speed
        """
        self.previous_speed = traci.vehicle.getSpeed(self.ID)

    def getRoute(self): #currently not used except in getters -> nothing depends on it in 1.3
        """
        :return:  route of the car. Either r1 or r2 currently.
        """
        try:
            self.route = int(traci.vehicle.getRoadID(self.ID)[1])
        except:
            pass #EDIT #Is this useful? I think it's a remenant. #Waleed
            #print("Warning,current route status: "+traci.vehicle.getRoadID(self.ID) )

    def getPose(self): #ROS #DO NOT EDIT WITHOUT CHECKING ImportantAssumptions.txt
        # DONE: Change this function to depend on  traci.vehicle.getPosition
        """
        :return: return the position of the vehicle's front tip in the lane (lane: 0,1 currently).
        Accounts for different routes.

        IMPORTANT: THIS FUNCTION ASSUMES HORIZONTAL LANES !!
        """

        #OLD
        # self.lane_pose = traci.vehicle.getLanePosition(self.ID)
        # if(self.route > self.base_route):
        #     self.lane_pose +=  self.length_of_base_route
        self.lane_pose = traci.vehicle.getPosition(self.ID)[0]
        #debug#print(f'Compare {self.lane_pose}, {traci.vehicle.getPosition(self.ID)}') #found to be equal
        return self.lane_pose

    def getAcc(self):  # ROS #currently not used except in getters -> nothing depends on it in 1.3
        """
        :return: Returns the acceleration in m/s^2 of the named vehicle within the last step.
        """
        self.accel = traci.vehicle.getAcceleration(self.ID)

    def getL(self): #ROS
        """
        :return: current lane of the agent, and sets index of the lane in which the vehicle resides inside the vehicle
            object of the agent.
        """
        self.lane = traci.vehicle.getLaneIndex(self.ID)
        return self.lane

    def chL(self, L):  # ROS #currently not used except in getters -> nothing depends on it in 1.3
        """
        :function: requests the lane change action from the SUMO environment.
        :param L: Index of Lane to change lane to.
        :return:  None, but sets the ane changed to, and pefroms the lane change action
        """
        traci.vehicle.changeLane(self.ID, L, SimTime)
        '''
        Forces a lane change to the lane with the given index; if successful,the lane will be chosen for the given amount of time (in s).
        SimtTime to keep the lane change till that the end of the simulation.
        '''
        self.lane = traci.vehicle.getLaneIndex(self.ID) #Force lane update right after to avoid lagging in information

    def chRight(self):
        lane = traci.vehicle.getLaneIndex(self.ID)
        traci.vehicle.changeLane(self.ID, lane-1 , SimTime)

    def chLeft(self):
        lane = traci.vehicle.getLaneIndex(self.ID)
        traci.vehicle.changeLane(self.ID, lane + 1, SimTime)

    def acc(self, spd: float, t: float):  # currently not used -> nothing depends on it in 1.3
        """
        :param spd: speed to reach after time (t)
        :param t: time after which to reach speed (spd)
        :return: None
        """
        traci.vehicle.slowDown(self.ID, spd, t)

    def inst_acc(self, acc):  # ROS
        """accelerate instantaneously"""
        self.getSpd()  # get current speed
        traci.vehicle.setSpeed(self.ID, max(0.0, min(int(self.spd + acc), int(self.max_speed))))
        # minimum velocity: 0.0 -> Handled by Code, not SUMO. Sumo ignores command if velocity is negative.
        # maximum velocity: self.max_speed -> Handled by SUMO, so code here is redundant.

    def __eq__(self, other):
        """
        :param other: other vehicle to compare with
        :return: return True if the two vehicles have the same ID, False otherwise
        """
        return self.ID == other.ID

    def __str__(self):
        return f"VehicleObjectWithID={self.ID}"
