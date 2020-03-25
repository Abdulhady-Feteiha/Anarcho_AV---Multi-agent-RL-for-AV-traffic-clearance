import traci
from Config import SimTime
class Vehicle:
    def __init__(self, ID):
        self.ID = ID
        self.base_route = 1 #Index of base route
        self.length_of_base_route = 100 #Length of base route
    def initialize(self):
        traci.vehicle.setLaneChangeMode(self.ID, 256)
        '''To disable all autonomous changing but still handle safety checks in the simulation,
        either one of the modes 256 (collision avoidance) or 512 (collision avoidance and safety-gap enforcement) may be used.
        ref: https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html#lane_change_mode_0xb6'''
        self.type = traci.vehicle.getTypeID(self.ID)
        self.max_speed = traci.vehicle.getMaxSpeed(self.ID)
        self.max_accel = traci.vehicle.getAccel(self.ID)
        self.max_decel = traci.vehicle.getDecel(self.ID)

    def getSpd(self): #ROS
        '''
        :return: vehicle speed in m/sec over last step. This is the speed that will be continued with if no intereference
        occurs from setSpeed or slowDown functions.
        '''
        self.spd = traci.vehicle.getSpeed(self.ID)

    def getRoute(self):
        '''
        :return:  route of the car. Either r1 or r2 currently.
        '''
        try:
            self.route = int(traci.vehicle.getRoadID(self.ID)[1])
        except:
            pass #EDIT #Is this useful? I think it's a remenant. #Waleed
            #print("Warning,current route status: "+traci.vehicle.getRoadID(self.ID) )

    def getPose(self): #ROS
        #DONE: Change this function to depend on  traci.vehicle.getPosition
        '''
        :return: return the position of the vehicle's front tip in the lane (lane: 0,1 currently).
        Accounts for different routes.
        '''

        #OLD
        # self.lane_pose = traci.vehicle.getLanePosition(self.ID)
        # if(self.route > self.base_route):
        #     self.lane_pose +=  self.length_of_base_route
        self.lane_pose = traci.vehicle.getPosition(self.ID)[0]
        #debug#print(f'Compare {self.lane_pose}, {traci.vehicle.getPosition(self.ID)}') #found to be equal
        return self.lane_pose

    def getAcc(self): #ROS
        '''
        :return: Returns the acceleration in m/s^2 of the named vehicle within the last step.
        '''
        self.accel = traci.vehicle.getAcceleration(self.ID)

    def getL(self): #ROS
        '''
        :return: None, but sets index of the lane in which the vehicle resides.
        '''
        self.lane = traci.vehicle.getLaneIndex(self.ID)

    def chL(self,L): #ROS
        '''
        :function: pefroms the lane change action
        :param L: Index of Lane to change lane to.
        :return:  None, but sets the ane changed to, and pefroms the lane change action
        '''
        traci.vehicle.changeLane(self.ID,L, SimTime)
        '''
        Forces a lane change to the lane with the given index; if successful,the lane will be chosen for the given amount of time (in s).
        SimtTime to keep the lane change till that the end of the simulation.
        '''
        self.lane = traci.vehicle.getLaneIndex(self.ID) #Force lane update right after to avoid lagging in information

    def acc(self,spd,t):
        '''
        :param spd: speed to reach after time (t)
        :param t: time after which to reach speed (spd)
        :return: None
        '''
        traci.vehicle.slowDown(self.ID,spd,t)

    def revertSpd(self):
        '''
        Sets speed instantaneously
        :return:
        '''
        traci.vehicle.setSpeed(self.ID,-1)

    def inst_acc(self, acc): #ROS
        ''' accelerate instantaneously'''
        self.getSpd() #get current speed
        traci.vehicle.setSpeed(self.ID, max( 0.0 , min(int(self.spd  + acc), int(self.max_speed))))
        #minimum velocity: 0.0 -> Handled by Code, not SUMO. Sumo ignores command if velocity is negative.
        #maximum velocity: self.max_speed -> Handled by SUMO, so code here is redundant.
