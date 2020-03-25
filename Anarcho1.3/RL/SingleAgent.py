class RLAlgorithm():
    '''
    Access order for the Q_table is [agent_vel][agent_lane][amb_vel][amb_lane][rel_amb_y][action].. remember to change the value rel_amb_y to be positive [0,58]
    '''

    def __init__(self, environment, name="Q-learning"):
        '''
        :param environment:  of class env, contains list of vehicles and observations.
        :param name:         string, currently not used except for display purposes
        '''
        self.name = name
        self.environment = environment

        self.QActions = self.environment.Actions
        self.action_to_string_dict = self.environment.action_to_string_dict

    def pickAction(self):
        self.Action = self.QActions[randrange(len(self.QActions))]

    def takeAction(self):
        if self.Action=="change_left":
            self.agent.chL(slow)
        elif self.Action=="change_right":
            self.agent.chL(fast)
        elif self.Action=="acc":
            self.agent.acc(40,10)
        elif self.Action=="dec":
            self.agent.acc(4,10)
        elif self.Action=="no_acc":
            pass
        else:
            raise("Error: Action not recognized. ")

    # def memory(self):
    #     self.initial_time = getArrivTime(self.emer,self.agent)

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
                Q[state,self.action_to_string_dict[self.Action]] += self.reward
                print("state ",state)
            else:
                self.reward = 1
                Q[5,self.action_to_string_dict[self.Action]] += self.reward
        else:
            self.reward = -1
            Q[6,self.action_to_string_dict[self.Action]] += self.reward
