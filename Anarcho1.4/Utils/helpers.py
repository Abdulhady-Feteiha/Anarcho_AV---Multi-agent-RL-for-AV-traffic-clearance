import traci
def getters(vs):
    for v in vs:
        try:
            v.getSpd()
            v.getRoute() #Note: getRoute must always be called before getPose
            v.getPose()
            v.getAcc()
            v.getL()
        except traci.exceptions.TraCIException:
            print(f'{v.ID} was requested after removal ...')
