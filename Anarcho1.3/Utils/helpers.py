def getters(vs):
    for v in vs:
        v.getSpd()
        v.getRoute() #Note: getRoute must always be called before getPose
        v.getPose()
        v.getAcc()
        v.getL()
