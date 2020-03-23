def CalcDist(emer,agent):
    #edit
    if emer.route>agent.route:
        dist = track_len-(emer.pose-agent.pose)
    elif emer.route<agent.route:
        dist = agent.pose-emer.pose
    else:
        if emer.pose>agent.pose:
            dist = track_len-(emer.pose-agent.pose)
        else:
            dist = agent.pose-emer.pose
    return dist
    
def getters(vs):
    for v in vs:
        v.getSpd()
        v.getRoute() #Note: getRoute must always be called before getPose
        v.getPose()
        v.getAcc()
        v.getL()
