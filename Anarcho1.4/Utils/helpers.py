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

def fill_str(in_str: str, to_length: int):
    """
    :param in_str: input string of length x, where x could be x<=to_length and x could be >=to_length
    :param to_length: length of the required output string (part to take from the in_str
    :return: infromation from in_str in length (to_length_
    """
    in_str = in_str[:min(len(in_str), to_length)]
    return in_str.ljust(to_length)


