def behavior2human(inlog):
    if inlog == 0:
        return 'Standby'
    elif inlog == 1:
        return 'Find BL'
    elif inlog == 2:
        return 'Goto WP/NoGW'
    elif inlog == 3:
        return 'Goto WP'
    elif inlog == 4:
        return 'Shoot'
    elif inlog == 5:
        return 'Find GW'
    elif inlog == 6:
        return 'Man Cal'
    elif inlog == 7:
        return 'A/T Cal'

def refbox2human(inlog):
    if inlog is 'W':
        return 'Welcome'
    elif inlog is 'S':
        return 'Stop'
    elif inlog is 's' :
        return 'Start'
    elif inlog is 'N':
        return 'DropBall'
    elif inlog is 'L':
        return 'Park'
    elif inlog is 'h':
        return 'EndPart'
    elif inlog is 'Z':
        return 'Reset'
    elif inlog is 'e':
        return 'EndGame'
    elif inlog is 'K':
        return 'Cyan KickOff'
    elif inlog is 'F':
        return 'Cyan FreeKick'
    elif inlog is 'G':
        return 'Cyan GoalKick'
    elif inlog is 'T':
        return 'Cyan ThrowIn'
    elif inlog is 'C':
        return 'Cyan Corner'
    elif inlog is 'P':
        return 'Cyan Penalty'
    elif inlog is 'A':
        return 'Cyan Goal'
    elif inlog is 'O':
        return 'Cyan Repair'
    elif inlog is 'R':
        return 'Cyan RedCard'
    elif inlog is 'Y':
        return 'Cyan YellowCard'
    elif inlog is 'k':
        return 'Magenta KickOff'
    elif inlog is 'f':
        return 'Magenta FreeKick'
    elif inlog is 'g':
        return 'Magenta GoalKick'
    elif inlog is 't':
        return 'Magenta ThrowIn'
    elif inlog is 'c':
        return 'Magenta Corner'
    elif inlog is 'p':
        return 'Magenta Penalty'
    elif inlog is 'a':
        return 'Magenta Goal'
    elif inlog is 'o':
        return 'Magenta Repair'
    elif inlog is 'r':
        return 'Magenta RedCard'
    elif inlog is 'y':
        return 'Magenta YellowCard'
