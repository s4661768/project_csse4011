from enum import Enum

class Gesture(Enum):
    STOP = 0
    FORWARD = 1
    BACK = 2
    LEFT = 3
    RIGHT = 4   
    HOME = 5
    

def get_gest_value(code: int) -> Gesture:
    gest = Gesture.STOP
    if code == 1:
        gest = Gesture.FORWARD
    elif code == 2:
        gest = Gesture.BACK
    elif code == 3:
        gest = Gesture.LEFT
    elif code == 4:
        gest = Gesture.RIGHT
    elif code == 5:
        gest = Gesture.HOME
    return gest