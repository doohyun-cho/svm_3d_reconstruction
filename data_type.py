from enum import Enum

class CamDirection(Enum):
    F = 1
    B = 2
    L = 3
    R = 4    

class CamType(Enum):
    Rectilinear = 1
    Fisheye = 2

class LinePos(Enum):
    Undefined = 0
    NextNextLeft = 1
    NextLeft = 2
    Left = 3
    Right = 4
    NextRight = 5
    NextNextRight = 6