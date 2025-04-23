import matplotlib.pyplot as plt
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from rosbags.typesys import Stores, get_typestore, get_types_from_idl, get_types_from_msg
from pathlib import Path
import math
    
def main():
    x0 = -0.6
    y0 = 1.2
    x1 = 0.0
    y1 = 0

    c1 = x0-x1
    c2 = y0-y1
    ipo = math.sqrt(pow(c1,2)+pow(c2,2))

    cos = math.acos(c2/ipo)
    sin = math.asin(c1/ipo)

    print(cos)
    print(sin)
    print(abs(cos)+abs(sin))
    print(sin+math.pi/2)

if __name__ == "__main__":
    main()
