#!/usr/bin/env python3

import numpy as np
from millihex_robot import Millihexapod

def main():
    millihex = Millihexapod()

    millihex.walk(pattern="bipod", h=(np.pi/3), w=(np.pi/3))
    # millihex.walk(pattern="bipod", h=(np.pi/4), w=(np.pi/4))
    # millihex.random_dancing()

if __name__ == '__main__':
    main()