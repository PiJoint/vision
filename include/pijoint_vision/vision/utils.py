#!/usr/bin/env python
"""!

    This file is part of PiJoint.

    @package pijoint_vision vision
    @author: Alessandro Mizzaro
    @version: 1.0.0
"""
import numpy as np

## Simulation flag
SIMULAZIONE = False

if SIMULAZIONE:
    ## Rotation matrix from camera to robot frame
    ro = np.array(
        [[0,-0.4995,0.8663], [1,0,0], [0,0.8663,0.4995]]
    )
    ## Translation vector from camera to robot frame
    tau = np.array([-0.90, -0.23, 0.35],
    
    ).reshape((3,1))
else:
    ## Rotation matrix from camera to robot frame
    ro = np.array(
        [[0.4995,0,0.8663], [0,-1,0], [0.8663,0,-0.4995]]
    )
    ## Translation vector from camera to robot frame
    tau = np.array([-0.90, -0.15, 0.35],
    
    ).reshape((3,1))


def trw(*vec):
    """!
        Rotational matrix from camera to robot frame
    """
    t = ro.dot(np.array(vec).reshape((3,1))) + tau

    return t.reshape(3,1)


if __name__ == '__main__':
    # TEST

    print(trw(np.array([-0.12, 0.12, 0.72])))