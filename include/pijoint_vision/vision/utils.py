import numpy as np


ro = np.array(
    [[1,0,0], [0,-1,0], [0,0,-1]]
)
tau = np.array([-0.5, 0.35,1.8],
   
).reshape((3,1))


def trw(*vec):
    return ro.dot(np.array(vec).reshape((3,1))) + tau



if __name__ == '__main__':
    # TEST

    print(trw(np.array([0.32, 0.70, 0.89])))