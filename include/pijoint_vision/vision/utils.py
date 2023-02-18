import numpy as np

SIMULAZIONE = False

if SIMULAZIONE:
    ro = np.array(
        [[0,-0.4995,0.8663], [1,0,0], [0,0.8663,0.4995]]
    )
    tau = np.array([-0.90, -0.23, 0.35],
    
    ).reshape((3,1))
else:
    ro = np.array(
        [[0.4995,0,0.8663], [0,-1,0], [0.8663,0,-0.4995]]
    )
    tau = np.array([-0.90, -0.15, 0.35],
    
    ).reshape((3,1))


def trw(*vec):
    """
    Rotational matrix from camera to robot frame
    """
    t = ro.dot(np.array(vec).reshape((3,1))) + tau

    return t.reshape(3,1)


if __name__ == '__main__':
    # TEST

    print(trw(np.array([-0.12, 0.12, 0.72])))