import numpy as np


ro = np.array(
    [[0,-0.8660,0.5], [1,0,0], [0,0.5,0.8660]]
)
tau = np.array([-0.5, 0,0.5],
   
).reshape((3,1))


def trw(*vec):
    t = ro.dot(np.array(vec).reshape((3,1))) + tau

    return t.reshape(3,1)



if __name__ == '__main__':
    # TEST

    print(trw(np.array([-0.12, 0.12, 0.72])))