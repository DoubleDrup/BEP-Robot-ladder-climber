import numpy as np

def rotx(radx):
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(radx), -np.sin(radx)],
                   [0, np.sin(radx), np.cos(radx)]])
    return Rx

def roty(rady):
    Ry = np.array([[np.cos(rady), 0, np.sin(rady)],
                   [0, 1, 0],
                   [-np.sin(rady), 0, np.cos(rady)]])
    return Ry


def rotz(radz):
    Rz = np.array([[np.cos(radz), -np.sin(radz), 0],
                   [np.sin(radz), np.cos(radz), 0],
                   [0, 0, 1]])
    return Rz

def rotzyx(vec_mat, yaw=0, pitch=0, roll=0):
    """Right hand systems, remember that every rotation is follows the right hand rule around the specific axis. The
    angles are in radiants"""

    vecyaw = rotz(yaw)@vec_mat
    # print(f"vecyaw = {vecyaw}")
    vecpitch = roty(pitch)@vecyaw
    # print(f"vecpitch = {vecpitch}")
    vecroll = rotx(roll)@vecpitch
    # print(f"vecroll = {vecroll}")
    return vecroll

########### Test ###########
def test_rotzyx_yaw(vec1, yaw):
    "testing if simple yaw works"
    if all(np.equal(np.around(rotzyx(vec1, yaw),3), np.around(rotz(yaw)@vec1,3))):
        print(f"Correct: vec1 = {vec1}, yaw = {np.rad2deg(yaw)} ==> result = {rotzyx(vec1, yaw)}")
    else:
        print(f"def rotzyx does not work, check math, {rotzyx(vec1, yaw)} != {rotz(yaw)@vec1}")

def test_rotzyx(vec1, yaw, pitch, roll):
    "testing if yaw, pitch and roll, works"
    if all(np.equal(np.around(rotzyx(vec1, yaw, pitch, roll), 3), np.around(rotx(roll)@roty(pitch)@rotz(yaw)@ vec1, 3))):
        print(f"Correct: vec1 = {vec1.T} (yaw,pitch,roll = {np.rad2deg(yaw)},{np.rad2deg(pitch)},{np.rad2deg(roll)}) ==> result = {np.around(rotzyx(vec1, yaw, pitch, roll), 3)}")
    else:
        print(f"def rotzyx does not work, check math, {np.around(rotzyx(vec1, yaw, pitch, roll), 3).T} != {np.around(rotx(roll)@roty(pitch)@rotz(yaw)@ vec1, 3).T}")

def test_rotzyx_matrix():
    "testing if matrices work for triads"
    mat = np.array([[1,0,0],
                    [0,1,0],
                    [0,0,1]])
    print(rotzyx(mat,1/2*np.pi,0,1/2*np.pi))


# test_rotzyx_yaw(np.array([1,0,0]).T, 1/2*np.pi)
# test_rotzyx(np.array([1,0,1]).T, 1/4*np.pi, 1/8*np.pi, 3)
# test_rotzyx_matrix()

def solve_soe(eq):
    vecs = []
    for eqn in eqns:
        vec = []
        for term in eqn.split(' '):
            if term[0] == '(':
                vec.append(np.array([float(x) for x in term[1:-1].split(',')]))
        vecs.append(vec)

