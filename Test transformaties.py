import numpy as np
import matplotlib.pyplot as plt


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

def roteren(vec, deg):
    rad = np.deg2rad(deg)
    options = 2 + (180 / deg - 1) * (360 / deg)
    # print(options)
    all_pos = np.zeros((int(options), 3))
    # print(all_pos.shape)
    all_pos[0] = vec
    all_pos[-1] = -vec
    for y in range(1, int(180 / deg)):
        a = roty(rad * y) @ vec
        # print(rad*y)
        for i in range(0, int(360 / deg)):
            # print(f"deg om y = {deg * y}, deg om z = {i * deg}")
            b = rotz(rad * i) @ a
            # print(f"vector is dan -> {b}")
            ind = 1 + i + (y - 1) * int(360 / deg)
            # print(ind)
            all_pos[ind] = b
    return all_pos


def parent_child(parent, child):
     new_options = np.zeros(parent.shape[0], child.shape[0], 3)
     for i in range(parent):
         c = child.shape[0]
         while c != 0:
             new_options[indx] =
             indx += 1
             c -= 1



    return

a = 5
b = 2

norm45 = roteren(np.array([0, 0, 1]), 45)
boptions = norm45*b
aoptions = norm45*a



