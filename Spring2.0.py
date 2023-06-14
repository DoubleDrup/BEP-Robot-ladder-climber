import numpy as np
import matplotlib.pyplot as plt
from Math import rotzyx
from Plot_functions import *


zmin, zmax = -30, 30
ymin, ymax = -45, 45
xmin, xmax = -90, 90


h_cor = 0.05
d_spring = 0.035

h_knot = 0.01

origin = np.array([0,0,0])
o_spring = np.array([d_spring,0,0])
o_cor = np.array([0, 0, h_cor])
cor_knot = np.array([d_spring, 0, -h_knot])
o_knot = o_cor + cor_knot
def cor_knot_rotated(v_cor_knot, anglez, angley, anglex):
    new = rotzyx(cor_knot, anglez, angley, anglex)
    return new

def spring_knot(v_o_cor, v_o_spring, cor_knot_new):
    v1 = v_o_cor + cor_knot_new
    spring_knot = v1 - v_o_spring
    return spring_knot

def calculate_spring_lengths(v_o_cor, v_o_spring, v_cor_knot, step_degrees):
    z_steps = int((zmax - zmin)/step_degrees)
    print(z_steps)
    y_steps = int((ymax - ymin)/step_degrees)
    x_steps = int((xmax - xmin)/step_degrees)
    zangles = np.linspace(zmin, zmax, z_steps)
    yangles = np.linspace(ymin, ymax, y_steps)
    xangles = np.linspace(xmin, xmax, x_steps)

    cor_knots = np.zeros((len(zangles), len(yangles), len(xangles), 3))
    spring_knots = np.zeros(np.shape(cor_knots))
    lengths = np.zeros((len(zangles), len(yangles), len(xangles)))
    for z in range(len(zangles)):
        for y in range(len(yangles)):
            for x in range(len(xangles)):
                z_rad, y_rad, x_rad = zangles[z]*np.pi/180, yangles[y]*np.pi/180, xangles[x]*np.pi/180
                # print(zangles[z],yangles[y],xangles[x])
                cor_knots[z,y,x,:] = cor_knot_rotated(v_cor_knot, z_rad, y_rad, x_rad)
                # print(cor_knots[z,y,x,:])
                spring_knots[z,y,x,:] = spring_knot(v_o_cor, v_o_spring, cor_knots[z,y,x])
                lengths[z,y,x] = np.linalg.norm(spring_knots[z,y,x,:])
    return zangles, yangles, xangles, cor_knots, spring_knots, lengths

def longest_spring(lengths):
    longest_index = []
    longest_length = 0
    # print(lengths.shape)
    for z in range(lengths.shape[0]):
        # print(z)
        for y in range(lengths.shape[1]):
            for x in range(lengths.shape[2]):
                if lengths[z,y,x] > longest_length:
                    longest_length = lengths[z,y,x]
                    longest_index = [[z,y,x]]
                elif lengths[z,y,x] == longest_length:
                    longest_index.append([z,y,x])
    return longest_index, longest_length


zangles, yangles, xangles, cor_knots, spring_knots, lengths = calculate_spring_lengths(o_cor, o_spring, cor_knot, 1)
# print(spring_knots.shape)
longindexes, longlength = longest_spring(lengths)
# print(lengths[2,3,6])
# print(lengths)
# print(spring_knots)
# print(f'Longest spring length = {lengths}')

for i in longindexes:
    print(f'Indexes of this longest spring {i}')
    print(f'corknot = {cor_knots[i[0],i[1],i[2],:]}, spring_knot = {spring_knots[i[0],i[1],i[2],:]}, length = {longlength}, length check = {np.linalg.norm(spring_knots[i[0],i[1],i[2],:])}')
    print(f'z-angle = {zangles[i[0]]}, y-angle = {yangles[i[1]]}, x-angle = {xangles[i[2]]}')
def plot():
    fig, xz = plt.subplots(figsize = (5,5))
    xz.set_aspect('equal')
    xz.set_title('wrist xz plane, longest springs')
    fig, yz = plt.subplots(figsize = (5,5))
    yz.set_aspect('equal')
    yz.set_title('wrist yz plane, longest springs')
    fig, xy = plt.subplots(figsize = (5,5))
    xy.set_aspect('equal')
    xy.set_title('wrist xy plane, longest springs')
    for i in longindexes:
        cor_knot_rot = cor_knots[i[0],i[1],i[2],:]
        plot_vecxz(xz, origin, o_cor)
        plot_vecxz(xz, origin, o_spring)  
        plot_vecxz(xz, o_cor, cor_knot_rot)
        plot_vecxz(xz, o_spring, spring_knots[i[0],i[1],i[2],:], color="blue")

        cor_knot_rot = cor_knots[i[0],i[1],i[2],:]
        plot_vecyz(yz, origin, o_cor)
        plot_vecyz(yz, origin, o_spring)
        plot_vecyz(yz, o_cor, cor_knot_rot)
        plot_vecyz(yz, o_spring, spring_knots[i[0],i[1],i[2],:], color="blue")

        cor_knot_rot = cor_knots[i[0], i[1], i[2], :]
        plot_vecxy(xy, origin, o_cor)
        plot_vecxy(xy, origin, o_spring)
        plot_vecxy(xy, o_cor, cor_knot_rot)
        plot_vecxy(xy, o_spring, spring_knots[i[0], i[1], i[2], :], color="blue")
    plt.show()

plot()
