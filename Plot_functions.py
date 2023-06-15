import numpy as np
import matplotlib.pyplot as plt

def plot_vecxy(plot_name, base_vec, direction_vec, color="black"):
    dir = np.array([direction_vec[0], direction_vec[1]])
    if np.linalg.norm(dir) == 0:
        return
    x = [base_vec[0], base_vec[0] + direction_vec[0]]
    y = [base_vec[1], base_vec[1] + direction_vec[1]]

    plot_name.plot(x, y, color=color)

def plot_vecxz(plot_name, base_vec, direction_vec, color="black"):
    dir = np.array([direction_vec[0],direction_vec[2]])
    if np.linalg.norm(dir) == 0:
        return
    x = [base_vec[0], base_vec[0] + direction_vec[0]]
    y = [base_vec[2],base_vec[2] + direction_vec[2]]

    plot_name.plot(x,y,color=color)

def plot_vecyz(plot_name, base_vec, direction_vec, color="black"):
    dir = np.array([direction_vec[1],direction_vec[2]])
    if np.linalg.norm(dir) == 0:
        return
    x = [base_vec[1], base_vec[1] + direction_vec[1]]
    y = [base_vec[2],base_vec[2] + direction_vec[2]]

    plot_name.plot(x,y,color=color)

def plot_vecxyz(plotname_xz, plotname_yz, plotname_xy, base_vec, direction_vec, color="black"):
    plot_vecxz(plotname_xz, base_vec,direction_vec, color=color)
    plot_vecyz(plotname_yz, base_vec,direction_vec, color=color)
    plot_vecxy(plotname_xy, base_vec,direction_vec, color=color)

# fig, ax = plt.subplots(figsize = (5,5))
# plot_vecxy(ax, np.array([0,0,0]), np.array([1,1,0]))
# plt.show()
