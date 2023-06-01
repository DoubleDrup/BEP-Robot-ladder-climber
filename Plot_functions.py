import numpy as np


def plot_vec(plot, base_vec, direction_vec):
    Base = np.array([base_vec[0], base_vec[1]])
    end = np.array([direction_vec[0],direction_vec[1]])
    plot.arrow(Base[0],Base[1],end[0],end[1])

