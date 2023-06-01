import numpy as np
from Math import rotzyx
from typing import Type
import matplotlib.pyplot as plt
rotzyx(np.array([1,1,1]),np.deg2rad(60))

class vector:
    def __init__(self, length, yawmin, yawmax):
        self.length = length
        self.yawmin = yawmin
        self.yawmax = yawmax

        self.__triad = np.eye(3)
        self.__parent = None
        self.__children = []
        self.angleyaw = yawmin
        self.vec = self.length*np.array([1,0,0])

    def rotate(self, yaw, pitch, roll):
        self.__triad = rotzyx(self.__triad,yaw,pitch,roll)

    def get_parent(self):
        return self.__parent

    def set_parent(self, parent):
        self.__parent = parent
        parent.__children.append

    def __repr__(self):
        return f"{self.vec}"

a = vector(3,1/4*np.pi,5/4*np.pi,[0,0,0])
print(a)
