import numpy as np
import math
from Math import roty, rotz
import matplotlib.pyplot as plt

r_arm = 0.040 #0.020
r_hook = 0.030 #0.015
dist = 0.045 #0.020


d = 0.0075
r = dist + d

def cable_length_rotz(angle, r_arm=r_arm, r_hook=r_hook, r=r, d=d):
    v1 = rotz(angle)@np.array([r_hook, r, r_arm-r_hook])
    v2 = np.array([r_arm, d, 0])
    v3 = v1 - v2
    return np.linalg.norm(v3)

def extension_rotz(angle_deg):
    angle0 = cable_length_rotz(0)
    angle = cable_length_rotz(angle_deg * np.pi / 180)
    extension = abs(angle - angle0)
    # print(f"length at 0 degrees = {angle0*1000} mm")
    # print(f"length at {angle_deg} degrees = {angle*1000} mm")
    # print(f"Extension = {extension*1000} mm")
    return extension

def cable_length_roty(angle, r_arm=r_arm, r_hook=r_hook, dist=dist):
    v4 = np.array([r_arm, 0, 0])
    v5 = roty(angle)@np.array([r_hook, 0, 0])
    v6 = v4 - v5 + np.array([0,dist,0])
    length = np.linalg.norm(v6)
    # print(f"V5: {v5}")
    # print(f"V6: {v6}")
    # print(f"angle: {angle}, length: {length}")
    return length
# print(cable_length_roty(15))
# print(cable_length_roty(-15))

def extension_roty(angle_deg):
    angle0 = cable_length_roty(0)
    angle = cable_length_roty(angle_deg * np.pi / 180)
    extension = abs(angle - angle0)
    # print(f"length at 0 degrees = {angle0*1000} mm")
    # print(f"length at {angle_deg} degrees = {angle*1000} mm")
    # print(f"Extension = {extension*1000} mm")
    return extension

angles = np.arange(-60,60,0.5)
extensions_roty = np.zeros(len(angles))
extensions_rotz = np.zeros(len(angles))



for i in range(len(angles)):
    extensions_roty[i] = extension_roty(angles[i])
    extensions_rotz[i] = extension_rotz(angles[i])

angle = 12 #deg
print(f"The maximum angle for the pin to enter the ball is {angle} degrees:")
print(f"    maximum extension of cables with a {angle} deg rotation about z (left-right):   {round(extension_rotz(angle)*1000,1)} mm")
print(f"    maximum extension of cables with a {angle} deg rotation about y (up-down):   {round(extension_roty(angle)*1000,1)} mm")
print(f"    maximum extension of cables with a {-angle} deg rotation about z (left-right):   {round(extension_rotz(-angle)*1000,1)} mm")
print(f"    maximum extension of cables with a {-angle} deg rotation about y (up-down):   {round(extension_roty(-angle)*1000,1)} mm")


# print(f"Stable length = {stable_length(w_out,w_ball,r,d)}")
# print(f"Length at angle = {length_angle(w_out, w_ball, r, d, angle)}")
plt.plot(angles, extensions_roty, label="Rotation up down")
plt.plot(angles, extensions_rotz, label="Rotation left right")
plt.legend()
plt.show()