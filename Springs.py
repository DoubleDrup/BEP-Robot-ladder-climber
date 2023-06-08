import numpy as np
import math
from Math import roty

def stable_length(w_out, w_ball, r, d):
    l_stable = math.sqrt((r - d)**2+(w_ball - w_out)**2)
    return l_stable

def length_angle(w_out, w_ball, r, d, angle):
    a_stable = math.atan(w_ball/d)
    print(f'\na_stable = {a_stable*180/math.pi}')
    a1 = a_stable + angle
    print(f'a1 = {a1*180/math.pi}')
    d2 = np.sqrt(d **2 + w_ball **2)
    print(f'd2 = {d2}')
    d1 = d2 * np.cos(a1)
    print(f'd1 = {d1}')
    d3 = d2 * np.sin(a1)
    print(f'd3 = {d3}')
    r2 = r - d1
    print(f'r2 = {r2}')
    r3 = d3 - w_out
    print(f'r3 = {r3}')
    l_angle = math.sqrt(r3 **2 + r2 **2)
    return l_angle

def rotation_lenght(r_big, r_small, dist, angle):
    v1 = np.array([r_big, 0, 0])
    v2 = roty(angle)@np.array([r_small, 0, 0])
    v3 = v1 - v2 + np.array([0,0,dist])
    length = np.linalg.norm(v3)

    return length

def calculate_length_rotation(r_big, r_small, dist, angle_deg):
    angle0 = rotation_lenght(r_big, r_small, dist, 0)
    angle = rotation_lenght(r_big, r_small, dist, angle_deg*np.pi/180)
    print(f"length at 0 degrees = {angle0}")
    print(f"length at {angle_deg} degrees = {angle}")
    print(f"Extension = {angle - angle0}")

angle = 15
r_big = 2.5
r_small = 0.5
dist = 2

calculate_length_rotation(r_big,r_small,dist,angle)
w_out = 2
w_ball = 3
r = 3
d = 0.5


# print(f"Angle = {angle*360/(2*math.pi)}")
# print(f"Stable length = {stable_length(w_out,w_ball,r,d)}")
# print(f"Length at angle = {length_angle(w_out, w_ball, r, d, angle)}")
# print(f"Cable extension = {angle}")