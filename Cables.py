import numpy as np
import math
friction_coef = 0.5
maz_z_angle = 90
angle_corner = 90

def cable_force_after_corner(tension_in, angle_deg, Cf=friction_coef):
    angle_rad = angle_deg*2*np.pi/360
    tension_out = tension_in/math.exp(Cf*angle_rad)
    print(f"Tension in = {tension_in}")
    print(f"    angle = {angle_rad/np.pi}pi rad")
    print(f"    Tension out = {tension_out} N")
    return tension_out

cable_force_after_corner(1, 180)