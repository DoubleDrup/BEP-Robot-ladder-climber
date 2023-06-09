import numpy as np
import math
friction_coef = 0.5


def cable_force_after_corner(tension_in, angle_deg, Cf=friction_coef):
    angle_rad = angle_deg*2*np.pi/360
    tension_out = tension_in*math.exp(Cf*angle_rad)
    # print(f"Tension in = {tension_in}")
    # print(f"    angle = {angle_rad/np.pi}pi rad")
    # print(f"    Tension out = {tension_out} N")
    return tension_out

def calculate_angle_to_offset(x_dist, y_dist):
    angle = math.atan(x_dist/y_dist)
    return angle*360/(2*np.pi)

def calculate_force_left_right(CoM_claw, spring_arm, weight):
    Fg = weight*9.81
    Fs = Fg*CoM_claw/(2*spring_arm)
    # print(f"The force of each spring should be {Fs} newton more than the opposing spring")
    return Fs

def calculate_force_up_down(CoM_claw, spring_arm, weight):
    Fg = weight * 9.81
    Fs = Fg * CoM_claw / (4 * spring_arm)
    return Fs


### Distances ###
angles = [90,90,calculate_angle_to_offset(0.03,0.06),calculate_angle_to_offset(0.03,0.06)]
spring_arm = 0.019
ball_to_connecting_plate_arm = 0.040
connecting_plate_to_CoM = 0.050
ball_to_CoM = ball_to_connecting_plate_arm + connecting_plate_to_CoM


pull_force = calculate_force_up_down(ball_to_CoM, spring_arm, 0.35)

for angle in angles:
    print(f"force before angle of {angle} deg: {pull_force} N")
    pull_force = cable_force_after_corner(pull_force, angle)


