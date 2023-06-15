import numpy as np
import matplotlib.pyplot as plt
from Math import rotzyx
from Plot_functions import *
from sympy import Symbol, Matrix, solve

k = Symbol('k')
h_cor = 0.05
d_spring = 0.035

h_knot_wrist = -0.01
h_knot_upper = 0.06

mass_clamp = 0.4
r_ball = 0.015

Fg = np.array([mass_clamp*9.81,0,0])
cor_CoM = np.array([0,0,0.12])
origin = np.array([0,0,0])
o_spring_wrist = np.array([d_spring, 0, 0])
cor_spring_claw = np.array([0, d_spring, 0])
o_cor = np.array([0, 0, h_cor])
cor_knot_wrist_right = np.array([d_spring, 0, h_knot_wrist])
cor_knot_wrist_left = np.array([-d_spring, 0, h_knot_wrist])
cor_knot_claw_right = np.array([0, d_spring, h_knot_upper])
cor_knot_claw_left = np.array([0, -d_spring, h_knot_upper])
o_knot = o_cor + cor_knot_wrist_right

def calculate_cor_knots(angle):
    cor_knot1 = rotzyx(cor_knot_wrist_right, angle, angle, angle)
    cor_knot2 = rotzyx(cor_knot_wrist_left, angle, angle, angle)
    cor_knot3 = rotzyx(cor_knot_claw_right, angle, angle, angle)
    cor_knot4 = rotzyx(cor_knot_claw_left, angle, angle, angle)
    return np.array([cor_knot1, cor_knot2, cor_knot3, cor_knot4])

def calculate_springs(angle):
    cor_knots = calculate_cor_knots(angle)
    spring1 = cor_knots[0] + o_cor - o_spring_wrist
    spring2 = cor_knots[1] + o_cor + o_spring_wrist
    spring3 = cor_knots[2] - cor_spring_claw
    spring4 = cor_knots[3] + cor_spring_claw
    return np.array([spring1, spring2, spring3, spring4])

def calculate_spring_extensions_directions(angle):
    springs0 = calculate_springs(0)
    springs_angle = calculate_springs(angle)
    extensions = np.zeros(len(springs0))
    directions = np.zeros((len(springs0), 3))
    for i in range(len(springs0)):
        extensions[i] = np.linalg.norm(springs0[i]) - np.linalg.norm(springs_angle[i])
        directions[i] = springs_angle[i] / np.linalg.norm(springs_angle[i])

    return extensions, directions


def calculate_spring_forces(angle, pre_tension):
    extensions, directions = calculate_spring_extensions_directions(angle)
    spring_forces = []
    mags = []
    for i in range(len(extensions)):
        spring_force = extensions[i]*k*Matrix(directions[i]) + Matrix(directions[i])*pre_tension
        mag = (spring_force[0]**2 + spring_force[1]**2 + spring_force[2]**2)**(1/2)

        # print(f"extension = {extensions[i]}")
        # print(f"direction = {Matrix(directions[i])}")
        # print(f"spring force {spring_force}")
        # print(f"First entry vector = {spring_force[0]}")
        # print(mag)

        spring_forces.append(spring_force)
        mags.append(mag)

    return spring_forces, mags

def calculate_moments_springs(angle, pre_tension):
    cor_knots = [Matrix(i) for i in calculate_cor_knots(angle)]
    spring_forces, mags = calculate_spring_forces(angle, pre_tension)
    moments_springs = []
    for i in range(len(cor_knots)):
        moment_spring = cor_knots[i].cross(spring_forces[i])
        moments_springs.append(moment_spring)
        # print(f"##### Spring {i} #####")
        # print(f"Arm of spring = {cor_knots[i]}")
        # print(f"Force of spring = {spring_forces[i]}")
        # print(f"Moment of spring = {moment_spring}")
    # print(moments_springs)
    return moments_springs
def calculate_Fg(angle):
    return rotzyx(Fg, angle, angle, angle)
def calculate_friction_force_moment_and_PoC(angle, pre_tension, c_friction):
    spring_forces, mags = calculate_spring_forces(angle, pre_tension)
    # print(f"spring forces : {spring_forces}")
    sum = Matrix([[0],[0],[0]])
    for i in spring_forces:
        # print(f"spring force {i}")
        sum += i
    N = (-sum - Matrix(calculate_Fg(angle)))/(1+c_friction)
    # print(f"N = {N}")
    N_dir = N/(N[0]**2 + N[1]**2 + N[2]**2)**(1/2)
    cor_PoC = N_dir * r_ball
    F_fric = N*c_friction
    M_fric = cor_PoC.cross(F_fric)
    return F_fric, M_fric, cor_PoC

def calculate_moment_gravity(angle):
    Fg_rot = Matrix(calculate_Fg(angle))
    r_g = Matrix(rotzyx(cor_CoM,angle,angle,angle))
    return r_g.cross(Fg_rot)

def solve_equation(angle, pre_tension, c_friction):
    M_springs = calculate_moments_springs(angle, pre_tension)
    F_fric, M_friction, cor_PoC = calculate_friction_force_moment_and_PoC(angle, pre_tension, c_friction)
    M_grav = calculate_moment_gravity(angle)
    sum_m = M_friction + M_grav
    # print(M_springs)
    for i in M_springs:
        sum_m += i

    print(sum_m)
    print(sum_m[0])
    ks = np.linspace(1,1000,1000)
    solutions0 = []
    solutions1 = []
    solutions2 = []
    for i in ks:
        solutions0.append(sum_m[0].subs(k, i))
        solutions1.append(sum_m[1].subs(k, i))
        solutions2.append(sum_m[2].subs(k, i))
    plt.plot(ks,solutions0)
    plt.plot(ks,solutions1)
    plt.plot(ks, solutions2)
    plt.show()

solve_equation(12,1,0.5)



def plot_at_max_angle(angle, pre_tension, c_friction):
    fig, xz = plt.subplots(figsize = (5,5))
    xz.set_aspect('equal')
    xz.set_title(f'wrist xz plane, rotation angles {angle*180/np.pi} deg')
    fig, yz = plt.subplots(figsize = (5,5))
    yz.set_aspect('equal')
    yz.set_title(f'wrist yz plane, rotation angles {angle*180/np.pi} deg')
    fig, xy = plt.subplots(figsize = (5,5))
    xy.set_aspect('equal')
    xy.set_title(f'wrist xy plane, rotation angles {angle*180/np.pi} deg')

    cor_CoM_rot = rotzyx(cor_CoM, angle, angle,angle)
    Fg_rot = calculate_Fg(angle)
    cor_knots = calculate_cor_knots(angle)
    springs = calculate_springs(angle)
    F_fric, M_fric, cor_PoC = calculate_friction_force_moment_and_PoC(angle, pre_tension, c_friction)

    plot_vecxyz(xz, yz, xy, o_cor, cor_spring_claw)
    plot_vecxyz(xz, yz, xy, o_cor, -cor_spring_claw)
    plot_vecxyz(xz,yz,xy, origin, o_cor)
    plot_vecxyz(xz, yz, xy, origin, o_spring_wrist)
    plot_vecxyz(xz, yz, xy, origin, -o_spring_wrist)

    plot_vecxyz(xz, yz, xy, o_spring_wrist, springs[0], color="blue")
    plot_vecxyz(xz, yz, xy, -o_spring_wrist, springs[1], color="blue")
    plot_vecxyz(xz, yz, xy, o_cor+cor_spring_claw, springs[2], color="blue")
    plot_vecxyz(xz, yz, xy, o_cor-cor_spring_claw, springs[3], color="blue")

    plot_vecxyz(xz,yz,xy, o_cor, cor_knots[0], color="orange")
    plot_vecxyz(xz,yz,xy, o_cor, cor_knots[1], color="orange")
    plot_vecxyz(xz,yz,xy, o_cor, cor_knots[2], color="orange")
    plot_vecxyz(xz,yz,xy, o_cor, cor_knots[3], color="orange")

    plot_vecxyz(xz,yz,xy, o_cor, cor_CoM_rot, color="orange")
    plot_vecxyz(xz,yz,xy, o_cor + cor_CoM_rot, Fg_rot/100, color="blue")
    # plot_vecxyz(xz,yz,xy, o_cor, cor_PoC, color="orange")

    plt.show()


plot_at_max_angle(12*np.pi/180, 1, 0.5)
plot_at_max_angle(0, 1, 0.5)
