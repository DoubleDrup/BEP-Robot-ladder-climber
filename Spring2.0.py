import numpy as np
import matplotlib.pyplot as plt
from Math import rotzyx
from Plot_functions import *
from sympy import Symbol, Matrix, solve, symbols, linsolve, sqrt

k = Symbol('k')
x,y,z = symbols('x, y, z')
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
    cor_knot1 = rotzyx(cor_knot_wrist_right, 0, angle, angle)
    cor_knot2 = rotzyx(cor_knot_wrist_left, 0, angle, angle)
    cor_knot3 = rotzyx(cor_knot_claw_right, 0, angle, angle)
    cor_knot4 = rotzyx(cor_knot_claw_left, 0, angle, angle)
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
        directions[i] = -springs_angle[i] / np.linalg.norm(springs_angle[i])

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
    # print(f"spring forces:  {spring_forces}")
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
        # # print(f"Moment of spring = {moment_spring}")
    # print(moments_springs)
    return moments_springs
def calculate_Fg(angle):
    return rotzyx(Fg, angle, 0, 0)
def calculate_friction_force(angle, pre_tension, c_friction):
    spring_forces, mags = calculate_spring_forces(angle, pre_tension)
    # print(f"spring forces : {spring_forces}")
    sum = Matrix([[0],[0],[0]])
    for i in spring_forces:
        # print(f"spring force {i}")
        sum += i
    N = (-sum - Matrix(calculate_Fg(angle)))/(1+c_friction)
    # print(f"N = {N}")
    N_len = (N[0]**2 + N[1]**2 + N[2]**2)**(1/2)
    F_fric = N_len*c_friction
    return F_fric

def calculate_moment_gravity(angle):
    Fg_rot = Matrix(calculate_Fg(angle))
    r_g = Matrix(rotzyx(cor_CoM,0, angle, angle))
    return r_g.cross(Fg_rot)

def solve_equation(angle, pre_tension, c_friction):
    spring_forces, mags = calculate_spring_forces(angle, pre_tension)
    Fg = Matrix(calculate_Fg(angle))
    M_springs = calculate_moments_springs(angle, pre_tension)
    M_grav = calculate_moment_gravity(angle)
    sum_m = M_grav
    sum_f = Fg
    # print(M_springs)
    for i in range(len(M_springs)):
        sum_m += M_springs[i]
        sum_f += spring_forces[i]
    print(f"spring_forces : {spring_forces}")
    print(sum_m)
    print(sum_f)
    r_unknown = Matrix([x, y, z])
    m_unknown = r_unknown.cross(-sum_f)
    sum0 = m_unknown - sum_m
    print(m_unknown)

    r_known_unreadable = linsolve([sum0[0], sum0[1], sum0[2]],(x,y,z))
    x_r_known,y_r_known,z_r_known = r_known_unreadable.args[0]
    r_len = sqrt(x_r_known**2 + y_r_known**2 + z_r_known**2)
    # k_calculated = solve(r_len-r_ball, k)

    ks = np.linspace(0,1000, 100)
    rs = np.zeros(len(ks))
    for i in range(len(ks)):
        rs[i] = r_len.subs(k, ks[i])
    plt.plot(ks, rs)
    plt.show()

    print(f"r_known = {r_known_unreadable}")
    print(f"x_r_known = {x_r_known}")
    print(f"y_r_known = {y_r_known}")
    print(f"z_r_known = {z_r_known}")
    print(f"r_len = {r_len}")
    # print(f"k calculated = {k_calculated}")



solve_equation(12*np.pi/180,10,0.5)


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

    cor_CoM_rot = rotzyx(cor_CoM, 0, angle, angle)
    Fg_rot = calculate_Fg(angle)
    cor_knots = calculate_cor_knots(angle)
    springs = calculate_springs(angle)
    F_fric = calculate_friction_force(angle, pre_tension, c_friction)
    spring_forces, mags = calculate_spring_forces(angle, pre_tension)

    for i in range(len(spring_forces)):
        spring_force = spring_forces[i].subs(k, 10)
        force = np.array([spring_force[0],spring_force[1],spring_force[2]])
        print(force)
        plot_vecxyz(xz,yz,xy, cor_knots[i], force, color="green")


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
