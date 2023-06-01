import numpy as np
import matplotlib.pyplot as plt

from Math import rotzyx
from Plot_functions import plot_vec

########################################################################################################################
# Robot dimensions
arm_lenght = 0.35
body_width = 0.30
body_height = 0.60
grip_offset = 0.05       # Distance from body to hand/feet position on ladder in x direction
dist_ladder = 0.2
max_arm_tol = 0.02      # Prevent full stretch of arms

# Ladder dimensions
sport_dist = 0.25
sport_width = 0.50
ladder_height = 3

# Plot settings
space_ladder = 0.1


def sport_heights():
    heights = np.zeros(int(ladder_height/sport_dist))
    for i in range(len(heights)):
        heights[i] = -ladder_height/2 + i * sport_dist
    return heights

def body_corners(cog):
    a = np.array([-body_width/2, body_height/2, 0])
    b = np.array([body_width/2, body_height/2, 0])
    left_up = cog + a
    right_up = cog + b
    right_down = cog - a
    left_down = cog - b
    corners = np.array([left_up, right_up, right_down, left_down])
    return corners

def closest_sport_above_below(point):
    sh = sport_heights()
    aboves = [sport-point[1] for sport in sh if sport-point[1] > 0]
    belows = [sport-point[1] for sport in sh if sport-point[1] < 0]
    if aboves:
        above = min(aboves)
    else:
        above = None
    if belows:
        below = max(belows)
    else:
        below = None
    return above, below

def start_pos():
    x_left, x_right = -(grip_offset + body_width/2), grip_offset + body_width/2
    dist_xy = (arm_lenght*2)**2 - dist_ladder**2
    dist_y = dist_xy**2 - grip_offset**2
    cog = np.array([0, dist_y+body_height/2-max_arm_tol, dist_ladder])
    bc = body_corners(cog)
    left_up_hand = np.array([x_left, closest_sport_above_below(bc[0])[0], 0])
    right_up_hand = np.array([x_right, closest_sport_above_below(bc[1]+dist_y)[1], 0])
    right_down_hand = np.array([x_right, closest_sport_above_below(bc[2])[0], 0])
    left_down_hand = np.array([x_left, closest_sport_above_below(bc[3]-dist_y)[1], 0])
    hand_pos = np.array([left_up_hand, right_up_hand, right_down_hand, left_down_hand])
    return cog, bc, hand_pos


def plot_ladder(plot):
    heights = sport_heights()
    for height in heights:
        plot_vec(plot, [-sport_width/2, height, 0], [sport_width,0,0])
    plot_vec(plot, [-sport_width/2,-ladder_height/2,0], [0,ladder_height,0])
    plot_vec(plot, [sport_width / 2, -ladder_height / 2, 0], [0, ladder_height, 0])


def plot_body(plot, cog):
    corners = body_corners(cog)
    plot.plot(cog[0], cog[1], color='red', marker = 'o')
    for i in range(len(corners)):
        if i != len(corners)-1:
            plot_vec(plot, corners[i], corners[i+1]-corners[i])
        else:
            plot_vec(plot, corners[i], corners[0]-corners[i])

def plot_arms(plot, corners, hands):
    for i in range(len(corners)):
        plot_vec(plot, corners[i], hands[i] - corners[i])












fig, ax = plt.subplots(figsize = (5,5))
ax.set_xlim([-(sport_width/2+space_ladder), sport_width/2+space_ladder])
ax.set_ylim([-(ladder_height/2), ladder_height/2])
ax.set_aspect('equal')
ax.set_title('Free body diagram of ladder climber')
# ax.grid()
plot_ladder(ax)
print(start_pos())
cog, corners, hands = start_pos()
plot_body(ax, cog)
plot_arms(ax, corners, hands)
plt.show()