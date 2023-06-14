import numpy as np
import matplotlib.pyplot as plt

from Math import rotzyx
from Plot_functions import plot_vecxy

########################################################################################################################
# Robot dimensions
arm_lenght = 0.35
body_width = 0.2
body_height = 1
grip_offset = 0.05       # Distance from body to hand/feet position on ladder in x direction
dist_ladder = 0.2
max_arm_tol = 0.02      # Prevent full stretch of arms

# Ladder dimensions
sport_dist = 0.25
sport_width = 1
ladder_height = 6

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
    aboves = [sport for sport in sh if sport-point[1] > 0]
    belows = [sport for sport in sh if sport-point[1] < 0]
    print(f"aboves {aboves}")
    print(f"belows {belows}")
    if aboves:
        above = min(aboves)
    else:
        above = max(belows)
    if belows:
        below = max(belows)
    else:
        below = min(aboves)
    print(f"corner: {point}, above {above}, below {below}")
    return above, below

def start_pos():
    x_left, x_right = -(grip_offset + body_width/2), grip_offset + body_width/2
    dist_xy = np.sqrt((arm_lenght*2)**2 - dist_ladder**2)
    print(dist_xy)
    dist_y = np.sqrt(dist_xy**2 - grip_offset**2)
    print(dist_y)
    cog = np.array([0, dist_y+body_height/2-max_arm_tol-ladder_height/2, dist_ladder])
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
        plot_vecxy(plot, [-sport_width / 2, height, 0], [sport_width, 0, 0])
    plot_vecxy(plot, [-sport_width / 2, -ladder_height / 2, 0], [0, ladder_height, 0])
    plot_vecxy(plot, [sport_width / 2, -ladder_height / 2, 0], [0, ladder_height, 0])


def plot_body(plot, cog):
    corners = body_corners(cog)
    plot.plot(cog[0], cog[1], color='orange', marker = 'o')
    for i in range(len(corners)):
        if i != len(corners)-1:
            plot_vecxy(plot, corners[i], corners[i + 1] - corners[i], color="orange")
        else:
            plot_vecxy(plot, corners[i], corners[0] - corners[i], color="orange")

def plot_arms(plot, corners, hands):
    for i in range(len(hands)):
        plot_vecxy(plot, corners[i], hands[i] - corners[i], color="y")












fig, ax = plt.subplots(figsize = (5,5))
ax.set_xlim([-(sport_width/2+space_ladder), sport_width/2+space_ladder])
ax.set_ylim([-(ladder_height/2), ladder_height/2])
ax.set_aspect('equal')
ax.set_title('Ladder climber')
# ax.grid()
plot_ladder(ax)

cog, corners, hands = start_pos()
plot_body(ax, cog)
plot_arms(ax, corners, hands)
plt.show()