import numpy as np
from Math import rotzyx
import matplotlib.pyplot as plt

### Lenghts (meters) ###
arm = [2,1,1,2] # upper arm up, upper arm down, lower arm up, lower arm down
L = 3 # body length
h = 0.5 # body heigth
lz = -3 # distance to ladder

### angles (degrees) ###
alphas = [60,90,225,270] # shoulder angles
betas = [0,0,0,0]

### coordinates ###
Z = np.array([0,0,0])
AB = Z + np.array([-h,L,0])
CD = Z + np.array([-h,-L,0])
elbows = np.zeros([3, 4])
elbows[:, 0] = arm[0] * rotzyx(AB, np.deg2rad(alphas[0]))/np.linalg.norm(AB)
elbows[:, 1] = arm[0] * rotzyx(AB, np.deg2rad(alphas[1]))/np.linalg.norm(AB)
elbows[:, 2] = arm[1] * rotzyx(CD, np.deg2rad(alphas[2]))/np.linalg.norm(CD)
elbows[:, 3] = arm[1] * rotzyx(CD, np.deg2rad(alphas[3]))/np.linalg.norm(CD)

def plot_vec(basevec, endvec):
    Base = np.array([basevec[0], basevec[1]])
    end = np.array([endvec[0],endvec[1]])
    ax.arrow(Base[0],Base[1],end[0],end[1])
    # mid = (start+end)/2
    # ax.annotate(f'vector {startvec, endvec}', xy=start+(mid/2), xytext=(mid/2+np.array([0.5, 0.5])), fontsize=12, arrowprops=dict(arrowstyle='->', lw=1.5, color='g'))

def calc_hand(vec, line, len):
    """Calculates the 2 possible vectors in 2D where a vector could be placed, where the tip is on the line x = line and
    the tail is a np.array. len is the length of the vector that should be connecting the two. it returns the distance
    in y."""
    d = line - vec[0]
    if d > len:
        print("length of arm is too short, cannot reach ladder in this shoulder position")
        return vec + np.array([-len,0,0])
    else:
        y_dis = np.sqrt(len**2 - d**2)
        # print(y_dis)
        return np.array([d, y_dis, 0])

hands = np.zeros([3,4])
hands[:,0] = calc_hand(elbows[:,0]+AB, lz, arm[2])
hands[:,1] = calc_hand(elbows[:,1]+AB, lz, arm[2])
hands[:,2] = calc_hand(elbows[:,2]+CD, lz, arm[3]) * np.array([1,-1,1])
hands[:,3] = calc_hand(elbows[:,3]+CD, lz, arm[3]) * np.array([1,-1,1])
### plot setup ###
fig, ax = plt.subplots(figsize = (5,5))
# lim = max_length = np.max(np.linalg.norm(elbows, axis=0))+0.2
lim = 8
ax.set_xlim([-lim, lim])
ax.set_ylim([-lim, lim])
ax.set_aspect('equal')
ax.set_title('Free body diagram of ladder climber')
ax.grid()

### vectors ###
plot_vec(Z, AB)
plot_vec(Z, CD)
plot_vec(AB, elbows[:, 0])
plot_vec(AB, elbows[:, 1])
plot_vec(CD, elbows[:, 2])
plot_vec(CD, elbows[:, 3])

print(AB, elbows[:, 0], AB + elbows[:,0], hands[:, 0], np.linalg.norm(hands[:,0]))

plot_vec(elbows[:, 0] + AB, hands[:, 0])
plot_vec(elbows[:, 1] + AB, hands[:, 1])
plot_vec(elbows[:, 2] + CD, hands[:, 2])
plot_vec(elbows[:, 3] + CD, hands[:, 3])


plt.show()

 