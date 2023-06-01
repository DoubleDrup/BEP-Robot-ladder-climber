import numpy as np
import sympy as sp
from sympy.vector import CoordSys3D, Vector
from sympy.solvers import solve
N = CoordSys3D('N')

forces_dict = {}
vars = []

def add_force(name, position, force=None):
    if isinstance(force, type(None)):
        symbols = []
        for i in range(3):
            global_vars = globals()
            global_vars[f'{name}{i+1}'] = sp.symbols(f"{name}{i + 1}")
            symbols.append(sp.symbols(f"{name}{i + 1}"))
        F = sp.MatrixSymbol(f'F{name}', 3, 1)
    else:
        F = sp.Matrix([[force[0]],[force[1]],[force[2]]])
    pos = sp.Matrix([[position[0]],[position[1]],[position[2]]])
    forces_dict[name] = [pos, F]

def moment(around, force_name):
    return (forces_dict[force_name][0]-forces_dict[around][0]).cross(forces_dict[force_name][1])


c = np.array([0,-1,0])
a = np.array([0,0,0])
add_force('a', a)
add_force('c', c)
print(forces_dict)
mz = moment('a', 'c')
print(mz)

