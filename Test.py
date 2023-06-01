from sympy import MatrixSymbol, Eq, solve

# create MatrixSymbol for the variables
X = MatrixSymbol('X', 2, 1)

# create the matrix equation
A = [[1, 2], [3, 4]]
B = [[5], [6]]
eq = Eq(X, B)

# solve the equation for X
sol = solve(eq, X)

# print the solution
print(sol)