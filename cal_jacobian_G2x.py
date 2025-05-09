import sympy as sp
import numpy as np
from sympy import sin, cos, tan, Matrix
from sympy.vector import Vector
from sympy.vector import CoordSys3D
from sympy.codegen.cfunctions import expm1
from sympy.codegen.rewriting import create_expand_pow_optimization

def replace_pow(expr):
    """Replace pow(x,n) with explicit multiplication for n=2"""
    return expr.replace(sp.Pow, lambda x, y: x*x if y == 2 else sp.Pow(x,y))   

expand_opt = create_expand_pow_optimization(3)
def replace_pow_with_mul(expr):
    """Recursively replace all pow(x,2) with x*x in a SymPy expression"""
    if expr.is_Atom:  # Base case: numbers/symbols
        return expr
    elif expr.func == sp.Pow and expr.exp == 2:  # Replace pow(x,2) with x*x
        return expr.base * expr.base
    else:  # Recursively apply to all arguments
        return expr.func(*[replace_pow_with_mul(arg) for arg in expr.args])
    

def print_eigen_matrix(matrix, name):
    """Prints a SymPy matrix in Eigen C++ format"""
    rows, cols = matrix.shape
    print(f"Eigen::MatrixXd {name}({rows}, {cols});")
    print(f"{name} << ", end="")
    
    for i in range(rows):
        if i > 0:
            print("    ", end="")  # Indentation for multi-line
        for j in range(cols):
            expr = matrix[i,j]
            # Print element with comma (except last element)
            print(f"{sp.ccode(expr)}", end=", " if j < cols-1 else "")
        
        # Print row separator (except last row)
        print(", " if i < rows-1 else "", end="\n")
    print(";") 
    

# Define state and measurement symbols
x = sp.symbols('x1:22', real=True)
z = sp.symbols('z1:7', real=True)
x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13, x14, x15, x16, x17, x18, x19, x20, x21 = x
z1, z2, z3, z4, z5, z6 = z

R_wb = sp.Matrix([
    [sp.cos(x6)*sp.cos(x5) - sp.sin(x4)*sp.sin(x6)*sp.sin(x5), -sp.cos(x4)*sp.sin(x6), sp.cos(x6)*sp.sin(x5) + sp.cos(x5)*sp.sin(x4)*sp.sin(x6)],
    [sp.cos(x5)*sp.sin(x6) + sp.cos(x6)*sp.sin(x4)*sp.sin(x5),  sp.cos(x4)*sp.cos(x6), sp.sin(x6)*sp.sin(x5) - sp.cos(x6)*sp.cos(x5)*sp.sin(x4)],
    [-sp.cos(x4)*sp.sin(x5),                               sp.sin(x4),                sp.cos(x4)*sp.cos(x5)]
])

R_wk = sp.Matrix([
    [sp.cos(x21)*sp.cos(x20) - sp.sin(x19)*sp.sin(x21)*sp.sin(x20), -sp.cos(x19)*sp.sin(x21), sp.cos(x21)*sp.sin(x20) + sp.cos(x20)*sp.sin(x19)*sp.sin(x21)],
    [sp.cos(x20)*sp.sin(x21) + sp.cos(x21)*sp.sin(x19)*sp.sin(x20),  sp.cos(x19)*sp.cos(x21), sp.sin(x21)*sp.sin(x20) - sp.cos(x21)*sp.cos(x20)*sp.sin(x19)],
    [-sp.cos(x19)*sp.sin(x20),                                   sp.sin(x19),                 sp.cos(x19)*sp.cos(x20)]
])

p_b = sp.Matrix(x[0:3])
p_k = sp.Matrix(x[15:18])

delta = p_b - p_k
z_pos = R_wk.T * delta
R_kb = sp.sympify(R_wk.T * R_wb)
# print("R_kb", R_kb)
phi = sp.asin(R_kb[2, 1])
theta = sp.atan2(-R_kb[2, 0], R_kb[2, 2])
psi = sp.atan2(-R_kb[0, 1], R_kb[1, 1])

z_sym = sp.Matrix([z_pos[0], z_pos[1], z_pos[2], phi, theta, psi])
dzdX= z_sym.jacobian(sp.Matrix(x))
# print("dzdX", dzdX)
# dzdX = sp.simplify(z_sym.jacobian(sp.Matrix(x)))
# print("dzdX", dzdX)
# dzdX = sp.simplify(sp.Matrix.vstack(z_pos, sp.Matrix([phi, theta, psi])).jacobian(sp.Matrix(x)))
# dzdX  # symbolic Jacobian


print_eigen_matrix(dzdX, "Ct")