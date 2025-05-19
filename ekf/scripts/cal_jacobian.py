import sympy as sp
import numpy as np
from sympy import sin, cos, tan, Matrix
from sympy.vector import Vector
from sympy.vector import CoordSys3D
from sympy.codegen.cfunctions import expm1
from sympy.codegen.rewriting import create_expand_pow_optimization
def linearize_uav_dynamics():
    # Define symbolic state variables
    # Position (x, y, z)
    # x, y, z = sp.symbols('x y z')
    # # Orientation (roll, pitch, yaw)
    # phi, theta, psi = sp.symbols('phi theta psi')
    # # Velocity (vx, vy, vz)
    # vx, vy, vz = sp.symbols('vx vy vz')
    # # Gyro bias (b_p, b_q, b_r)
    # b_p, b_q, b_r = sp.symbols('b_p b_q b_r')
    # # Accelerometer bias (b_ax, b_ay, b_az)
    # b_ax, b_ay, b_az = sp.symbols('b_ax b_ay b_az')
    
    # # Define control inputs (angular rates and thrust)
    # p, q, r = sp.symbols('p q r')  # Angular rates (roll, pitch, yaw rates)
    # ax, ay, az = sp.symbols('ax ay az')  # Specific force (acceleration - gravity)
    
    # # Define gravity
    # g = sp.symbols('g')  # Gravity constant
    
    # # Define the state vector X = [x1,x2,x3,x4,x5] x1 = [x,y,z], x2 = [phi,theta,psi], x3 = [vx,vy,vz], x4 = [b_p,b_q,b_r], x5 = [b_ax,b_ay,b_az]
    # X = Matrix([x, y, z, phi, theta, psi, vx, vy, vz, b_p, b_q, b_r, b_ax, b_ay, b_az])
    x = sp.symbols('x1:16')  # x1-x15
    X = Matrix(x)  # Shape: 15×1
    n = sp.symbols('n1:13')  # n1-n12
    N = Matrix(n)  # Shape: 12×1
    u = sp.symbols('u1:7')  # u1-u6 [p, q, r, ax, ay, az]
    U = Matrix(u)  # Shape: 6×1
    # Define gravity constant
    g = sp.symbols('g')
    pos = X[:3]
    phi, theta, psi = X[3], X[4], X[5]
    # Velocity (x7-x9)
    vel = Matrix(X[6:9] )
     # Gyro biases (x10-x12)
    b_gyro = Matrix(X[9:12])
    n_gyro = Matrix(N[:3])
    print("n_gyro", n_gyro) 
    # Accelerometer biases (x13-x15)
    b_accel = Matrix(X[12:15])
    n_accel = Matrix(N[3:6])
    G = Matrix([[cos(theta), 0, -cos(phi)*sin(theta)],
                    [0, 1, sin(phi)],
                    [sin(theta), 0, cos(phi)*cos(theta)]])
    G_inv = G.inv()
    # print("G_inv", G_inv)
    Rot = Matrix([[cos(psi)*cos(theta)-sin(psi)*sin(phi)*sin(theta), -cos(phi)*sin(psi), cos(psi)*sin(theta)+cos(theta)*sin(psi)*sin(phi)],
                 [cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta), cos(phi)*cos(psi), sin(psi)*sin(theta)-cos(psi)*cos(theta)*sin(phi)],
                 [-cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta)]])
    # print G_inv in C++ eigen format
    
    
    
    #process model for gyroscope
    # w_m = U[:3] + b_gyro + N[:3]
    # a_m =
    w_m = Matrix(U[:3])
    # process model for accelerometer
    a_m = Matrix(U[3:6])
    
    dpos = vel 
    # print("size of dpos", dpos.shape)
    dang = G_inv * (w_m - b_gyro- n_gyro)
    # print("size of dang", dang.shape)
    dvel = Matrix([0,0,g]) + Rot * (a_m - b_accel - n_accel)
    # print("size of dvel", dvel.shape)
    db_gyro = Matrix(N[6:9])
    db_accel = Matrix(N[9:12])
    # Define the state transition function
    f = Matrix.vstack(dpos, dang, dvel, db_gyro, db_accel)
    print ("f", f.shape)
    A_j = f.jacobian(X)  # Jacobian of f with respect to X
    U_j = f.jacobian(N)  # Jacobian of f with respect to N noise
    # B_ = f.jacobian(U)  # Jacobian of f with respect to U control input
    return A_j, U_j

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
        print(" \\" if i < rows-1 else "", end="\n")
    print(";") 
    

# Example usage
if __name__ == "__main__":
    # Get the linearization function and symbolic matrices
    A,U = linearize_uav_dynamics()
    # Print the Jacobian matrices in Eigen format
    print_eigen_matrix(A, "A_j")
    print_eigen_matrix(U, "U_j")

    
    