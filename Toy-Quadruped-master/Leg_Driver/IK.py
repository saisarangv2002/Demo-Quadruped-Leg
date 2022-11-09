import numpy as np

T = 90
C = 40
R = 40
CU = 63.654148805222384
O = 65
L = 120

pi = np.pi

def findToe(alpha, beta):

    f = np.sqrt(O*O + T*T - 2*O*T*np.cos(alpha))

    theta = np.arcsin(T*np.sin(alpha)/f)
    delta = np.arcsin(O*np.sin(alpha)/f)

    epsilon = pi - beta - theta

    t = np.sqrt(f*f + C*C - 2*f*C*np.cos(epsilon))

    x = np.arccos((R*R + CU*CU - t*t)/(2*R*CU))

    phi = np.arcsin(CU*np.sin(x)/t) + np.arcsin(C*np.sin(epsilon)/t)

    gamma = 2*pi - delta - phi - 1.9198

    l_x = T*np.cos(alpha) - L*np.cos(gamma - alpha)
    l_y = T*np.sin(alpha) + L*np.sin(gamma - alpha)

    return l_x, l_y

def Func(alpha, beta, X, Y):

    f = np.array(findToe(alpha, beta))
    f[0] -= X
    f[1] -= Y

    return f

def Jacobian(alpha, beta):

    J = np.zeros((2,2))

    X, Y = findToe(alpha, beta)
    Xa, Ya = findToe(alpha + 0.001, beta)
    Xb, Yb = findToe(alpha, beta + 0.001)

    J[0][0] = (Xa - X)/0.001
    J[1][0] = (Ya - Y)/0.001
    J[0][1] = (Xb - X)/0.001
    J[1][1] = (Yb - Y)/0.001

    return J

def findSol(X, Y, alpha_guess = pi/4, beta_guess = 0):


    alpha_guess = pi/4
    beta_guess = 0

    x = np.array([alpha_guess, beta_guess])

    for i in range(3):

        f = Func(x[0], x[1], X, Y)

        J = Jacobian(x[0], x[1])

        k = np.dot(np.linalg.inv(J), f)

        x = x - k

    return x

z = findSol(0, 160)

print(findToe(z[0], z[1]))

