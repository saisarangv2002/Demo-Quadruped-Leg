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

def Jacobian(theta, beta, f, t, epsilon, x, alpha, gamma, delta, phi):

    J = np.zeros((10, 10))

    # First column
    J[4][0] = 1
    J[7][0] = f*np.cos(theta)

    # Second Column
    J[4][1] = 1

    # Third Column
    J[5][2] = 2*C*np.cos(epsilon) - 2*f
    J[6][2] = 2*f
    J[7][2] = np.sin(theta)
    J[8][2] = np.cos(theta)

    # Fourth Column
    J[3][3] = (1/(np.sqrt(t*t - CU*CU*np.sin(x)*np.sin(x))) + 1/(np.sqrt(t*t - C*C*np.sin(epsilon)*np.sin(epsilon))))/t
    J[5][3] = 2*t
    J[9][3] = -2*t

    # Fifth Column
    J[3][4] = -t*np.cos(epsilon)/(np.sqrt(t*t - C*C*np.sin(epsilon)*np.sin(epsilon)))
    J[4][4] = 1
    J[5][4] = -2*f*C*np.sin(epsilon)

    # Sixth Column
    J[3][5] = -t*np.cos(x)/(np.sqrt(t*t - CU*CU*np.sin(x)*np.sin(x)))
    J[9][5] = 2*R*CU*np.sin(x)

    # Seventh Column
    J[1][6] = -T*np.sin(alpha) - L*np.sin(gamma-alpha)
    J[2][6] = T*np.cos(alpha) - L*np.cos(gamma - alpha)
    J[6][6] = -2*O*T*np.sin(alpha)
    J[7][6] = -T*np.cos(alpha)
    J[8][6] = -O*np.cos(alpha)

    # Eigth Column
    J[0][7] = -1
    J[1][7] = L*np.sin(gamma - alpha)
    J[3][7] = L*np.cos(gamma - alpha)

    # Ninth Column
    J[0][8] = -1
    J[8][8] = f*np.cos(delta)

    # Tenth Column
    J[0][9] = -1
    J[3][9] = 1

    return J

def nonLinearSystem(X, Y, theta, beta, f, t, epsilon, x, alpha, gamma, delta, phi):

    F = np.zeros(10)

    F[0] = 2*pi - gamma - delta - phi - (110/180)*pi
    F[1] = T*np.cos(alpha) - L*np.cos(gamma - alpha) - X
    F[2] = T*np.sin(alpha) + L*np.sin(gamma - alpha) - Y
    F[3] = phi - np.arcsin(CU*np.sin(x)/t) - np.arcsin(C*np.sin(epsilon)/t)
    F[4] = epsilon - pi + theta + beta
    F[5] = t*t - f*f - C*C + 2*f*C*np.cos(epsilon)
    F[6] = f*f - O*O - T*T + 2*O*T*np.cos(alpha)
    F[7] = f*np.sin(theta) - T*np.sin(alpha)
    F[8] = f*np.sin(delta) - O*np.sin(alpha)
    F[9] = R*R + CU*CU - 2*R*CU*np.cos(x) - t*t

    return F

def findSol(X, Y):

    theta = pi*88.8/180
    beta = 0
    f = CU
    t = 75.899
    epsilon = pi - theta
    x = epsilon
    alpha = pi/4
    gamma = pi*115/180
    delta = pi*46.2/180
    phi = pi*88.8/180

    print(findToe(alpha, beta))

    z = np.array([theta, beta, f, t, epsilon, x, alpha, gamma, delta, phi])

    #F = nonLinearSystem(X, Y, theta, beta, f, t, epsilon, x, alpha, gamma, delta, phi)
    #J = Jacobian(theta, beta, f, t, epsilon, x, alpha, gamma, delta, phi)

    for i in range(3):

        F = nonLinearSystem(X, Y, theta, beta, f, t, epsilon, x, alpha, gamma, delta, phi)
        J = Jacobian(theta, beta, f, t, epsilon, x, alpha, gamma, delta, phi)

        #print(F)

        k = np.dot(np.linalg.inv(J), F)
        print(k)
        z -= k

        theta = z[0]
        beta = z[1]
        f = z[2]
        t = z[3]
        epsilon = z[4]
        x = z[5]
        alpha = z[6]
        gamma = z[7]
        delta = z[8]
        phi = z[9]

        print("Iteration " + str(i) + ": ")
        print(z)
        print("Toe Position: ")
        print(findToe(z[6], z[1]))

findSol(22.5, 176.4)