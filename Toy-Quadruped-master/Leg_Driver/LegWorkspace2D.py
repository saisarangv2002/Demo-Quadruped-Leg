import numpy as np
import matplotlib.pyplot as plt

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

def betaLim(alpha):

    f = np.sqrt(O*O + T*T - 2*O*T*np.cos(alpha))

    theta = np.arcsin(T*np.sin(alpha)/f)

    betaHIGH = pi - theta - np.arccos(((C+CU)*(C+CU) + f*f - R*R)/(2*(C+CU)*f))
    betaLOW = pi - theta - np.arccos((C*C + f*f - (R+CU)*(R+CU))/(2*C*f))

    return betaLOW, betaHIGH

alphaList = np.linspace(pi/2, pi/4, 50)
counter = 0
color_seq = "rbgkm"*20

for alpha in alphaList:
    beta_range = betaLim(alpha)
    betaList = np.linspace(beta_range[0] + 0.05, beta_range[1] - 0.05, num=100)

    x_list = []
    y_list = []

    for beta in betaList:

        x,y = findToe(alpha, beta)
        x_list.append(x)
        y_list.append(-y)

    plt.scatter(x_list, y_list, color = color_seq[counter])
    counter = counter + 1

plt.show()

    
