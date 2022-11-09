import numpy as np
import matplotlib.pyplot as plt

T = 90
C = 40
R = 40
CU = 63.654148805222384
O = 65
L = 120

pi = np.pi

def betaLim(alpha):

    f = np.sqrt(O*O + T*T - 2*O*T*np.cos(alpha))

    theta = np.arcsin(T*np.sin(alpha)/f)

    betaHIGH = pi - theta - np.arccos(((C+CU)*(C+CU) + f*f - R*R)/(2*(C+CU)*f))
    betaLOW = pi - theta - np.arccos((C*C + f*f - (R+CU)*(R+CU))/(2*C*f))

    return betaLOW, betaHIGH

alphaList = np.linspace(pi/2, pi/4, 100)
counter = 0
color_seq = "rbgkm"*20

points_alpha = []
points_beta = []

for alpha in alphaList:
    beta_range = betaLim(alpha)
    betaList = np.linspace(beta_range[0] + 0.05, beta_range[1] - 0.05, num=100)

    for beta in betaList:
        points_alpha.append(alpha*180/pi)
        points_beta.append(beta*180/pi)

plt.plot(points_alpha, points_beta)

plt.xlabel(r'$\alpha$')
plt.ylabel(r'$\beta$')
plt.show()

    
