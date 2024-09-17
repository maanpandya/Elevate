import numpy as np
from AirfoildataGenerator import get_aifoildata

# question = ('Cd', 'Cl', 'ClCd', 'AlphaClCd', 'ClCdmax')
# re_angs = [[10**5, 5]]
# answer = get_aifoildata('NACA 2412', re_angs, question)
# print(answer)

# Global Inputs
P = 52199
V = 49.17        # 10 m/s at hover according to https://www.scopus.com/inward/record.uri?eid=2-s2.0-85119987670&doi=10.3390%2fapp112311083&partnerID=40&md5=1aa6253871a54672b3f55d84f0cce64f (p11)
D = 0.3 * 5.75
D_hub = 0.3
B = 2
a = 340
M_tip = 2400 * D * np.pi / 60 / a      # No more than 0.6 to reduce noise.

rho = 1.225
dyn_viscosity = 1.789e-5
kin_viscosity = dyn_viscosity/rho

# Program Inputs
nr_sect = 20
zeta = 0.1
zeta_prev = 100
zeta_acc = 0.01
Re = np.full(nr_sect, 100000)
foil = 'NACA 2412'

# Calculations
R = D/2
omega = a*M_tip/R
v_ratio = V/(omega*R)
P_c = 2 * P / (rho * V**3 * np.pi * R**2)

# Section Calculations
r = np.linspace(D_hub/2, D/2, nr_sect)
xi = r/R
#print(xi)

while abs(zeta_prev/(zeta+0.0001) - 1) >= zeta_acc:
    phi_tip = np.arctan2(v_ratio*(1+zeta/2), 1)
    f = (B/2)*(1-xi)/np.sin(phi_tip)
    F = (2/np.pi)*np.arccos(np.exp(-f))
    phi = np.arctan2(np.tan(phi_tip), xi)
    print(phi)
    G = F * np.cos(phi) * np.sin(phi)
    x = omega * r / V

    re_angs = np.insert(np.expand_dims(Re, axis=1), 1, np.zeros((1, nr_sect)), axis=1)
    re_angs[-1, 0] = 100000
    data = get_aifoildata(foil, re_angs, ('Cl', 'Cd'))
    alpha, cl, cdcl = np.radians(data[:, 0]), data[:, 1], 1/data[:, 2]
    cd0=cdcl*cl

    Wc = 4 * np.pi * v_ratio * G * V * R * zeta / (cl * B)
    Re_prv = Re
    Re = Wc / kin_viscosity
    print('ReRatio', Re_prv/(Re+1))             #Fix this!!!
    param1 = 1 - cdcl * np.tan(phi)
    param2 = 1 + cdcl / np.tan(phi)
    ax_interf = zeta / 2 * (np.cos(phi))**2 * param1
    rot_interf = zeta / (2 * x) * np.cos(phi) * np.sin(phi) * param2
    W = V * (1 + ax_interf) / np.sin(phi)
    c = Wc / W
    twist = phi + alpha

    i1 = 4 * xi * G * param1
    i2 = v_ratio * i1 / (2 * xi) * param2 * np.sin(phi) * np.cos(phi)
    j1 = 4 * xi * G * param2
    j2 = j1 / 2 * param1 * (np.cos(phi))**2

    I1 = np.trapz(i1, x=xi)
    I2 = np.trapz(i2, x=xi)
    J1 = np.trapz(j1, x=xi)
    J2 = np.trapz(j2, x=xi)

    zeta_prev = zeta
    zeta = -J1 / (2 * J2) + np.sqrt((J1 / (2 * J2))**2 + P_c / J2)
    print('ZetaRatio', zeta_prev/zeta)



T_c = I1 * zeta - I2 * zeta**2
#P_c = J1*zeta + J2*zeta**2
prop_efficiency = T_c / P_c
print('Final Design:\nTwists;', twist, '\nChords;', c,  '\nEfficiency;', prop_efficiency)
solidarity = B*c/(2*np.pi*R)



# if data:
#     npdata = np.array(data)
#     n = 0
#     for i in np.arange(angles[0], angles[1], angles[2]):
#         if round(i, 4) not in np.around(npdata[:, 0], 4):
#             print(round(i, 4))
#             n += 1
#     print('Rate:', 1-(n/len(np.arange(angles[0], angles[1], angles[2]))))
# else:
#     print('error!')
# print(data)

# W = V*(1 + a)/np.sin(phi)
