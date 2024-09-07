import numpy as np
from AirfoildataGenerator import get_aifoildata

# question = ('Cd', 'Cl', 'ClCd', 'AlphaClCd', 'ClCdmax')
# re_angs = [[10**5, 5]]
# answer = get_aifoildata('NACA 2412', re_angs, question)
# print(answer)

# Global Inputs
power = 52200
mach_tip = 0.6
a = 340
velocity = 49.17
diameter = 0.3 * 5.75
dia_hub = 0.3
nr_blades = 2
dyn_viscosity = 1.789e-5
rho = 1.225
kin_viscosity = dyn_viscosity/rho

# Program Inputs
nr_sect = 6
dzeta = 1
dzeta_prev = 0
dzeta_acc = 0.01
Re = np.full(nr_sect, 100000)
foil = 'NACA 2412'

# Calculations
radius = diameter/2
ang_velocity = a*mach_tip/radius
v_ratio = velocity/(ang_velocity*radius)
power_coef = 2 * power / (rho * velocity**3 * np.pi * radius**2)

# Section Calculations
sections = np.linspace(dia_hub/2, diameter/2, nr_sect)
nd_radius = sections/radius

while abs(dzeta_prev/dzeta - 1) >= dzeta_acc:
    flow_angle_tip = np.arctan(v_ratio*(1+dzeta/2))
    f = (nr_blades/2)*(1-nd_radius)/np.sin(flow_angle_tip)
    F = (2/np.pi)*np.arccos(np.exp(-f))
    flow_angle = np.arctan(np.tan(flow_angle_tip)/nd_radius)
    circulation = F * np.cos(flow_angle) * np.sin(flow_angle)
    x = ang_velocity * sections / velocity

    re_angs = np.insert(np.expand_dims(Re, axis=1), 1, np.zeros((1, nr_sect)), axis=1)
    re_angs[-1, 0] = 100000
    data = get_aifoildata(foil, re_angs, ('AlphaClCd', 'ClCdmaxCl', 'ClCdmax'))
    alpha, cl, cdcl = data[:, 0], data[:, 1], 1/data[:, 2]

    Wc = 4 * np.pi * v_ratio * circulation * velocity * radius * dzeta / (cl * nr_blades)
    Re_prv = Re
    Re = Wc / kin_viscosity
    print('ReRatio', Re_prv/(Re+1))             #Fix this!!!
    param1 = 1 - cdcl * np.tan(flow_angle)
    param2 = 1 + cdcl / np.tan(flow_angle)
    ax_interf = dzeta / 2 * (np.cos(flow_angle))**2 * param1
    rot_interf = dzeta / 2 * x * np.cos(flow_angle) * np.sin(flow_angle) * param2
    W = velocity * (1 + ax_interf) / np.sin(flow_angle)
    c = Wc / W
    twist = flow_angle + alpha
    print(flow_angle, alpha)

    i1 = 4 * nd_radius * circulation * param1
    i2 = v_ratio * i1 / 2 * nd_radius * param2 * np.sin(flow_angle) * np.cos(flow_angle)
    j1 = 4 * nd_radius * circulation * param2
    j2 = j1 / 2 * param1 * (np.cos(flow_angle))**2

    I1 = np.trapz(i1, x=sections)
    I2 = np.trapz(i2, x=sections)
    J1 = np.trapz(j1, x=sections)
    J2 = np.trapz(j2, x=sections)

    dzeta_prev = dzeta
    dzeta = -J1 / 2 * J2 + np.sqrt((J1 / 2 * J2)**2 + power_coef / J2)
    print('DzRatio', dzeta_prev/dzeta)

thrust_coef = I1 * dzeta + I2 * dzeta**2
prop_efficiency = thrust_coef * velocity * np.pi * 2 / (power_coef * diameter * ang_velocity)
print('Final Design:\nTwists;', twist, '\nChords;', c, '\nEfficiency;', prop_efficiency)




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

# W = velocity*(1 + a)/np.sin(flow_angle)
