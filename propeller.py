import numpy as np
from AirfoilData import get_aifoildata
import time

#Measure start time:
start_time = time.time()

# Global Inputs
power = 52199
velocity = 49.17        # 10 m/s at hover according to https://www.scopus.com/inward/record.uri?eid=2-s2.0-85119987670&doi=10.3390%2fapp112311083&partnerID=40&md5=1aa6253871a54672b3f55d84f0cce64f (p11)
diameter = 0.3 * 5.75
dia_hub = 0.3
nr_blades = 2
a = 340
mach_tip = 2400 * diameter * np.pi / 60 / a      # No more than 0.6 to reduce noise.
foil = 'NACA 4415'

rho = 1.225
dyn_viscosity = 1.789e-5
kin_viscosity = dyn_viscosity/rho

# Program Inputs
nr_sect = 6
zeta_acc = 0.001
zeta = 1
zeta_prev = 0
Re = np.full(nr_sect, 100000)

# Calculations
radius = diameter/2
ang_velocity = a*mach_tip/radius
v_ratio = velocity/(ang_velocity*radius)
power_coef = 2 * power / (rho * velocity**3 * np.pi * radius**2)

# Section Calculations
sections = np.linspace(dia_hub/2, diameter/2, nr_sect)
nd_radius = sections/radius

while abs(zeta_prev/zeta - 1) >= zeta_acc:
    flow_angle_tip = np.arctan2(v_ratio*(1+zeta/2), 1)
    f = (nr_blades/2)*(1-nd_radius)/np.sin(flow_angle_tip)
    F = (2/np.pi)*np.arccos(np.exp(-f))
    flow_angle = np.arctan2(np.tan(flow_angle_tip), nd_radius)
    circulation = F * np.cos(flow_angle) * np.sin(flow_angle)
    x = ang_velocity * sections / velocity

    re_angs = np.insert(np.expand_dims(Re, axis=1), 1, np.zeros((1, nr_sect)), axis=1)
    re_angs[-1, 0] = 100000
    data = get_aifoildata(foil, re_angs, ('AlphaClCd', 'ClCdmaxCl', 'ClCdmax'))
    alpha, cl, cdcl = np.radians(data[:, 0]), data[:, 1], 1/data[:, 2]

    Wc = 4 * np.pi * v_ratio * circulation * velocity * radius * zeta / (cl * nr_blades)
    Re_prv = Re
    Re = Wc / kin_viscosity
    param1 = 1 - cdcl * np.tan(flow_angle)
    param2 = 1 + cdcl / np.tan(flow_angle)
    ax_interf = zeta / 2 * (np.cos(flow_angle))**2 * param1
    rot_interf = zeta / (2 * x) * np.cos(flow_angle) * np.sin(flow_angle) * param2
    W = velocity * (1 + ax_interf) / np.sin(flow_angle)
    c = Wc / W
    twist = flow_angle + alpha

    i1 = 4 * nd_radius * circulation * param1
    i2 = v_ratio * i1 / (2 * nd_radius) * param2 * np.sin(flow_angle) * np.cos(flow_angle)
    j1 = 4 * nd_radius * circulation * param2
    j2 = j1 / 2 * param1 * (np.cos(flow_angle))**2

    I1 = np.trapz(i1, x=sections)
    I2 = np.trapz(i2, x=sections)
    J1 = np.trapz(j1, x=sections)
    J2 = np.trapz(j2, x=sections)

    zeta_prev = zeta
    zeta = -J1 / (2 * J2) + np.sqrt((J1 / (2 * J2))**2 + power_coef / J2)

print(f'Convergence reached! ({(time.time() - start_time)} seconds)\nZetaRatio; {zeta_prev/zeta} ReRatio; {Re_prv/(Re+0.01)}')

thrust_coef = I1 * zeta + I2 * zeta**2
power_coef = J1*zeta + J2*zeta**2
prop_efficiency = thrust_coef / power_coef
print('Final Design:\nSections;', sections,'\nTwists;', twist, '\nChords;', c, '\nEfficiency;', prop_efficiency)
solidarity = nr_blades*c/(2*np.pi*radius)