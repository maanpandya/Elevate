import numpy as np
from AirfoilData import get_aifoildata
import time
import matplotlib.pyplot as plt

start_time = time.time()

# Global Inputs


def powers(D, T, V, theta):
    foil = 'NACA 4415'
    D_hub = 0.3048
    B = 2
    a = 340
    mach_tip = 0.6     # No more than 0.6 to reduce noise.
    rho = 1.225
    dyn_viscosity = 1.789e-5
    kin_viscosity = dyn_viscosity/rho

    nr_sect = 100
    zeta_acc = 0.001
    zeta = 0.1
    zeta_prev = 0
    Re = np.full(nr_sect, 100000)

    R = D/2
    omega = a*mach_tip/R
    v_ratio = V/(omega*R)
    T_c = 2 * T / (rho * V**2 * np.pi * R**2)

    r = np.linspace(D_hub/2, D/2, nr_sect)
    dr=(R-D_hub/2)/(nr_sect-1)
    xi = r/R

    while abs(zeta_prev/zeta - 1) >= zeta_acc:
        phi_tip = np.arctan2(v_ratio*(1+zeta/2), 1)
        f = (B/2)*(1-xi)/np.sin(phi_tip)
        F = (2/np.pi)*np.arccos(np.exp(-f))
        #F=np.array([1]*nr_sect)
        phi = np.arctan2(np.tan(phi_tip), xi)
        x = omega * r / V
        G = F * x * np.cos(phi) * np.sin(phi)
        gamma=2*np.pi*V**2*zeta*G/(B*omega)
        gamma_slope=[gamma[0]/dr]
        for i in range(nr_sect-2):
            gamma_slope.append((gamma[i+2]-gamma[i])/(2*dr))
        gamma_slope.append(-gamma[-2]/dr)
        gamma_slope=np.array(gamma_slope)

        V_ind_lst=[]
        for i in range(nr_sect):
            V_ind = -gamma_slope[i] / (2 * r[i]) * dr/(4*np.pi)
            for j in range(nr_sect - 1):
                index = (i + 1 + j) % nr_sect
                V_ind += gamma_slope[index] / (
                        r[i] - r[index]) * dr/(4*np.pi)
                V_ind += -gamma_slope[index] / (
                        r[i] + r[index]) * dr/(4*np.pi)
            V_ind_lst.append(V_ind)
        V_ind=np.array(V_ind_lst)


        re_angs = np.insert(np.expand_dims(Re, axis=1), 1, np.zeros((1, nr_sect)), axis=1)
        re_angs[-1, 0] = 100000
        data = get_aifoildata(foil, re_angs, ('AlphaClCd', 'ClCdmaxCl', 'ClCdmax'))
        alpha, cl, cdcl = np.radians(data[:, 0]), data[:, 1], 1/data[:, 2]

        Wc = 4 * np.pi * v_ratio * G * V * R * zeta / (cl * B)
        Re_prv = Re
        Re = Wc / kin_viscosity
        param1 = 1 - cdcl * np.tan(phi)
        param2 = 1 + cdcl / np.tan(phi)
        ax_interf = zeta / 2 * (np.cos(phi))**2 * param1
        rot_interf = zeta / (2 * x) * np.cos(phi) * np.sin(phi) * param2
        W = V * (1 + ax_interf) / np.sin(phi)
        c = Wc / W
        alpha_ind=V_ind/W
        beta = phi + alpha + alpha_ind

        i1 = 4 * xi * G * param1
        i2 = v_ratio * i1 / (2 * xi) * param2 * np.sin(phi) * np.cos(phi)
        j1 = 4 * xi * G * param2
        j2 = j1 / 2 * param1 * (np.cos(phi))**2

        I1 = np.trapz(i1, x=r)
        I2 = np.trapz(i2, x=r)
        J1 = np.trapz(j1, x=r)
        J2 = np.trapz(j2, x=r)

        zeta_prev = zeta
        zeta = I1/(2*I2)-np.sqrt((I1/(2*I2))**2-T_c/I2)

    #print(f'Convergence reached! ({(time.time() - start_time)} seconds)\nZetaRatio; {zeta_prev/zeta} ReRatio; {Re_prv/(Re+0.01)}')

    P_c = J1*zeta + J2*zeta**2
    P=P_c*rho*V**3*np.pi*R**2/2
    prop_efficiency = T_c / P_c
    #print('Final Design:\nr;', r,'\nbetas;', beta, '\nChords;', c, '\nEfficiency;', prop_efficiency)
    solidarity = B*c/(2*np.pi*R)
    plt.plot(r, alpha_ind)
    plt.show()
    return P

print(powers(D=0.3048 * 5.75, T=923.49528, V=50, theta=0))


