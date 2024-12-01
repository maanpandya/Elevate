import numpy as np
from AirfoilData import get_aifoildata
import time
import matplotlib.pyplot as plt
import pickle

start_time = time.time()

# Global Inputs
g=9.81
rho = 1.225
dyn_viscosity = 1.789e-5
kin_viscosity = dyn_viscosity/rho
a = 340
B=2
foil = 'e395.DAT'
foilpath = foil.replace(" ", "_")

def design(D, T_hv, V):
    #print(T_hv)
    nr_sect = 100
    D_hub = D/5.75
    mach_tip = 0.6
    zeta_acc = 0.001
    V_displ=20
    zeta = V_displ/V
    zeta_prev = 0
    Re = np.full(nr_sect, 100000)

    R = D/2
    omega = a*mach_tip/R
    v_ratio = V/(omega*R)
    T_c = 2 * T_hv / (rho * V**2 * np.pi * R**2)

    r = np.linspace(D_hub/2, D/2, nr_sect)
    xi = r/R

    while abs(zeta_prev/zeta - 1) >= zeta_acc:
        phi_tip = np.arctan2(v_ratio*(1+zeta/2), 1)
        #print(phi_tip)
        f = (B/2)*(1-xi)/np.sin(phi_tip)
        F = (2/np.pi)*np.arccos(np.exp(-f))
        #F=np.array([1]*nr_sect)
        phi = np.arctan2(np.tan(phi_tip), xi)
        #print(phi)
        x = omega * r / V
        G = F * x * np.cos(phi) * np.sin(phi)

        re_angs = np.insert(np.expand_dims(Re, axis=1), 1, np.zeros((1, nr_sect)), axis=1)
        re_angs[-1, 0] = 100000
        data = get_aifoildata(foil, re_angs, ('AlphaClCd', 'ClCdmaxCl', 'ClCdmax'))
        alpha, cl, cdcl = np.radians(data[:, 0]), data[:, 1], 1/data[:, 2]
        #print(cl)

        Wc = 4 * np.pi * v_ratio * G * V * R * zeta / (cl * B)
        #print(Wc)
        Re = Wc / kin_viscosity
        #print(Re)
        param1 = 1 - cdcl * np.tan(phi)
        param2 = 1 + cdcl / np.tan(phi)
        ax_interf = zeta / 2 * (np.cos(phi))**2 * param1
        rot_interf = zeta / (2 * x) * np.cos(phi) * np.sin(phi) * param2
        #print(ax_interf)
        W = V * (1 + ax_interf) / np.sin(phi)
        c = Wc / W
        beta = phi + alpha

        i1 = 4 * xi * G * param1
        i2 = v_ratio * i1 / (2 * xi) * param2 * np.sin(phi) * np.cos(phi)
        j1 = 4 * xi * G * param2
        j2 = j1 / 2 * param1 * (np.cos(phi))**2

        I1 = np.trapz(i1, x=xi)
        I2 = np.trapz(i2, x=xi)
        J1 = np.trapz(j1, x=xi)
        J2 = np.trapz(j2, x=xi)

        zeta_prev = zeta
        zeta = I1/(2*I2)-np.sqrt((I1/(2*I2))**2-T_c/I2)
        #print(zeta)


    P_c = J1*zeta + J2*zeta**2
    P=P_c*rho*V**3*np.pi*R**2/2
    cl_mean=np.mean(cl)
    #print(P)
    #prop_efficiency = T_c / P_c
    #print('Final Design:\nr;', r,'\nbetas;', beta, '\nChords;', c, '\nEfficiency;', prop_efficiency)
    #plt.plot(r, r**2*np.sqrt(F))
    #plt.plot(r, rot_interf)
    #plt.show()
    #print(c)
    #print(beta)
    return r, c, beta, phi, Re, omega, P, cl_mean

def powers(D, T_hv, lst, wind_lst):
    r, c, beta, phi_hv, Re_hv, omega_hv, P_hv, cl_mean = design(D, T_hv, 0.001)

    powers = [r, c, beta, cl_mean, P_hv]

    #print(P_hv)
    nr_sect=np.size(r)
    R=r[-1]
    xi=r/R
    sigma=B*c/(2*np.pi*r)

    with open('AirfoilData/' + foilpath + '/cl_func.pkl', 'rb') as file:
        cl_func = pickle.load(file)
    with open('AirfoilData/' + foilpath + '/cd_func.pkl', 'rb') as file:
        cd_func = pickle.load(file)

    winds=[0]
    winds.extend(wind_lst)
    wind_lst=winds
    for point in lst:
        var_lst=[]

        for wind_speed in wind_lst:

            if len(point)==2:
                T_conv, V_n = point
                V_disc=wind_speed
            elif len(point)==3:
                T_conv, V_n, V_disc = point
                theta=np.arctan2(V_n, V_disc)
                V_n+=wind_speed*np.sin(theta)
                V_disc += wind_speed * np.cos(theta)

            T = 0
            omega=omega_hv
            phi=phi_hv+0.05
            Re = Re_hv

            while np.abs((T_conv-T)/T_conv)>0.001:
                a_prev = np.full(1, nr_sect)
                a = np.zeros(nr_sect)
                while not np.max(np.abs((a - a_prev) / a_prev)) < 0.0005:
                    f=B/2*(1-xi)/np.sin(np.arctan2(xi*np.tan(phi),1))
                    F=2/np.pi*np.arccos(np.exp(-f))
                    alpha=beta-phi

                    points=[]
                    for j in range(nr_sect):
                        points.append([np.log10(Re[j]), np.degrees(alpha[j])])

                    C_l=cl_func(points)
                    C_d = cd_func(points)
                    epsilon=C_d/C_l
                    #epsilon=np.zeros(nr_sect)

                    C_y = C_l*(np.cos(phi) - epsilon*np.sin(phi))
                    C_x = C_l*(np.sin(phi)+epsilon*np.cos(phi))

                    K = C_y/(4*np.sin(phi)**2)
                    K_prime = C_x/(4*np.sin(phi)*np.cos(phi))

                    a_prev=a
                    a = sigma*K/(F-sigma*K)
                    a_prime = sigma*K_prime/(F+sigma*K_prime)
                    a[-1]=a[-2]
                    a_prime[-1]=a_prime[-2]

                    #i_lst.append(i)
                    #a_lst.append(a[0])

                    W=V_n*(1+a)/np.sin(phi)
                    Re=W*c/kin_viscosity
                    Re[-1] = 100000

                    delta_phi=np.arctan2(V_n*(1+a), omega*r*(1-a_prime))-phi
                    phi+=delta_phi/200



                C_T_prime = np.pi ** 3 / 4 * sigma * C_y * xi ** 3 * F ** 2 / ((F + sigma * K_prime) * np.cos(phi)) ** 2
                C_P_prime = C_T_prime * np.pi * xi * C_x / C_y
                C_T_prime[-1] = C_T_prime[-2]
                C_P_prime[-1] = C_P_prime[-2]

                #plt.plot(xi, C_l)
                #plt.title(f'baseline, omega={omega}')
                #plt.show()

                C_T = np.trapz(C_T_prime, x=xi)
                C_P = np.trapz(C_P_prime, x=xi)
                #print(4 / np.pi ** 3 * C_P * rho * omega ** 3 * R ** 5)
                #print()

                phi_eq=phi
                Re_eq=Re
                a_prime_eq=a_prime

                n_psi=11
                T=0
                P=0
                blade_drag=0
                for psi in np.linspace(0, 180, n_psi):
                    psi=np.radians(psi)
                    V_t=V_disc*np.cos(psi)
                    phi=phi_eq
                    Re=Re_eq

                    a_prev=np.full(1, nr_sect)
                    a=np.zeros(nr_sect)
                    while not np.max(np.abs((a-a_prev)/a_prev))<0.0005:
                        f = B / 2 * (1 - xi) / np.sin(np.arctan2(xi * np.tan(phi), 1))
                        F = 2 / np.pi * np.arccos(np.exp(-f))
                        alpha = beta - phi

                        points = []
                        for j in range(nr_sect):
                            points.append([np.log10(Re[j]), np.degrees(alpha[j])])

                        C_l = cl_func(points)
                        C_d = cd_func(points)
                        epsilon = C_d / C_l
                        #epsilon=np.zeros(nr_sect)

                        C_y = C_l * (np.cos(phi) - epsilon * np.sin(phi))
                        C_x = C_l * (np.sin(phi) + epsilon * np.cos(phi))

                        K = C_y / (4 * np.sin(phi) ** 2)
                        K_prime = C_x / (4 * np.sin(phi) * np.cos(phi))

                        a_prev=a
                        a = sigma * K / (F - sigma * K)
                        a_prime = sigma * K_prime / (F + sigma * K_prime)
                        a[-1] = a[-2]
                        a_prime[-1] = a_prime[-2]

                        #i_lst.append(i)
                        #a_lst.append(a[0])

                        W = np.sqrt((V_n*(1+a))**2+(omega*r*(1-a_prime)+V_t)**2)
                        Re = W*c/kin_viscosity
                        Re[-1] = 100000

                        delta_phi = np.arctan2(V_n * (1 + a), omega * r * (1 - a_prime)+V_t) - phi
                        phi += delta_phi / 200
                        #print(psi)
                        #print(phi)



                    C_T_prime = np.pi**3/4*sigma*C_y*xi**3*W**2/(omega*r)**2
                    C_P_prime = C_T_prime*np.pi*xi*C_x/C_y
                    C_T_prime[-1] = C_T_prime[-2]
                    C_P_prime[-1] = C_P_prime[-2]
                    H_prime=0.5*rho*W**2*B*c*C_x

                    #plt.plot(xi, a_prime)


                    C_T=np.trapz(C_T_prime, x=xi)
                    C_P=np.trapz(C_P_prime, x=xi)
                    H=np.trapz(H_prime, x=r)
                    T+= 4 / np.pi ** 2 * C_T * rho * omega ** 2 * R ** 4/n_psi
                    P+=4/np.pi**3*C_P*rho*omega**3*R**5/n_psi
                    blade_drag+=H*np.cos(psi)/n_psi
                    #print(4/np.pi**3*C_P*rho*omega**3*R**5)
                    #print(W)

                    #plt.plot(xi, phi)

                #plt.title(f'psi={psi}')
                #plt.show()
                #print(C_P_prime)
                #print(f'P: {P}')
                #print(f'T: {T}')
                #print(f'blade_drag: {blade_drag}')
                #print()
                #plt.title(f'omega={omega}, psi={psi}')
                #plt.show()
                #print(P)
                #print(T)
                #print()

                omega*=np.sqrt(T_conv/T)
            var_lst.append([P, blade_drag, omega])
        powers.append(var_lst)
    return powers

#print(powers(D=2, T_hv=583, lst=[[600.2130211835558, 4.941286552985221, 29.687954154618136], [360.9971821340844, 7.697425215730051, 29.062173380815263], [592.4494820678754, 5], [350.5568701406026,5]], wind_lst=[1, -1]))

#print(design(2, 600, 0.001))

