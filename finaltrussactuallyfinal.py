import numpy as np
from scipy.integrate import quad

T = 833  # N (force)
L = 1.8  # m (arm length)
sigmaY = 280  # MPa (yield stress)
rho = 2770  # kg/m^3 (density)
htruss = 0.5  # m (attachment point height)
E = 71 * 10**9  # Pa (Young's modulus)
v = 0.33  # Poisson's ratio

# Combined truss and no-truss configuration function
def structure_config_loop(T, L, sigmaY, rho, E, v, t_min, t_max, d_min, d_max, 
                          ttruss_min=None, ttruss_max=None, dtruss_min=None, dtruss_max=None, truss=True):
    min_score = float('inf')
    best_params = None
    
    if truss:
        htruss = 0.5
        weight_deflection = 0.1
        weight_mass = 0.5
        weight_stress = 0.1
        weight_buckling = 0.3
        
        for Ltruss in np.arange(0.3, 0.7*L, 0.01):
            if Ltruss >= htruss:
                Xtruss = np.sqrt(Ltruss**2 - htruss**2)
                for ttruss in np.arange(ttruss_min, ttruss_max, 0.001):
                    for dtruss in np.arange(dtruss_min, dtruss_max, 0.01):
                        for t in np.arange(t_min, t_max, 0.001):
                            for d in np.arange(d_min, d_max, 0.01):
                                Itruss = np.pi * ttruss * dtruss**3 / 8
                                I = np.pi * t * d**3 / 8
                                mtruss = np.pi * ((0.5 * dtruss)**2 - (0.5 * dtruss - ttruss)**2) * Ltruss * rho
                                mbeam = np.pi * ((0.5 * d)**2 - (0.5 * d - t)**2) * L * rho
                                m = mtruss + mbeam

                                # Calculate buckling stress (sigma_x) inline
                                P = (np.pi**2 * 0.25 * E * Itruss) / (Ltruss**2)
                                sigma_x = P / (np.pi * ((0.5 * dtruss)**2 - (0.5 * dtruss - ttruss)**2))

                                theta = np.arcsin(htruss / Ltruss)
                                M = L * T
                                Fty = M / Xtruss
                                Ft = Fty / np.sin(theta)

                                Vty = (L / Xtruss)
                                Vt = (1 - Vty)
                                Fo = T - Fty

                                def Bm(z):
                                    return ((-Fo * z) * (-Vt * z)) / (E * I)
                                result1, _ = quad(Bm, 0, Xtruss)

                                def Vm(z):
                                    return ((-Fo * Xtruss + T * (z - Xtruss)) * (-Xtruss * Vt + 1 * (z - 3))) / (E * I)
                                result2, _ = quad(Vm, Xtruss, L)

                                Def1 = result1 + result2

                                def Sm(z):
                                    return (Fty * (Vty)) / (E * np.pi * ((0.5 * dtruss)**2 - (0.5 * dtruss - ttruss)**2))
                                result3, _ = quad(Sm, 0, Ltruss)

                                Deflectiontotal = (result3) / (np.sin(theta)) - Def1

                                Mx = T * (L - Xtruss)
                                sigma1 = (Mx * (d * 0.5)) / I

                                A_truss = np.pi * ((0.5 * dtruss)**2 - (0.5 * dtruss - ttruss)**2)
                                sigmatruss = Ft / A_truss

                                # Check conditions
                                if sigma1 * 1.5 > sigmaY * 10**6 or sigmatruss * 1.5 > sigmaY * 10**6 or sigmatruss > sigma_x or Deflectiontotal > 0.03:
                                    continue

                                score = (weight_deflection * Deflectiontotal +
                                         weight_mass * m +
                                         weight_stress * ((sigma1 / (sigmaY * 10**6)) + (sigmatruss / (sigmaY * 10**6))) +
                                         weight_buckling * (sigmatruss / sigma_x))

                                if score < min_score:
                                    min_score = score
                                    best_params = {
                                        'config': 'truss',
                                        'Ltruss': Ltruss,
                                        'ttruss': ttruss,
                                        'dtruss': dtruss,
                                        't': t,
                                        'd': d,
                                        'deflection': Deflectiontotal,
                                        'stress_beam': sigma1,
                                        'stress_truss': sigmatruss,
                                        'mass': m,
                                        'buckling_stress': sigma_x,
                                        'load': Fty
                                    }
    else:
        weight_deflection = 0.1
        weight_mass = 0.5
        weight_stress = 0.1

        for t in np.arange(t_min, t_max, 0.001):
            for d in np.arange(d_min, d_max, 0.01):
                I = np.pi * t * d**3 / 8
                mbeam = np.pi * ((0.5 * d)**2 - (0.5 * d - t)**2) * L * rho
                M = L * T

                def Km(z):
                    return ((T * z) * z) / (E * I)
                res, _ = quad(Km, 0, L)

                sigma1 = (M * (d * 0.5)) / I

                if sigma1 * 1.5 > sigmaY * 10**6 or res > 0.03:
                    continue

                score = (weight_deflection * res + 
                         weight_mass * mbeam + 
                         weight_stress * (sigma1 / (sigmaY * 10**6)))

                if score < min_score:
                    min_score = score
                    best_params = {
                        'config': 'no_truss',
                        't': t,
                        'd': d,
                        'deflection': res,
                        'stress_beam': sigma1,
                        'mass': mbeam
                    }
    
    return best_params, min_score


# Main optimization function
def optimize_structure(T=5000, L=1.8, sigmaY=280, rho=2770, htruss=0.5, E=71*10**9, v=0.33, 
                       t_min=0.0005, t_max=0.006, d_min=0.02, d_max=0.1, 
                       ttruss_min=0.001, ttruss_max=0.009, dtruss_min=0.07, dtruss_max=0.1, 
                       weight_deflection=0.1, weight_mass=0.5, weight_stress=0.1, weight_buckling=0.3, 
                       truss=True):

    best_params, min_score = structure_config_loop(T, L, sigmaY, rho, E, v, t_min, t_max, d_min, d_max, 
                                                   ttruss_min, ttruss_max, dtruss_min, dtruss_max, truss=truss)
    return best_params


# Example usage:
#configuration = int(input("Select configuration: No truss (1), Truss (2): "))
#truss_option = True if configuration == 2 else False
#result = optimize_structure(T=5000, L=1.8, truss=True)
#print(result)
#print(result["mass"])

#print("\nOptimal Configuration:")
#for key, value in result.items():
#    print(f"{key}: {value}")


