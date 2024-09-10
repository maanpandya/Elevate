import numpy as np
from scipy.integrate import quad

# Parameters
T = 600  # N (force)
L = 2.5  # m (arm length)
sigmaY = 400  # MPa (yield stress)
rho = 2000  # kg/m^3 (density)
htruss = 0.6  # m (attachment point height)
E = 600 * 10**9  # Pa (Young's modulus)

# Limits for thickness and diameter
t_min, t_max = 0.0005, 0.006  # Thickness range for beam (m)
d_min, d_max = 0.02, 0.1  # Diameter range for beam (m)
ttruss_min, ttruss_max = 0.0004, 0.006  # Thickness range for truss (m)
dtruss_min, dtruss_max = 0.02, 0.10  # Diameter range for truss (m)

# Adjustable weights for deflection, mass, and stress in the combined score
weight_deflection = 0.4
weight_mass = 0.4
weight_stress = 0.2

print("Configuration options: No truss (1), truss (2)")
truss = int(input("Select configuration: "))

min_score = float('inf')
best_Ltruss = None
best_t = None
best_d = None
best_ttruss = None
best_dtruss = None
best_deflection = None
best_stress = None
best_m = None

if truss == 2:
    for Ltruss in np.arange(0.3, 0.7*L, 0.01):
        if Ltruss >= htruss:
            Xtruss = np.sqrt(Ltruss**2 - htruss**2)
            # Iterate over all thickness and diameter combinations for both truss and beam
            for ttruss in np.arange(ttruss_min, ttruss_max, 0.001):
                for dtruss in np.arange(dtruss_min, dtruss_max, 0.01):
                    for t in np.arange(t_min, t_max, 0.001):
                        for d in np.arange(d_min, d_max, 0.01):
                            # Calculate inertia and mass for the truss and beam
                            Itruss = np.pi * ttruss * dtruss**3 / 8
                            I = np.pi * t * d**3 / 8
                            mtruss = np.pi * ((0.5 * dtruss)**2 - (0.5 * dtruss - ttruss)**2) * Ltruss * rho
                            mbeam = np.pi * ((0.5 * d)**2 - (0.5 * d - t)**2) * L * rho
                            m = mtruss + mbeam
                            
                            # Calculate theta
                            theta = np.arccos((Ltruss**2 + Xtruss**2 - htruss**2) / (2 * Ltruss * Xtruss))
                            
                            # Calculate moment and forces
                            M = L * T  # Moment around 0
                            Fty = M / Xtruss  # Projected force in truss
                            Ft = Fty / np.sin(theta)  # Force in the truss (compression)
                            
                            # Virtual parameters
                            Vty = (L / Xtruss)
                            Vt = (1 - Vty)
                            Fo = T - Fty  # Attachment force
                            
                            # First function: deflection in the arm
                            def Bm(z):
                                return ((-Fo * z) * (-Vt * z)) / (E * I)
                            result1, error1 = quad(Bm, 0, Xtruss)
                            
                            # Second function: deflection in arm 2
                            def Vm(z):
                                return ((-Fo * Xtruss + T * (z - Xtruss)) * (-Xtruss * Vt + 1 * (z - 3))) / (E * I)
                            result2, error2 = quad(Vm, Xtruss, L)
                            
                            # Deflection without rod
                            Def1 = result1 + result2
                            
                            # Third function: Stress integration (deflection due to truss force)
                            def Sm(z):
                                return (Ft * (Vt / np.sin(theta))) / (E * Itruss)
                            result3, error3 = quad(Sm, 0, Ltruss)
                            
                            # Total deflection
                            Deflectiontotal = -(Def1 + result3)
                            
                            # Bending stress in the beam
                            Mx = T * (L - Xtruss)  # Moment for part 2
                            sigma1 = (Mx * (d * 0.5)) / I  # Bending stress in the beam
                            
                            # Tension stress in the truss
                            A_truss = np.pi * ((0.5 * dtruss)**2 - (0.5 * dtruss - ttruss)**2)  # Cross-sectional area of truss
                            sigmatruss = Ft / A_truss  # Truss stress
                            
                            # Skip invalid configurations where stress exceeds material yield strength
                            if sigma1 * 1.5 > sigmaY * 10**6 or sigmatruss * 1.5 > sigmaY * 10**6:
                                continue
                            
                            # Calculate a combined score
                            score = (weight_deflection * Deflectiontotal +
                                     weight_mass * m +
                                     weight_stress * ((sigma1 / (sigmaY * 10**6)) + (sigmatruss / (sigmaY * 10**6))))
                            
                            # Save the best Ltruss for which the score is the lowest
                            if score < min_score:
                                min_score = score
                                best_Ltruss = Ltruss
                                best_ttruss = ttruss
                                best_dtruss = dtruss
                                best_t = t
                                best_d = d
                                best_deflection = Deflectiontotal
                                best_stress = (sigma1, sigmatruss)
                                best_m = m

    # Output the optimal results
    print(f"\nTruss Configuration")
    print(f"Best Ltruss: {best_Ltruss:.2f} m, with combined score: {min_score:.4f}")
    print(f"Optimal truss thickness: {best_ttruss:.4f} m")
    print(f"Optimal truss diameter: {best_dtruss:.4f} m")
    print(f"Optimal beam thickness: {best_t:.4f} m")
    print(f"Optimal beam diameter: {best_d:.4f} m")
    print(f"Mass of the structure: {best_m:.4f} kg")
    print(f"Xtruss length (horizontal component): {np.sqrt(best_Ltruss**2 - htruss**2):.4f} m")
    print(f"Deflection for best Ltruss: {best_deflection:.6f} m")
    print(f"Bending stress in beam: {best_stress[0]:.6f} Pa")
    print(f"Tension stress in truss: {best_stress[1]:.6f} Pa")
if truss == 1:
    # Iterate over thickness and diameter
    for t in np.arange(t_min, t_max, 0.001):
        for d in np.arange(d_min, d_max, 0.01):
            # Calculate inertia and mass for the beam
            I = np.pi * t * d**3 / 8
            mbeam = np.pi * ((0.5 * d)**2 - (0.5 * d - t)**2) * L * rho
            M = L * T
            # Deflection calculation for simple beam
            def Km(z):
                return ((T * z) * z) / (E * I)
            res, er = quad(Km, 0, L)
            # Bending stress in the beam
            sigma1 = (M * (d * 0.5)) / I
            
            # Skip invalid configurations where stress exceeds material yield strength
            if sigma1*1.5 > sigmaY * 10**6:  # Convert MPa to Pa
                continue
            
            # Calculate a combined score using deflection, mass, and stress
            score = (weight_deflection * res + 
                     weight_mass * mbeam + 
                     weight_stress * (sigma1 / (sigmaY * 10**6)))  # Normalized stress
            
            # Save the best configuration with the lowest score
            if score < min_score:
                min_score = score
                best_t = t
                best_d = d
                best_deflection = res
                best_stress = sigma1
                best_mbeam = mbeam

    # Output the optimal results
    print(f"\nNo Truss Configuration")
    print(f"Optimal thickness: {best_t:.4f} m")
    print(f"Optimal diameter: {best_d:.4f} m")
    print(f"Optimal mass of the structure: {best_mbeam:.4f} kg")
    print(f"Optimal deflection: {best_deflection:.6f} m")
    print(f"Optimal bending stress: {best_stress:.6f} Pa")
    print(f"Combined score: {min_score:.4f}")