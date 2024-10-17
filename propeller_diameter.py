import numpy as np

# Returns maximal blade diameter as a function of the configuration, nr of blades and a few geometrical parameters.
# N_blades should be larger than 1.
# Possible configurations are "hori_fold", "verti_fold" and "level_fold".
# Assuming sharp blade tips and no margin with the side of the truck.

def diamgenerator(config, N_blades, D_hub, Cr_blade, W_arm, W_pin, H_pin, H_prop):
    W_max_flight = 8.5 - 2 * 0.5
    W_fuselage = 1.2
    W_max = 2.4
    H_max = 4.1
    L_max = 8.53 #14.63 for a semi-truck (https://ops.fhwa.dot.gov/freight/publications/size_regs_final_rpt/).

    if config == "hori_fold":
        if N_blades == 2:
            L_it = L_max
            D_1, D_2 = 1, 0
            while D_1 > D_2:
                b = (L_it + W_max_flight)
                D_1 = (- b + np.sqrt(b**2 + 2 * (W_max_flight**2 + L_it**2))) / 2
                D_2 = np.sqrt(W_max**2 + ((L_max - L_it) / 2)**2)
                L_it -= 0.01
            D_max = D_1
        elif N_blades == 4:
            phi = np.pi / 4
            D_max = -1000
            while D_max == -1000 or D_max - D_prev > 0:
                phi += np.pi/100
                D_prev = D_max
                Z = max(((D_hub - np.sin(phi) * Cr_blade) * np.tan(phi) - D_hub - np.cos(phi) * Cr_blade) / (1 - np.tan(phi)), 0)
                D_max = (W_max - Z - D_hub) / np.sin(phi)
        elif N_blades % 2 == 0:
            print("Result not yet verified!")
            S_hub = max(D_hub + W_arm, (Cr_blade+D_hub)/2/np.sin(np.pi/N_blades))
            D_max = (W_max - S_hub) / np.cos(np.pi/N_blades)
        else:
            print("Result only verified for 3 blades!")
            D_max = W_max / np.cos(np.pi/2/N_blades)
        if N_blades != 2 or N_blades != 4:
            L = D_max * (2 + np.cos(np.pi / N_blades))
            if L > L_max:
                print(f"Design too long, length is {L} meters!")

    elif config == "verti_fold":
        S_max = np.sqrt((W_max - W_pin)**2 + (H_max - H_pin)**2)
        if N_blades == 2:
            D_max = S_max + W_pin - D_hub/2
        else:
            D_max = (S_max + W_pin) / (1 + np.cos(np.pi / (1 + N_blades % 2) / N_blades)/2)

    elif config == "level_fold":
        S_hub = max(D_hub, (Cr_blade + D_hub) / 2 / np.sin(np.pi / N_blades))
        L_arm = np.sqrt((W_pin - S_hub)**2 + (H_max - H_pin - H_prop)**2)
        D_max1 = L_arm / np.cos(np.pi/6) + W_pin    # Assuming rotors on plane of symmetry, otherwise np.cos() => 1.
        if N_blades == 2:
            D_max2 = D_max1
        elif N_blades % 2 == 0:
            D_max2 = (W_max - S_hub) / np.cos(np.pi / N_blades)
        else:
            D_max2 = W_max - S_hub
        D_max = min(D_max1, D_max2)

    else:
        print(f"Configuration doesn't exist, use 'hori_fold', 'verti_fold' or 'level_fold'.")

    return (D_max)


#D = diamgenerator("hori_fold", 2, 0.3, 0.1, 0.08, 0.5, 1.1, 0.2)
#print(D)
