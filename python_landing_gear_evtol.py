import numpy as np
import matplotlib.pyplot as plt 

##! Main function to run the optimization
def main(mtow): #INPUT: Mtow OUTPUT: ((tube_radius, tube_thickness, gear_deflection),total_landing_gear_optimal_mass)

    ##! Constants
    g = 9.80665 # m/s^2
    emod_al = 73.4 * 10**9 # Pa (Young's modulus for aluminum)
    density_al = 2780 # kg/m^3 (Density of aluminum)
    N_gear = 4 #No. of landing gears

    ##! Function to calculate beam lengths
    def beam_lengths(phi, L):
        len_AB = L/np.cos(phi)
        len_AC = L
        len_AD = L * np.tan(phi)
        len_BC = L * np.tan(phi)
        len_CD = L/np.cos(phi)
        return len_AB, len_AC, len_AD, len_BC, len_CD

    ##! Function to calculate total mass
    def total_mass(phi, A, L, rho):
        mass = (1 + 2/np.cos(phi) + 2*np.tan(phi)) * rho * A * L
        return mass

    ##! Function to calculate deflection
    def deflection_B(phi, F, L, E, A):
        deflection = (F * L * (1/(np.tan(phi)**2) + np.tan(phi) + 1/(np.cos(phi)*np.sin(phi)**2))) / (E * A)
        return deflection

    ##! Function to calculate cross-sectional area of tube
    def cross_section_area(radius, thickness):
        A = np.pi * ((radius + thickness/2)**2 - (radius - thickness/2)**2)
        return A

    ##! Function to find optimal tube geometry based on deflection criteria
    def find_optimal_geometry(mtow, max_load_factor=1.5, L=0.5, phi_deg=45, max_deflection=0.03):  # L is chosen here because of stability 
        # Convert phi from degrees to radians
        phi = np.radians(phi_deg)
        
        # Calculate the load
        F = mtow * g * max_load_factor
        print(F)

        # Define ranges for thickness and radius to iterate through
        radius_range = np.arange(0.02, 0.06, 0.001)  # Radii between 0.02 and 0.06 meters
        thickness_range = np.arange(0.001, 0.01, 0.0005)  # Thicknesses between 0.001 and 0.01 meters
        
        optimal_mass = float('inf')  # Initialize optimal mass with a high value
        optimal_params = None  # Store the optimal parameters
        
        # Iterate over different radius and thickness values
        for radius in radius_range:
            for thickness in thickness_range:
                A = cross_section_area(radius, thickness)  # Calculate cross-sectional area
                mass = total_mass(phi, A, L, density_al)  # Calculate mass
                deflection = deflection_B(phi, F, L, emod_al, A)  # Calculate deflection
                
                # Check if deflection is below the limit and update the optimal solution
                if deflection <= max_deflection and mass < optimal_mass:
                    optimal_mass = mass
                    optimal_params = (radius, thickness, deflection)
        
        # Return optimal parameters (radius, thickness, deflection, and mass)
        return optimal_params, optimal_mass


    optimal_params, optimal_mass = find_optimal_geometry(mtow)
    
    if optimal_params:
        radius, thickness, deflection = optimal_params
        print(f"Optimal Radius: {radius:.4f} m")
        print(f"Optimal Thickness: {thickness:.4f} m")
        print(f"Deflection: {deflection*1000:.4f} mm")
        print(f"Optimal Mass ONE Landing Gear: {optimal_mass:.4f} kg")
    else:
        print("No solution found with deflection below the limit.")

    return optimal_params, optimal_mass*N_gear

##! Example usage with MTOW
mtow = 300  # kg
print(main(mtow))






## LEGACY CODE ## AND DRAWING AND PHI OPT CODE ##
""" #Imports
import numpy as np 
import matplotlib.pyplot as plt
from cross_section_gear import cross_section_area



##!Constants
g = 9.80665 #m/s^2
emod_al = 73.4 * 10**9 #Pa
density_al = 2780 #kg/m^3

##!Global Input Parameters
mtow = 340 #kg
max_load_factor = 1.5 #Picked for now


##!Define Truss Shape (see gear_drawing.png)
L = 0.5 #For stability (choice)
radius_pipe = 0.03 #m
thickness = 0.002 #m
A = np.pi * (radius_pipe + thickness/2)*(radius_pipe + thickness/2) - np.pi * (radius_pipe - thickness/2)*(radius_pipe - thickness/2)
phideg = 45 #Iterate
phi = np.radians(phideg) 

def beam_lengths(phi,L):
    len_AB = L/np.cos(phi)
    len_AC = L
    len_AD = L *np.tan(phi)
    len_BC = L *np.tan(phi)
    len_CD = L/np.cos(phi)
    return len_AB,len_AC,len_AD,len_BC,len_CD


##!Calculate Mass
def total_mass(phi,A,L,rho):
    mass = (1 + 2/np.cos(phi) + 2*np.tan(phi))*rho*A*L
    return mass    

##!Calculate Loads 
F = mtow*g*max_load_factor
def beam_loads(phi):
    F = mtow*g*max_load_factor
    F_AB = 0
    F_AC = F/np.tan(phi)
    F_AD = 0
    F_BC = -F
    F_CD = -F/np.sin(phi)
    return F,F_AB,F_AC,F_AD,F_BC,F_CD

##!Calculate Deflections
def deflection_B(phi,F,L,E,A):
    deflection = (F*L*(1/(np.tan(phi)**2) + np.tan(phi) + 1/(np.cos(phi)*np.sin(phi)**2)))/(E*A)
    return deflection


##!Iterate to find optimal thickness
phideg_lst = []
A_lst = []
mass_lst = []
deflection_lst = []
deflection_lst_normalised = []
mass_lst_normalised = []

deflection_factor = F*L/(emod_al*A)
mass_factor = density_al * emod_al* A

for phideg in range(30,80):
    mass =  total_mass(np.radians(phideg),A,L,density_al)
    deflection = deflection_B(np.radians(phideg),F,L,emod_al,A)
    phideg_lst.append(phideg)
    A_lst.append(A)
    mass_lst_normalised.append(mass/mass_factor)
    deflection_lst.append(deflection)
    deflection_lst_normalised.append(deflection/deflection_factor)


fig, ax1 = plt.subplots(figsize=(8, 8))
ax2 = ax1.twinx()

ax1.plot(phideg_lst, mass_lst_normalised)
ax1.set_ylabel("mass (kg)")
ax1.set_xlabel("phi (degrees)")

ax2.plot(phideg_lst, deflection_lst_normalised)
ax2.set_ylabel("deflection in FL/EA")

plt.title("Mass and Deflection vs Phi")
#plt.savefig("phi_iteration")
plt.show()


def draw_gear(phideg1,L):

    print(phideg1)
    phi1 = np.radians(phideg1)
    print(phi1)
    A_cords = [0,0]
    C_cords = [L,0]
    CD_len = L/np.cos(phi1)
    D_cords = [0,CD_len * np.sin(phi1)]
    AB_len = CD_len
    B_cords = [L,-1*np.sin(phi1)*AB_len]
    
    x_cords = [A_cords[0],C_cords[0],D_cords[0],A_cords[0],B_cords[0], C_cords[0],A_cords[0],D_cords[0]]
    y_cords = [A_cords[1], C_cords[1],D_cords[1],A_cords[1],B_cords[1],C_cords[1],A_cords[1],D_cords[1]]

    plt.plot(x_cords,y_cords)
    plt.title(f"mass:{mass}[kg], deflection={deflection*1000}[mm]")
    plt.xlim(-5,5)
    plt.ylim(-5, 5)
    #plt.savefig("gear")
    plt.show()
    return 

#Phi = 45deg based on iteration
deflection = deflection_B(np.radians(phideg),F,L,emod_al,A)
mass =  total_mass(np.radians(phideg),A,L,density_al)
draw_gear(45,L)
 """
