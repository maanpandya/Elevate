import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import cumulative_trapezoid

''' Assumptions (yes, there are loads):
- beam only connects to pin via 2 bearings
- bearings can only take transverse loads (perpendicular to it's axis) and not axial loads
- point forces between bearings and pin and between pin and lugs at half the bearing/lug thickness
- cylindrical pin
- same material for both the lug and the bolt
 - ... more ...

'''

# Define variables
beam_length = 1.8  # [m]
thrust = 600  # [N]
sleeve_height = 0.03  # [m]

lug_thickness = 0.012  # [m]
lug_width = 0.035  # [m]
shear_strength_lug = 200 * 10**6  # [Pa]
yield_strength_lug = 345 * 10**6 # [Pa]
center_lug_hole = 0.03 # [m] distance from edge of lug to center of hole in lug

diameter_pin = 0.025  # [m]
bolt_head_height = 0.01  # [m]
nut_height = 0.01  # [m]
depth_of_contact = 0.005  # [m] depth of contact point between the pin and the beam with upper (or lower) surface as reference
shear_strength_bolt = 190 * 10**6  # [Pa]
yield_strength_bolt = 410 * 10**6  # [Pa]


safety_factor = 2 # [-]
thread_height = 0.03 # [m]

# DO NOT CHANGE THE VARIABLE "bearing_thickness" AS IT IS USED IN THE CALCULATIONS. 
# CHANGE THE VARIABLE "depth_of_contact" INSTEAD, WHICH IS THE DEPTH OF CONTACT POINT
# BETWEEN THE BOLT AND THE BEAM WHEN LOADED.
bearing_thickness = 2*depth_of_contact  # [m]

# This will determine the thread engagement ratio: length of thread engagement=thread_engagement_ratio*diameter_thread

Material_bolt = "Steel" # Steel or Aluminium (it is assumed that the lug is made of steel)
if Material_bolt == "Steel":
    thread_engagement_ratio = 1
    density_bolt=8000 # [kg/m^3]
elif Material_bolt == "Aluminium":
    thread_engagement_ratio = 2
    density_aluminium= 2780 # [kg/m^3]
density_aluminium= 2780 # [kg/m^3]
density_steel= 8000 # [kg/m^3]
# Calculate the pitch of the thread
screw_thread_pitch=0.007 # [m]
pitch_circle_diameter_thread=diameter_pin-0.64952*screw_thread_pitch # [m]


# Calculate bolt diameter based on internal bencing and shear forces
def calculate_bolt_parameters(thrust, safety_factor, beam_length, sleeve_height, lug_thickness, bolt_head_height, nut_height, bearing_thickness, shear_strength_bolt, yield_strength_bolt):
    # Calculate other parameters
    total_bolt_length = 2 * lug_thickness + sleeve_height + bolt_head_height + nut_height
    Moment = thrust * beam_length

    # Calculate forces on bolt
    F_bearing = thrust * beam_length / (sleeve_height - bearing_thickness)  # [N]
    F_lug = F_bearing * ((sleeve_height - bearing_thickness) / (sleeve_height + lug_thickness))  # [N]

    # Calculate shear force and bending moment
    max_internal_shear = -F_lug  # [N]
    max_internal_bending = max_internal_shear * 0.5 * (bearing_thickness + lug_thickness)  # [Nm]

    # Create x-axis for plotting
    x = np.linspace(0, total_bolt_length, 1000)
    y_shear = np.zeros_like(x)

    # Set the values of shear force based on the given conditions
    y_shear[(x >= 0) & (x <= bolt_head_height + 0.5 * lug_thickness)] = 0
    y_shear[(x > bolt_head_height + 0.5 * lug_thickness) & (x <= bolt_head_height + lug_thickness + 0.5 * bearing_thickness)] = -F_lug
    y_shear[(x > bolt_head_height + lug_thickness + 0.5 * bearing_thickness) & (x <= bolt_head_height + lug_thickness + sleeve_height - bearing_thickness * 0.5)] = F_bearing - F_lug
    y_shear[(x > bolt_head_height + lug_thickness + sleeve_height - bearing_thickness * 0.5) & (x <= total_bolt_length)] = -F_lug
    y_shear[(x > total_bolt_length - nut_height - 0.5 * lug_thickness)] = 0  # Set shear force to 0 after total_bolt_length

    # Plotting the shear force diagram
    '''
    plt.figure()
    plt.plot(x, y_shear)
    plt.xlabel('x [m]')
    plt.ylabel('Shear Force [N]')
    plt.title('Internal Shear Force Diagram')
    plt.grid(True)
    '''
    # Integrate shear force to get bending moment
    y_moment = np.zeros_like(x)
    y_moment = cumulative_trapezoid(y_shear, x, initial=0)

    # Plotting the bending moment diagram
    '''
    plt.figure()
    plt.plot(x, y_moment)
    plt.xlabel('x [m]')
    plt.ylabel('Bending Moment [Nm]')
    plt.title('Internal Bending Moment Diagram')
    plt.grid(True)
    plt.show()
'''
    # find the maximum internal shear and moment
    max_internal_shear = max(abs(y_shear))
    max_internal_moment = max(abs(y_moment))

    # Calculate diameter of bolt required to support the shear force * safety factor
    #diameter_bolt_shear = np.sqrt(16 * max_internal_shear * safety_factor / (3 * np.pi * shear_strength_bolt))
    max_supportable_internal_shear_force = (3 * np.pi * shear_strength_bolt * (diameter_pin ** 2)) / 16
    #diameter_bolt_bending = (32 * max_internal_moment * safety_factor / (np.pi * yield_strength_bolt)) ** (1/3)
    max_supportable_internal_bending_moment = yield_strength_bolt * (diameter_pin ** 3) * np.pi / 32

    FoS_shear = max_supportable_internal_shear_force / max_internal_shear
    FoS_bending = max_supportable_internal_bending_moment / max_internal_moment

    #print(f'The maximum internal shear force is {max_internal_shear:.2f} N')
    #print(f'The maximum internal bending moment is {max_internal_moment:.2f} Nm')
    print(f"The safety factor for shear is: {FoS_shear:.2f}")
    print(f"The safety factor for bending is: {FoS_bending:.2f}")
    return F_lug, FoS_shear, FoS_bending


# Force on head of bolt
def calculate_bolt_head_parameters(thrust, safety_factor, yield_strength_bolt):
    F_head = thrust * safety_factor
    Area_bolt_head = F_head / yield_strength_bolt
    side_length_bolt_head = np.sqrt(Area_bolt_head / (3 * np.sqrt(3) / 2))
    print(f"Area of the bolt head: {Area_bolt_head:.2f} mm^2")
    print(f"Side length of the bolt head: {side_length_bolt_head:.2f} mm")  
    return Area_bolt_head, side_length_bolt_head

#### MALE THREAD CALCULATIONS ####

##TENSILE AREA CALCULATIONS##
# Tensile Stress Area of male screw
def calculate_Tensile_Stress_Area(diameter_pin, screw_thread_pitch):
    At= (np.pi/4)*(diameter_pin-0.938194*screw_thread_pitch)**2
    print(f"Tensile stress area: {At*10**6:.2f} mm^2")
    return At
# Length of Thread Engagement of male screw
def calculate_Length_Thread_Engagement(Tensile_area, pitch_circle_diameter_thread):
    L_thread_engagement= 2*Tensile_area/(0.5*np.pi*pitch_circle_diameter_thread)
    print(f"Minimum length of thread engagement: {L_thread_engagement:.2f} m")
    return L_thread_engagement

# Safety Factor for Tensile Loading
def calculate_bolt_thread_safety(thrust, thread_area):
    F_thread = thrust
    Effective_load= F_thread/thread_area
    FoS= yield_strength_bolt/Effective_load
    print(f"Factor of safety for the bolt thread: {FoS}")
    return FoS


##SHEAR AREA CALCULATIONS##
# Thread Shear Area of male screw
def calculate_Thread_Shear_Area(L_thread_engagement, pitch_circle_diameter_thread):
    A_ss= 0.5*np.pi*pitch_circle_diameter_thread*L_thread_engagement
    return A_ss

### FEMALE THREAD CALCULATIONS ###

# Equation showing the dependence of K_t on the dimensionless ratio diameter/lug_width
def calculate_kt(diameter, lug_width):
    kt = 3 - 3.14 * (diameter / lug_width) + 3.667 * (diameter / lug_width)**2 - 1.527 * (diameter / lug_width)**3
    return kt

# Calculate the net section stress
def calculate_net_section(diameter, lug_width):
    net_area = (lug_width-diameter)*lug_thickness
    nominal_stress = F_lug/net_area
    max_stress = nominal_stress/calculate_kt(diameter, lug_width)
    FoS = yield_strength_lug/max_stress
    print(f"factor of safety for net stress: {FoS:.2f}")
    return FoS

# Calculate at what force the bolt will fail in shear out and check if it will occur now
def calculate_shear_out(diameter, center_lug_hole, lug_thickness):
    area_shear_out = 2*(center_lug_hole - diameter/2)*lug_thickness
    ultimate_shear_load = area_shear_out*shear_strength_lug
    FoS = ultimate_shear_load/F_lug
    print(f"The safety factor for shear out is: {FoS:.2f}")
    return FoS

# Calculate the bearing failure of the and lug
def calculate_bearing_failure(diameter, lug_thickness):
    area_bearing = diameter*lug_thickness
    ultimate_bearing_load = area_bearing*shear_strength_bolt
    FoS = ultimate_bearing_load/F_lug
    print(f"The safety factor for bearing failure is: {FoS:.2f}")


F_lug, FoS_internal_shear, FoS_internal_moment = calculate_bolt_parameters(thrust, safety_factor, beam_length, sleeve_height, lug_thickness, bolt_head_height, nut_height, bearing_thickness, shear_strength_bolt, yield_strength_bolt)
net_section_safety_factor = calculate_net_section(diameter_pin, lug_width)
shear_out_safety_factor = calculate_shear_out(diameter_pin, center_lug_hole, lug_thickness)
bearing_safety_factor = calculate_bearing_failure(diameter_pin, lug_thickness)
#A_t=calculate_Tensile_Stress_Area(diameter_pin, screw_thread_pitch)
#FoS_tensile_bolt=calculate_bolt_thread_safety(thrust, A_t)
#L_thread_engagement=calculate_Length_Thread_Engagement(A_t, pitch_circle_diameter_thread)
#print(f"According to a rule of thumb, the thread engagement should be at least {thread_engagement_ratio} times the diameter of the bolt, depending on the material which is {thread_engagement_ratio*diameter_pin:.2f} m")

#Weight Calculation
def total_weight(density_hinge, density_bolt, beam_length, sleeve_height, lug_thickness, bolt_head_height, nut_height, bearing_thickness, diameter_pin, thread_height):
    volume_pin=np.pi*(diameter_pin/2)**2*(2 * lug_thickness + sleeve_height + bolt_head_height + nut_height)
    print(2 * lug_thickness + sleeve_height + bolt_head_height + nut_height)
    volume_lug= lug_thickness*lug_width
    volume_bearing= bearing_thickness*diameter_pin
    volume_sleeve= sleeve_height*center_lug_hole
    weight= density_hinge*(volume_lug+volume_bearing+volume_sleeve)
    weight+= density_bolt*(volume_pin)
    return weight
print(f"The total weight of the structure is: {total_weight(density_aluminium, density_bolt, beam_length, sleeve_height, lug_thickness, bolt_head_height, nut_height, bearing_thickness, diameter_pin, thread_height):.2f} N")