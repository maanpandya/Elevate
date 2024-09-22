import numpy as np
import matplotlib.pyplot as plt

''' Assumptions (yes, there are loads):
- beam only connects to pin via 2 bearings
- bearings can only take transverse loads (perpendicular to it's axis) and not axial loads
- point forces between bearings and pin and between pin and lugs at half the bearing/lug thickness
- cylindrical pin
 - ... more ...

'''

# Define variables
beam_width = 0.2  # [m]
lug_thickness = 0.05  # [m]
bearing_thickness = 0.03  # [m]
beam_length = 2  # [m]
thrust = 600  # [N]
diameter_pin = 0.0224  # [m]
bolt_head_height = 0.01  # [m]
nut_height = 0.01  # [m]
E_mod = 73.1 *10**9 # [Pa]
shear_strength = 235 * 10**6  # [Pa]
yield_strength = 345 * 10**6  # [Pa]
safety_factor = 2 # [-]
center_lug_hole = 0.04 # [m]

# Calculate bolt diameter based on internal bencing and shear forces
def calculate_bolt_parameters(thrust, safety_factor, beam_length, beam_width, lug_thickness, bolt_head_height, nut_height, bearing_thickness, shear_strength, yield_strength):
    # Calculate other parameters
    total_bolt_length = 2 * lug_thickness + beam_width + bolt_head_height + nut_height
    Moment = thrust * beam_length

    # Calculate forces on bolt
    F_bearing = thrust * beam_length / (beam_width - bearing_thickness)  # [N]
    F_lug = F_bearing * ((beam_width - bearing_thickness) / (beam_width + lug_thickness))  # [N]

    # Calculate shear force and bending moment
    max_internal_shear = -F_lug  # [N]
    max_internal_bending = max_internal_shear * 0.5 * (bearing_thickness + lug_thickness)  # [Nm]

    # Create x-axis for plotting
    x = np.linspace(0, total_bolt_length, 1000)
    y_shear = np.zeros_like(x)

    # Set the values of shear force based on the given conditions
    y_shear[(x >= 0) & (x <= bolt_head_height + 0.5 * lug_thickness)] = 0
    y_shear[(x > bolt_head_height + 0.5 * lug_thickness) & (x <= bolt_head_height + lug_thickness + 0.5 * bearing_thickness)] = -F_lug
    y_shear[(x > bolt_head_height + lug_thickness + 0.5 * bearing_thickness) & (x <= bolt_head_height + lug_thickness + beam_width - bearing_thickness * 0.5)] = F_bearing - F_lug
    y_shear[(x > bolt_head_height + lug_thickness + beam_width - bearing_thickness * 0.5) & (x <= total_bolt_length)] = -F_lug
    y_shear[(x > total_bolt_length - nut_height - 0.5 * lug_thickness)] = 0  # Set shear force to 0 after total_bolt_length

    # Plotting the shear force diagram
    plt.figure()
    plt.plot(x, y_shear)
    plt.xlabel('x [m]')
    plt.ylabel('Shear Force [N]')
    plt.title('Internal Shear Force Diagram')
    plt.grid(True)

    # Integrate shear force to get bending moment
    y_moment = np.zeros_like(x)

    # Numerical integration of shear force to get bending moment
    for i in range(1, len(x)):
        y_moment[i] = y_moment[i - 1] + (y_shear[i - 1] + y_shear[i]) / 2 * (x[i] - x[i - 1])

    # Plotting the bending moment diagram
    plt.figure()
    plt.plot(x, y_moment)
    plt.xlabel('x [m]')
    plt.ylabel('Bending Moment [Nm]')
    plt.title('Internal Bending Moment Diagram')
    plt.grid(True)
    plt.show()

    # find the maximum internal shear and moment
    max_internal_shear = max(y_shear)
    max_internal_moment = max(y_moment)

    # Calculate diameter of bolt required to support the shear force * safety factor
    #diameter_bolt_shear = np.sqrt(16 * max_internal_shear * safety_factor / (3 * np.pi * shear_strength))
    max_supportable_internal_shear_force = (3 * np.pi * shear_strength * (diameter_pin ** 2)) / 16
    #diameter_bolt_bending = (32 * max_internal_moment * safety_factor / (np.pi * yield_strength)) ** (1/3)
    max_supportable_internal_bending_moment = yield_strength * (diameter_pin ** 3) * np.pi / 32

    FoS_shear = max_supportable_internal_shear_force / max_internal_shear
    FoS_bending = max_supportable_internal_bending_moment / max_internal_moment

    #print(f'The maximum internal shear force is {max_internal_shear:.2f} N')
    #print(f'The maximum internal bending moment is {max_internal_moment:.2f} Nm')
    print(f"The safety factor for shear is: {FoS_shear:.2f}")
    print(f"The safety factor for bending is: {FoS_bending:.2f}")
    return F_lug, FoS_shear, FoS_bending

# Force on head of bolt
def calculate_bolt_head_parameters(thrust, safety_factor, yield_strength):
    F_head = thrust * safety_factor
    Area_bolt_head = F_head / yield_strength
    side_length_bolt_head = np.sqrt(Area_bolt_head / (3 * np.sqrt(3) / 2))
    print(f"Area of the bolt head: {Area_bolt_head:.2f} mm^2")
    print(f"Side length of the bolt head: {side_length_bolt_head:.2f} mm")  
    return Area_bolt_head, side_length_bolt_head

# Force on thread
def calculate_bolt_thread_parameters(thrust, safety_factor, yield_strength, diameter_bolt_bending):
    F_thread = thrust * safety_factor
    Area_bolt_thread = F_thread / (yield_strength / 10**6)
    height_bolt_thread = Area_bolt_thread / (np.pi * diameter_bolt_bending)
    print(f"Area of the bolt thread: {Area_bolt_thread:.2f} mm^2")
    print(f"Height of the bolt thread: {height_bolt_thread:.2f} mm")
    return Area_bolt_thread, height_bolt_thread

# Equation showing the dependence of K_t on the dimensionless ratio diameter/lug_width
def calculate_kt(diameter, lug_width):
    kt = 3 - 3.14 * (diameter / lug_width) + 3.667 * (diameter / lug_width)**2 - 1.527 * (diameter / lug_width)**3
    return kt

# Calculate the net section stress
def calculate_net_section(diameter, lug_width):
    net_area = (lug_width-diameter)*lug_thickness
    nominal_stress = F_lug/net_area
    max_stress = nominal_stress/calculate_kt(diameter, lug_width)
    FoS = yield_strength/max_stress
    print(f"factor of safety for net stress: {FoS:.2f}")
    return FoS

# Calculate at what force the bolt will fail in shear out and check if it will occur now
def calculate_shear_out(diameter, center_lug_hole, lug_thickness):
    area_shear_out = 2*(center_lug_hole - diameter)*lug_thickness
    ultimate_shear_load = area_shear_out*shear_strength
    FoS = ultimate_shear_load/F_lug
    print(f"The safety factor for shear out is: {FoS:.2f}")
    return FoS

# Calculate the bearing failure of the bolt and lug
def calculate_bearing_failure(diameter, lug_thickness):
    area_bearing = diameter*lug_thickness
    ultimate_bearing_load = area_bearing*shear_strength
    FoS = ultimate_bearing_load/F_lug
    print(f"The safety factor for bearing failure is: {FoS:.2f}")



F_lug, FoS_internal_shear, FoS_internal_moment = calculate_bolt_parameters(thrust, safety_factor, beam_length, beam_width, lug_thickness, bolt_head_height, nut_height, bearing_thickness, shear_strength, yield_strength)
net_section_safety_factor = calculate_net_section(diameter_pin, 0.03)
shear_out_safety_factor = calculate_shear_out(diameter_pin, center_lug_hole, lug_thickness)
bearing_safety_factor = calculate_bearing_failure(diameter_pin, lug_thickness)
