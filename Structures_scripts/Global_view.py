import numpy as np
import matplotlib.pyplot as plt

''' Assumptions (yes, there are loads):
- beam only connects to pin via 2 bearings
- bearings can only take transverse loads (perpendicular to it's axis)
- point forces between bearings and pin and between pin and lugs at half the bearing/lug thickness
- cylindrical pin

'''

# Define variables
beam_width = 0.07  # [m]
lug_thickness = 0.01  # [m]
bearing_thickness = 0.01  # [m]
beam_length = 2  # [m]
thrust = 200  # [N]
bolt_head_height = 0.01  # [m]
nut_height = 0.01  # [m]
E_mod = 73.1 *10**9 # [Pa]
shear_strength = 235 * 10**6  # [Pa]
yield_strength = 345 * 10**6  # [Pa]
safety_factor = 2 # [-]


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
y_shear[(x > bolt_head_height + lug_thickness + 0.5 * bearing_thickness) & (x <= bolt_head_height + lug_thickness + beam_width - lug_thickness * 0.5)] = F_bearing - F_lug
y_shear[(x > bolt_head_height + lug_thickness + beam_width - lug_thickness * 0.5) & (x <= total_bolt_length)] = -F_lug
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
max_shear = max(y_shear)
max_moment = max(y_moment)

# Calculate diameter of bolt required to support the shear force * safety factor
diameter_bolt_shear = np.sqrt(16 * max_shear * safety_factor / (3 * np.pi * shear_strength))
diamter_bolt_bending = (32*max_moment*safety_factor/(np.pi*yield_strength))**(1/3)


print(f'The maximum internal shear force is {max_shear:.2f} N')
print(f'The maximum internal bending moment is {max_moment:.2f} Nm')
print(f'The required bolt diameter to support the shear force is {diameter_bolt_shear*1000:.3g} mm')
print(f'The required bolt diameter to support the bending moment is {diamter_bolt_bending*1000:.3g} mm')



