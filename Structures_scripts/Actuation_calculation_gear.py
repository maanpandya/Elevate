import numpy as np
import matplotlib.pyplot as plt

# Constants
m_rod = 1  # mass
m_motor = 10 # mass at end of beam
L = 1  # length
I = (1/3) * m_rod * L**2+ m_motor*L**2  # Moment of inertia

theta_f = (4/9) * np.pi  # Final angle to cover
t_f = 30  # Total time  
angular_freq= 2*np.pi/(t_f) # Angular frequency
A = theta_f/(angular_freq*t_f-angular_freq**2*np.sin(angular_freq*t_f))

# Time array
t = np.linspace(0, t_f, 1000)

# Angular acceleration, velocity, and position functions
alpha_t = A * np.sin(angular_freq * t)
omega_t = -A * (angular_freq) * np.cos(angular_freq*t) + A * (angular_freq)
theta_t = -A * (angular_freq**2) * np.sin(angular_freq*t)+ A * (angular_freq)*t

# Torque as a function of time
torque_t = I * alpha_t

# Plotting the results
plt.figure(figsize=(10, 6))

plt.subplot(3, 1, 1)
plt.plot(t, theta_t)
plt.title('Angular Position (θ) vs Time')
plt.ylabel('θ (rad)')

plt.subplot(3, 1, 2)
plt.plot(t, omega_t)
plt.title('Angular Velocity (ω) vs Time')
plt.ylabel('ω (rad/s)')

plt.subplot(3, 1, 3)
plt.plot(t, torque_t)
plt.title('Torque (τ) vs Time')
plt.ylabel('Torque (N·m)')
plt.xlabel('Time (s)')

plt.tight_layout()
plt.show()

print("The maximum torque required is:", np.max(torque_t))
print("The maximum angle is:", np.rad2deg(np.max(theta_t)))