import numpy as np
import matplotlib.pyplot as plt

def calculate_thrust_and_power(mass, velocity, acceleration, rho=1.225):
    g = 9.81  # gravitational acceleration (m/s^2)
    drag = 0.5 * 0.808256 * rho * velocity**2  + mass * g
    thrust = mass * acceleration + drag
    power = thrust * velocity
    return thrust, power

def mission_profile(max_speed, climb_altitude, acceleration):
    t_acc = max_speed / acceleration
    d_acc = 0.5 * acceleration * t_acc**2
    
    if 2 * d_acc > climb_altitude:
        t_acc = np.sqrt(climb_altitude / acceleration)
        max_speed = acceleration * t_acc
        t_cruise = 0
    else:
        t_cruise = (climb_altitude - 2 * d_acc) / max_speed
    
    total_time = 2 * t_acc + t_cruise
    
    return t_acc, t_cruise, total_time, max_speed

def generate_data(mass, max_speed, climb_altitude, acceleration, rho=1.225):
    t_acc, t_cruise, total_time, actual_max_speed = mission_profile(max_speed, climb_altitude, acceleration)
    
    time = np.linspace(0, total_time, 1000)
    altitude = np.zeros_like(time)
    velocity = np.zeros_like(time)
    thrust = np.zeros_like(time)
    power = np.zeros_like(time)
    
    for i, t in enumerate(time):
        if t <= t_acc:
            velocity[i] = acceleration * t
            altitude[i] = 0.5 * acceleration * t**2
            thrust[i], power[i] = calculate_thrust_and_power(mass, velocity[i], acceleration, rho)
        elif t <= t_acc + t_cruise:
            velocity[i] = actual_max_speed
            altitude[i] = 0.5 * acceleration * t_acc**2 + actual_max_speed * (t - t_acc)
            thrust[i], power[i] = calculate_thrust_and_power(mass, velocity[i], 0, rho)
        else:
            t_decel = t - (t_acc + t_cruise)
            velocity[i] = actual_max_speed - acceleration * t_decel
            altitude[i] = climb_altitude - 0.5 * acceleration * t_decel**2
            thrust[i], power[i] = calculate_thrust_and_power(mass, velocity[i], -acceleration, rho)
    
    return time, altitude, velocity, thrust, power

def plot_results(time, altitude, velocity, thrust, power):
    fig, axes = plt.subplots(3, 2, figsize=(15, 20))
    
    # Thrust plots
    axes[0, 0].plot(time, thrust)
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Thrust (N)')
    axes[0, 0].set_title('Thrust vs Time')
    axes[0, 0].grid(True)
    
    axes[0, 1].plot(altitude, thrust)
    axes[0, 1].set_xlabel('Altitude (m)')
    axes[0, 1].set_ylabel('Thrust (N)')
    axes[0, 1].set_title('Thrust vs Altitude')
    axes[0, 1].grid(True)
    
    # Plot altitude vs time
    axes[1, 0].plot(time, altitude)
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Altitude (m)')
    axes[1, 0].set_title('Altitude vs Time')
    axes[1, 0].grid(True)
    
    axes[1, 1].plot(altitude, power / 1000)  # Convert to kW
    axes[1, 1].set_xlabel('Altitude (m)')
    axes[1, 1].set_ylabel('Power (kW)')
    axes[1, 1].set_title('Power vs Altitude')
    axes[1, 1].grid(True)
    
    # Velocity plots
    axes[2, 0].plot(time, velocity)
    axes[2, 0].set_xlabel('Time (s)')
    axes[2, 0].set_ylabel('Velocity (m/s)')
    axes[2, 0].set_title('Velocity vs Time')
    axes[2, 0].grid(True)
    
    axes[2, 1].plot(altitude, velocity)
    axes[2, 1].set_xlabel('Altitude (m)')
    axes[2, 1].set_ylabel('Velocity (m/s)')
    axes[2, 1].set_title('Velocity vs Altitude')
    axes[2, 1].grid(True)
    
    plt.tight_layout()
    plt.show()

mass = 5000  # kg
max_speed = 20  # m/s
climb_altitude = 500  # m
acceleration = 9.81  # m/s^2
rho = 1.225  # kg/m^3

time, altitude, velocity, thrust, power = generate_data(mass, max_speed, climb_altitude, acceleration, rho)
plot_results(time, altitude, velocity, thrust, power)