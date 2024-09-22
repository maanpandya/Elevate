import numpy as np
import matplotlib.pyplot as plt

def calculate_thrust_and_power(mass, velocity, acceleration, rho=1.225):
    g = 9.81  # gravitational acceleration (m/s^2)
    drag = 0.5 * rho * velocity**2 * 0.808256  + mass * g
    thrust = mass * acceleration + drag
    power = thrust * velocity
    return thrust, power

def mission_profile(max_speed, climb_distance, acceleration):
    t_acc = max_speed / acceleration
    d_acc = 0.5 * acceleration * t_acc**2
    
    if 2 * d_acc > climb_distance:
        t_acc = np.sqrt(climb_distance / acceleration)
        max_speed = acceleration * t_acc
        t_cruise = 0
    else:
        t_cruise = (climb_distance - 2 * d_acc) / max_speed
    
    total_time = 2 * t_acc + t_cruise
    
    return t_acc, t_cruise, total_time, max_speed

def generate_data(mass, max_speed, climb_distance, acceleration, rho=1.225):
    t_acc, t_cruise, total_time, actual_max_speed = mission_profile(max_speed, climb_distance, acceleration)
    
    time = np.linspace(0, total_time, 1000)
    distance = np.zeros_like(time)
    velocity = np.zeros_like(time)
    thrust = np.zeros_like(time)
    power = np.zeros_like(time)
    
    for i, t in enumerate(time):
        if t <= t_acc:
            velocity[i] = acceleration * t
            distance[i] = 0.5 * acceleration * t**2
            thrust[i], power[i] = calculate_thrust_and_power(mass, velocity[i], acceleration, rho)
        elif t <= t_acc + t_cruise:
            velocity[i] = actual_max_speed
            distance[i] = 0.5 * acceleration * t_acc**2 + actual_max_speed * (t - t_acc)
            thrust[i], power[i] = calculate_thrust_and_power(mass, velocity[i], 0, rho)
        else:
            t_decel = t - (t_acc + t_cruise)
            velocity[i] = actual_max_speed - acceleration * t_decel
            distance[i] = climb_distance - 0.5 * acceleration * t_decel**2
            thrust[i], power[i] = calculate_thrust_and_power(mass, velocity[i], -acceleration, rho)
    
    return time, distance, velocity, thrust, power

def plot_results(time, distance, thrust, power):
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 15))
    
    ax1.plot(time, thrust)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Thrust (N)')
    ax1.set_title('Thrust vs Time')
    ax1.grid(True)
    
    ax2.plot(distance, thrust)
    ax2.set_xlabel('Distance (m)')
    ax2.set_ylabel('Thrust (N)')
    ax2.set_title('Thrust vs Distance')
    ax2.grid(True)
    
    ax3.plot(time, power / 1000)  # Convert to kW
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Power (kW)')
    ax3.set_title('Power vs Time')
    ax3.grid(True)
    
    ax4.plot(distance, power / 1000)  # Convert to kW
    ax4.set_xlabel('Distance (m)')
    ax4.set_ylabel('Power (kW)')
    ax4.set_title('Power vs Distance')
    ax4.grid(True)
    
    plt.tight_layout()
    plt.show()

mass = 1000  # kg
max_speed = 20  # m/s
climb_distance = 500  # m
acceleration = 9.81  # m/s^2
rho = 1.225  

time, distance, velocity, thrust, power = generate_data(mass, max_speed, climb_distance, acceleration, rho)
plot_results(time, distance, thrust, power)