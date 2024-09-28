import numpy as np
import matplotlib.pyplot as plt

def calculate_thrust_and_power(mass, velocity, acceleration, rho=1.225):
    g = 9.80665 #m/s^2
    equivalent_flat_plate_area = 0.808256 #m^2 (equivalent flat plate area source)  
    drag = 0.5 * equivalent_flat_plate_area * rho * velocity**2 + mass * g
    thrust = mass * acceleration + drag
    power = thrust * velocity
    return thrust, power

def mission_profile(max_speed, climb_altitude, descent_altitude, acceleration, deceleration):
    # Climb phase
    t_acc = max_speed / acceleration
    d_acc = 0.5 * acceleration * t_acc**2
    
    if 2 * d_acc > climb_altitude:
        t_acc = np.sqrt(climb_altitude / acceleration)
        max_speed = acceleration * t_acc
        t_cruise = 0
    else:
        t_cruise = (climb_altitude - 2 * d_acc) / max_speed
    
    # Descent phase
    t_decel = max_speed / deceleration
    d_decel = 0.5 * deceleration * t_decel**2
    
    if 2 * d_decel > descent_altitude:
        t_decel = np.sqrt(descent_altitude / deceleration)
        t_descent = t_decel
    else:
        t_descent = (descent_altitude - 2 * d_decel) / max_speed + t_decel
    
    total_time = 2 * t_acc + t_cruise + 2 * t_decel
    
    return t_acc, t_cruise, t_decel, t_descent, total_time, max_speed

def generate_data(mass, max_speed, climb_altitude, descent_altitude, acceleration, deceleration, rho=1.225):
    t_acc, t_cruise, t_decel, t_descent, total_time, actual_max_speed = mission_profile(max_speed, climb_altitude, descent_altitude, acceleration, deceleration)
    
    time = np.linspace(0, total_time, 1000)
    altitude = np.zeros_like(time)
    velocity = np.zeros_like(time)
    thrust = np.zeros_like(time)
    power = np.zeros_like(time)
    distance = np.zeros_like(time)
    # make arrays to store velocity and thrust in different phases
    vel_climb = []
    vel_cruise = []
    vel_decel = []
    thrust_climb = []
    thrust_cruise = []
    thrust_decel = []
    
    current_distance = 0  # Tracking total distance

    for i, t in enumerate(time):
        # track velocity in different phases
        if t <= t_acc:  # Acceleration / Climb phase
            velocity[i] = acceleration * t
            vel_climb.append(velocity[i])
            altitude[i] = 0.5 * acceleration * t**2
            thrust[i], power[i] = calculate_thrust_and_power(mass, velocity[i], acceleration, rho)
            thrust_climb.append(thrust[i])
            current_distance = 0.5 * acceleration * t**2  # distance covered during climb
        elif t <= t_acc + t_cruise:  # Cruise phase
            velocity[i] = actual_max_speed
            vel_cruise.append(velocity[i])
            altitude[i] = climb_altitude
            thrust[i], power[i] = calculate_thrust_and_power(mass, velocity[i], 0, rho)
            thrust_cruise.append(thrust[i])
            current_distance = current_distance + actual_max_speed * (t - t_acc)  # Accumulate cruise distance
        elif t <= t_acc + t_cruise + t_decel:  # Deceleration / Descent phase
            t_decel_local = t - (t_acc + t_cruise)
            velocity[i] = actual_max_speed - deceleration * t_decel_local
            vel_decel.append(velocity[i])
            altitude[i] = climb_altitude - 0.5 * deceleration * t_decel_local**2
            thrust[i], power[i] = calculate_thrust_and_power(mass, velocity[i], -deceleration, rho)
            thrust_decel.append(thrust[i])
            current_distance = current_distance + actual_max_speed * t_decel_local - 0.5 * deceleration * t_decel_local**2  # Cumulative distance
        else:  # Post mission hover or zero velocity phase (if applicable)
            velocity[i] = 0
            altitude[i] = climb_altitude - descent_altitude
            thrust[i], power[i] = calculate_thrust_and_power(mass, velocity[i], 0, rho)

        distance[i] = current_distance  # Update total distance
        
        # make arrays to store velocity and thrust in different phases
    vel_climb = np.array(vel_climb)
    vel_decel = np.array(vel_decel)
    vel_decel = np.array(vel_decel)
    thrust_climb = np.array(thrust_climb)
    thrust_cruise = np.array(thrust_cruise)
    thrust_decel = np.array(thrust_decel)

    mission_profile = [time, altitude, velocity, thrust, power, distance, vel_climb, vel_decel, thrust_climb, thrust_cruise, thrust_decel]
    return mission_profile

def plot_results(time, altitude, velocity, thrust, power, distance):
    fig, axes = plt.subplots(3, 2, figsize=(15, 20))
    
    # Thrust plots
    axes[0, 0].plot(time, thrust)
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Thrust (N)')
    axes[0, 0].set_title('Thrust vs Time')
    axes[0, 0].grid(True)
    
    axes[0, 1].plot(distance, thrust)
    axes[0, 1].set_xlabel('Distance (m)')
    axes[0, 1].set_ylabel('Thrust (N)')
    axes[0, 1].set_title('Thrust vs Distance')
    axes[0, 1].grid(True)
    
    # Altitude and velocity plots
    axes[1, 0].plot(time, altitude)
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Altitude (m)')
    axes[1, 0].set_title('Altitude vs Time')
    axes[1, 0].grid(True)
    
    axes[1, 1].plot(time, velocity)
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Velocity (m/s)')
    axes[1, 1].set_title('Velocity vs Time')
    axes[1, 1].grid(True)
    
    # Power plot
    axes[2, 0].plot(time, power / 1000)  # Convert to kW
    axes[2, 0].set_xlabel('Time (s)')
    axes[2, 0].set_ylabel('Power (kW)')
    axes[2, 0].set_title('Power vs Time')
    axes[2, 0].grid(True)
    
    axes[2, 1].plot(distance, power / 1000)  # Converting to kW
    axes[2, 1].set_xlabel('Distance (m)')
    axes[2, 1].set_ylabel('Power (kW)')
    axes[2, 1].set_title('Power vs Distance')
    axes[2, 1].grid(True)
    
    plt.tight_layout()
    plt.show()


mass = 500  # kg
max_speed = 20  # m/s
climb_altitude = 500  # m
descent_altitude = 500  # m
acceleration = 9.81  # m/s^2
deceleration = 9.81  # m/s^2
rho = 1.225  # kg/m^3

mission_profile = generate_data(mass, max_speed, climb_altitude, descent_altitude, acceleration, deceleration, rho)
#plot_results(time, altitude, velocity, thrust, power, distance) # toggle comment to plot the results






