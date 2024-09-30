import numpy as np
import matplotlib.pyplot as plt

def calculate_thrust_and_power(mass, velocity, acceleration, altitude, rho=1.225):
    g = 9.80665  # m/s^2
    equivalent_flat_plate_area = 0.808256  # m^2 (equivalent flat plate area source)
    rho = rho * np.exp(-altitude / 8500)  # Atmospheric density adjustment with altitude
    drag = 0.5 * equivalent_flat_plate_area * rho * velocity**2
    weight = mass * g
    thrust = mass * acceleration + drag + weight  # Always add full weight for vertical phases
    power = thrust * velocity
    return thrust, power

def mission_profile(max_speed, climb_altitude, cruise_distance, descent_altitude, acceleration, deceleration):
    t_acc = max_speed / acceleration
    d_acc = 0.5 * acceleration * t_acc**2
    
    if d_acc > climb_altitude:
        t_acc = np.sqrt(2 * climb_altitude / acceleration)
        max_speed = acceleration * t_acc
        t_cons_climb = 0
    else:
        t_cons_climb = (climb_altitude - d_acc) / max_speed
    
    t_cruise = cruise_distance / max_speed
    
    t_decel = max_speed / deceleration
    d_decel = 0.5 * deceleration * t_decel**2
    
    if d_decel > descent_altitude:
        t_decel = np.sqrt(2 * descent_altitude / deceleration)
        t_cons_descent = 0
    else:
        t_cons_descent = (descent_altitude - d_decel) / max_speed
    
    t_descent = t_cons_descent + t_decel
    total_time = t_acc + t_cons_climb + t_cruise + t_descent
    
    return t_acc, t_cons_climb, t_cruise, t_cons_descent, t_decel, total_time, max_speed

def generate_data(mass, max_speed, climb_altitude, cruise_distance, descent_altitude, acceleration, deceleration, rho=1.225):
    t_acc, t_cons_climb, t_cruise, t_cons_descent, t_decel, total_time, actual_max_speed = mission_profile(
        max_speed, climb_altitude, cruise_distance, descent_altitude, acceleration, deceleration)
    
    time = np.linspace(0, total_time, 1000)
    altitude = np.zeros_like(time)
    velocity = np.zeros_like(time)
    thrust = np.zeros_like(time)
    power = np.zeros_like(time)
    horizontal_distance = np.zeros_like(time)
    vertical_distance = np.zeros_like(time)
    
    # New arrays for separate phases
    thrust_climb = []
    thrust_cruise = []
    thrust_descent = []
    velocity_climb = []
    velocity_cruise = []
    velocity_descent = []
    
    t_climb = t_acc + t_cons_climb
    t_cruise_end = t_climb + t_cruise
    t_const_descent_end = t_cruise_end + t_cons_descent
    
    for i, t in enumerate(time):
        if t <= t_acc:  # Acceleration / Climb phase
            velocity[i] = acceleration * t
            altitude[i] = 0.5 * acceleration * t**2
            thrust[i], power[i] = calculate_thrust_and_power(mass, velocity[i], acceleration, altitude[i], rho)
            thrust_climb.append(thrust[i])
            velocity_climb.append(velocity[i])
        elif t <= t_climb:  # Constant velocity climb
            velocity[i] = actual_max_speed
            altitude[i] = 0.5 * acceleration * t_acc**2 + actual_max_speed * (t - t_acc)
            thrust[i], power[i] = calculate_thrust_and_power(mass, velocity[i], 0, altitude[i], rho)
            thrust_climb.append(thrust[i])
            velocity_climb.append(velocity[i])
        elif t <= t_cruise_end:  # Cruise phase
            velocity[i] = actual_max_speed
            altitude[i] = climb_altitude
            thrust[i], power[i] = calculate_thrust_and_power(mass, velocity[i], 0, altitude[i], rho)
            thrust_cruise.append(thrust[i])
            velocity_cruise.append(velocity[i])
        elif t <= t_const_descent_end:  # Constant velocity descent
            velocity[i] = actual_max_speed
            altitude[i] = climb_altitude - actual_max_speed * (t - t_cruise_end)
            thrust[i], power[i] = calculate_thrust_and_power(mass, velocity[i], 0, altitude[i], rho)
            thrust_descent.append(thrust[i])
            velocity_descent.append(velocity[i])
        else:  # Deceleration / Final descent phase
            t_decel_local = t - t_const_descent_end
            velocity[i] = max(0, actual_max_speed - deceleration * t_decel_local)
            altitude[i] = max(0, altitude[int(t_const_descent_end * 1000 / total_time)] - 
                              (actual_max_speed * t_decel_local - 0.5 * deceleration * t_decel_local**2))
            thrust[i], power[i] = calculate_thrust_and_power(mass, velocity[i], -deceleration, altitude[i], rho)
            thrust_descent.append(thrust[i])
            velocity_descent.append(velocity[i])
        
        vertical_distance[i] = altitude[i]
        
        if t <= t_climb:
            horizontal_distance[i] = 0
        elif t <= t_cruise_end:
            horizontal_distance[i] = actual_max_speed * (t - t_climb)
        else:
            horizontal_distance[i] = cruise_distance

    # Convert lists to numpy arrays
    thrust_climb = np.array(thrust_climb)
    thrust_cruise = np.array(thrust_cruise)
    thrust_descent = np.array(thrust_descent)
    velocity_climb = np.array(velocity_climb)
    velocity_cruise = np.array(velocity_cruise)
    velocity_descent = np.array(velocity_descent)

    return (time, altitude, velocity, thrust, power, horizontal_distance, vertical_distance,
            thrust_climb, thrust_cruise, thrust_descent, velocity_climb, velocity_cruise, velocity_descent)


def plot_results(time, altitude, velocity, thrust, power, horizontal_distance, vertical_distance):
    fig, axes = plt.subplots(3, 2, figsize=(15, 20))
    
    axes[0, 0].plot(time, thrust)
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Thrust (N)')
    axes[0, 0].set_title('Thrust vs Time')
    axes[0, 0].grid(True)
    
    axes[0, 1].plot(horizontal_distance, thrust)
    axes[0, 1].set_xlabel('Horizontal Distance (m)')
    axes[0, 1].set_ylabel('Thrust (N)')
    axes[0, 1].set_title('Thrust vs Horizontal Distance')
    axes[0, 1].grid(True)
    
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
    
    axes[2, 0].plot(time, power / 1000)  # Convert to kW
    axes[2, 0].set_xlabel('Time (s)')
    axes[2, 0].set_ylabel('Power (kW)')
    axes[2, 0].set_title('Power vs Time')
    axes[2, 0].grid(True)
    
    axes[2, 1].plot(horizontal_distance, vertical_distance)
    axes[2, 1].set_xlabel('Horizontal Distance (m)')
    axes[2, 1].set_ylabel('Vertical Distance (m)')
    axes[2, 1].set_title('Flight Path')
    axes[2, 1].grid(True)
    
    plt.tight_layout()
    plt.show()





