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

def mission_profile(climb_velocity, cruise_velocity, descent_velocity, climb_altitude, cruise_distance, descent_altitude, acceleration, deceleration):
    # Climb phase
    t_acc_climb = climb_velocity / acceleration
    d_acc_climb = 0.5 * acceleration * t_acc_climb**2
    t_cons_climb = (climb_altitude - 2 * d_acc_climb) / climb_velocity
    t_decel_climb = climb_velocity / deceleration

    # Cruise phase
    t_acc_cruise = cruise_velocity / acceleration
    d_acc_cruise = 0.5 * acceleration * t_acc_cruise**2
    t_cons_cruise = (cruise_distance - 2 * d_acc_cruise) / cruise_velocity
    t_decel_cruise = cruise_velocity / deceleration

    # Descent phase
    t_acc_descent = descent_velocity / acceleration
    d_acc_descent = 0.5 * acceleration * t_acc_descent**2
    t_cons_descent = (descent_altitude - 2 * d_acc_descent) / descent_velocity
    t_decel_descent = descent_velocity / deceleration

    total_time = (t_acc_climb + t_cons_climb + t_decel_climb +
                  t_acc_cruise + t_cons_cruise + t_decel_cruise +
                  t_acc_descent + t_cons_descent + t_decel_descent)

    return (t_acc_climb, t_cons_climb, t_decel_climb, 
            t_acc_cruise, t_cons_cruise, t_decel_cruise, 
            t_acc_descent, t_cons_descent, t_decel_descent, 
            total_time, climb_velocity, cruise_velocity, descent_velocity)

def generate_data(mass, climb_velocity, cruise_velocity, descent_velocity, climb_altitude, cruise_distance, descent_altitude, acceleration, deceleration, rho=1.225):
    # Generate mission profile data
    (t_acc_climb, t_cons_climb, t_decel_climb, 
     t_acc_cruise, t_cons_cruise, t_decel_cruise, 
     t_acc_descent, t_cons_descent, t_decel_descent, 
     total_time, v_const_climb, v_cons_cruise, v_cons_descent) = mission_profile(
        climb_velocity, cruise_velocity, descent_velocity, climb_altitude, cruise_distance, descent_altitude, acceleration, deceleration)
    
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
    
    # Time divisions for each phase
    t_climb_end = t_acc_climb + t_cons_climb + t_decel_climb
    t_cruise_start = t_climb_end
    t_cruise_end = t_cruise_start + t_acc_cruise + t_cons_cruise + t_decel_cruise
    t_descent_start = t_cruise_end
    
    for i, t in enumerate(time):
        if t <= t_acc_climb:  # Climb acceleration
            velocity[i] = acceleration * t
            altitude[i] = 0.5 * acceleration * t**2
            acc = acceleration
            T, p = calculate_thrust_and_power(mass, velocity[i], acc, altitude[i], rho)
            thrust_climb.append(T)
            velocity_climb.append(velocity[i])
        elif t <= t_acc_climb + t_cons_climb:  # Constant velocity climb
            velocity[i] = climb_velocity
            altitude[i] = 0.5 * acceleration * t_acc_climb**2 + climb_velocity * (t - t_acc_climb)
            acc = 0
            T, p = calculate_thrust_and_power(mass, velocity[i], acc, altitude[i], rho)
            thrust_climb.append(T)
            velocity_climb.append(velocity[i])
        elif t <= t_climb_end:  # Climb deceleration
            t_local = t - (t_acc_climb + t_cons_climb)
            velocity[i] = climb_velocity - deceleration * t_local
            altitude[i] = climb_altitude - 0.5 * deceleration * t_local**2
            acc = -deceleration
            T, p = calculate_thrust_and_power(mass, velocity[i], acc, altitude[i], rho)
            thrust_climb.append(T)
            velocity_climb.append(velocity[i])
        elif t <= t_cruise_start + t_acc_cruise:  # Cruise acceleration
            t_local = t - t_cruise_start
            velocity[i] = acceleration * t_local
            altitude[i] = climb_altitude
            acc = acceleration
            T, p = calculate_thrust_and_power(mass, velocity[i], acc, altitude[i], rho)
            thrust_cruise.append(T)
            velocity_cruise.append(velocity[i])
        elif t <= t_cruise_start + t_acc_cruise + t_cons_cruise:  # Constant cruise
            velocity[i] = cruise_velocity
            altitude[i] = climb_altitude
            acc = 0
            T, p = calculate_thrust_and_power(mass, velocity[i], acc, altitude[i], rho)
            thrust_cruise.append(T)
            velocity_cruise.append(velocity[i])
        elif t <= t_cruise_end:  # Cruise deceleration
            t_local = t - (t_cruise_start + t_acc_cruise + t_cons_cruise)
            velocity[i] = cruise_velocity - deceleration * t_local
            altitude[i] = climb_altitude
            acc = -deceleration
            T, p = calculate_thrust_and_power(mass, velocity[i], acc, altitude[i], rho)
            thrust_cruise.append(T)
            velocity_cruise.append(velocity[i])
        elif t <= t_descent_start + t_acc_descent:  # Descent acceleration
            t_local = t - t_descent_start
            velocity[i] = -acceleration * t_local
            altitude[i] = climb_altitude - 0.5 * acceleration * t_local**2
            acc = -acceleration
            T, p = calculate_thrust_and_power(mass, abs(velocity[i]), acc, altitude[i], rho)
            thrust_descent.append(T)
            velocity_descent.append(abs(velocity[i]))
        elif t <= t_descent_start + t_acc_descent + t_cons_descent:  # Constant descent
            velocity[i] = -descent_velocity
            altitude[i] = climb_altitude - (0.5 * acceleration * t_acc_descent**2 + 
                                            descent_velocity * (t - t_descent_start - t_acc_descent))
            acc = 0
            T, p = calculate_thrust_and_power(mass, abs(velocity[i]), acc, altitude[i], rho)
            thrust_descent.append(T)
            velocity_descent.append(abs(velocity[i]))
        else:  # Descent deceleration
            t_local = t - (t_descent_start + t_acc_descent + t_cons_descent)
            velocity[i] = -descent_velocity + deceleration * t_local
            altitude[i] = max(0, descent_altitude - (descent_velocity * t_local - 0.5 * deceleration * t_local**2))
            acc = deceleration
            T, p = calculate_thrust_and_power(mass, abs(velocity[i]), acc, altitude[i], rho)
            thrust_descent.append(T)
            velocity_descent.append(abs(velocity[i]))

        thrust[i], power[i] = calculate_thrust_and_power(mass, abs(velocity[i]), acc, altitude[i], rho)
        vertical_distance[i] = altitude[i]
        

        if t <= t_climb_end:
            horizontal_distance[i] = 0
        elif t <= t_cruise_end:
            t_local = t - t_climb_end
            if t_local <= t_acc_cruise:
                horizontal_distance[i] = 0.5 * acceleration * t_local**2
            elif t_local <= t_acc_cruise + t_cons_cruise:
                horizontal_distance[i] = 0.5 * acceleration * t_acc_cruise**2 + cruise_velocity * (t_local - t_acc_cruise)
            else:
                t_decel = t_local - (t_acc_cruise + t_cons_cruise)
                horizontal_distance[i] = (0.5 * acceleration * t_acc_cruise**2 + 
                                          cruise_velocity * t_cons_cruise +
                                          cruise_velocity * t_decel - 0.5 * deceleration * t_decel**2)
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




