import numpy as np
from matplotlib import pyplot as plt
from propeller_diameter import diamgenerator
from productivity_mission_profile import generate_data

#----------------------------------------------------------------------------#
#             INITIAL REMARKS AND FEATURES TO BE IMPLEMENTED                 #
#----------------------------------------------------------------------------#

#Propeller coaxial effects have not been taken into account
#Rotorcraft theory needs to be further improved and understood for power calculations
#Remaining two missions need to be implemented
#Implement wind
#Look at regulations and how it may affect these calculations
#Look into battery relations for size in relation to if it is better to have smaller but more and larger but lesser batteries
#Implement diversion to other landing zones, which is an additional distance added to the cruise range


#----------------------------------------------------------------------------#
#                  INITIAL PARAMETERS AND CONSTANTS                          #
#----------------------------------------------------------------------------#

payload_masses = [57.0, 2.9, 18.2] #kg (Alex, rebar and sandbag individual masses. 93kg was the original selected payload weight)
g = 9.80665 #m/s^2
air_density = 1.225 #kg/m^3 (sea level)
air_temperature = 288.15 #K (sea level)
air_specific_heat_ratio = 1.4 #(standard conditions)
air_gas_constant = 287.0 #(standard conditions)
air_speed_of_sound = np.sqrt(air_specific_heat_ratio*air_gas_constant*air_temperature)
number_of_propellers = 6.0 #Design choice
number_of_blades = 2.0 #Design choice
propeller_hub_diameter = 0.3 #m (Design choice, Noam)
blade_root_chord = 0.1 #m (Design choice, Noam)
propeller_beam_width = 0.08 #m (Design choice, Noam)
propeller_beam_pin_height_position = 1.1 #m (Design choice, Noam)
propeller_beam_pin_width_position = 0.5 #m (Design choice, Noam)
propeller_height_difference = 0.2 #m (Design choice, Noam)
propeller_diameter_clearance = 0.2 #m (Design choice, Noam)

plot_sample_productivity_mission_profile = False
plot_sample_power_curve = True

#----------------------------------------------------------------------------#
#                      CLASS I WEIGHT ESTIMATION                             #
#----------------------------------------------------------------------------#

statistical_payload_masses = np.array([100, 120, 79.8, 99.8, 113.4, 158.8, 200, 120, 150, 200, 100, 130, 95.3, 70, 180, 100]) #kg
statistical_operational_empty_masses = np.array([260, 240, 327.1, 113.4, 195.9, 290.3, 360.2, 230, 250, 300, 300, 270, 114.8, 200, 270, 230]) #kg
statistical_maximum_take_off_masses = np.array([360, 360, 406.9, 213.2, 309.3, 449.1, 560.2, 350, 400, 500, 400, 400, 210, 270, 450, 330]) #kg
slope, intercept = np.polyfit(statistical_payload_masses, statistical_operational_empty_masses, 1)

#----------------------------------------------------------------------------#
#                      Payload Values Generation                             #
#----------------------------------------------------------------------------#

payload_mass = []
payload_mass_identifier = []
for i in range(13):
    for j in range(4):
        payload_mass.append(payload_masses[0] + (payload_masses[1] * i) + (payload_masses[2] * j))
        identifier = "Payload: Alex, " + str(j) + " sandbags and " + str(i) + " rebars."
payload_mass = np.array(payload_mass) #kg (Contains all possible payload combinations)

##############################################################################
#----------------------------------------------------------------------------#
#                        ITERATION LOOP START                                #
#----------------------------------------------------------------------------#
##############################################################################

#--------------------Class I Weight Estimation Result------------------------#

class_I_operational_empty_mass = payload_mass*slope + intercept #kg
class_I_maximum_take_off_mass = class_I_operational_empty_mass + payload_mass #kg
#print(class_I_maximum_take_off_mass)
maximum_thrust_to_weight = 2.0 #Design choice, for maneuvering conditions (could be 1.5)
maximum_maneuvering_total_thrust = class_I_maximum_take_off_mass * g * maximum_thrust_to_weight #N
maximum_maneuvering_thrust_per_propeller = maximum_maneuvering_total_thrust / number_of_propellers #N
loaded_cruise_total_thrust = class_I_maximum_take_off_mass * g #N (Vertical thrust component for L=W)
unloaded_cruise_total_thrust = class_I_operational_empty_mass * g #N (Vertical thrust component for L=W)

#-------------------------Rotor Sizing----------------------------#

#First rotor size estimate is purely based on geometrical limitations, we cannot go further than that
propeller_diameter_max = diamgenerator("hori_fold", number_of_blades, propeller_hub_diameter, blade_root_chord, propeller_beam_width, propeller_beam_pin_width_position, propeller_beam_pin_height_position, propeller_height_difference) - propeller_diameter_clearance #m (Maximum propeller diameter from geometrical constraints)
propeller_area_max = np.pi * (propeller_diameter_max / 2.0) * (propeller_diameter_max / 2.0) #m^2
total_propeller_area_max = propeller_area_max * number_of_propellers #m^2
disk_loading_max = (maximum_maneuvering_total_thrust / g) / total_propeller_area_max #kg/m^2

#Estimate rotor size according to statistics, assuming it gives smaller size than from the geometrical limitations
statistical_disk_loading = 98.0 #kg/m^2 (disk loading source)
statistical_total_propeller_area = (maximum_maneuvering_total_thrust / g) / statistical_disk_loading #m^2
statistical_single_propeller_area = statistical_total_propeller_area / number_of_propellers #m^2 
propeller_diameter_min = 2.0 * np.sqrt(statistical_single_propeller_area / np.pi) #m (minimum propeller diameter from statistics)
blade_tip_mach_number = 0.7 #Should stay below 0.8 for drag divergence and possibly below 0.6 for noise
blade_tip_velocity = blade_tip_mach_number * air_speed_of_sound #m/s

#The propeller diameter ranges are added in the coming loop

#-------------------Mission Velocity & Thrust Profiles-----------------------#

cruise_velocity = np.arange(10, 105, 5) #m/s 
cruise_height = 300 #m (Design choice, could be bound by regulations)
max_acceleration = g #m/s^2 (Design choice, eVTOLs don't generally accelerate more than this)
mission_distance = 3000.0 #m

productivty_mission_profiles = [] #Contains the profile of the productivity mission for the different payload options
#It contains as many profiles as payload options, for each you have the mission profile for loaded and unloaded conditions, the cruise
#velocity and the propeller data. The propeller data has the propeller diameter, area and angular velocity ranges.

#Loop through each payload option and all cruise velocity options 
for h in range(len(payload_mass)):

    payload_specific_productivity_mission_profile = []

    for k in range(len(cruise_velocity)):

        #Loaded mission profile
        time, altitude, velocity, thrust, power, horizontal_distance, vertical_distance, thrust_climb, thrust_cruise, thrust_descent, velocity_climb, velocity_cruise, velocity_descent = generate_data(class_I_maximum_take_off_mass[h], cruise_velocity[k], cruise_height, mission_distance, cruise_height, max_acceleration, max_acceleration, air_density)
        thrust_cruise_vertical = np.full(thrust_cruise.shape, loaded_cruise_total_thrust[h])
        thrust_cruise_horizontal = thrust_cruise
        thrust_cruise = np.sqrt(thrust_cruise_horizontal*thrust_cruise_horizontal + thrust_cruise_vertical*thrust_cruise_vertical)
        cruise_angle_of_attack = np.arctan2(thrust_cruise_vertical, thrust_cruise_horizontal)
        rotor_normal_cruise_velocity = velocity_cruise * np.sin(cruise_angle_of_attack)
        rotor_tangential_cruise_velocity = velocity_cruise * np.cos(cruise_angle_of_attack)
        
        loaded_mission_profile = [time, altitude, velocity, thrust, power, horizontal_distance, vertical_distance, thrust_climb, thrust_cruise_vertical, thrust_cruise_horizontal, thrust_cruise, cruise_angle_of_attack, thrust_descent, velocity_climb, velocity_cruise, rotor_normal_cruise_velocity, rotor_tangential_cruise_velocity, velocity_descent]
        
        #Unloaded mission profile
        time, altitude, velocity, thrust, power, horizontal_distance, vertical_distance, thrust_climb, thrust_cruise, thrust_descent, velocity_climb, velocity_cruise, velocity_descent = generate_data(class_I_operational_empty_mass[h], cruise_velocity[k], cruise_height, mission_distance, cruise_height, max_acceleration, max_acceleration, air_density)
        thrust_cruise_vertical = np.full(thrust_cruise.shape, unloaded_cruise_total_thrust[h])
        thrust_cruise_horizontal = thrust_cruise
        thrust_cruise = np.sqrt(thrust_cruise_horizontal*thrust_cruise_horizontal + thrust_cruise_vertical*thrust_cruise_vertical)
        cruise_angle_of_attack = np.arctan2(thrust_cruise_vertical, thrust_cruise_horizontal)
        rotor_normal_cruise_velocity = velocity_cruise * np.sin(cruise_angle_of_attack)
        rotor_tangential_cruise_velocity = velocity_cruise * np.cos(cruise_angle_of_attack)

        unloaded_mission_profile = [time, altitude, velocity, thrust, power, horizontal_distance, vertical_distance, thrust_climb, thrust_cruise_vertical, thrust_cruise_horizontal, thrust_cruise, cruise_angle_of_attack, thrust_descent, velocity_climb, velocity_cruise, rotor_normal_cruise_velocity, rotor_tangential_cruise_velocity, velocity_descent]

        #Propeller diameter range generation with previous sizing
        propeller_diameter = np.linspace(propeller_diameter_min[h], propeller_diameter_max, 50) #m
        propeller_area = np.pi * (propeller_diameter / 2.0) * (propeller_diameter / 2.0) #m^2  
        propeller_angular_velocity = blade_tip_velocity / (propeller_diameter / 2.0) #rad/s
        propeller_data = [propeller_diameter, propeller_area, propeller_angular_velocity]

        velocity_specific_productivty_mission_profile = [loaded_mission_profile, unloaded_mission_profile, cruise_velocity[k], propeller_data]
        payload_specific_productivity_mission_profile.append(velocity_specific_productivty_mission_profile)

    productivty_mission_profiles.append(payload_specific_productivity_mission_profile)


if plot_sample_productivity_mission_profile:

    fig, axes = plt.subplots(2, 3, figsize=(12, 6)) 

    fig.tight_layout(pad=3.0)

    # Subplot 1: Altitude vs Time
    axes[0, 0].plot(productivty_mission_profiles[0][0][0][0], productivty_mission_profiles[0][0][0][1], label="Loaded") 
    axes[0, 0].plot(productivty_mission_profiles[0][0][1][0], productivty_mission_profiles[0][0][1][1], label="Unloaded") 
    axes[0, 0].set_title('Altitude vs Time')  
    axes[0, 0].set_xlabel('Time (s)')  
    axes[0, 0].set_ylabel('Altitude (m)') 
    axes[0, 0].legend()
    axes[0, 0].grid(True)

    # Subplot 2: Velocity vs Time
    axes[0, 1].plot(productivty_mission_profiles[0][0][0][0], productivty_mission_profiles[0][0][0][2], label="Loaded") 
    axes[0, 1].plot(productivty_mission_profiles[0][0][1][0], productivty_mission_profiles[0][0][1][2], label="Unloaded") 
    axes[0, 1].set_title('Velocity vs Time')  
    axes[0, 1].set_xlabel('Time (s)')  
    axes[0, 1].set_ylabel('Velocity (m/s)') 
    axes[0, 1].legend()
    axes[0, 1].grid(True)

    # Subplot 3: Thrust vs Time
    axes[0, 2].plot(productivty_mission_profiles[0][0][0][0], productivty_mission_profiles[0][0][0][3], label="Loaded") 
    axes[0, 2].plot(productivty_mission_profiles[0][0][1][0], productivty_mission_profiles[0][0][1][3], label="Unloaded") 
    axes[0, 2].set_title('Thrust vs Time')  
    axes[0, 2].set_xlabel('Time (s)')  
    axes[0, 2].set_ylabel('Thrust (N)') 
    axes[0, 2].legend()
    axes[0, 2].grid(True)

    # Subplot 4: Altitude vs Distance
    axes[1, 0].plot(productivty_mission_profiles[0][0][0][5], productivty_mission_profiles[0][0][0][1], label="Loaded") 
    axes[1, 0].plot(productivty_mission_profiles[0][0][1][5], productivty_mission_profiles[0][0][1][1], label="Unloaded") 
    axes[1, 0].set_title('Altitude vs Distance')  
    axes[1, 0].set_xlabel('Distance (m)')  
    axes[1, 0].set_ylabel('Altitude (m)') 
    axes[1, 0].legend()
    axes[1, 0].grid(True)

    # Subplot 5: Velocity vs Distance
    axes[1, 1].plot(productivty_mission_profiles[0][0][0][5], productivty_mission_profiles[0][0][0][2], label="Loaded") 
    axes[1, 1].plot(productivty_mission_profiles[0][0][1][5], productivty_mission_profiles[0][0][1][2], label="Unloaded") 
    axes[1, 1].set_title('Velocity vs Distance')  
    axes[1, 1].set_xlabel('Distance (m)')  
    axes[1, 1].set_ylabel('Velocity (m/s)') 
    axes[1, 1].legend()
    axes[1, 1].grid(True)

    # Subplot 6: Thrust vs Distance
    axes[1, 2].plot(productivty_mission_profiles[0][0][0][5], productivty_mission_profiles[0][0][0][3], label="Loaded") 
    axes[1, 2].plot(productivty_mission_profiles[0][0][1][5], productivty_mission_profiles[0][0][1][3], label="Unloaded") 
    axes[1, 2].set_title('Thrust vs Distance')  
    axes[1, 2].set_xlabel('Distance (m)')  
    axes[1, 2].set_ylabel('Thrust (N)') 
    axes[1, 2].legend()
    axes[1, 2].grid(True)

    plt.show()

#----------------------------------------------------------------------------#
#                        CLASS II WEIGHT ESTIMATION                          #
#----------------------------------------------------------------------------#

#--------------Rotor Geometry Design and Power Estimation--------------------#

#Obtained from Tamas's code








#--------------------Analytical Rotor Power Estimation------------------------#

vertical_climb_speed = 2.5 #m/s (Literature but can be variable too)
vertical_descent_speed = -2.5 #m/s (Literature but can be variable too)
rotor_solidity = 0.065 #(running variable between 0.05-0.08, or obtained from Tamas)
blade_profile_drag_coefficient = 0.01 #Literature (basic helicopter aerodynamics by Seddon)
hover_correction_factor = 1.15 #Literature (basic helicopter aerodynamics by Seddon)
cruise_correction_factor = 1.2 #Literature (basic helicopter aerodynamics by Seddon)
cruise_blade_profile_drag_correction_factor = 4.65 ##Literature (basic helicopter aerodynamics by Seddon), can run between 4.5-4.7
airframe_equivalent_flat_plate_area = 0.808256 #m^2 (equivalent flat plate area source)

for n in range(len(productivty_mission_profiles)): #Loop over each payload combination
    for j in range(len(productivty_mission_profiles[n])): #Loop over each mission profile for 1 payload type
        mission_velocity_specific_power_values = []
        for l in range(len(productivty_mission_profiles[n][j][3][0])): #Loop over each rotor size for 1 mission profile and payload type
            propeller_specific_power_values = [productivty_mission_profiles[n][j][3][0][l]] #List contains the propeller size and corresponding loaded and unloaded power values
            for g in range(2): #Loop through 1 loaded and 1 unloaded flight
                power_values = []

                #Hover Power 
                hover_thrust_coefficient = np.full(productivty_mission_profiles[n][j][g][7].shape, loaded_cruise_total_thrust[n]) / (air_density * productivty_mission_profiles[n][j][3][0][l] * blade_tip_velocity * blade_tip_velocity)/number_of_propellers
                induced_hover_power_coefficient = (hover_correction_factor * (hover_thrust_coefficient**(1.5))) / (np.sqrt(2.0))
                profile_power_coefficient = (rotor_solidity * blade_profile_drag_coefficient) / 8.0
                hover_power = (induced_hover_power_coefficient + profile_power_coefficient) * air_density * productivty_mission_profiles[n][j][3][0][l] * number_of_propellers * blade_tip_velocity * blade_tip_velocity * blade_tip_velocity #W
                #print(hover_power[0])
                print()
                power_values.append(hover_power)

                #Climb and descent power
                thrust_equivalent_vertical_flight_induced_velocity = np.sqrt(productivty_mission_profiles[n][j][g][7] / (2.0 * air_density * productivty_mission_profiles[n][j][3][0][l] * number_of_propellers)) #m/s
                climb_power = hover_power * ((vertical_climb_speed / (2.0 * thrust_equivalent_vertical_flight_induced_velocity)) + np.sqrt((vertical_climb_speed / (2 * thrust_equivalent_vertical_flight_induced_velocity))**2 + 1)) #W
                descent_power = hover_power #W

                #if abs(vertical_descent_speed) >= (2.0 * thrust_equivalent_vertical_flight_induced_velocity):
                #    descent_power = hover_power * ((vertical_descent_speed / (2.0 * thrust_equivalent_vertical_flight_induced_velocity)) + np.sqrt((vertical_descent_speed / (2.0 * thrust_equivalent_vertical_flight_induced_velocity))**2 + 1.0)) #W
                #else:
                #    descent_power = hover_power #W

                power_values.append(climb_power)
                power_values.append(descent_power)

                #Cruise power
                cruise_thrust_coefficient = productivty_mission_profiles[n][j][g][10] / (air_density * productivty_mission_profiles[n][j][3][0][l] * blade_tip_velocity * blade_tip_velocity)/number_of_propellers
                cruise_induced_velocity = np.sqrt(-0.5 * productivty_mission_profiles[n][j][2] * productivty_mission_profiles[n][j][2] + 0.5 * np.sqrt(productivty_mission_profiles[n][j][2]**4 + 4.0 * (productivty_mission_profiles[n][j][g][10] / (2.0 * air_density * productivty_mission_profiles[n][j][3][0][l] * number_of_propellers))**2)) #m/s
                cruise_induced_velocity_inflow_factor = cruise_induced_velocity / blade_tip_velocity
                cruise_advance_ratio = productivty_mission_profiles[n][j][2] / blade_tip_velocity
                induced_cruise_power_coefficient = cruise_correction_factor * cruise_thrust_coefficient * cruise_induced_velocity_inflow_factor
                cruise_profile_power_coefficient = 0.125 * rotor_solidity * blade_profile_drag_coefficient * (1 + cruise_blade_profile_drag_correction_factor * cruise_advance_ratio * cruise_advance_ratio)
                cruise_parasitic_drag_power_coefficient = (0.5 * cruise_advance_ratio**3 * airframe_equivalent_flat_plate_area) / (productivty_mission_profiles[n][j][3][0][l] * number_of_propellers)
                cruise_power = (induced_cruise_power_coefficient + cruise_profile_power_coefficient + cruise_parasitic_drag_power_coefficient) * air_density * productivty_mission_profiles[n][j][3][0][l] * number_of_propellers * blade_tip_velocity**3 #W
                power_values.append(cruise_power)
                print(cruise_power[0])
                
                propeller_specific_power_values.append(power_values)
            mission_velocity_specific_power_values.append(propeller_specific_power_values)
        productivty_mission_profiles[n][j][3].append(mission_velocity_specific_power_values)

if plot_sample_power_curve:
    cruise_velocity_list = []
    cruise_power_list = []
    for y in range(len(productivty_mission_profiles[0])):
        cruise_velocity_list.append(productivty_mission_profiles[0][y][2])
        cruise_power_list.append(np.mean(productivty_mission_profiles[0][y][3][3][0][1][3]))

    plt.plot(cruise_velocity_list, cruise_power_list)
    plt.title("Sample Cruise Power Plot with Varying Cruise Velocity")
    plt.xlabel("Cruise Velocity (m/s)")
    plt.ylabel("Cruise Power (W)")
    plt.show()


#-------------------Productivity Mission Modelling-----------------------#

# Mission starts with unloaded flight, then loaded and repeating this pattern until 90 minutes runs out, trying to get as close as possible.
# The mission is defined as the sum of all runs and one run can be any defined number of flights
# Each run is flown with a new battery pack

cruise_distance = mission_distance  # m (Mission requirement, single flight distance)
cruise_height = cruise_height  # m (Design choice, might be modified due to regulations)
loiter_hover_time = 40.0  # s (both at start and end of the entire mission, could be modified due to regulations)
ground_turnover_time = 120.0 # s (assumed for payload operations, added every loaded flight)
battery_turnover_time = 60.0  # s (assumed for battery replacement, added every run (1 loaded and 1 unloaded flight))

#Function to add the corresponding time taken to complete either a loaded or unloaded flight
def add_flight_type_specific_time(flight_type_identifier, mission_time, loaded_climb_time, loaded_cruise_time, unloaded_climb_time, unloaded_cruise_time, ground_turnover_time):

    global loaded_flight_counter

    if flight_type_identifier % 2 == 0:  # Even identifier means loaded flight

        mission_time = mission_time + (loaded_cruise_time + (2.0 * loaded_climb_time) + ground_turnover_time) #Add time values for loaded flight
        loaded_flight_counter = loaded_flight_counter + 1

    else:  # Odd identifier means unloaded flight

        mission_time = mission_time + (unloaded_cruise_time + (2.0 * unloaded_climb_time)) #Add time values for unloaded flight

    return mission_time

for p in range(len(productivty_mission_profiles)): #Loop through all payload combinations
    for t in range(len(productivty_mission_profiles[p])): #Loop through all cruise velocities for 1 payload combination
        for k in range(len(productivty_mission_profiles[p][t][3][0])): #Loop through all propeller sizes for 1 payload and 1 cruise velocity combination

            mission_time = 0.0  #s
            mission_time = mission_time + loiter_hover_time  # Add loiter time since it is always there
            flight_type_identifier = 0  #Odd values identify unloaded and even identify loaded flights since we start unloaded
            flight_number_identifier_previous = 0 #Used to count how many flights have been flown to identify when the battery must be changed
            flight_number_identifier_new = 0 #Used to count how many flights have been flown to identify when the battery must be changed
            single_charge_flight_number = 2 #Determines per how many flights we want to change the battery, if it is 2 then every 2 flights it is changed and it if is 3 then every 3
            loaded_flight_counter = 0 #Stores the amount of loaded flights
            battery_change_times = 0 #Stores the amount of battery changes

            loaded_climb_time = np.sum(productivty_mission_profiles[p][t][0][0][:len(productivty_mission_profiles[p][t][0][7])]) #s
            unloaded_climb_time = np.sum(productivty_mission_profiles[p][t][1][0][:len(productivty_mission_profiles[p][t][1][7])]) #s
            loaded_cruise_time = np.sum(productivty_mission_profiles[p][t][0][0][len(productivty_mission_profiles[p][t][0][7]):len(productivty_mission_profiles[p][t][0][7])+len(productivty_mission_profiles[p][t][0][10])]) #s
            unloaded_cruise_time = np.sum(productivty_mission_profiles[p][t][0][0][len(productivty_mission_profiles[p][t][1][7]):len(productivty_mission_profiles[p][t][1][7])+len(productivty_mission_profiles[p][t][1][10])]) #s

            #Mission loop
            while mission_time < (90 * 60):  #Mission requirement

                flight_type_identifier = flight_type_identifier + 1
                flight_number_identifier_new = flight_number_identifier_new + 1

                if (flight_number_identifier_new - flight_number_identifier_previous) == single_charge_flight_number:  # Means that enough flights have been flown to change the battery in this loop

                    battery_change_times = battery_change_times + 1
                    flight_number_identifier_previous = flight_number_identifier_previous + single_charge_flight_number  # Updates the counter so that after the next batch of flights the difference is correct to enable the battery change
                    temporary_mission_time = add_flight_type_specific_time(flight_type_identifier, mission_time, loaded_climb_time, loaded_cruise_time, unloaded_climb_time, unloaded_cruise_time, ground_turnover_time) + battery_turnover_time  # Before 90 min check

                    if temporary_mission_time < (90 * 60):  # Maximum mission time requirement is not exceeded with latest run hence mission time is updated

                        mission_time = temporary_mission_time

                    else:  # Maximum mission time requirement is exceeded with latest run but if it is the last flight then we could make it still if we do remove the battery turnover time since it is the last flight

                        if (temporary_mission_time - battery_turnover_time) < (90 * 60):  # Check if we exceed mission requirement by removing last battery time change since it is useless, if so then break the loop since another flight is not possible anyway
                            battery_change_times = battery_change_times - 1
                            mission_time = temporary_mission_time - battery_turnover_time
                            last_flight_battery_state = "on the replacement flight."
                            break

                        else:  # We exceed even without replacing the last battery, hence the last run cannot fit, do not update mission time
                            last_flight_battery_state = "on its last flight before replacement."
                            break

                else:  # Not needed to change the battery yet in this loop

                    temporary_mission_time = add_flight_type_specific_time(flight_type_identifier, mission_time, loaded_climb_time, loaded_cruise_time, unloaded_climb_time, unloaded_cruise_time, ground_turnover_time)  # Before 90 min check

                    if temporary_mission_time < (90 * 60):  # Maximum mission time requirement is not exceeded with latest run hence mission time is updated

                        mission_time = temporary_mission_time

                    else:  # Maximum mission time requirement is exceeded with latest run hence mission time is not updated and loop broken
                        last_flight_battery_state = "on an intermediate flight."
                        break

            if (flight_type_identifier - 1) % 2 == 0:
                last_flight_type = "loaded"

            else:
                last_flight_type = "unloaded"

            total_flight_number = flight_type_identifier - 1
            loaded_flight_number = loaded_flight_counter
            unloaded_flight_number = total_flight_number - loaded_flight_number
            mission_climb_time = loaded_flight_number * loaded_climb_time + unloaded_flight_number * unloaded_climb_time #s
            mission_cruise_time = loaded_flight_number * loaded_cruise_time + unloaded_flight_number * unloaded_cruise_time #s
            battery_pack_number = battery_change_times

            #print("#----Productivity Mission Summary----#\n")
            #print("The total mission time is " + str(round(mission_time / 60.0, 2)) + " min, the last flight was " + last_flight_type + " and the battery was " + last_flight_battery_state)
            #print("A total of " + str(mission_climb_time) + "s is spent climbing or " + str(round((mission_climb_time / mission_time) * 100, 2)) + "% of the total mission")
            #print("A total of " + str(mission_descent_time) + "s is spent descending or " + str(round((mission_descent_time / mission_time) * 100, 2)) + "% of the total mission")
            #print("A total of " + str(mission_cruise_time) + "s is spent cruising or " + str(round((mission_cruise_time / mission_time) * 100, 2)) + "% of the total mission")
            #print("In total " + str(flight_type_identifier - 1) + " flights are flown, with " + str(loaded_flight_counter) + " of those being loaded. A total of " + str(loaded_flight_counter * payload_mass) + " kg of payload is transported overall.")
            #print("Finally, " + str(battery_change_times) + " battery packs are needed.")

            #These are the energy values actually contained in the battery, since it is changed per as many runs as previously defined
            run_hover_energy = hover_power_values * loiter_hover_time #J
            run_climb_energy = vertical_climb_power_values * (climb_time * single_charge_flight_number) #J
            run_descent_energy = vertical_descent_climb_power_values * (climb_time * single_charge_flight_number) #J
            run_cruise_energy = cruise_power_values * (cruise_time * single_charge_flight_number) #J
            total_run_energy = run_cruise_energy + run_descent_energy + run_climb_energy + run_hover_energy #J (This is the energy stored in one battery)

#-------------------Component Weight Estimation-----------------------#

#Battery sizing
minimum_battery_state_of_charge = 0.2 #20% of the battery charge is preserved to improve the longevity of the battery and can be used as an emergence energy reserve as well
battery_efficiency = 0.92
battery_energy_density = 270.0 * 3600.0 #J/kg (could range between 170-350 Wh/kg, 3600 is conversion factor from Wh to J)
battery_mass = 1.05 * (total_run_energy * (1.0 + minimum_battery_state_of_charge)) / (battery_energy_density * battery_efficiency) #kg (Individual battery pack mass, factor of 1.05 for aviones and other sytems consumption)
total_battery_packs_mass = battery_mass * battery_change_times #kg (Total mass of all battery packs needed)

#Original Excel Weight Formulas
fuselage_length = 2.0 #m (Assumed)
fuselage_height = 0.7 #m (Assumed)
fuselage_width = 0.7 #m (Assumed)
fuselage_perimiter = (fuselage_width * 2.0) + (fuselage_length * 2.0) #m
number_of_passengers = 1.0 #Alex

total_fuselage_mass_1 = 14.86 * (class_one_maximum_take_off_mass**(0.144)) * ((fuselage_length**(0.778))/(fuselage_perimiter)) * (fuselage_length**(0.383)) * (number_of_passengers**(0.455)) #kg 
propeller_blades_mass_1 = (0.144 * ((propeller_diameter * vertical_climb_power_values[0] * np.sqrt(number_of_blades))/(number_of_propellers))**(0.782)) * number_of_propellers #kg (Preferred from manufacturer)
propeller_motor_mass_1 = ((0.165 * vertical_climb_power_values[0])  / number_of_propellers) * number_of_propellers #kg (Preferred from manufacturer)

#Rohit's Class II Weight Estimation (1-6)

#propeller_mass_calibration_factor = 1 #Used to match the weight of some desired industry propeller series, needs to be determined
#fuselage_skin_material_density = 1500 #kg/m^3 (Given by Viktor, average between plastics and composites)
#bulkhead_material_density = 1200 #Determined from structures, bulkhead separates passenger from battery and other units (Given by Viktor, plastic)
#landing_impact_factor = 1.5 #Landing load factor? Definition unclear
#landing_gear_retaining_bolt_ultimate_strength = 1 #Determined from structures
#landing_gear_pad_ultimate_strength = 1 #Determined from structures
#landing_gear_pad_material_density = 1 #Determined from structures

#fuselage_wetted_area = 4.0 * np.pi * (0.3333 * ((fuselage_length * fuselage_width)**(8.0/5.0) + (fuselage_length * (fuselage_height/4.0))**(8.0/5.0) + (fuselage_width + (fuselage_height / 4.0))**(8.0/5.0)))**(5.0/8.0) #m^2
#bulkhead_wetted_area = (3.0 * np.pi /4.0) * (fuselage_height * fuselage_width) #m^2
#landing_force = class_one_maximum_take_off_mass * landing_impact_factor * np.sin((2.0 * np.pi /360.0) * 40.0) #N (40º landing angle with respect to surfaced)
#landing_gear_retaining_bolt_diameter = 2.0 * np.sqrt(landing_force / (np.pi * landing_gear_retaining_bolt_ultimate_strength)) #m
#landing_gear_pad_thickness = landing_force / (landing_gear_retaining_bolt_diameter * landing_gear_pad_ultimate_strength) #m
#landing_gear_pad_volume = (np.pi * (20.0 * landing_gear_pad_thickness)**(2)) * (landing_gear_pad_thickness/3.0) #m^3

#propeller_blades_mass_2 = 0.144 * propeller_mass_calibration_factor * (((propeller_diameter * 3.281 * max(cruise_power_values[0], hover_power_values[0], vertical_climb_power_values[0]) * 1.360 * np.sqrt(number_of_blades))**(0.782)) / (2.74)) * number_of_propellers #kg
#fuselage_skin_mass = fuselage_wetted_area * fuselage_skin_material_density #kg
#bulkhead_mass = bulkhead_material_density * bulkhead_wetted_area #kg
#landing_gear_mass_2 = 4 * landing_gear_pad_volume * landing_gear_pad_material_density #kg
#total_fuselage_mass_2 = fuselage_skin_mass + bulkhead_mass #kg (Rohits class II formula sheet source, soem definitions aren't the clearest and it depends a lot on structural considerations)

#Rohit's Class II Weight Estimation (7)

fuselage_wetted_area_3 = (2.0 * fuselage_length * fuselage_height) + (fuselage_height * fuselage_width * 2.0) + (fuselage_width * fuselage_length * 2.0) #m^2 (Assuming a prism and that all the area is wetted)
number_of_landing_gears = 4.0

pound_to_kilo_conversion_factor = 0.45359237 #Unit conversion
kilo_to_pound_conversion_factor = 1.0 / pound_to_kilo_conversion_factor #Unit conversion
meters_to_feet_conversion_factor = 3.28084 #Unit conversion
square_meters_to_square_feet_conversion_factor = meters_to_feet_conversion_factor * meters_to_feet_conversion_factor #Unit conversion

propeller_blades_mass_3 = (2.20462/1000.0) * ((7200.0/500.0) * (thrust_values[0] * 4.4482 - 300.0) + 800.0) * number_of_propellers * pound_to_kilo_conversion_factor #kg (Also includes hub mass)
#additional_hub_mass = 0.0037 * (number_of_blades)**(0.28) * (propeller_diameter/2.0)**(1.5) * (blade_tip_velocity)**(0.43) * (0.01742 * (number_of_blades)**(0.66) * propeller_chord * (propeller_diameter / 2.0)**(1.3) * (blade_tip_velocity)**(0.67) + g * (np.pi * (propeller_diameter/2.0)**(0.5))**(0.5))**(0.55) kg (Only used in the second iteration and correct units to imperial)
propeller_motor_mass_3 = 2.20462 * ((58.0 / 990.0) * ((max(cruise_power_values[0], hover_power_values[0], vertical_climb_power_values[0]) / 1000.0) * propeller_angular_velocity * 1.3558) + 2) * number_of_propellers * pound_to_kilo_conversion_factor #kg
motor_controller_mass = 2.20462 * ((49.9/398.0) * ((max(cruise_power_values[0], hover_power_values[0], vertical_climb_power_values[0])/1000.0) - 2) + 0.1) * number_of_propellers * pound_to_kilo_conversion_factor #kg
total_fuselage_mass_3 = 6.9 * ((class_one_maximum_take_off_mass * kilo_to_pound_conversion_factor)/1000.0)**(0.49) * (fuselage_length * meters_to_feet_conversion_factor)**(0.61) * (fuselage_wetted_area_3 * square_meters_to_square_feet_conversion_factor)**(0.25) * pound_to_kilo_conversion_factor #kg
landing_gear_mass_3 = 40 * (class_one_maximum_take_off_mass/1000.0)**(0.47) * number_of_landing_gears**(0.54) * pound_to_kilo_conversion_factor #kg
flight_control_system_mass = 11.5 * ((class_one_maximum_take_off_mass * kilo_to_pound_conversion_factor)/1000.0)**(0.4) * pound_to_kilo_conversion_factor #kg
avionics_mass = 0.0268**(class_one_maximum_take_off_mass * kilo_to_pound_conversion_factor) * pound_to_kilo_conversion_factor #kg
furnishings_mass = 13 * ((class_one_maximum_take_off_mass * kilo_to_pound_conversion_factor) / 1000)**(1.3) * pound_to_kilo_conversion_factor #kg

#Rohit's Class II Weight Estimation (9)

maximum_battery_power = max(cruise_power_values[0], hover_power_values[0], vertical_climb_power_values[0]) / 1000.0 #kW (Assuming it is the same as the propeller, should be modified)
maximum_motor_power = max(cruise_power_values[0], hover_power_values[0], vertical_climb_power_values[0]) / 1000.0 #kW
battery_management_system_power_density = 20.0 #kW/kg (Needs to be found)
electric_motor_power_density = 5.0 #kW/kg (Needs to be found)

battery_management_system_mass = maximum_battery_power / battery_management_system_power_density #kg
electric_motor_mass_4 = maximum_motor_power / electric_motor_power_density #kg
battery_thermal_management_system_mass = 0.521 * ((1.0 - battery_efficiency)/(battery_efficiency)) * maximum_battery_power + 1.863 #kg

#Component sizing result

class_II_weight_estimation_results = np.array([battery_management_system_mass, battery_thermal_management_system_mass, total_fuselage_mass_3, propeller_motor_mass_3, propeller_blades_mass_3, motor_controller_mass, landing_gear_mass_3, flight_control_system_mass, avionics_mass, furnishings_mass, payload_mass]) #kg
class_II_maximum_take_off_mass = np.sum(class_II_weight_estimation_results) #kg

#Plot the weight distribution
weight_name_labels = ["Battery Management", "Thermal Management", "Fuselage", "Motors", "Propeller Blades & Hubs", "Motor Controller", "Landing Gear", "Flight Control", "Avionics", "Furnishings", "Payload"]
class_II_relative_weight_estimation_results = class_II_weight_estimation_results / class_II_maximum_take_off_mass #kg

print(class_II_maximum_take_off_mass)
print(class_II_weight_estimation_results)

plt.pie(class_II_relative_weight_estimation_results, labels=weight_name_labels, shadow=True)
plt.axis('equal')  
plt.title('Class II Weight Estimation Distribution')
plt.show()

#-------------------Adversity Mission Modelling-----------------------#
#-------------------Maneuvering Mission Modelling---------------------#