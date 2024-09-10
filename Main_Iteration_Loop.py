import numpy as np

#Propeller coaxial effects have not been taken into account
#Rotorcraft theory needs to be further improved and understood for power calculations
#Remaining two missions need to be implemented
#Implement wind
#Look at regulations and how it may affect these calculations
#Look into battery relations for size in relation to if it is better to have smaller but more and larger but lesser batteries
#Implement diversion to other landing zones, which is an additional distance added to the cruise range


#Initial parameters and constants

payload_mass = 93 #kg
g = 9.80665 #m/s^2
air_density = 1.225 #kg/m^3 (sea level)
air_temperature = 288.15 #K (sea level)
air_specific_heat_ratio = 1.4 #(standard conditions)
air_gas_constant = 287 #(standard conditions)
air_speed_of_sound = np.sqrt(air_specific_heat_ratio*air_gas_constant*air_temperature)

#----Class I Weight Estimation----#

payload_masses = np.array([100, 120, 79.8, 99.8, 113.4, 158.8, 200, 120, 150, 200, 100, 130, 95.3, 70, 180, 100]) #kg
operational_empty_masses = np.array([260, 240, 327.1, 113.4, 195.9, 290.3, 360.2, 230, 250, 300, 300, 270, 114.8, 200, 270, 230]) #kg
maximum_take_off_masses = np.array([360, 360, 406.9, 213.2, 309.3, 449.1, 560.2, 350, 400, 500, 400, 400, 210, 270, 450, 330]) #kg

slope, intercept = np.polyfit(payload_masses, operational_empty_masses, 1)
class_one_operational_empty_mass = payload_mass*slope + intercept #kg
class_one_maximum_take_off_mass = class_one_operational_empty_mass + payload_mass #kg

#-----Class II Weight Estimation----#

#General Propulsion & Performance

thrust_to_weight = 2 #Design choice, for maneuvering conditions (could be 1.5)
number_of_propellers = 6 #Design choice (variable)
disk_loading = 98 #kg/m^2 (disk loading source)
number_of_blades = 2 #Design choice (variable)
propeller_angular_velocity = 418.8792 #rad/s (variable)
rotor_solidity = 0.065 #(running variable between 0.05-0.08)
blade_profile_drag_coefficient = 0.01 #Literature (basic helicopter aerodynamics by Seddon)
hover_correction_factor = 1.15 #Literature (basic helicopter aerodynamics by Seddon)
cruise_correction_factor = 1.2 #Literature (basic helicopter aerodynamics by Seddon)
cruise_blade_profile_drag_correction_factor = 4.65 ##Literature (basic helicopter aerodynamics by Seddon), can run between 4.5-4.7
airframe_equivalent_flat_plate_area = 0.808256 #m^2 (equivalent flat plate area source)
vertical_climb_speed = 2.5 #m/s (Literature but can be variable too)
vertical_descent_speed = -2.5 #m/s (Literature but can be variable too)
cruise_speed = 20 #m/s (Design choice but can be variable)

maximum_maneuvering_total_thrust = class_one_maximum_take_off_mass * g * thrust_to_weight #N
maximum_maneuvering_thrust_per_propeller = maximum_maneuvering_total_thrust / number_of_propellers #N
total_propeller_area = (maximum_maneuvering_total_thrust / g) / disk_loading #m^2
single_propeller_area = total_propeller_area / number_of_propellers #m^2
propeller_diameter = 2 * np.sqrt(single_propeller_area / np.pi) #m
blade_tip_velocity = (propeller_diameter / 2) * propeller_angular_velocity #m/s
blade_tip_mach_numer = blade_tip_velocity / air_speed_of_sound
loaded_cruise_total_thrust = class_one_maximum_take_off_mass * g #N
unloaded_cruise_total_thrust = class_one_operational_empty_mass * g #N

#Arrays for power calculations, ordered as maneuvering (T/W=2 and maximum weight), loaded (T/W=1 and maximum weight) and unloaded (T/W=1 and no payload)
weight_values = np.array([class_one_maximum_take_off_mass, class_one_maximum_take_off_mass, class_one_operational_empty_mass]) #kg
thrust_values = np.array([maximum_maneuvering_total_thrust, loaded_cruise_total_thrust, unloaded_cruise_total_thrust]) #N

thrust_coefficient = thrust_values / (air_density * total_propeller_area * (blade_tip_velocity**2))
induced_hover_power_coefficient = (hover_correction_factor * (thrust_coefficient**(3/2))) / (np.sqrt(2))
profile_power_coefficient = (rotor_solidity * blade_profile_drag_coefficient) / 8
hover_power_values = (induced_hover_power_coefficient + profile_power_coefficient) * air_density * total_propeller_area * (blade_tip_velocity**3) #W

thrust_equivalent_vertical_flight_induced_velocity = np.sqrt((thrust_values)/(2 * air_density * total_propeller_area)) #m/s
vertical_climb_power_values = hover_power_values * ((vertical_climb_speed/(2 * thrust_equivalent_vertical_flight_induced_velocity)) + np.sqrt((vertical_climb_speed/(2 * thrust_equivalent_vertical_flight_induced_velocity))**2 + 1))
vertical_descent_climb_power_values = []
for i in range(len(thrust_equivalent_vertical_flight_induced_velocity)):
    if abs(vertical_descent_speed) >= (2 * thrust_equivalent_vertical_flight_induced_velocity[i]):
        descent_power = hover_power_values[i] * ((vertical_descent_speed/(2 * thrust_equivalent_vertical_flight_induced_velocity[i])) + np.sqrt((vertical_descent_speed/(2 * thrust_equivalent_vertical_flight_induced_velocity[i]))**2 + 1))
        vertical_descent_climb_power_values.append(descent_power)
    else:
        vertical_descent_climb_power_values.append(hover_power_values[i])
vertical_descent_climb_power_values = np.array(vertical_descent_climb_power_values) #W

cruise_induced_velocity = np.sqrt(-0.5 * cruise_speed**2 + 0.5 * np.sqrt(cruise_speed**4 + 4 * (thrust_values/(2 * air_density * total_propeller_area))**2)) #m/s
cruise_induced_velocity_inflow_factor = cruise_induced_velocity / blade_tip_velocity
cruise_advance_ratio = cruise_speed / blade_tip_velocity
induced_cruise_power_coefficient = cruise_correction_factor * thrust_coefficient * cruise_induced_velocity_inflow_factor
cruise_profile_power_coefficient = 1/8 * rotor_solidity * blade_profile_drag_coefficient * (1 + cruise_blade_profile_drag_correction_factor * cruise_advance_ratio**2)
cruise_parasitic_drag_power_coefficient = (0.5 * cruise_advance_ratio**3 * airframe_equivalent_flat_plate_area) / total_propeller_area
cruise_power_values = (induced_cruise_power_coefficient + cruise_profile_power_coefficient + cruise_parasitic_drag_power_coefficient) * air_density * total_propeller_area * blade_tip_velocity**3 #W


#Productivty Mission & Energy

# Mission starts with unloaded flight, then loaded and repeating this pattern until 90 minutes runs out, trying to get as close as possible.
# The mission is defined as the sum of all runs and one run can be any defined number of flights
# Each run is flown with a new battery pack
cruise_distance = 3000  # m (Mission requirement, single flight distance)
cruise_height = 100  # m (Design choice, might be modified due to regulations)
loiter_hover_time = 40  # s (both at start and end of the entire mission, could be modified due to regulations)
climb_time = cruise_height / vertical_climb_speed  # s (Time to climb to the required altitude, descent time is the same)
cruise_time = cruise_distance / cruise_speed  # s (Time to cover the single flight distance)
ground_turnover_time = 120  # s (assumed for payload operations, added every loaded flight)
battery_turnover_time = 60  # s (assumed for battery replacement, added every run (1 loaded and 1 unloaded flight))
mission_time = 0  #s
mission_time = mission_time + loiter_hover_time  # Add loiter time since it is always there
mission_climb_time = 0 #s (Accumulates all the time used for climbing in the mission)
mission_descent_time = 0 #s (Accumulates all the time used for descending in the mission)
mission_cruise_time = 0 #s (Accumulates all the time used for cruising in the mission)
flight_type_identifier = 0  #Odd values identify unloaded and even identify loaded flights since we start unloaded
flight_number_identifier_previous = 0 #Used to count how many flights have been flown to identify when the battery must be changed
flight_number_identifier_new = 0 #Used to count how many flights have been flown to identify when the battery must be changed
single_charge_flight_number = 2 #Determines per how many flights we want to change the battery, if it is 2 then every 2 flights it is changed and it if is 3 then every 3
loaded_flight_counter = 0


#Function to add the corresponding time taken to complete either a loaded or unloaded flight
def add_flight_type_specific_time(flight_type_identifier, mission_time, climb_time, cruise_time, ground_turnover_time):

    global loaded_flight_counter

    if flight_type_identifier % 2 == 0:  # Even identifier means loaded flight

        mission_time = mission_time + (cruise_time + (2 * climb_time) + ground_turnover_time) #Add time values for loaded flight
        loaded_flight_counter = loaded_flight_counter + 1

    else:  # Odd identifier means unloaded flight

        mission_time = mission_time + (cruise_time + (2 * climb_time)) #Add time values for unloaded flight

    return mission_time

#Mission loop
while mission_time < (90 * 60):  #Mission requirement

    flight_type_identifier = flight_type_identifier + 1
    flight_number_identifier_new = flight_number_identifier_new + 1

    if (flight_number_identifier_new - flight_number_identifier_previous) == single_charge_flight_number:  # Means that enough flights have been flown to change the battery in this loop

        flight_number_identifier_previous = flight_number_identifier_previous + single_charge_flight_number  # Updates the counter so that after the next batch of flights the difference is correct to enable the battery change
        temporary_mission_time = add_flight_type_specific_time(flight_type_identifier, mission_time, climb_time, cruise_time, ground_turnover_time) + battery_turnover_time  # Before 90 min check

        if temporary_mission_time < (90 * 60):  # Maximum mission time requirement is not exceeded with latest run hence mission time is updated

            mission_time = temporary_mission_time

        else:  # Maximum mission time requirement is exceeded with latest run but if it is the last flight then we could make it still if we do remove the battery turnover time since it is the last flight

            if (temporary_mission_time - battery_turnover_time) < (90 * 60):  # Check if we exceed mission requirement by removing last battery time change since it is useless, if so then break the loop since another flight is not possible anyway

                mission_time = temporary_mission_time - battery_turnover_time
                last_flight_battery_state = "on the replacement flight."
                break

            else:  # We exceed even without replacing the last battery, hence the last run cannot fit, do not update mission time
                last_flight_battery_state = "on its last flight before replacement."
                break

    else:  # Not needed to change the battery yet in this loop

        temporary_mission_time = add_flight_type_specific_time(flight_type_identifier, mission_time, climb_time, cruise_time, ground_turnover_time)  # Before 90 min check

        if temporary_mission_time < (90 * 60):  # Maximum mission time requirement is not exceeded with latest run hence mission time is updated

            mission_time = temporary_mission_time

        else:  # Maximum mission time requirement is exceeded with latest run hence mission time is not updated and loop broken
            last_flight_battery_state = "on an intermediate flight."
            break

    mission_climb_time = mission_climb_time + climb_time
    mission_descent_time = mission_descent_time + climb_time
    mission_cruise_time = mission_cruise_time + cruise_time

if (flight_type_identifier - 1) % 2 == 0:
    last_flight_type = "loaded"

else:
    last_flight_type = "unloaded"

print("The total mission time is " + str(round(mission_time / 60, 2)) + " min, the last flight was " + last_flight_type + " and the battery was " + last_flight_battery_state)
print("A total of " + str(mission_climb_time) + "s is spent climbing or " + str(round((mission_climb_time / mission_time) * 100, 2)) + "% of the total mission")
print("A total of " + str(mission_descent_time) + "s is spent descending or " + str(round((mission_descent_time / mission_time) * 100, 2)) + "% of the total mission")
print("A total of " + str(mission_cruise_time) + "s is spent cruising or " + str(round((mission_cruise_time / mission_time) * 100, 2)) + "% of the total mission")
print("Finally, " + str(flight_type_identifier - 1) + " flights are flown with " + str(loaded_flight_counter) + " of those being loaded. A total of " + str(loaded_flight_counter * payload_mass) + " kg of payload is transported overall.")


total_mission_hover_energy = hover_power_values * loiter_hover_time #J
total_mission_climb_energy = vertical_climb_power_values * mission_climb_time #J
total_mission_descent_energy = vertical_descent_speed * mission_descent_time #J
total_mission_cruise_energy = cruise_power_values * mission_cruise_time #J

#These are the energy values actually contained in the battery since it is changed per as many runs as previously defined
run_hover_energy = hover_power_values * loiter_hover_time #J
run_climb_energy = vertical_climb_power_values * (climb_time * single_charge_flight_number) #J
run_descent_energy = vertical_descent_climb_power_values * (climb_time * single_charge_flight_number) #J
run_cruise_energy = cruise_power_values * (cruise_time * single_charge_flight_number) #J
total_run_energy = run_cruise_energy + run_descent_energy + run_climb_energy + run_hover_energy #J (This is the energy stored in one battery)

minimum_battery_state_of_charge = 0.2 #20% of the battery charge is preserved to improve the longevity of the battery and can be used as an emergence energy reserve as well
battery_efficiency = 0.92
battery_energy_density = 270 * 3600 #J/kg (could range between 170-350 Wh/kg, 3600 is conversion factor from Wh to J)
battery_mass = (total_run_energy * (1 + minimum_battery_state_of_charge)) / (battery_energy_density * battery_efficiency)

#Remaining Class II weight formulas

fuselage_length = 2 #m
fuselage_height = 0.7 #m
fuselage_width = 0.7 #m
fuselage_perimiter = (fuselage_width * 2) + (fuselage_length * 2) #m
number_of_passengers = 1 #Alex
fuselage_mass = 14.86 * (class_one_maximum_take_off_mass**(0.144)) * ((fuselage_length**(0.778))/(fuselage_perimiter)) * (fuselage_length**(0.383)) * (number_of_passengers**(0.455)) #kg
#landing_gear_mass =
propeller_blades_mass = (0.144 * ((propeller_diameter * vertical_climb_power_values[0] * np.sqrt(number_of_blades))/(number_of_propellers))**(0.782)) * number_of_propellers #kg (Preferred from manufacturer)
propeller_motor_mass = ((0.165 * vertical_climb_power_values[0])  / number_of_propellers) * number_of_propellers #kg (Preferred from manufacturer)
#avionics_mass =
#cables_mass =
#rotor_beam_mass =
#rotor_guard_mass =