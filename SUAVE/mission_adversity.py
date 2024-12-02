# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import SUAVE

assert SUAVE.__version__ == '2.5.2', 'This simulation will only work with the SUAVE 2.5.2 release'
from SUAVE.Core import Units

import pylab as plt

from copy import deepcopy
import os

from SUAVE.Plots.Performance.Mission_Plots               import *
from SUAVE.Methods.Propulsion                            import propeller_design
from SUAVE.Methods.Power.Battery.Sizing                  import initialize_from_mass
from SUAVE.Methods.Propulsion.electric_motor_sizing      import size_from_kv
from SUAVE.Attributes.Gases                              import Air
from SUAVE.Plots.Performance.Mission_Plots               import *
from SUAVE.Input_Output.OpenVSP                          import write
from SUAVE.Methods.Geometry.Two_Dimensional.Planform     import segment_properties, wing_segmented_planform, wing_planform
from SUAVE.Methods.Propulsion                            import propeller_design
from SUAVE.Methods.Propulsion.electric_motor_sizing      import size_optimal_motor
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_polars import compute_airfoil_polars
from SUAVE.Methods.Performance                           import payload_range


# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():
    # build the vehicle, configs, and analyses
    configs, analyses = full_setup()

    configs.finalize()
    analyses.finalize()

    # mission analysis
    mission = analyses.missions.base
    results = mission.evaluate()

    # plot results
    plot_mission(results)

    return


# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup():
    # Collect baseline vehicle data and changes when using different configuration settings
    vehicle = vehicle_setup()
    configs = configs_setup(vehicle)

    # vehicle analyses
    configs_analyses = analyses_setup(configs)

    # mission analyses
    mission = mission_setup(configs_analyses, vehicle)
    missions_analyses = missions_setup(mission)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs = configs_analyses
    analyses.missions = missions_analyses

    return configs, analyses


# ----------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------

def vehicle_setup():
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------

    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'eVTOL'

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------
    # mass properties

    vehicle.mass_properties.takeoff = 351.25821329134055 * Units.kg
    vehicle.mass_properties.operating_empty = 204.85821329134058 * Units.kg
    vehicle.mass_properties.max_takeoff = 351.25821329134055 * Units.kg
    vehicle.mass_properties.max_payload = 60 * Units.kg

    # basic parameters
    vehicle.reference_area = 13.36

    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'

    wing.areas.reference = vehicle.reference_area
    wing.spans.projected = 0.0001 * Units.m
    wing.aspect_ratio = (wing.spans.projected ** 2) / wing.areas.reference
    wing.sweeps.quarter_chord = 5.0 * Units.deg
    wing.thickness_to_chord = 0.0001
    wing.taper = 1.0
    wing.dynamic_pressure_ratio = 1.0
    wing.chords.mean_aerodynamic = 0.162 * Units.m
    wing.twists.root = 0.0 * Units.degrees
    wing.twists.tip = 0.0 * Units.degrees
    wing.high_lift = False
    wing.vertical = False
    wing.symmetric = True

    # add to vehicle
    vehicle.append_component(wing)



    # ------------------------------------------------------------------
    # Propulsor
    # ------------------------------------------------------------------

    # Build Network
    net = SUAVE.Components.Energy.Networks.Battery_Propeller()
    net.number_of_lift_rotor_engines = 6.
    net.voltage = 400.
    net.identical_propellers = False

    # Component 1 the ESC
    esc = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency = 0.95  # Gundlach for brushless motors
    net.esc = esc

    # Component 2 the Battery
    bat = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion()
    bat.mass_properties.mass = 96.95207658382327 * Units.kg
    bat.specific_energy = 275. * Units.Wh / Units.kg
    bat.resistance = 0.05   # Ohms
    bat.max_voltage = 400.  # V
    initialize_from_mass(bat)
    net.battery = bat

    # Component 3 the Propeller
    # Design the Propeller
    lift_rotor                            = SUAVE.Components.Energy.Converters.Lift_Rotor()
    lift_rotor.tip_radius                 = 1.00081061852 * Units.m
    lift_rotor.hub_radius                 = 0.12 * Units.m
    lift_rotor.number_of_blades           = 2
    lift_rotor.design_tip_mach            = 0.59521184238
    lift_rotor.freestream_velocity        = 500. * Units['ft/min']
    lift_rotor.angular_velocity           = lift_rotor.design_tip_mach* Air().compute_speed_of_sound()/lift_rotor.tip_radius
    lift_rotor.design_Cl                  = 1.2
    lift_rotor.design_altitude            = 0. * Units.m
    lift_rotor.design_thrust              = 6897.049482939595*Units.N/6
    lift_rotor.variable_pitch             = False
    lift_rotor.airfoil_geometry           = ['./Airfoils/e_395.txt']
    lift_rotor.airfoil_polars             = [['./Airfoils/Polars/xf-e395-il-50000.txt' ,
                                         './Airfoils/Polars/xf-e395-il-100000.txt' ,
                                         './Airfoils/Polars/xf-e395-il-200000.txt' ,
                                         './Airfoils/Polars/xf-e395-il-500000.txt' ,
                                         './Airfoils/Polars/xf-e395-il-1000000.txt' ]]

    lift_rotor.airfoil_polar_stations   = np.zeros((20), dtype=np.int8).tolist()
    lift_rotor                          = propeller_design(lift_rotor)

    # Appending rotors with different origins
    rotations = [1, -1, -1, 1, 1, -1]
    origins = [[1.0182548106, 0, 1.],
               [2.443811545, 1.618254811, 1.],
               [2.443811545, -1.618254811, 1.],
               [5.984188455, 1.618254811, 1.],
               [5.984188455, -1.618254811, 1.],
               [7.4097451894, 0, 1.]]
             # [length, width, height]

    for ii in range(6):
        lift_rotor          = deepcopy(lift_rotor)
        lift_rotor.tag      = 'lift_rotor'
        lift_rotor.rotation = rotations[ii]
        lift_rotor.origin   = [origins[ii]]
        net.lift_rotors.append(lift_rotor)

    # Component 4 the Motor
    lift_rotor_motor = SUAVE.Components.Energy.Converters.Motor()
    lift_rotor_motor.efficiency = 0.85
    lift_rotor_motor.nominal_voltage = bat.max_voltage * 3 / 4
    lift_rotor_motor.mass_properties.mass = 3. * Units.kg
    lift_rotor_motor.origin = lift_rotor.origin
    lift_rotor_motor.propeller_radius = lift_rotor.tip_radius
    lift_rotor_motor.gearbox_efficiency = 1.0
    lift_rotor_motor.no_load_current = 4.0
    lift_rotor_motor = size_optimal_motor(lift_rotor_motor, lift_rotor)

    for _ in range(6):
        lift_rotor_motor = deepcopy(lift_rotor_motor)
        lift_rotor_motor.tag = 'motor'
        net.lift_rotor_motors.append(lift_rotor_motor)

    # ------------------------------------------------------------------
    # Others
    # ------------------------------------------------------------------

    # Component 5 the Payload
    payload = SUAVE.Components.Energy.Peripherals.Payload()
    payload.power_draw = 0.  # Watts
    payload.mass_properties.mass = 60 * Units.kg
    net.payload = payload

    # Component 6 the Avionics
    avionics = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw = 300.  # Watts
    net.avionics = avionics

    # add the network to the vehicle
    vehicle.append_component(net)

    # Now account for things that have been overlooked for now:
    vehicle.excrescence_area = 13.36

    return vehicle



# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs = SUAVE.Components.Configs.Config.Container()

    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)

    # Without Alex
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'no_alex'
    config.mass_properties.takeoff = 204.85821329134058 * Units.kg
    config.mass_properties.operating_empty = 204.85821329134058 * Units.kg
    config.mass_properties.max_takeoff = 351.25821329134055 * Units.kg
    config.mass_properties.max_payload = 0 * Units.kg
    configs.append(config)

    return configs


# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):
    analyses = SUAVE.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag, config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses


def base_analysis(vehicle):
    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------
    analyses = SUAVE.Analyses.Vehicle()

    # ------------------------------------------------------------------
    #  Basic Geometry Relations
    sizing = SUAVE.Analyses.Sizing.Sizing()
    sizing.features.vehicle = vehicle
    analyses.append(sizing)

    # ------------------------------------------------------------------
    #  Weights
    weights = SUAVE.Analyses.Weights.Weights()
    weights.settings.empty_weight_method = \
        SUAVE.Methods.Weights.Correlations.UAV.empty
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.AERODAS()
    aerodynamics.geometry = vehicle
    aerodynamics.settings.drag_coefficient_increment = 0.0146
    aerodynamics.settings.maximum_lift_coefficient = 1.5
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Energy
    energy = SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.networks
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = SUAVE.Analyses.Planets.Planet()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)

    # done!
    return analyses


# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
def mission_setup(analyses, vehicle):
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'Adversity Mission'

    mission.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()
    mission.planet = SUAVE.Attributes.Planets.Earth()

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()
    base_segment.process.initialize.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.iterate.conditions.planet_position = SUAVE.Methods.skip
    base_segment.state.numerics.number_control_points = 30

    # ------------------------------------------------------------------
    #  Hover Climb
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Hover.Climb(base_segment)
    segment.tag = "Climb1"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.battery_energy = vehicle.networks.battery_propeller.battery.max_energy
    segment.altitude_start = 0.
    segment.altitude_end = 10. * Units.m
    segment.climb_rate = 5.0 * Units.m / Units.s
    segment.true_course = 0.0 * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.1)
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Accelerated Cruise
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag = "Cruise1"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude = 10 * Units.m
    segment.acceleration = 0.3125 * Units['m/s/s']
    segment.air_speed_start = 2.5 * Units['m/s']
    segment.air_speed_end = 5. * Units['m/s']
    segment.true_course = 0.0 * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.07)
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Cruise
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag = "Cruise2"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude = 10 * Units.m
    segment.acceleration = -0.3125 * Units['m/s/s']
    segment.air_speed_start = 5. * Units['m/s']
    segment.air_speed_end = 2.5 * Units['m/s']
    segment.true_course = 240. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.07)
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Descend
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Hover.Descent(base_segment)
    segment.tag = "Descend1"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude_start = 10. * Units.m
    segment.altitude_end = 0. * Units.m
    segment.descent_rate = 5.0 * Units['m/s']
    segment.true_course = 240. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.04)

    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Ground
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Hover.Hover(base_segment)
    segment.tag = "Ground1"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude = 0. * Units.m
    segment.time = 120. * Units.s
    segment.true_course = 240. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.04)

    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Hover Climb
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Hover.Climb(base_segment)
    segment.tag = "Climb2"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude_start = 0.
    segment.altitude_end = 10. * Units.m
    segment.climb_rate = 5.0 * Units.m / Units.s
    segment.true_course = 60. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.1)
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Accelerated Cruise
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag = "Cruise3"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude = 10 * Units.m
    segment.acceleration = 0.3125 * Units['m/s/s']
    segment.air_speed_start = 2.5 * Units['m/s']
    segment.air_speed_end = 5. * Units['m/s']
    segment.true_course = 60. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.07)
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Cruise
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag = "Cruise4"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude = 10 * Units.m
    segment.acceleration = -0.3125 * Units['m/s/s']
    segment.air_speed_start = 5. * Units['m/s']
    segment.air_speed_end = 2.5 * Units['m/s']
    segment.true_course = 300. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.07)
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Descend
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Hover.Descent(base_segment)
    segment.tag = "Descend2"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude_start = 10. * Units.m
    segment.altitude_end = 0. * Units.m
    segment.descent_rate = 5.0 * Units['m/s']
    segment.true_course = 300. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.04)

    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Ground
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Hover.Hover(base_segment)
    segment.tag = "Ground2"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude = 0. * Units.m
    segment.time = 120. * Units.s
    segment.true_course = 300. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.04)

    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Hover Climb
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Hover.Climb(base_segment)
    segment.tag = "Climb3"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude_start = 0.
    segment.altitude_end = 10. * Units.m
    segment.climb_rate = 5.0 * Units.m / Units.s
    segment.true_course = 120. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.1)
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Accelerated Cruise
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag = "Cruise5"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude = 10 * Units.m
    segment.acceleration = 0.3125 * Units['m/s/s']
    segment.air_speed_start = 2.5 * Units['m/s']
    segment.air_speed_end = 5. * Units['m/s']
    segment.true_course = 120. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.07)
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Cruise
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag = "Cruise6"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude = 10 * Units.m
    segment.acceleration = -0.3125 * Units['m/s/s']
    segment.air_speed_start = 5. * Units['m/s']
    segment.air_speed_end = 2.5 * Units['m/s']
    segment.true_course = 0. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.07)
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Descend
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Hover.Descent(base_segment)
    segment.tag = "Descend3"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude_start = 10. * Units.m
    segment.altitude_end = 0. * Units.m
    segment.descent_rate = 5.0 * Units['m/s']
    segment.true_course = 0. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.04)

    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Ground
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Hover.Hover(base_segment)
    segment.tag = "Ground3"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude = 0. * Units.m
    segment.time = 120. * Units.s
    segment.true_course = 0. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.04)

    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Hover Climb
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Hover.Climb(base_segment)
    segment.tag = "Climb4"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude_start = 0.
    segment.altitude_end = 10. * Units.m
    segment.climb_rate = 5.0 * Units.m / Units.s
    segment.true_course = 180. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.1)
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Accelerated Cruise
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag = "Cruise7"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude = 10 * Units.m
    segment.acceleration = 0.3125 * Units['m/s/s']
    segment.air_speed_start = 2.5 * Units['m/s']
    segment.air_speed_end = 5. * Units['m/s']
    segment.true_course = 180. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.07)
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Cruise
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag = "Cruise8"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude = 10 * Units.m
    segment.acceleration = -0.3125 * Units['m/s/s']
    segment.air_speed_start = 5. * Units['m/s']
    segment.air_speed_end = 2.5 * Units['m/s']
    segment.true_course = 60. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.07)
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Descend
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Hover.Descent(base_segment)
    segment.tag = "Descend4"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude_start = 10. * Units.m
    segment.altitude_end = 0. * Units.m
    segment.descent_rate = 5.0 * Units['m/s']
    segment.true_course = 60. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.04)

    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Ground
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Hover.Hover(base_segment)
    segment.tag = "Ground4"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude = 0. * Units.m
    segment.time = 120. * Units.s
    segment.true_course = 60. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.04)

    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Hover Climb
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Hover.Climb(base_segment)
    segment.tag = "Climb5"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude_start = 0.
    segment.altitude_end = 10. * Units.m
    segment.climb_rate = 5.0 * Units.m / Units.s
    segment.true_course = 240. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.1)
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Accelerated Cruise
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag = "Cruise9"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude = 10 * Units.m
    segment.acceleration = 0.3125 * Units['m/s/s']
    segment.air_speed_start = 2.5 * Units['m/s']
    segment.air_speed_end = 5. * Units['m/s']
    segment.true_course = 240. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.07)
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Cruise
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag = "Cruise10"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude = 10 * Units.m
    segment.acceleration = -0.3125 * Units['m/s/s']
    segment.air_speed_start = 5. * Units['m/s']
    segment.air_speed_end = 2.5 * Units['m/s']
    segment.true_course = 120. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.07)
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Descend
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Hover.Descent(base_segment)
    segment.tag = "Descend5"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude_start = 10. * Units.m
    segment.altitude_end = 0. * Units.m
    segment.descent_rate = 5.0 * Units['m/s']
    segment.true_course = 120. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.04)

    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Ground
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Hover.Hover(base_segment)
    segment.tag = "Ground5"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude = 0. * Units.m
    segment.time = 120. * Units.s
    segment.true_course = 120. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.04)

    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Hover Climb
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Hover.Climb(base_segment)
    segment.tag = "Climb6"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude_start = 0.
    segment.altitude_end = 10. * Units.m
    segment.climb_rate = 5.0 * Units.m / Units.s
    segment.true_course = 300. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.1)
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Accelerated Cruise
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag = "Cruise11"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude = 10 * Units.m
    segment.acceleration = 0.3125 * Units['m/s/s']
    segment.air_speed_start = 2.5 * Units['m/s']
    segment.air_speed_end = 5. * Units['m/s']
    segment.true_course = 300. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.07)
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Cruise
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag = "Cruise12"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude = 10 * Units.m
    segment.acceleration = -0.3125 * Units['m/s/s']
    segment.air_speed_start = 5. * Units['m/s']
    segment.air_speed_end = 2.5 * Units['m/s']
    segment.true_course = 180. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.07)
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Descend
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Hover.Descent(base_segment)
    segment.tag = "Descend6"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # segment attributes
    segment.altitude_start = 10. * Units.m
    segment.altitude_end = 0. * Units.m
    segment.descent_rate = 5.0 * Units['m/s']
    segment.true_course = 180. * Units.degrees

    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, \
                                                                                       initial_power_coefficient=0.04)

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Mission definition complete
    # -------------------------------------------------------------------

    return mission


def missions_setup(base_mission):
    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------

    missions.base = base_mission

    # done!
    return missions


# ----------------------------------------------------------------------
#   Plot Results
# ----------------------------------------------------------------------
def plot_mission(results):
    # Plot Flight Conditions
    plot_flight_conditions(results)

    # Plot Aerodynamic Coefficients
    plot_aerodynamic_coefficients(results)

    # Plot Aircraft Flight Speed
    plot_aircraft_velocities(results)

    # Plot Aircraft Electronics
    plot_battery_pack_conditions(results)

    # Plot Propeller Conditions
    plot_propeller_conditions(results)

    # Plot Electric Motor and Propeller Efficiencies
    plot_eMotor_Prop_efficiencies(results)

    # Plot propeller Disc and Power Loading
    plot_disc_power_loading(results)

    # Plot 3D Trajectory
    plot_flight_trajectory(results)

    # Plot Stability Coefficients
    plot_stability_coefficients(results)

    return


if __name__ == '__main__':
    main()

    plt.show()


