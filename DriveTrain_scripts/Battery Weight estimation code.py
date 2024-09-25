import pandas as pd
import matplotlib.pyplot as plt 
import numpy as np

#! means that it needs to be looked at

#parameters and constants

Required_Energy = 1000 #kWh From other code
Energy_Density = 250 # Wh/KG

Efficiency_Powertrain = 0.9
Efficiency_rotor = 0.89
End_state_of_charge = 0.2 # 20% it,s just a buffer



#formulas
Efficiency = Efficiency_Powertrain * Efficiency_rotor * (1-End_state_of_charge)
battery_mass = Required_Energy /( Energy_Density * Efficiency)

print("the Battery mass is:" ,battery_mass)
