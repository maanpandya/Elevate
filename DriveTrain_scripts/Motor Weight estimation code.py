import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Parameters and constants

Climb_Power = 1000  # ! I hope i get this from other code
Power_Margin = 0.5  # ! No idea what value this needs to be
number_of_motors = 6  # Design choice
Power_to_Weight_ratio = 0.165  # Based on other aircraft data




#formulas

# Motor mass calculation
motor_mass = Power_to_Weight_ratio * ((Climb_Power * (1 + Power_Margin)) / number_of_motors)
print("The motor mass is:", motor_mass)

# Motor mass calculation via kadhiresan method
def calculate_motor_weight(torque):
    return 2.20462 * ((58 / 990) * (torque - 10) + 2) * number_of_motors



#Excel data

# Read the CSV file and skip some rows and columns
data = pd.read_csv(r'C:\Users\Gebruiker\Desktop\Elevate\Elevate\DriveTrain_scripts\motor comparison(Sheet2).csv', encoding='latin-1', skiprows=4)
data = data.iloc[:, 1:]

# Remove outliers
data = data.drop([16])
data = data.drop([35])

# Extract columns
Names = data['Name']
Weight = data['Weight(kg)(AC)']
Power = data['P(kW)2']
Torque = data['Torque(Nm)'] 

# Drop any rows with NaN values
data_cleaned = pd.DataFrame({'Names': Names, 'Weight': Weight, 'Power': Power, 'Torque': Torque}).dropna()

# Extract Weight, Power, Torque, and updated Names from the cleaned data
X_power = data_cleaned['Power'].values
Y_weight = data_cleaned['Weight'].values
X_torque = data_cleaned['Torque'].values
updated_names = data_cleaned['Names'].values

# Create plots
fig, axs = plt.subplots(1, 2, figsize=(14, 6))  # Two plots side by side

# First subplot: Power vs Weight with Linear Regression
axs[0].scatter(X_power, Y_weight, color='b', label='Data points')

# Perform linear regression
slope_power, intercept_power = np.polyfit(X_power, Y_weight, 1)
regression_line_power = slope_power * X_power + intercept_power

# Plot the regression line
axs[0].plot(X_power, regression_line_power, color='r', label=f'Linear fit: y = {slope_power:.2f}x + {intercept_power:.2f}')

# Annotate each data point with the corresponding name
for i in range(len(X_power)):
    axs[0].text(X_power[i], Y_weight[i], updated_names[i], fontsize=9, ha='right', color='green')

axs[0].set_xlabel('Power (kW)')
axs[0].set_ylabel('Weight (kg)')
axs[0].set_title('Power vs Weight with Linear Regression')
axs[0].legend()
axs[0].grid(True)

# Second subplot: Weight vs Torque
axs[1].scatter(X_torque, Y_weight, color='blue', label='Data points')

# Perform linear regression for Torque vs Weight
slope_torque, intercept_torque = np.polyfit(X_torque, Y_weight, 1)
regression_line_torque = slope_torque * X_torque + intercept_torque

# Plot the regression line
axs[1].plot(X_torque, regression_line_torque, color='red', label=f'Linear fit: y = {slope_torque:.2f}x + {intercept_torque:.2f}')

# Annotate each data point with the corresponding name
for i in range(len(X_torque)):
    axs[1].text(X_torque[i], Y_weight[i], updated_names[i], fontsize=9, ha='right', color='green')

axs[1].set_xlabel('Torque (Nm)')
axs[1].set_ylabel('Weight (kg)')
axs[1].set_title('Weight vs Max Torque with Linear Regression')
axs[1].legend()
axs[1].grid(True)

# Adjust layout
plt.tight_layout()  # Automatically adjust subplot parameters for a better fit
plt.show()
