import pandas as pd
import matplotlib.pyplot as plt 
import numpy as np

#! means that it needs to be looked at

#parameters and constants

Climb_Power = 1000 #! I hope i get this from other code
Power_Margin = 0.5 #! No idea what value this needs to be
number_of_motors = 6 #design choice
Power_to_Weight_ratio = 0.165 #! based on the data of other aircraft

#formulas

motor_mass = Power_to_Weight_ratio*((Climb_Power*(1 + Power_Margin))/(number_of_motors))

print("the motor mass is:" ,motor_mass)



#graph

# Read the CSV with specific delimiter
data = pd.read_csv(r'C:\Users\Gebruiker\Desktop\Elevate\Elevate\DriveTrain_scripts\motor comparison(Sheet1).csv', delimiter=';', encoding='ISO-8859-1')

# Extract columns
Names = data.iloc[6:44, 9]  # Column K 
Weight = data.iloc[6:44, 18] # Column T 
Power = data.iloc[6:44, 20]  # Column V (

# Convert Weight and Power to numeric values, coercing errors to NaN
Weight = pd.to_numeric(Weight.str.replace(',', '.'), errors='coerce')
Power = pd.to_numeric(Power.str.replace(',', '.'), errors='coerce')

# Drop any rows with NaN values
data_cleaned = pd.DataFrame({'Names': Names, 'Weight': Weight, 'Power': Power}).dropna()

#remove row 20, because it is an outlier
data_cleaned = data_cleaned.drop([20])

# Display the extracted data
print(data_cleaned)

# Extract Weight and Power as numpy arrays for regression
X = data_cleaned['Power'].values
Y = data_cleaned['Weight'].values

# Perform linear regression
slope, intercept = np.polyfit(X, Y, 1)

# Create the regression line
regression_line = slope * X + intercept

# Plot the data and the regression line
plt.figure(figsize=(10, 6))
plt.scatter(X, Y, color='b', label='Data points')
plt.plot(X, regression_line, color='r', label=f'Linear fit: y = {slope:.2f}x + {intercept:.2f}')
plt.xlabel('Power (kW)')
plt.ylabel('Weight (kg)')
plt.title('Power vs Weight with Linear Regression')
plt.legend()
plt.grid(True)
plt.show()

# Print the equation of the line
print(f'The linear function that best fits the data is: y = {slope:.2f}x + {intercept:.2f}')
