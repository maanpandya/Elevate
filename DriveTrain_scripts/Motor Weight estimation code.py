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

# Read the CSV file and skip some rows and columns
data = pd.read_csv(r'C:\Users\Gebruiker\Desktop\Elevate\Elevate\DriveTrain_scripts\motor comparison(Sheet2).csv', encoding='latin-1', skiprows=4)
data = data.iloc[:, 1:] 

# Remove row 16 and 35 because they are outliers
data = data.drop([16])
data = data.drop([35])

# Extract columns
Names = data['Name'] 
Weight = data['Weight(kg)(AC)'] 
Power = data['P(kW)2']

# Drop any rows with NaN values
data_cleaned = pd.DataFrame({'Names': Names, 'Weight': Weight, 'Power': Power}).dropna()

# Display the extracted data
print(data_cleaned)

# Extract Weight, Power, and updated Names from the cleaned data
X = data_cleaned['Power'].values
Y = data_cleaned['Weight'].values
updated_names = data_cleaned['Names'].values  # Updated names after removing rows

# Perform linear regression
slope, intercept = np.polyfit(X, Y, 1)

# Create the regression line
regression_line = slope * X + intercept

# Plot the data and the regression line
plt.figure(figsize=(10, 6))
plt.scatter(X, Y, color='b', label='Data points')

# Annotate each data point with the corresponding name
for i in range(len(X)):
    plt.text(X[i], Y[i], updated_names[i], fontsize=9, ha='right', color='green')  # Use updated_names

# Plot the regression line
plt.plot(X, regression_line, color='r', label=f'Linear fit: y = {slope:.2f}x + {intercept:.2f}')

# Add labels and title
plt.xlabel('Power (kW)')
plt.ylabel('Weight (kg)')
plt.title('Power vs Weight with Linear Regression')

# Show legend and grid
plt.legend()
plt.grid(True)

# Show the plot
plt.show()

# Print the equation of the line
print(f'The linear function that best fits the data is: y = {slope:.2f}x + {intercept:.2f}')