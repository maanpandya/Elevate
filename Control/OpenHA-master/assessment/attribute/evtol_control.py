# Main Script

import math
from itertools import combinations

import numpy as np
import scipy
import sympy as sp

from degree_of_controllablity import control_allocation
from degree_of_controllablity import acai
from degree_of_controllablity import doc_gramian
from degree_of_controllablity import doc_recovery_region
from degree_of_controllablity import doc_disturbance_rejection_kang

from profust_reliability import profust_reliability
from profust_reliability import trapezoidal_membership_func


# Note: Adapted for a hexacopter design
m = 357.35955149404106
g = 9.81

hub1 = [2.3182548105986953,0]
hub2 = [1.018254810598695,1.6182548105986951]
hub3 = [-1.018254810598695,-1.6182548105986951]
hub4 = [-2.3182548105986953,0]
hub5 = [-1.018254810598695,1.6182548105986951]
hub6 = [1.018254810598695,-1.6182548105986951]

phi1 = 0
phi2 = math.atan(hub2[1]/hub2[0])
phi3 = math.pi - phi2
phi4 = math.pi
phi5 = math.pi + phi2
phi6 = 2*math.pi - phi2

d1 = math.sqrt(hub1[0]**2 + hub1[1]**2)
d2 = math.sqrt(hub2[0]**2 + hub2[1]**2)
d3 = math.sqrt(hub3[0]**2 + hub3[1]**2)
d4 = math.sqrt(hub4[0]**2 + hub4[1]**2)
d5 = math.sqrt(hub5[0]**2 + hub5[1]**2)
d6 = math.sqrt(hub6[0]**2 + hub6[1]**2)

throttle = 50  #percentage
n = 6
d = [d1, d2, d3, d4, d5, d6]
ku = [0.80, 0.80, 0.80, 0.80, 0.80, 0.80]  
init_angle = [phi1, phi2, phi3, phi4, phi5, phi6]
drct = [-1, 1, -1, 1, -1, 1]
eta = 0.01 * throttle * np.array([1, 1, 1, 1, 1, 1])
giveup_yaw = False
giveup_height = False

bf = control_allocation(
    n,
    d,
    ku,
    init_angle=init_angle,
    drct=drct,
    eta=eta,
    giveup_yaw=giveup_yaw,
    giveup_height=giveup_height
    )

bf = np.array(bf)

#print(bf)




fmax = np.array([10000, 10000, 10000, 10000, 10000, 10000])
fmin = -1*fmax

G = np.array([m*g, 0, 0, 0])
if giveup_yaw and giveup_height:
    G = np.array([1, 0])
elif giveup_yaw or giveup_height:
    G = np.array([1, 0, 0])

doc_value = acai(bf, fmax, fmin, G)

print("Degree of Controllability (ACAI based):", doc_value)


"""
Notes:

- init_angle is measured from the body axis of the eVTOL to the respective propeller arm, viewed from top
- in drct, 1 = counter-clockwise, -1 = clockwise directions
- by turning on giveup_yaw and giveup_height, you are essentially disabling control in yaw direction and height direction...
... this will reduce the number of rows in the bf matrix by either 1 or 2 rows depending on if you turn both on or only one of them
- if there is no height control i.e. giveup_height = True, it means you are analysing the EVTOL in hovering mode (likewise for giveup_yaw, but for spinning mode)
- G should have the same number of columns as the number of rows that the bf matrix has (hence the if and elif statements)
- control effectiveness matrix bf has n columns (since n propellers). Normally it has 4 rows, representing the thrust, and the moments about all three axes produced by each propeller...
... hence a 4xn matrix. This makes sense because if you give up e.g. yaw control, then giveup_yaw = True and like mentioned before, this bf matrix will now have 3 rows because...
now the moment about the vertical axis (controlling yaw) is irrelevant in our matrix calculation since we do not care about yaw control. Hence, 3 rows for the bf matrix...
... you multiply this bf matrix by a vector [f1, f2, f2, ... , fn] representing the force produced by each propeller, and the result is (as mentioned before), a vector...
... [total thrust, xmoment, ymoment, zmoment]. For more info, refer to Quan Quan page 128, equation 6.25 and the equations on page 239-240
- The four terms in the G vector represents the constant disturbance of total thrust produce by all propellers, and total moments about x, y, and z axes respectively...
... basically, G = [f, taox, taoy, taoz]. In this case of hovering, the only constant disturbance we look at is gravity, hence f = m*g in the code above, and everything else is 0
- fmax and fmin are the maximum and minimum thrusts produced by the propellers 


"""











