import numpy as np
import matplotlib.pyplot as plt
#Define variables
l_beam=1
w_beam=0.3
F_lift=50
M=F_lift*l_beam
tau=283*10**6
sigma=324*10**6

V_shear=100
#Define the functions
def thickness_shear(V, tau):
    return np.sqrt(256*V/3*np.pi*tau)
def thickness_bending(M, sigma):
    return (64*M/(np.pi*sigma))**1/3