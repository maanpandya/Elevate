import numpy as np
import scipy.interpolate as interp
import matplotlib.pyplot as plt
import os
from parapy.lib.xfoil import run_xfoil
try:
    import cPickle as pickle
except ImportError:
    import pickle

def make_airfoildata(airfoil, foilpath, angles, re_nrs, verify):
    points = np.array([])
    cl_data = np.array([])
    cd_data = np.array([])

    re_points = []
    re_failed = []

    #Collect data from xfoil:
    for re_nr in re_nrs:
        data = np.array(run_xfoil(airfoil, 10 ** re_nr, (angles[0], angles[-1], angles[1] - angles[0]), ncrit=9))
        if data.size != 0:
            points = np.append(points, np.insert(np.expand_dims(np.array(data[:, 0]), axis=1), 0, re_nr, axis=1))
            cl_data = np.append(cl_data, data[:, 1])
            cd_data = np.append(cd_data, data[:, 2])

            re_points.append(re_nr)
        else:
            re_failed.append(re_nr)

    #Construct interpolation functions:
    points = np.reshape(points, (-1, 2))
    area_func = interp.LinearNDInterpolator(points, np.zeros(len(points)))
    cl_func = interp.RBFInterpolator(points, cl_data.ravel(), smoothing=0, kernel='cubic')
    cd_func = interp.RBFInterpolator(points, cd_data.ravel(), smoothing=0, kernel='cubic')

    if verify:
        # Test points for verification graphs:
        test_re, test_alpha = np.linspace(re_nrs[0], re_nrs[-1], len(re_nrs) * 2 - 1), np.linspace(angles[0],
                                                                                                   angles[-1],
                                                                                                   len(angles) * 2 - 1)
        testgrid_re, testgrid_alpha = np.meshgrid(test_re, test_alpha)
        test_points = np.stack([testgrid_re.ravel(), testgrid_alpha.ravel()], -1)

        test_cl = cl_func(test_points).reshape(testgrid_re.shape)
        test_cd = cd_func(test_points).reshape(testgrid_re.shape)

        test_area = area_func(testgrid_re, testgrid_alpha)

        # Plot interpolated surfaces for verification:
        fig = plt.figure()
        ax = fig.add_subplot(1, 2, 1, projection='3d')
        ax.plot_surface(testgrid_re, testgrid_alpha, test_cl, rstride=1, cstride=1, cmap='viridis', edgecolor='none')
        ax.set_title('Cl Verification Plot')
        ax.set_xlabel('Re')
        ax.set_ylabel('Alpha')
        ax.set_zlabel('Cl')

        ax = fig.add_subplot(1, 2, 2, projection='3d')
        ax.plot_surface(testgrid_re, testgrid_alpha, test_cd, rstride=1, cstride=1, cmap='viridis', edgecolor='none')
        ax.plot_wireframe(testgrid_re, testgrid_alpha, test_area, color='gray')
        ax.set_title('Cd Verification Plot')
        ax.set_xlabel('Re')
        ax.set_ylabel('Alpha')
        ax.set_zlabel('Cd')

        plt.show()

    # Save interpolation functions for later use:
    os.mkdir('AirfoilData/' + foilpath)
    with open('AirfoilData/' + foilpath + '/info.txt', "a") as file:
        file.write(
            "This folder contains the interpolated XFoildata of the " + airfoil + " airfoil.\nThe reynolds number ranges from " + str(
                re_nrs[0]) + " to " + str(re_nrs[1]) + " with a step size of " + str(
                re_nrs[1] - re_nrs[0]) + ".\nThe angle of attack ranges from " + str(angles[0]) + " to " + str(
                angles[-1]) + " with a step size of " + str(angles[1] - angles[
                0]) + ".\nXFoil failed to find datapoints for the following reynolds numbers: " + str(
                re_failed) + "\nData was generated at the following points:\n[log10(Re_nr), Alpha]\n" + str(points))
    with open('AirfoilData/' + foilpath + '/area_func.pkl', 'wb') as file:
        pickle.dump(area_func, file)
    with open('AirfoilData/' + foilpath + '/cl_func.pkl', 'wb') as file:
        pickle.dump(cl_func, file)
    with open('AirfoilData/' + foilpath + '/cd_func.pkl', 'wb') as file:
        pickle.dump(cd_func, file)

def get_aifoildata(foil, query):
    #Check if data already exists, if not, generate new:
    foilpath = foil.replace(" ", "_")
    if not os.path.exists('AirfoilData/' + foilpath):
        print('Hey!!')
        angles, re_nrs = np.arange(20, -20, -0.2), np.arange(3, 8, 0.25)
        make_airfoildata(foil, foilpath, angles, re_nrs, True)

    functions = []
    for property in query:
        if property == 'Cl':
            with open('AirfoilData/' + foilpath + '/cl_func.pkl', 'rb') as file:
                cl_func = pickle.load(file)
            functions.append(cl_func)
        elif property == 'Cd':
            with open('AirfoilData/' + foilpath + '/cd_func.pkl', 'rb') as file:
                cd_func = pickle.load(file)
            functions.append(cd_func)

    return functions

# Test code: (Should yield '[array([0.98095212]), array([0.01600014]), array([54.56383772]), array([7.2]), array([56.7724934]), nan]'.)
question = ('Cl', 'Cd')
answer = get_aifoildata('NACA 4415', question)

# re_angs = np.insert(np.expand_dims(np.arange(-10, 10, 0.2), axis=1), 0, 10**5, axis=1)
# print(re_angs)
# answer = get_aifoildata('NACA 2412', re_angs, question)
# alpha_ind = np.radians(5)
# l_ind = answer[:, 0]*np.cos(alpha_ind)-answer[:, 1]*np.sin(alpha_ind)
# d_ind = answer[:, 0]*np.sin(alpha_ind)+answer[:, 1]*np.cos(alpha_ind)
# ld_ind = l_ind/d_ind
# ld_ind[np.where(l_ind<=0)] = 0 #Don't count negative lift values, as they are useless.
# print(max(ld_ind), re_angs[np.argmax(ld_ind)], answer[0, [-2, -1]])
