import numpy as np
import os
try:
    import cPickle as pickle
except ImportError:
    import pickle

def get_aifoildata(foil, re_angs, query):
    #Check if data already exists, if not, generate new:
    foilpath = foil.replace(" ", "_")
    if not os.path.exists('AirfoilData/' + foilpath):
        print("If you get a ModuleNotFoundError, it is because you use an airfoil that doesn't have data generated yet."
              "\nPlease send a message with the airfoil to Tamás or Noam to get the data generated.")
        from AirfoildataGenerator import make_airfoildata
        angles, re_nrs = np.arange(20, -20, -0.2), np.arange(3, 8, 0.25)
        make_airfoildata(foil, foilpath, angles, re_nrs, True)

    re_angs = np.array(re_angs)
    re_angs[:, 0] = np.log10(re_angs[:, 0])

    with open('AirfoilData/' + foilpath + '/area_func.pkl', 'rb') as file:
        area_func = pickle.load(file)

    ind = np.where(np.array(area_func(re_angs))==0)
    analise = re_angs[ind]

    result = []
    for property in query:
        if property == 'Cl':
            with open('AirfoilData/' + foilpath + '/cl_func.pkl', 'rb') as file:
                cl_func = pickle.load(file)
            result.append(cl_func(analise))
        elif property == 'Cd':
            with open('AirfoilData/' + foilpath + '/cd_func.pkl', 'rb') as file:
                cd_func = pickle.load(file)
            result.append(cd_func(analise))
        elif property == 'ClCd':
            with open('AirfoilData/' + foilpath + '/clcd_func.pkl', 'rb') as file:
                clcd_func = pickle.load(file)
            result.append(clcd_func(analise))
        elif property == "AlphaClCd":
            with open('AirfoilData/' + foilpath + '/clcdangle_func.pkl', 'rb') as file:
                clcdangle_func = pickle.load(file)
            result.append(clcdangle_func(analise[:, 0]))
        elif property == "ClCdmax":
            with open('AirfoilData/' + foilpath + '/clcdmax_func.pkl', 'rb') as file:
                clcdmax_func = pickle.load(file)
            result.append(clcdmax_func(analise[:, 0]))
        elif property == "ClCdmaxCl":
            with open('AirfoilData/' + foilpath + '/clcdmaxcl_func.pkl', 'rb') as file:
                clcdmaxcl_func = pickle.load(file)
            result.append(clcdmaxcl_func(analise[:, 0]))
        else:
            result.append([np.NAN]*len(analise))

    result = np.transpose(np.array(result))
    results = [0] * len(re_angs)
    j = 0
    for i in range(len(re_angs)):
        if i in ind[0]:
            results[i] = result[j]
            j += 1
        else:
            results[i] = [np.nan] * len(query)

    return np.array(results)

# Test code, should yield: [[1.17880000e+00 4.30800002e-02 2.73630455e+01 6.40000000e+00 4.99226092e+01 9.03100000e-01]]
#question = ('Cl', 'Cd')
#answer = get_aifoildata('NACA 2412', [[1e5, 12]], question)
#print(answer)