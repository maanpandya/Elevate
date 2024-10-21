import numpy as np


class Configuration:
    w_max_flight = 8.5 - 2 * 1
    w_fuselage = 1.2    # Outer arm connection point is also assumed to be at this width.
    l_fuselage = 2.6    # Center arm connections point also assumed to be at this point.
    l_outerpin = 2.1    # Max outer arm connection point?
    w_max = 2.6
    h_max = 4.1
    l_max = 8.53


    def __init__(self, type, N_blades, D_hub, H_hub, Chord_root):
        self.type = type
        self.N = N_blades
        self.D_hub = D_hub
        self.H_hub = H_hub
        self.Cr = Chord_root

    def max_diam(self, folding):
        # Max diameter due to flight width restrictions:
        if folding:
            # If mid-flight folding is allowed, the outer propellers can be hinged inwards until they touch:
            max_diam_flight = self.w_max_flight / 2
        else:
            L_it = self.l_max - 0.01
            D_1, D_2 = 1, 0
            while D_1 > D_2:
                b = (L_it + self.w_max_flight)
                # Max diameter due to max flight width limit:
                D_1 = (- b + np.sqrt(b ** 2 + 2 * (self.w_max_flight ** 2 + L_it ** 2))) / 2
                # Max diameter due to truck dimension limits:
                if self.N == 2:
                    D_2 = np.sqrt(self.w_max ** 2 + (self.l_max - L_it) ** 2)
                else:
                    D_2 = min((self.l_max - L_it) / np.sin(np.pi / self.N), self.w_max / np.cos(np.pi / self.N))
                    # At some point the 6-bladed propeller switches away from having a blade in the axial direction.
                    if self.N == 6 and np.cos(np.pi / 6) < self.w_max / (self.l_max - L_it) < 1:
                        D_2 = (self.l_max - L_it)
                L_it -= 0.01
            # Max diameter due to fuselage having to fit in between two propellers:
            D_3 = (self.w_max_flight - self.w_fuselage) / 2
            # Select most limiting one:
            max_diam_flight = min(D_1, D_2, D_3)

        # Max diameter due to outer propellers having to fit in the truck.
        # Case for two blades isn't constrained by outer propellers and is thus governed by mid-flight conditions.
        if self.N == 2:
            max_diam_truck = max_diam_flight
        elif self.N == 3:
            # Center propeller is limiting as it is equally close to both truck walls.
            max_diam_truck = self.w_max / np.sin(np.pi/3)
        elif self.N == 4:
            phi = np.pi / 4
            max_diam_truck = -1000
            while max_diam_truck == -1000 or max_diam_truck - D_prev > 0:
                phi += np.pi / 100
                D_prev = max_diam_truck
                # Z is the spacing between the hearts of the props such that the blades don't intersect under an angle.
                Z = max(((self.D_hub - np.sin(phi) * self.Cr) * np.tan(phi) - self.D_hub - np.cos(phi) * self.Cr) / (
                            1 - np.tan(phi)), 0)
                max_diam_truck = (self.w_max - Z - self.D_hub) / np.sin(phi)
        elif self.N == 6:
            Z = max(self.Cr/np.tan(np.pi/12), self.D_hub)
            max_diam_truck = (self.w_max - Z)
        else:
            print('Warning! Requested Nr of blades not implemented, choose 2, 3, 4 or 6.')
            max_diam_truck = max_diam_flight

        return min(max_diam_flight, max_diam_truck)

    def arm(self, D_prop, folding):
        Z = self.D_hub
        w = 0
        if self.N == 2:
            if D_prop**2 >= self.w_max**2 + self.D_hub**2:
                frontmargin = np.sqrt(D_prop**2 - self.w_max**2) / 2
            else:
                frontmargin = self.D_hub / 2
            if D_prop**2 >= (self.w_max - Z)**2 + self.D_hub**2:
                frontmargin_out = np.sqrt(D_prop**2 - (self.w_max - Z)**2) / 2
            else:
                frontmargin_out = self.D_hub / 2
        elif self.N == 3:
            while w < self.w_max:
                Z += 0.01
                phi = np.arcsin(self.Cr / Z)
                w = Z + D_prop / np.sin(np.pi/6 + phi)
            Z -= 0.01
            frontmargin_out = D_prop * np.cos(np.pi/6 - phi) / 2
            frontmargin = D_prop * np.cos(np.pi/3) / 2
        elif self.N == 4:
            while w < self.w_max:
                Z += 0.01
                phi = np.arcsin(self.Cr / Z / np.sqrt(2))
                w = Z + D_prop * np.cos(phi + np.pi/4)
            Z -= 0.01
            frontmargin_out = D_prop * np.sin(phi + np.pi/4) / 2
            frontmargin = D_prop / np.sqrt(2) / 2
        elif self.N == 6:
            Z = self.w_max - D_prop
            frontmargin = D_prop * np.cos(np.pi / 6) / 2
            frontmargin_out = D_prop / 2
        else:
            print('Warning! Requested Nr of blades not implemented, choose 2, 3, 4 or 6.')
            frontmargin = self.D_hub / 2
            frontmargin_out = self.D_hub / 2

        smallarm = False
        if D_prop > self.w_fuselage:
            l_centerhub = D_prop/2 + np.sqrt(D_prop**2 - ((D_prop + self.w_fuselage)/2)**2)
            if l_centerhub > (D_prop + self.l_fuselage) / 2:
                l_arm = self.l_max
                l_armprev = 2 * self.l_max
                widthok = True
                lengthok = False
                while (l_arm < l_armprev or not lengthok) and widthok:

                    if l_centerhub < D_prop/2 + np.sqrt(D_prop**2 - ((D_prop + self.w_fuselage)/2)**2) - 0.001:
                        w_outerhub_prev = w_outerhub
                        l_outerhub_prev = l_outerhub
                    w_outerhub = np.sqrt(D_prop**2 - (l_centerhub - D_prop/2)**2)
                    l_outerarm = max(D_prop / 2, w_outerhub - self.w_fuselage / 2)

                    l_outerfolded_max = frontmargin_out + D_prop/2 + np.sqrt(D_prop**2 - (self.w_fuselage - Z)**2) / 2
                    if self.l_max/2 < l_outerfolded_max:
                        u = self.l_max - frontmargin_out*2 - D_prop
                        l_outerpin = D_prop/2 - ((w_outerhub*2 - self.w_fuselage)**2 - u**2 - (self.w_fuselage - Z)**2) / 2 / u
                        l_outerpin = min(self.l_outerpin, l_outerpin)
                    else:
                        if D_prop < self.l_outerpin:
                            l_outerpin = D_prop/2
                        else:
                            l_outerpin = self.l_outerpin/2

                    l_outerarm = np.sqrt(l_outerarm ** 2 + ((D_prop - l_outerpin) / 2) ** 2)

                    l_armprev = l_arm
                    l_arm = max(D_prop/2, l_centerhub - self.l_fuselage/2, l_outerarm)

                    l_outerhub = min(l_outerpin, D_prop) / 2 + np.sqrt(l_arm**2 - ((self.w_fuselage - Z)/2)**2)
                    l_folded = max(l_centerhub + frontmargin, l_outerhub + frontmargin_out)
                    if l_folded*2 < self.l_max:
                        lengthok = True
                    else:
                        lengthok = False

                    if (((not folding and w_outerhub * 2 + D_prop > self.w_max_flight) or
                        l_centerhub < (D_prop + self.l_fuselage) / 2)
                        and not l_centerhub > D_prop/2 + np.sqrt(D_prop**2 - ((D_prop + self.w_fuselage)/2)**2) - 0.001):
                        widthok = False
                        l_arm = l_armprev
                        w_outerhub = w_outerhub_prev
                        l_outerhub = l_outerhub_prev
                        l_centerhub += 0.01

                    l_centerhub -= 0.01

                    if l_centerhub < 0:
                        print('No solution!')
                        widthok = False

            else:
                smallarm = True
        else:
            smallarm = True

        if smallarm:
            #Continue, smallest arm length is this:
            l_arm = D_prop / 2
            if D_prop < self.l_outerpin:
                l_outerpin = D_prop/2
            else:
                l_outerpin = self.l_outerpin / 2
            l_outerhub = min(l_outerpin, D_prop) / 2 + np.sqrt(l_arm**2 - ((self.w_fuselage - Z)/2)**2)
            l_centerhub = l_arm + self.l_fuselage / 2
            w_outerhub = (D_prop + self.w_fuselage) / 2

        hub_coords = [['Center_hub: (L, W)', l_centerhub, 0],
                      ['Outer_hub: (L, W)', D_prop/2, w_outerhub],
                      ['Outer_hub_folded: (L, W)', l_outerhub, Z/2],
                      ['Pin_location: (L, W)', l_outerpin, self.w_fuselage/2]]
        l_folded = max(l_outerhub + frontmargin_out, l_centerhub + frontmargin)
        width = w_outerhub * 2 + D_prop
        angle_hinge = np.arctan2(self.H_hub, (w_outerhub-Z/2))

        return l_arm, width, 2 * l_folded, np.degrees(angle_hinge), hub_coords


config1 = Configuration("Hori_fold", 4, 0.3, 0.2, 0.1)
fold = True     # Limits in-flight width.
diameter_limit = 4
dia_max = config1.max_diam(fold)
print(dia_max)
print(config1.arm(min(dia_max, diameter_limit), fold))
