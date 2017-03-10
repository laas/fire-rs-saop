from collections import namedtuple

import cython
import numpy as np
import environment
cimport environment

from environment cimport *
from environment import fuel_models, moisture_scenarios
from libc.math cimport exp

@cython.cdivision(True)
cdef inline double div(double num, double denum):
    """Returns the division num/denum, always returning 0 if num==0"""
    if num != 0.:
        return num / denum
    else:
        return 0.

@cython.cdivision(True)
cdef RothermelResult ros_detailed(FuelModel fuel, MoistureScenario moistures, double wind, double slope):
    """Computes the Rate of Spread of a wildfire using the Rothermel model.

    :param modeltype: Either environment.STATIC_FUEL_MODEL or environment.DYNAMIC_FUEL_MODEL
    :param loads: fuel loads (t/ha) for fuel classes [1-hr, 10-hr, 100-hr, live-herbs, live-woody]
    :param savs: surface to volume ratios (m2/m3) for the five fuel classes
    :param depth: fuel depth (cm)
    :param mx_dead: value of dead fuel moisture of extinction (percent)
    :param heat_contents: heat content (kJ/kg) for the five fuel classes
    :param moistures: percent of moisture on a dry weight basis (percent) for the five fuel classes
    :param wind: midflame wind speed (km/h)
    :param slope: value of site slope (percent)
    :return: A tuple (ros, summary) where ros is the rate of spread in [m/s] and summary is a dictionary
     containing intermediate values in the computation of the rate of spread.
    """

    cdef double[5] m, w, s, h
    for i in range(5):
        m[i] = moistures.moistures[i] / 100
        w[i] = fuel.loads[i] / 10 * 0.2048
        if w[i] == 0.:
            s[i] = 99999  # this is simply to avoid dividing by 0, each time s[i] is used, w[i] appears as a factor of the expression anyway
        else:
            s[i] = fuel.savs[i] /3.281
        h[i] = fuel.heat_contents[i] * 0.429922614

    cdef double delta = fuel.depth * 0.0328084
    cdef double mx_dead = fuel.mx_dead / 100
    cdef double u = wind * 54.6806649
    slope /= 100

    assert m[3] >= 0.3, "Moisture of live herbs should be greater than 30%"

    # If model is dynamic and moisture of live herbs is below 120%, transfer some load to dead fuel
    cdef double kt, f1, f4
    if fuel.is_dynamic == DYNAMIC_FUEL_MODEL and 0.3 <= m[3] < 1.2:
        kt = (1.2 - m[3]) / 0.9
        f1 = w[0] * s[0] / 32
        f4 = w[3] * kt * s[3] / 32
        s[0] = (f1*s[0] + f4*s[3]) / (f1+f4)

        w[0] += w[3] * kt
        w[3] -= w[3] * kt

    cdef double rho_p = 32  # 513*0.0624279606 [Scott and Burgan 2005]
    cdef double st = 0.0555
    cdef double se = 0.01

    # area fractions and weights
    cdef double[5] a
    for i in range(5):
        a[i] = s[i] * w[i] / rho_p
    cdef double a_dead = a[0] + a[1] + a[2]
    cdef double a_live = a[3] + a[4]
    cdef double a_tot = a_dead + a_live

    # careful, could result in division by 0
    cdef double[5] f
    for i in range(5):
        if i < 3:
            f[i] = div(a[i], a_dead)
        else:
            f[i] = div(a[i], a_live)

    cdef double f_dead = a_dead / a_tot
    cdef double f_live = a_live / a_tot

    # net (weighted) fuel loadings
    cdef double[5] wn  # Albini 1976
    for i in range(5):
        wn[i] = w[i] * (1-st)
    cdef double wn_dead = f[0]*wn[0]+f[1]*wn[1]+f[2]*wn[2]
    cdef double wn_live = wn[3]+wn[4]  # corrects models w/ 2 live fuel classes  (undocumented)

    # weighted fuel moisture
    cdef double mf_dead = f[0]*m[0]+f[1]*m[1]+f[2]*m[2]
    cdef double mf_live = f[3]*m[3]+f[4]*m[4]

    # weighted SAV ratio
    cdef double sigma_dead = f[0]*s[0]+f[1]*s[1]+f[2]*s[2]
    cdef double sigma_live = f[3]*s[3]+f[4]*s[4]
    cdef double sigma_tot = (f_dead*sigma_dead + f_live*sigma_live)  # characteristic SAV

    # weighted heat content
    cdef double h_dead = f[0]*h[0]+f[1]*h[1]+f[2]*h[2]
    cdef double h_live = f[3]*h[3]+f[4]*h[4]

    # mean packing ratio for fuel complex
    # beta = 1/delta*(w[0]/rho_p+w[1]/rho_p+w[2]/rho_p+w[3]/rho_p+w[4]/rho_p)
    cdef double beta = (w[0]+w[1]+w[2]+w[3]+w[4]) / (delta * rho_p)

    # live fuel moisture of extinction.
    # if there is no live fuel, should be set to mx_dead (currently, would crash)
    cdef double mx_live, W, mfpd
    if w[3] + w[4] == 0:
        mx_live = mx_dead
    else:
        W = ((w[0]*exp(-138/s[0]) + w[1]*exp(-138/s[1]) + w[2]*exp(-138/s[2])) /
             (w[3]*exp(-500/s[3])+w[4]*exp(-500/s[4])))

        mfpd = ((w[0]*m[0]*exp(-138/s[0]) + w[1]*m[1]*exp(-138/s[1]) + w[2]*m[2]*exp(-138/s[2])) /
                (w[0]*exp(-138/s[0]) + w[1]*exp(-138/s[1])+w[2]*exp(-138/s[2])))

        mx_live = 2.9*W*(1-mfpd/mx_dead) - 0.226

        if mx_live < mx_dead:
            mx_live = mx_dead

    # damping coefficients
    cdef double ns = 0.174*se**(-0.19)

    cdef double nm_dead = 1 - 2.59*(mf_dead/mx_dead) + 5.11*(mf_dead/mx_dead)**2 - 3.52*(mf_dead/mx_dead)**3
    cdef double nm_live = 1 - 2.59*(mf_live/mx_live) + 5.11*(mf_live/mx_live)**2 - 3.52*(mf_live/mx_live)**3

    # stop propagation if moisture beyond extinction [Andrews 2005]
    if mf_dead > mx_dead:
        nm_dead = 0
    if mf_live > mx_live:
        nm_live = 0

    # optimum packing ratio
    cdef double beta_op = 3.348*sigma_tot**(-0.8189)
    cdef double rpr = beta/beta_op  # relative packing ratio

    # maximum reaction velocity
    cdef double gamma_max = (sigma_tot**1.5) / (495 + 0.0594*sigma_tot**1.5)

    # reaction intensity
    cdef double sum_dead = wn_dead * h_dead * nm_dead * ns
    cdef double sum_live = wn_live * h_live * nm_live * ns

    # A=(6.7229*sigma.tot^0.1-7.27)  # [Rothermel 72]
    cdef double A = 133*sigma_tot**(-0.7913)  # alternate formulation from [Albini 76]
    cdef double ir_dead = gamma_max*(rpr*exp(1-rpr))**A * sum_dead  # *f_dead removed by [Frandsen 73]
    cdef double ir_live = gamma_max*(rpr*exp(1-rpr))**A * sum_live  # *f.live removed by [Frandsen 73]
    cdef double ir = ir_dead + ir_live

    # propagating flux ratio
    cdef double xi = (192+0.2595*sigma_tot)**(-1) * exp((0.792 + 0.681*sigma_tot**0.5)*(beta+0.1))

    # wind coefficient
    cdef double C = 7.47 * exp(-0.133 * sigma_tot**.55)
    cdef double B = 0.02526 * sigma_tot**.54
    cdef double E = 0.715 * exp(-3.59 * 10**(-4) * sigma_tot)
    cdef double fw = C * u**B * rpr**(-E)

    # slope coefficient
    cdef double fs = 5.275*beta**(-0.3) * slope**2

    # heat sink
    cdef double rho_b = 0.  # oven-dry bulk density
    for i in range(5):
        rho_b += w[i] / delta
    cdef double[5] qig
    for i in range(5):
        qig[i] = 250 + 1116 * m[i]
        if qig[i] < 0:
            qig[i] = 0

    cdef double eps = f_dead*(f[0]*qig[0]*exp(-138/s[0])+f[1]*qig[1]*exp(-138/s[1])+f[2]*qig[2]*exp(-138/s[2])) \
        + f_live*(f[3]*qig[3]*exp(-138/s[3])+f[4]*qig[4]*exp(-138/s[4]))

    # ROS
    cdef double r = (ir * xi * (1+fw+fs)) / (rho_b * eps)
    r = 0.3048 * r / 60  # change to m/s

    # summary = {
    #     'ROS [m/min]': r,
    #     'Wind Factor': fw,
    #     'Slope Factor': fs,
    #     # Factor applied to (1 + slope_factor + wind_factor) to get the ROS
    #     'ROS multiplier': ir * xi / (rho_b * eps) * 0.3048,
    #     # Equivalent slope gives the strength of the wind that would give a similar effect to the one of the slope [Lopes 02]
    #     'Equivalent slope [km/h]': (fs / C * rpr**E)**(1/B) / 54.6806649,  # last factor is to transform output in km/h
    #     "Characteristic dead fuel moisture [%]": mf_dead * 100,
    #     "Characteristic live fuel moisture [%]": mf_live * 100,
    #     "Live fuel moisture of extinction [%]": mx_live * 100,
    #     "Characteristic SAV [m2/m3]": sigma_tot*3.281,
    #     "Bulk density [kg/m3]": rho_b * 16.0184634,
    #     "Packing ratio [dimensionless]": beta,
    #     "Relative packing ratio [dimensionless]": rpr,
    #     "Dead fuel Reaction intensity [kW/m2]": ir_dead * 0.1893,
    #     "Live fuel Reaction intensity [kW/m2]": ir_live * 0.1893,
    #     "Reaction intensity [kW/m2]": ir * 0.1893,
    #     "Heat source [kW/m2]": ir*xi*(1+fw+fs)*0.1893,
    #     "Heat sink [kJ/m3]": rho_b * eps * 37.258994580781
    # }

    # Equivalent slope gives the strength of the wind that would give a similar effect to the one of the slope [Lopes 02]
    cdef double equivalent_slope=(fs / C * rpr**E)**(1/B) / 54.6806649  # last factor is to transform output in km/h
    result = RothermelResult(r, fw, fs, equivalent_slope)
    return result

cdef class RothermelResult:
    def __cinit__(self, double ros, double wind_factor, double slope_factor, double equivalent_slope):
        self.ros = ros
        self.wind_factor = wind_factor
        self.slope_factor = slope_factor
        self.equivalent_slope = equivalent_slope

RothermelSummary = namedtuple('RothermelSummary', ['ros', 'wind_factor', 'slope_factor', 'equivalent_slope'])


def ros(fuel_type, moisture_scenario, wind, slope):
    """Computes the Rate of Spread of a wildfire using the Rothermel model for known fuel and moisture scenarios.

    :param fuel_type: A string identifying the fuel type. The fuel type should be present in the standard fuel type
    of the environment module.
    :param moisture_scenario: A string identifying the moisture scenario. The scenario should be present in the standard
    moisture scenarii of the environment module.
    :param wind: Local wind in [km/h]
    :param slope: Local slope in percents
    :return: A tuple (ros, summary) where ros is the rate of spread in [m/s] and summary is a dictionary
     containing intermediate values in the computation of the rate of spread.
    """

    assert fuel_type in fuel_models, "Unknown fuel model: {}".format(fuel_type)
    assert moisture_scenario in moisture_scenarios, 'Unknown moisture scenario: {}'.format(moisture_scenario)
    f = fuel_models[fuel_type]
    moistures = moisture_scenarios[moisture_scenario].moistures
    #return ros_detailed(f.is_dynamic, f.loads, f.savs, f.depth, f.mx_dead, f.heat_contents, moistures, wind, slope)
    return ros_detailed(f, moisture_scenarios[moisture_scenario], wind, slope)


if __name__ == '__main__':
    #ros_detailed(environment.DYNAMIC_FUEL_MODEL, [2., 1., 0.5, 3., 8.], [5600., 358., 98., 6200., 8000.], 50., 30., [18622., 18622., 18622., 19500., 20000.], [7., 8., 9., 40., 60.], 5., 10.)
    print(ros('SH6', 'D2L2', 10, 0.))

    import timeit

    print(timeit.timeit('ros_detailed(environment.DYNAMIC_FUEL_MODEL, [2., 1., 0.5, 3., 8.], [5600., 358., 98., 6200., 8000.], 50., 30., [18622., 18622., 18622., 19500., 20000.], [7., 8., 9., 40., 60.], 5., 10.)',
                        'from rothermel import ros_detailed\nimport environment', number=10000)/10000)
    print(timeit.timeit('ros("SH4", "D1L1", 10, 30)', 'from rothermel import ros', number=10000)/10000)




