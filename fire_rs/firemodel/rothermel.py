import numpy as np
from numpy import exp
from collections import namedtuple

import environment
from environment import fuel_models, moisture_scenarios


def ros_detailed(
        modeltype,
        loads,
        savs,
        depth,
        mx_dead,
        heat_contents,
        moistures,
        wind,
        slope):
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

    assert len(loads) == len(savs) and len(loads) == len(heat_contents) and len(loads) == len(moistures)

    m = np.array(moistures) / 100
    w = np.array(loads) / 10 * 0.2048
    s = np.array(savs) / 3.281
    for i in range(5):
        if w[i] == 0.:
            s[i] = 999999  # this is simply to avoid dividing by 0, each time s[i] is used, m[i] appears as a factor of the expression anyway
    h = np.array(heat_contents) * 0.429922614
    delta = depth * 0.0328084
    mx_dead /= 100
    u = wind * 54.6806649
    slope /= 100

    assert m[3] >= 0.3, "Moisture of live herbs should be greater than 30%"

    # If model is dynamic and moisture of live herbs is below 120%, transfer some load to dead fuel
    if modeltype == environment.DYNAMIC_FUEL_MODEL and 0.3 <= m[3] < 1.2:
        kt = (1.2 - m[3]) / 0.9
        f1 = w[0] * s[0] / 32
        f4 = w[3] * kt * s[3] / 32
        s[0] = (f1*s[0] + f4*s[3]) / (f1+f4)

        w[0] += w[3] * kt
        w[3] -= w[3] * kt

    rho_p = 32  # 513*0.0624279606 [Scott and Burgan 2005]
    st = 0.0555
    se = 0.01

    # area fractions and weights
    a = np.multiply(s, w) / rho_p
    a_dead = a[0] + a[1] + a[2]
    a_live = a[3] + a[4]
    a_tot = a_dead + a_live

    # careful, could result in division by 0
    f = np.array([a[0]/a_dead, a[1]/a_dead, a[2]/a_dead, a[3]/a_live, a[4]/a_live])

    f_dead = a_dead / a_tot
    f_live = a_live / a_tot

    # net (weighted) fuel loadings
    wn = w*(1-st)  # Albini 1976
    wn_dead = f[0]*wn[0]+f[1]*wn[1]+f[2]*wn[2]
    wn_live = wn[3]+wn[4]  # corrects models w/ 2 live fuel classes  (undocumented)

    # weighted fuel moisture
    mf_dead = f[0]*m[0]+f[1]*m[1]+f[2]*m[2]
    mf_live = f[3]*m[3]+f[4]*m[4]

    # weighted SAV ratio
    sigma_dead = f[0]*s[0]+f[1]*s[1]+f[2]*s[2]
    sigma_live = f[3]*s[3]+f[4]*s[4]
    sigma_tot = (f_dead*sigma_dead + f_live*sigma_live)  # characteristic SAV

    # weighted heat content
    h_dead = f[0]*h[0]+f[1]*h[1]+f[2]*h[2]
    h_live = f[3]*h[3]+f[4]*h[4]

    # mean packing ratio for fuel complex
    # beta = 1/delta*(w[0]/rho_p+w[1]/rho_p+w[2]/rho_p+w[3]/rho_p+w[4]/rho_p)
    beta = 1 / delta * (w / rho_p).sum()

    # live fuel moisture of extinction.
    # if there is no live fuel, should be set to mx_dead (currently, would crash)
    W = ((w[0]*exp(-138/s[0]) + w[1]*exp(-138/s[1]) + w[2]*exp(-138/s[2])) /
         (w[3]*exp(-500/s[3])+w[4]*exp(-500/s[4])))

    mfpd = ((w[0]*m[0]*exp(-138/s[0]) + w[1]*m[1]*exp(-138/s[1]) + w[2]*m[2]*exp(-138/s[2])) /
            (w[0]*exp(-138/s[0]) + w[1]*exp(-138/s[1])+w[2]*exp(-138/s[2])))

    mx_live = 2.9*W*(1-mfpd/mx_dead) - 0.226

    if mx_live < mx_dead:
        mx_live = mx_dead

    # damping coefficients
    ns = 0.174*se**(-0.19)

    nm_dead = 1 - 2.59*(mf_dead/mx_dead) + 5.11*(mf_dead/mx_dead)**2 - 3.52*(mf_dead/mx_dead)**3
    nm_live = 1 - 2.59*(mf_live/mx_live) + 5.11*(mf_live/mx_live)**2 - 3.52*(mf_live/mx_live)**3

    # stop propagation if moisture beyond extinction [Andrews 2005]
    if mf_dead > mx_dead:
        nm_dead = 0
    if mf_live > mx_live:
        nm_live = 0

    # optimum packing ratio
    beta_op = 3.348*sigma_tot**(-0.8189)
    rpr = beta/beta_op  # relative packing ratio

    # maximum reaction velocity
    gamma_max = (sigma_tot**1.5) / (495 + 0.0594*sigma_tot**1.5)

    # reaction intensity
    sum_dead = wn_dead * h_dead * nm_dead * ns
    sum_live = wn_live * h_live * nm_live * ns

    # A=(6.7229*sigma.tot^0.1-7.27)  # [Rothermel 72]
    A = 133*sigma_tot**(-0.7913)  # alternate formulation from [Albini 76]
    ir_dead = gamma_max*(rpr*exp(1-rpr))**A * sum_dead  # *f_dead removed by [Frandsen 73]
    ir_live = gamma_max*(rpr*exp(1-rpr))**A * sum_live  # *f.live removed by [Frandsen 73]
    ir = ir_dead + ir_live

    # propagating flux ratio
    xi = (192+0.2595*sigma_tot)**(-1) * exp((0.792 + 0.681*sigma_tot**0.5)*(beta+0.1))

    # wind coefficient
    C = 7.47 * exp(-0.133 * sigma_tot**.55)
    B = 0.02526 * sigma_tot**.54
    E = 0.715 * exp(-3.59 * 10**(-4) * sigma_tot)
    fw = C * u**B * rpr**(-E)

    # slope coefficient
    fs = 5.275*beta**(-0.3) * slope**2

    # heat sink
    rho_b = (1/delta) * w.sum()  # oven-dry bulk density
    qig = 250 + 1116*m
    qig = np.maximum(qig, np.zeros(5))  # ensure qig is always >= 0

    eps = f_dead*(f[0]*qig[0]*exp(-138/s[0])+f[1]*qig[1]*exp(-138/s[1])+f[2]*qig[2]*exp(-138/s[2])) \
        + f_live*(f[3]*qig[3]*exp(-138/s[3])+f[4]*qig[4]*exp(-138/s[4]))

    # ROS
    r = (ir * xi * (1+fw+fs)) / (rho_b * eps)
    r = 0.3048 * r

    # summary = {
    #     'ROS [m/min]': r,
    #     'Wind Factor': fw,
    #     'Slope Factor': fs,
    #     # Factor applied to (1 + slope_factor + wind_factor) to get the ROS
    #     'ROS multiplier': ir * xi / (rho_b * eps) * 0.3048,
    #     # Equivalent slope gives the strength of the wind that would give a similar effect to the one of the slope [Lopes 02]
    #     'Equivalent slope [km/h]': (fs / C * rpr**E)**(1/B) / 54.6806649  # last factor is to transform output in km/h
    #     # "Characteristic dead fuel moisture [%]": mf_dead * 100,
    #     # "Characteristic live fuel moisture [%]": mf_live * 100,
    #     # "Live fuel moisture of extinction [%]": mx_live * 100,
    #     # "Characteristic SAV [m2/m3]": sigma_tot*3.281,
    #     # "Bulk density [kg/m3]": rho_b * 16.0184634,
    #     # "Packing ratio [dimensionless]": beta,
    #     # "Relative packing ratio [dimensionless]": rpr,
    #     # "Dead fuel Reaction intensity [kW/m2]": ir_dead * 0.1893,
    #     # "Live fuel Reaction intensity [kW/m2]": ir_live * 0.1893,
    #     # "Reaction intensity [kW/m2]": ir * 0.1893,
    #     # "Heat source [kW/m2]": ir*xi*(1+fw+fs)*0.1893,
    #     # "Heat sink [kJ/m3]": rho_b * eps * 37.258994580781
    # }
    summary = RothermelSummary(
        ros=r/60,  # Rate of Spread [m/s]
        wind_factor=fw,
        slope_factor=fs,
        # Equivalent slope gives the strength of the wind that would give a similar effect to the one of the slope [Lopes 02]
        equivalent_slope=(fs / C * rpr**E)**(1/B) / 54.6806649  # last factor is to transform output in km/h
    )
    return r/60, summary


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
    return ros_detailed(f.type, f.loads, f.savs, f.depth, f.mx_dead, f.heats, moistures, wind, slope)


if __name__ == '__main__':
    #ros_detailed(environment.DYNAMIC_FUEL_MODEL, [2., 1., 0.5, 3., 8.], [5600., 358., 98., 6200., 8000.], 50., 30., [18622., 18622., 18622., 19500., 20000.], [7., 8., 9., 40., 60.], 5., 10.)
    print(ros('SH6', 'D2L2', 10, 0.))

    import timeit

    print(timeit.timeit('ros_detailed(environment.DYNAMIC_FUEL_MODEL, [2., 1., 0.5, 3., 8.], [5600., 358., 98., 6200., 8000.], 50., 30., [18622., 18622., 18622., 19500., 20000.], [7., 8., 9., 40., 60.], 5., 10.)',
                        'from rothermel import ros_detailed\nimport environment', number=10000)/10000)
    print(timeit.timeit('ros("SH4", "D1L1", 10, 30)', 'from rothermel import ros', number=10000)/10000)




