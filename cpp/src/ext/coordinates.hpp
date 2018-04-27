/* Copyright (c) 2017, CNRS-LAAS
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#ifndef PLANNING_CPP_COORDINATE_SYSTEMS_H
#define PLANNING_CPP_COORDINATE_SYSTEMS_H

#include <cmath>


namespace SAOP {

    namespace ext {

        /* Copied from DUNE source code */

        //! Semi-major axis.
        static const double c_wgs84_a = 6378137.0;
        //! Semi-minor axis.
        static const double c_wgs84_b = 6356752.3142;
        //! First eccentricity squared.
        static const double c_wgs84_e2 = 0.00669437999013;
        //! Second (prime) eccentricity squared.
        static const double c_wgs84_ep2 = 0.00673949674228;
        //! Flattening.
        static const double c_wgs84_f = 0.0033528106647475;

        static void toWGS84(double north, double east, int zone, bool in_north_hem, double* lat, double* lon) {
            double c_k0 = 0.9996;
            double c_ref_easting = 500000;

            double b = c_wgs84_b;
            double eSquared = c_wgs84_e2;
            double e2Squared = c_wgs84_ep2;
            double tn;
            double ap;
            double bp;
            double cp;
            double dp;
            double ep;
            double tmd;
            double sr;
            double sn;
            double ftphi;
            double s;
            double c;
            double t;
            double eta;
            double de;
            double dlam;
            double olam;
            double dn;

            tn = (c_wgs84_a - b) / (c_wgs84_a + b);
            ap = c_wgs84_a * (1.0 - tn + 5.0 * ((tn * tn) - (tn * tn * tn)) / 4.0 + 81.0 *
                                                                                    ((tn * tn * tn * tn) - (tn * tn * tn * tn * tn)) / 64.0);
            bp = 3.0 * c_wgs84_a * (tn - (tn * tn) + 7.0 * ((tn * tn * tn)
                                                            - (tn * tn * tn * tn)) / 8.0 + 55.0 * (tn * tn * tn * tn * tn) / 64.0) / 2.0;
            cp = 15.0 * c_wgs84_a * ((tn * tn) - (tn * tn * tn) + 3.0 * ((tn * tn * tn * tn)
                                                                         - (tn * tn * tn * tn * tn)) / 4.0) / 16.0;
            dp = 35.0 * c_wgs84_a * ((tn * tn * tn) - (tn * tn * tn * tn) + 11.0
                                                                            * (tn * tn * tn * tn * tn) / 16.0) / 48.0;
            ep = 315.0 * c_wgs84_a * ((tn * tn * tn * tn) - (tn * tn * tn * tn * tn)) / 512.0;

            if(!in_north_hem)
                north -= 10000000.0;

            tmd = north / c_k0;

            sr = c_wgs84_a * (1.0 - eSquared);

            ftphi = tmd / sr;

            double t10, t11, t14, t15;
            for (int i = 0; i < 5; i++) {
                t10 = (ap * ftphi) - (bp * std::sin(2.0 * ftphi)) + (cp * std::sin(4.0 * ftphi)) - (dp * std::sin(6.0 * ftphi)) + (ep * std::sin(8.0 * ftphi));
                dn = std::sqrt(1.0 - eSquared * (std::sin(ftphi) * std::sin(ftphi)));
                sr = c_wgs84_a * (1.0 - eSquared) / (dn * dn * dn);
                ftphi = ftphi + (tmd - t10) / sr;
            }
            dn = std::sqrt(1.0 - eSquared * (std::sin(ftphi) * std::sin(ftphi)));
            sr = c_wgs84_a * (1.0 - eSquared) / (dn * dn * dn);
            sn = c_wgs84_a / std::sqrt(1.0 - eSquared * (std::sin(ftphi) * std::sin(ftphi)));

            s = std::sin(ftphi);
            c = std::cos(ftphi);
            t = s / c;
            eta = e2Squared * (c * c);
            de = east - c_ref_easting;
            t10 = t / (2.0 * sr * sn * (c_k0 * c_k0));
            t11 = t * (5.0 + 3.0 * (t * t) + eta - 4.0 * (eta * eta) - 9.0 * (t * t)
                                                                       * eta) / (24.0 * sr * (sn * sn * sn) * (c_k0 * c_k0 * c_k0 * c_k0));
            *lat = ftphi - (de * de) * t10 + (de * de * de * de) * t11;
            t14 = 1.0 / (sn * c * c_k0);
            t15 = (1.0 + 2.0 * (t * t) + eta) / (6 * (sn * sn * sn) * c
                                                 * (c_k0 * c_k0 * c_k0));
            dlam = de * t14 - (de * de * de) * t15;
            olam = (zone * 6 - 183.0) * M_PI/180;
            *lon = olam + dlam;
        }

        static void fromWGS84(double lat, double lon, double* north, double* east, int* zone, bool* in_north_hem) {
            double c_k0 = 0.9996; // scale on central meridian
            double c_ref_easting = 500000; // false easting

            double ref_lon;
            double hemi_northing;

            ref_lon = std::floor((lon * 180 / M_PI) / 6) * 6 + 3;
            // UTM zone
            *zone = (int)std::floor(ref_lon / 6) + 31;

            ref_lon *= M_PI / 180;

            *in_north_hem = (lat > 0);

            if(lat < 0)
                hemi_northing = 10000000.0;
            else
                hemi_northing = 0.0;

            // Equations parameters
            double eqn_n = c_wgs84_a / std::sqrt(1 - c_wgs84_e2 * (std::sin(lat) * std::sin(lat)));
            // eqn_n: radius of curvature of the earth perpendicular to meridian plane
            double eqn_t = std::tan(lat) * std::tan(lat);
            double eqn_c = ((c_wgs84_e2) / (1 - c_wgs84_e2)) * std::cos(lat) * std::cos(lat);
            double eqn_a = (lon - ref_lon) * std::cos(lat);

            // M: true distance along the central meridian from the equator to lat
            double eqn_m = c_wgs84_a * ((1 - c_wgs84_e2 / 4 - 3 * (c_wgs84_e2 * c_wgs84_e2) / 64
                                         - 5 * (c_wgs84_e2 * c_wgs84_e2 * c_wgs84_e2) / 256) * lat
                                        - (3 * c_wgs84_e2 / 8 + 3 * (c_wgs84_e2 * c_wgs84_e2) / 32 + 45
                                                                                                     * (c_wgs84_e2 * c_wgs84_e2 * c_wgs84_e2) / 1024) * std::sin(2 * lat)
                                        + (15 * (c_wgs84_e2 * c_wgs84_e2) / 256 + 45 * (c_wgs84_e2 * c_wgs84_e2 * c_wgs84_e2) / 1024)
                                          * std::sin(4 * lat) - (35 * (c_wgs84_e2 * c_wgs84_e2 * c_wgs84_e2) / 3072) * std::sin(6 * lat));

            // easting
            *east = c_ref_easting + c_k0 * eqn_n * (eqn_a + (1 - eqn_t + eqn_c) * (eqn_a * eqn_a * eqn_a) / 6
                                                    + (5 - 18 * eqn_t + (eqn_t * eqn_t) + 72 * eqn_c - 58 * c_wgs84_ep2) * (eqn_a * eqn_a * eqn_a * eqn_a * eqn_a) / 120);

            // northing
            *north = hemi_northing + c_k0 * eqn_m + c_k0 * eqn_n * std::tan(lat) * ((eqn_a * eqn_a) / 2 + (5 - eqn_t + 9 * eqn_c + 4 * (eqn_c * eqn_c))
                                                                                                          * (eqn_a * eqn_a * eqn_a * eqn_a) / 24 + (61 - 58 * eqn_t + (eqn_t * eqn_t) + 600 * eqn_c - 330 * c_wgs84_ep2)
                                                                                                                                                   * (eqn_a * eqn_a * eqn_a * eqn_a * eqn_a * eqn_a) / 720);
        }

    }
}
#endif //PLANNING_CPP_COORDINATE_SYSTEMS_H
