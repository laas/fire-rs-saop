/* Copyright (c) 2018, CNRS-LAAS
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

#include "dubinswind.hpp"

namespace SAOP {

    DubinsWind::DubinsWind(const Waypoint3d& from, const Waypoint3d& to, const WindVector& constant_wind,
                           double uav_air_speed, double turn_radius) {

        wp_s = from;
        wp_e = to;
        r_min = turn_radius;
        wind_vector = constant_wind;
        air_speed = uav_air_speed;

        if (!ALMOST_EQUAL(from.z, to.z)) {
            throw DubinsWindPathNotFoundException(from, to, wind_vector, uav_air_speed,
                                                  "the trajectory is not flat");

        } else if (ALMOST_EQUAL(wind_vector.x(), 0) && ALMOST_EQUAL(wind_vector.y(), 0)) {
            // Do the regular Dubins2D procedure instead of the DubinsWind one
            d_star = 0.;
            double orig_air[3] = {from.x, from.y, from.dir};
            double dest_air[3] = {to.x, to.y, to.dir};
            auto ret = dubins_init(orig_air, dest_air, turn_radius, &air_path);
            ASSERT(ret == 0);

        } else {
//            if (from.as_point().hor_dist(to.as_point()) <= 2 * r_min) {
//                throw DubinsWindPathNotFoundException(from, to, wind_vector, uav_air_speed,
//                                                      "wp are closer than 4*r_min");
//            }

            // We only accept CSC traj types for now
            auto dubins_types = std::vector<Dubins2dPathType> {
                    Dubins2dPathType::RSR,
                    Dubins2dPathType::RSL,
                    Dubins2dPathType::LSR,
                    Dubins2dPathType::LSL,
            };
            // d* is chosen among these d values that are going to be computed
            auto dd = std::vector<double>(dubins_types.size(), std::numeric_limits<double>::infinity());
            d_star = std::numeric_limits<double>::infinity();

            // Finding d for each dubins type == finding d for each piece of G(d)
            for (size_t i = 0ul; i < dubins_types.size(); ++i) {
                // Find rdv between UAV dubins path in air frame and the target point
                DubinsPath a_path;
                a_path.type = dubins_types[i];
                dd[i] = find_d(wp_s, wp_e, wind_vector, air_speed, turn_radius, &a_path);
                if (dd[i] < d_star) {
                    air_path = a_path;
                    d_star = dd[i];
                }
            }

            if (air_path.type == -1) {
                throw DubinsWindPathNotFoundException(from, to, wind_vector, uav_air_speed, "d* not found");
            }
        }
    }

    std::vector<Waypoint3d> DubinsWind::sampled_airframe(double l_step) const {
        ASSERT(l_step > 0);

        std::vector<Waypoint3d> wps = {};

        double current_l = 0.0;
        double length = dubins_path_length(&air_path);
        while (current_l < length) {
            double q[3];
            dubins_path_sample(&air_path, current_l, q);
            wps.emplace_back(Waypoint3d{q[0], q[1], wp_s.z, q[3]});
            current_l += l_step;
        }

        return wps;
    }

    std::vector<Waypoint3d> DubinsWind::sampled(double l_step) const {
        ASSERT(l_step > 0);

        std::vector<Waypoint3d> wp_air = sampled_airframe(l_step);
        std::vector<Waypoint3d> wp_ground = {};

        double t_step = l_step / air_speed;

        double current_l = 0.0;
        double length = dubins_path_length(&air_path);

        Waypoint3d q_prev = wp_s;
        Waypoint3d p_prev = wp_s;
        while (current_l < length) {
            double q[3];
            dubins_path_sample(&air_path, current_l, q);
            // p⃗_g = p⃗_a + w⃗ ⨯ Δt
            Waypoint3d q_now{q[0], q[1], wp_s.z, q[2]};

            auto p_x = p_prev.x + (q_now.x - q_prev.x) + t_step * wind_vector.x();
            auto p_y = p_prev.y + (q_now.y - q_prev.y) + t_step * wind_vector.y();
            auto p_theta = WindVector(wind_vector.x() + air_speed * std::cos(q[2]),
                                      wind_vector.y() + air_speed * std::sin(q[2])).dir();
            Waypoint3d p_now{p_x, p_y, wp_s.z, p_theta};
            wp_ground.emplace_back(p_now);

            p_prev = p_now;
            q_prev = q_now;

            current_l += l_step;
        }

        return wp_ground;
    }

    std::pair<std::vector<Waypoint3d>, std::vector<double>> DubinsWind::sampled_with_time(double l_step) const {
        ASSERT(l_step > 0);

        std::vector<Waypoint3d> wp_air = sampled_airframe(l_step);
        std::vector<Waypoint3d> wp_ground = {};
        std::vector<double> time = {};

        double t_step = l_step / air_speed;

        double current_l = 0.0;
        double current_t = 0.0;
        double length = dubins_path_length(&air_path);

        Waypoint3d q_prev = wp_s;
        Waypoint3d p_prev = wp_s;
        while (current_l < length) {
            double q[3];
            dubins_path_sample(&air_path, current_l, q);
            // p⃗_g = p⃗_a + w⃗ ⨯ Δt
            Waypoint3d q_now{q[0], q[1], wp_s.z, q[2]};

            auto p_x = p_prev.x + (q_now.x - q_prev.x) + t_step * wind_vector.x();
            auto p_y = p_prev.y + (q_now.y - q_prev.y) + t_step * wind_vector.y();
            auto p_theta = WindVector(wind_vector.x() + air_speed * std::cos(q[2]),
                                      wind_vector.y() + air_speed * std::sin(q[2])).dir();
            Waypoint3d p_now{p_x, p_y, wp_s.z, p_theta};
            wp_ground.emplace_back(p_now);
            time.emplace_back(current_t);

            p_prev = p_now;
            q_prev = q_now;

            current_l += l_step;
            current_t += t_step;
        }

        return {wp_ground, time};
    }

    double DubinsWind::find_d(const Waypoint3d& from, const Waypoint3d& to, WindVector wind, double uav_speed,
                              double turn_radius, DubinsPath* dubins_air_conf) {
        double da = 0; // G(0) > 0 (Remark 2)
        double db = 2; // lim d→∞ (G(d)) < 0 (Remark 2)

        // G(da) is positive by definition
        // Increase db until G(db) < 0
        {
            int n_iter = 0;
            double g_db = 1;
            while (g_db > 0 && db < std::numeric_limits<double>::infinity()) {
                auto opt_g_db = G(db, from, to, wind, uav_speed, turn_radius, dubins_air_conf);
                if (opt_g_db) {
                    g_db = *opt_g_db;
                    if (std::isnan(g_db)) {
                        db = std::numeric_limits<double>::infinity();
                    }
                    if (g_db > 0) {
                        db = std::pow(db, 2);
                    }
                } else {
                    db = std::numeric_limits<double>::infinity();
                }
                ++n_iter;
            }
            std::cout << "# db iters: " << n_iter << std::endl;

            if (db >= std::numeric_limits<double>::infinity()) {
                return std::numeric_limits<double>::infinity();
            }

        }
        ASSERT((dubins_air_conf->type >= 0) && (dubins_air_conf->type <= 5));

        double max_iterations = 100;
        double epsilon = 0.001;

        double n = 0;

        while (n < max_iterations) {
            auto dc = (da + db) / 2; // Set new midpoint
            auto opt_g = G(dc, from, to, wind, uav_speed, turn_radius, dubins_air_conf);

            if (!opt_g) {
                // TODO: Path does not exist! What to do? idk :-(
                return std::numeric_limits<double>::infinity(); // Solution not found
            } else {
                auto g = *opt_g;
                if (fabs(g) < epsilon || ((db - da) / 2) < epsilon) {
                    std::cout << "# iterations: " << n << std::endl;
                    return dc;
                }
                auto opt_g_da = G(da, from, to, wind, uav_speed, turn_radius, dubins_air_conf);
                if (!opt_g_da) {
                    return std::numeric_limits<double>::infinity();
                }

                if (std::signbit(g) == std::signbit(*opt_g_da)) { da = dc; }
                else { db = dc; } // G(da) must be positive, G(db) must be negative
            }
            ++n;
        }

        return std::numeric_limits<double>::infinity(); // Solution not found
    }

    opt<double> DubinsWind::G(double d, const Waypoint3d& from, const Waypoint3d& to, WindVector wind, double uav_speed,
                              double turn_radius, DubinsPath* dubins_air_conf) {
        auto to_air = to.move(-d, wind.dir());
        auto t_vt = d / wind.speed();
        double orig_air[3] = {from.x, from.y, from.dir};
        double dest_air[3] = {to_air.x, to_air.y, to_air.dir};
        auto ret = dubins_init_with_type(orig_air, dest_air, turn_radius, dubins_air_conf, dubins_air_conf->type);
        if (ret == EDUBOK) {
            return dubins_path_length(dubins_air_conf) / uav_speed - t_vt;
        }
        return {};
    }

}