/* Copyright (c) 2017-2018, CNRS-LAAS
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
#ifndef PLANNING_CPP_GEOGRAPHY_HPP
#define PLANNING_CPP_GEOGRAPHY_HPP

#include <stdexcept>
#include <vector>

#include <gdal/ogr_spatialref.h>

#include "../core/waypoint.hpp"
#include "../ext/coordinates.hpp"

namespace SAOP {

    namespace neptus {

        static const int EPSG_RGF93_LAMBERT93 = 2154;
        static const int EPSG_RGF93 = 4171; // Equivalent in practice to WGS84
        static const int EPSG_ETRS89_LAEA = 3035;
        static const int EPSG_ETRS89 = 4258; // Equivalent in practice to WGS84
        static const int EPSG_WGS84_UTM29N = 32629;
        static const int EPSG_WGS84 = 4326; // Equivalent in practice to WGS84

        static std::vector<Waypoint3d> WGS84_waypoints(std::vector<Waypoint3d> waypoints,
                                                       Position origin, int utm_zone = 29,
                                                       bool northern_hemisphere = true) {
            auto wgs84_wp = std::vector<Waypoint3d>();
            auto wp0 = waypoints[0];

            for (auto& wp: waypoints) {
                double lat;
                double lon;
                SAOP::ext::toWGS84(wp.y - wp0.y + origin.y, wp.x - wp0.x + origin.x,
                                   utm_zone, northern_hemisphere, &lat, &lon);
                wgs84_wp.emplace_back(Waypoint3d(lon, lat, wp.z, wp.dir));
            }

            return wgs84_wp;
        }

        static std::vector<Waypoint3d> transform_coordinates(const std::vector<Waypoint3d>& from_wp, int from, int to) {
            OGRSpatialReference dst_sr;
            OGRSpatialReference src_sr;
            src_sr.importFromEPSG(from);
            dst_sr.importFromEPSG(to);

            auto poCT = OGRCreateCoordinateTransformation(&src_sr, &dst_sr);
            if (poCT == nullptr) {
                // If the conversion cannot take place, poCT is null
                throw std::invalid_argument("The conversion cannot take between the given coordinate systems");
            }

            double xx;
            double yy;
            auto to_wp = std::vector<Waypoint3d>();

            for (auto it = from_wp.begin(); it < from_wp.end(); ++it) {
                xx = it->x;
                yy = it->y;
                poCT->Transform(1, &xx, &yy); // Again Transform must succed

                to_wp.emplace_back(Waypoint3d(xx / 180 * M_PI, yy / 180 * M_PI, it->z, it->dir));
                xx = 0;
                yy = 0;
            }
            delete poCT;

            return to_wp;
        }

        /*Convert Lambert93 points to WGS84 (lat, lon) coordinates*/
        static std::vector<Waypoint3d> lambert93_to_world_coordinates(std::vector<Waypoint3d> lambert93_wp) {
            return transform_coordinates(lambert93_wp, EPSG_RGF93_LAMBERT93, EPSG_RGF93);
        }

        /*Convert LAEA points to ETRS89 (lat, lon) coordinates*/
        static std::vector<Waypoint3d> laea_to_world_coordinates(const std::vector<Waypoint3d>& laea_wp) {
            return transform_coordinates(laea_wp, EPSG_ETRS89_LAEA, EPSG_ETRS89);
        }

        static std::vector<Waypoint3d> utm29n_to_world_coordinates(const std::vector<Waypoint3d>& utm29n_wp) {
            return transform_coordinates(utm29n_wp, EPSG_WGS84_UTM29N, EPSG_WGS84);
        }

        /*Convert LAEA points to ETRS89 (lat, lon) coordinates*/
        static Position3d laea_to_world_coordinates(Position3d laea_position) {
            return transform_coordinates(
                    std::vector<Waypoint3d>({Waypoint3d(laea_position.x, laea_position.y, laea_position.z, 0.)}),
                    EPSG_ETRS89_LAEA, EPSG_ETRS89)[0].as_point();
        }

        static Position3d utm29n_to_world_coordinates(Position3d utm29n_position) {
            return transform_coordinates(
                    std::vector<Waypoint3d>({Waypoint3d(utm29n_position.x, utm29n_position.y, utm29n_position.z, 0.)}),
                    EPSG_WGS84_UTM29N, EPSG_WGS84)[0].as_point();
        }

        /*Convert Lambert93 points to WGS84 (lat, lon) coordinates*/
        static Position3d lambert93_to_world_coordinates(Position3d laea_position) {
            return transform_coordinates(
                    std::vector<Waypoint3d>({Waypoint3d(laea_position.x, laea_position.y, laea_position.z, 0.)}),
                    EPSG_RGF93_LAMBERT93, EPSG_RGF93)[0].as_point();
        }
    }
}


#endif //PLANNING_CPP_GEOGRAPHY_HPP
