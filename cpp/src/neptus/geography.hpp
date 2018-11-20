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

#include <vector>

#include <gdal/ogr_spatialref.h>

#include "../core/waypoint.hpp"
#include "../ext/coordinates.hpp"

namespace SAOP {

    namespace neptus {

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

        /*Convert Lambert93 (EPSG:2154) points to WGS84 (lat, lon) coordinates*/
        static std::vector<Waypoint3d> lambert93_to_wgs84(std::vector<Waypoint3d> lambert93_wp) {
            OGRSpatialReference wgs84_gcs;
            OGRSpatialReference lambert93_pcs;
            wgs84_gcs.importFromEPSG(4171); // RGF93 (EPSG:4171) is in practice equal to WGS84 (EPSG:4326)
            lambert93_pcs.importFromEPSG(2154);

            auto poCT = OGRCreateCoordinateTransformation(&lambert93_pcs, &wgs84_gcs);
            ASSERT(poCT); // If the conversion cannot take place, poCT is null

            double xx;
            double yy;

            auto wgs84_wp = std::vector<Waypoint3d>();

            for (auto it = lambert93_wp.begin(); it < lambert93_wp.end();++it) {
                xx = it->x;
                yy = it->y;
//                std::cout << it->x << " " << it->y << std::endl;
//                std::cout << xx << " " << yy << std::endl;

                poCT->Transform(1, &xx, &yy); // Again Transform must succed

                wgs84_wp.emplace_back(Waypoint3d(xx/180*M_PI, yy/180*M_PI, it->z, it->dir));
                xx = 0;
                yy = 0;
            }

            OGRCoordinateTransformation::DestroyCT(poCT);
            return wgs84_wp;
        }

        /*Convert Lambert93 (EPSG:2154) points to WGS84 (lat, lon) coordinates*/
        static std::vector<Waypoint3d> wgs84_to_lambert93(std::vector<Waypoint3d> wgs84_wp) {
            OGRSpatialReference wgs84_gcs;
            OGRSpatialReference lambert93_pcs;
            wgs84_gcs.importFromEPSG(4171); // RGF93 (EPSG:4171) is in practice equal to WGS84 (EPSG:4326)
            lambert93_pcs.importFromEPSG(2154);

            auto poCT = OGRCreateCoordinateTransformation(&wgs84_gcs, &lambert93_pcs);
            ASSERT(poCT); // If the conversion cannot take place, poCT is null

            double xx;
            double yy;

            auto lambert93 = std::vector<Waypoint3d>();

            for (auto it = wgs84_wp.begin(); it < wgs84_wp.end();++it) {
                xx = it->x;
                yy = it->y;

                poCT->Transform(1, &xx, &yy); // Again Transform must succed

                lambert93.emplace_back(Waypoint3d(xx/180*M_PI, yy/180*M_PI, it->z, it->dir));
                xx = 0;
                yy = 0;
            }

            OGRCoordinateTransformation::DestroyCT(poCT);
            return lambert93;
        }
    }
}


#endif //PLANNING_CPP_GEOGRAPHY_HPP
