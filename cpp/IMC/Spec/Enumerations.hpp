//***************************************************************************
// Copyright 2017 OceanScan - Marine Systems & Technology, Lda.             *
//***************************************************************************
// Licensed under the Apache License, Version 2.0 (the "License");          *
// you may not use this file except in compliance with the License.         *
// You may obtain a copy of the License at                                  *
//                                                                          *
// http://www.apache.org/licenses/LICENSE-2.0                               *
//                                                                          *
// Unless required by applicable law or agreed to in writing, software      *
// distributed under the License is distributed on an "AS IS" BASIS,        *
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
// See the License for the specific language governing permissions and      *
// limitations under the License.                                           *
//***************************************************************************
// Author: Ricardo Martins                                                  *
//***************************************************************************
// Automatically generated.                                                 *
//***************************************************************************
// IMC XML MD5: 4d8734a1111656aac56f803acdc90c22                            *
//***************************************************************************

#ifndef IMC_ENUMERATIONS_HPP_INCLUDED_
#define IMC_ENUMERATIONS_HPP_INCLUDED_

namespace IMC
{
  //! Boolean Value.
  enum Boolean
  {
    //! False.
    BOOL_FALSE = 0,
    //! True.
    BOOL_TRUE = 1
  };

  //! Controlled Mode.
  enum ControlledMode
  {
    //! Relinquish / Handoff Control.
    CTLMD_RELINQUISH_HANDOFF_CTL = 0,
    //! Request Control.
    CTLMD_REQUEST_CTL = 1,
    //! Override Control.
    CTLMD_OVERRIDE_CTL = 2
  };

  //! Speed Units.
  enum SpeedUnits
  {
    //! Meters per second.
    SUNITS_METERS_PS = 0,
    //! RPM.
    SUNITS_RPM = 1,
    //! Percentage.
    SUNITS_PERCENTAGE = 2
  };

  //! System Type.
  enum SystemType
  {
    //! CCU.
    SYSTEMTYPE_CCU = 0,
    //! Human-portable Sensor.
    SYSTEMTYPE_HUMANSENSOR = 1,
    //! UUV.
    SYSTEMTYPE_UUV = 2,
    //! USV.
    SYSTEMTYPE_USV = 3,
    //! UAV.
    SYSTEMTYPE_UAV = 4,
    //! UGV.
    SYSTEMTYPE_UGV = 5,
    //! Static sensor.
    SYSTEMTYPE_STATICSENSOR = 6,
    //! Mobile sensor.
    SYSTEMTYPE_MOBILESENSOR = 7,
    //! Wireless Sensor Network.
    SYSTEMTYPE_WSN = 8
  };

  //! Z Units.
  enum ZUnits
  {
    //! None.
    Z_NONE = 0,
    //! Depth.
    Z_DEPTH = 1,
    //! Altitude.
    Z_ALTITUDE = 2,
    //! Height.
    Z_HEIGHT = 3
  };

  //! RSSI Units.
  enum RSSIUnits
  {
    //! Decibel.
    RSSIUNITS_dB = 0,
    //! Percentage.
    RSSIUNITS_PERCENTAGE = 1
  };

  //! UAV Type.
  enum UAVType
  {
    //! Fixed-Wing.
    UAVTYPE_FIXEDWING = 0,
    //! Copter.
    UAVTYPE_COPTER = 1,
    //! Vtol.
    UAVTYPE_VTOL = 2
  };
}

#endif
