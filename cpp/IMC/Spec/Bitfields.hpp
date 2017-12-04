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

#ifndef IMC_BITFIELDS_HPP_INCLUDED_
#define IMC_BITFIELDS_HPP_INCLUDED_

namespace IMC
{
  //! Control Loops Mask.
  enum CLoopsMask
  {
    //! None.
    CL_NONE = 0x00000000,
    //! Path Control.
    CL_PATH = 0x00000001,
    //! Teleoperation Control.
    CL_TELEOPERATION = 0x00000002,
    //! Altitude Control.
    CL_ALTITUDE = 0x00000004,
    //! Depth Control.
    CL_DEPTH = 0x00000008,
    //! Roll Control.
    CL_ROLL = 0x00000010,
    //! Pitch Control.
    CL_PITCH = 0x00000020,
    //! Yaw Control.
    CL_YAW = 0x00000040,
    //! Speed Control.
    CL_SPEED = 0x00000080,
    //! Yaw Rate Control.
    CL_YAW_RATE = 0x00000100,
    //! Vertical Rate Control.
    CL_VERTICAL_RATE = 0x00000200,
    //! Torque Control.
    CL_TORQUE = 0x00000400,
    //! Force Control.
    CL_FORCE = 0x00000800,
    //! Velocity Control.
    CL_VELOCITY = 0x00001000,
    //! Throttle Control.
    CL_THROTTLE = 0x00002000,
    //! Unspecified External Control.
    CL_EXTERNAL = 0x40000000,
    //! Non-overridable control.
    CL_NO_OVERRIDE = 0x80000000,
    //! All.
    CL_ALL = 0xFFFFFFFF
  };

  //! Operational Limits Mask.
  enum OpLimitsMask
  {
    //! Maximum Depth.
    OPL_MAX_DEPTH = 0x01,
    //! Minimum Altitude.
    OPL_MIN_ALT = 0x02,
    //! Maximum Altitude.
    OPL_MAX_ALT = 0x04,
    //! Minimum Speed.
    OPL_MIN_SPEED = 0x08,
    //! Maximum Speed.
    OPL_MAX_SPEED = 0x10,
    //! Maximum Vertical Rate.
    OPL_MAX_VRATE = 0x20,
    //! Operation Area.
    OPL_AREA = 0x40
  };
}

#endif
