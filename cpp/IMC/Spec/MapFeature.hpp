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

#ifndef IMC_MAPFEATURE_HPP_INCLUDED_
#define IMC_MAPFEATURE_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <ostream>
#include <string>
#include <vector>

// IMC headers.
#include "../Base/Config.hpp"
#include "../Base/Message.hpp"
#include "../Base/InlineMessage.hpp"
#include "../Base/MessageList.hpp"
#include "../Base/JSON.hpp"
#include "../Base/Serialization.hpp"
#include "../Spec/Enumerations.hpp"
#include "../Spec/Bitfields.hpp"
#include "../Spec/MapPoint.hpp"

namespace IMC
{
  //! Map Feature.
  class MapFeature: public Message
  {
  public:
    //! FeatureType.
    enum FeatureTypeEnum
    {
      //! Point of Interest.
      FTYPE_POI = 0,
      //! Filled Polygon.
      FTYPE_FILLEDPOLY = 1,
      //! Countoured Polygon.
      FTYPE_CONTOUREDPOLY = 2,
      //! Line.
      FTYPE_LINE = 3,
      //! Transponder.
      FTYPE_TRANSPONDER = 4,
      //! Start Location.
      FTYPE_STARTLOC = 5,
      //! Home Reference.
      FTYPE_HOMEREF = 6
    };

    //! Identifier.
    std::string id;
    //! FeatureType.
    uint8_t feature_type;
    //! RedComponent.
    uint8_t rgb_red;
    //! GreenComponent.
    uint8_t rgb_green;
    //! BlueComponent.
    uint8_t rgb_blue;
    //! Feature.
    MessageList<MapPoint> feature;

    static uint16_t
    getIdStatic(void)
    {
      return 603;
    }

    static MapFeature*
    cast(Message* msg__)
    {
      return (MapFeature*)msg__;
    }

    MapFeature(void)
    {
      m_header.mgid = MapFeature::getIdStatic();
      clear();
      feature.setParent(this);
    }

    MapFeature*
    clone(void) const
    {
      return new MapFeature(*this);
    }

    void
    clear(void)
    {
      id.clear();
      feature_type = 0;
      rgb_red = 0;
      rgb_green = 0;
      rgb_blue = 0;
      feature.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::MapFeature& other__ = static_cast<const MapFeature&>(msg__);
      if (id != other__.id) return false;
      if (feature_type != other__.feature_type) return false;
      if (rgb_red != other__.rgb_red) return false;
      if (rgb_green != other__.rgb_green) return false;
      if (rgb_blue != other__.rgb_blue) return false;
      if (feature != other__.feature) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(id, ptr__);
      ptr__ += IMC::serialize(feature_type, ptr__);
      ptr__ += IMC::serialize(rgb_red, ptr__);
      ptr__ += IMC::serialize(rgb_green, ptr__);
      ptr__ += IMC::serialize(rgb_blue, ptr__);
      ptr__ += feature.serialize(ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(id, bfr__, size__);
      bfr__ += IMC::deserialize(feature_type, bfr__, size__);
      bfr__ += IMC::deserialize(rgb_red, bfr__, size__);
      bfr__ += IMC::deserialize(rgb_green, bfr__, size__);
      bfr__ += IMC::deserialize(rgb_blue, bfr__, size__);
      bfr__ += feature.deserialize(bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(id, bfr__, size__);
      bfr__ += IMC::deserialize(feature_type, bfr__, size__);
      bfr__ += IMC::deserialize(rgb_red, bfr__, size__);
      bfr__ += IMC::deserialize(rgb_green, bfr__, size__);
      bfr__ += IMC::deserialize(rgb_blue, bfr__, size__);
      bfr__ += feature.reverseDeserialize(bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return MapFeature::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "MapFeature";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 4;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(id) + feature.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "id", id, nindent__);
      IMC::toJSON(os__, "feature_type", feature_type, nindent__);
      IMC::toJSON(os__, "rgb_red", rgb_red, nindent__);
      IMC::toJSON(os__, "rgb_green", rgb_green, nindent__);
      IMC::toJSON(os__, "rgb_blue", rgb_blue, nindent__);
      feature.toJSON(os__, "feature", nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      feature.setTimeStamp(value__);
    }

    void
    setSourceNested(uint16_t value__)
    {
      feature.setSource(value__);
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      feature.setSourceEntity(value__);
    }

    void
    setDestinationNested(uint16_t value__)
    {
      feature.setDestination(value__);
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      feature.setDestinationEntity(value__);
    }
  };
}

#endif
