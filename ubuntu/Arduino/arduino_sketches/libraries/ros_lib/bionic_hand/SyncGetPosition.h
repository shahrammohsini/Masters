#ifndef _ROS_SERVICE_SyncGetPosition_h
#define _ROS_SERVICE_SyncGetPosition_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace bionic_hand
{

static const char SYNCGETPOSITION[] = "bionic_hand/SyncGetPosition";

  class SyncGetPositionRequest : public ros::Msg
  {
    public:
      typedef uint8_t _id1_type;
      _id1_type id1;
      typedef uint8_t _id2_type;
      _id2_type id2;

    SyncGetPositionRequest():
      id1(0),
      id2(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->id1 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id1);
      *(outbuffer + offset + 0) = (this->id2 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->id1 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->id1);
      this->id2 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->id2);
     return offset;
    }

    virtual const char * getType() override { return SYNCGETPOSITION; };
    virtual const char * getMD5() override { return "533dfb55d158e8e581d9e0f578d53185"; };

  };

  class SyncGetPositionResponse : public ros::Msg
  {
    public:
      typedef int32_t _position1_type;
      _position1_type position1;
      typedef int32_t _position2_type;
      _position2_type position2;

    SyncGetPositionResponse():
      position1(0),
      position2(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_position1;
      u_position1.real = this->position1;
      *(outbuffer + offset + 0) = (u_position1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position1);
      union {
        int32_t real;
        uint32_t base;
      } u_position2;
      u_position2.real = this->position2;
      *(outbuffer + offset + 0) = (u_position2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_position1;
      u_position1.base = 0;
      u_position1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position1 = u_position1.real;
      offset += sizeof(this->position1);
      union {
        int32_t real;
        uint32_t base;
      } u_position2;
      u_position2.base = 0;
      u_position2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position2 = u_position2.real;
      offset += sizeof(this->position2);
     return offset;
    }

    virtual const char * getType() override { return SYNCGETPOSITION; };
    virtual const char * getMD5() override { return "a1a390f5c43969c5caaab57cf13dc84b"; };

  };

  class SyncGetPosition {
    public:
    typedef SyncGetPositionRequest Request;
    typedef SyncGetPositionResponse Response;
  };

}
#endif
