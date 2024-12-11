#ifndef _ROS_bionic_hand_ControlCommands_h
#define _ROS_bionic_hand_ControlCommands_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace bionic_hand
{

  class ControlCommands : public ros::Msg
  {
    public:
      typedef int32_t _ID_type;
      _ID_type ID;
      typedef float _PWM_type;
      _PWM_type PWM;

    ControlCommands():
      ID(0),
      PWM(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_ID;
      u_ID.real = this->ID;
      *(outbuffer + offset + 0) = (u_ID.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ID.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ID.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ID.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ID);
      offset += serializeAvrFloat64(outbuffer + offset, this->PWM);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_ID;
      u_ID.base = 0;
      u_ID.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ID.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ID.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ID.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ID = u_ID.real;
      offset += sizeof(this->ID);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->PWM));
     return offset;
    }

    virtual const char * getType() override { return "bionic_hand/ControlCommands"; };
    virtual const char * getMD5() override { return "0710b5797f0353030d05fc8a9f52589b"; };

  };

}
#endif
