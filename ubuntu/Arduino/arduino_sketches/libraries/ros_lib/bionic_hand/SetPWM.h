#ifndef _ROS_bionic_hand_SetPWM_h
#define _ROS_bionic_hand_SetPWM_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace bionic_hand
{

  class SetPWM : public ros::Msg
  {
    public:
      typedef uint8_t _id_type;
      _id_type id;
      typedef int32_t _pwm_type;
      _pwm_type pwm;

    SetPWM():
      id(0),
      pwm(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id);
      union {
        int32_t real;
        uint32_t base;
      } u_pwm;
      u_pwm.real = this->pwm;
      *(outbuffer + offset + 0) = (u_pwm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pwm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pwm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pwm.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pwm);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->id);
      union {
        int32_t real;
        uint32_t base;
      } u_pwm;
      u_pwm.base = 0;
      u_pwm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pwm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pwm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pwm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pwm = u_pwm.real;
      offset += sizeof(this->pwm);
     return offset;
    }

    virtual const char * getType() override { return "bionic_hand/SetPWM"; };
    virtual const char * getMD5() override { return "7dcb9d04de9857b62a1d688544986dcd"; };

  };

}
#endif
