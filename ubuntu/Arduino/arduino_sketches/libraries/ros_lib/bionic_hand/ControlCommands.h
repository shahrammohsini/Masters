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
      typedef float _PWM_type;
      _PWM_type PWM;

    ControlCommands():
      PWM(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->PWM);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->PWM));
     return offset;
    }

    virtual const char * getType() override { return "bionic_hand/ControlCommands"; };
    virtual const char * getMD5() override { return "630d1348e66951f61746659ef3574616"; };

  };

}
#endif
