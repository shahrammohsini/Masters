#ifndef _ROS_bionic_hand_FingerJoints_h
#define _ROS_bionic_hand_FingerJoints_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace bionic_hand
{

  class FingerJoints : public ros::Msg
  {
    public:
      typedef float _theta_M_type;
      _theta_M_type theta_M;
      typedef float _theta_P_type;
      _theta_P_type theta_P;
      typedef float _theta_D_type;
      _theta_D_type theta_D;

    FingerJoints():
      theta_M(0),
      theta_P(0),
      theta_D(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->theta_M);
      offset += serializeAvrFloat64(outbuffer + offset, this->theta_P);
      offset += serializeAvrFloat64(outbuffer + offset, this->theta_D);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->theta_M));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->theta_P));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->theta_D));
     return offset;
    }

    virtual const char * getType() override { return "bionic_hand/FingerJoints"; };
    virtual const char * getMD5() override { return "87bf9b04d1a94d2eda566ee32685c210"; };

  };

}
#endif
