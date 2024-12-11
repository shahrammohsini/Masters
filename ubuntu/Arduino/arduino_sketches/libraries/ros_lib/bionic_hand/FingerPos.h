#ifndef _ROS_bionic_hand_FingerPos_h
#define _ROS_bionic_hand_FingerPos_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace bionic_hand
{

  class FingerPos : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _theta_M_type;
      _theta_M_type theta_M;
      typedef float _theta_P_type;
      _theta_P_type theta_P;
      typedef float _theta_D_type;
      _theta_D_type theta_D;

    FingerPos():
      header(),
      theta_M(0),
      theta_P(0),
      theta_D(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->theta_M);
      offset += serializeAvrFloat64(outbuffer + offset, this->theta_P);
      offset += serializeAvrFloat64(outbuffer + offset, this->theta_D);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->theta_M));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->theta_P));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->theta_D));
     return offset;
    }

    virtual const char * getType() override { return "bionic_hand/FingerPos"; };
    virtual const char * getMD5() override { return "4b76b67765bb2cfec63fff0018dee699"; };

  };

}
#endif
