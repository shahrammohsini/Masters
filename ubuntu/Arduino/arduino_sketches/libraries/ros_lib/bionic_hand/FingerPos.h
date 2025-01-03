#ifndef _ROS_bionic_hand_FingerPos_h
#define _ROS_bionic_hand_FingerPos_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "bionic_hand/FingerJoints.h"

namespace bionic_hand
{

  class FingerPos : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef bionic_hand::FingerJoints _index_type;
      _index_type index;
      typedef bionic_hand::FingerJoints _middle_type;
      _middle_type middle;
      typedef bionic_hand::FingerJoints _thumb_type;
      _thumb_type thumb;

    FingerPos():
      header(),
      index(),
      middle(),
      thumb()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->index.serialize(outbuffer + offset);
      offset += this->middle.serialize(outbuffer + offset);
      offset += this->thumb.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->index.deserialize(inbuffer + offset);
      offset += this->middle.deserialize(inbuffer + offset);
      offset += this->thumb.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "bionic_hand/FingerPos"; };
    virtual const char * getMD5() override { return "ddb23831c46fbe010de19d051a0b3b5e"; };

  };

}
#endif
