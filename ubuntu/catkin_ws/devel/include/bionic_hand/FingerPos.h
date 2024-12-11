// Generated by gencpp from file bionic_hand/FingerPos.msg
// DO NOT EDIT!


#ifndef BIONIC_HAND_MESSAGE_FINGERPOS_H
#define BIONIC_HAND_MESSAGE_FINGERPOS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace bionic_hand
{
template <class ContainerAllocator>
struct FingerPos_
{
  typedef FingerPos_<ContainerAllocator> Type;

  FingerPos_()
    : header()
    , theta_M(0.0)
    , theta_P(0.0)
    , theta_D(0.0)  {
    }
  FingerPos_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , theta_M(0.0)
    , theta_P(0.0)
    , theta_D(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _theta_M_type;
  _theta_M_type theta_M;

   typedef double _theta_P_type;
  _theta_P_type theta_P;

   typedef double _theta_D_type;
  _theta_D_type theta_D;





  typedef boost::shared_ptr< ::bionic_hand::FingerPos_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bionic_hand::FingerPos_<ContainerAllocator> const> ConstPtr;

}; // struct FingerPos_

typedef ::bionic_hand::FingerPos_<std::allocator<void> > FingerPos;

typedef boost::shared_ptr< ::bionic_hand::FingerPos > FingerPosPtr;
typedef boost::shared_ptr< ::bionic_hand::FingerPos const> FingerPosConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bionic_hand::FingerPos_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bionic_hand::FingerPos_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::bionic_hand::FingerPos_<ContainerAllocator1> & lhs, const ::bionic_hand::FingerPos_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.theta_M == rhs.theta_M &&
    lhs.theta_P == rhs.theta_P &&
    lhs.theta_D == rhs.theta_D;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::bionic_hand::FingerPos_<ContainerAllocator1> & lhs, const ::bionic_hand::FingerPos_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace bionic_hand

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::bionic_hand::FingerPos_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bionic_hand::FingerPos_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bionic_hand::FingerPos_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bionic_hand::FingerPos_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bionic_hand::FingerPos_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bionic_hand::FingerPos_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bionic_hand::FingerPos_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4b76b67765bb2cfec63fff0018dee699";
  }

  static const char* value(const ::bionic_hand::FingerPos_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4b76b67765bb2cfeULL;
  static const uint64_t static_value2 = 0xc63fff0018dee699ULL;
};

template<class ContainerAllocator>
struct DataType< ::bionic_hand::FingerPos_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bionic_hand/FingerPos";
  }

  static const char* value(const ::bionic_hand::FingerPos_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bionic_hand::FingerPos_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"float64 theta_M\n"
"float64 theta_P\n"
"float64 theta_D\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::bionic_hand::FingerPos_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bionic_hand::FingerPos_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.theta_M);
      stream.next(m.theta_P);
      stream.next(m.theta_D);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FingerPos_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bionic_hand::FingerPos_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bionic_hand::FingerPos_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "theta_M: ";
    Printer<double>::stream(s, indent + "  ", v.theta_M);
    s << indent << "theta_P: ";
    Printer<double>::stream(s, indent + "  ", v.theta_P);
    s << indent << "theta_D: ";
    Printer<double>::stream(s, indent + "  ", v.theta_D);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BIONIC_HAND_MESSAGE_FINGERPOS_H
