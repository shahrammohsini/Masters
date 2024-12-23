// Generated by gencpp from file bionic_hand/FingerJoints.msg
// DO NOT EDIT!


#ifndef BIONIC_HAND_MESSAGE_FINGERJOINTS_H
#define BIONIC_HAND_MESSAGE_FINGERJOINTS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace bionic_hand
{
template <class ContainerAllocator>
struct FingerJoints_
{
  typedef FingerJoints_<ContainerAllocator> Type;

  FingerJoints_()
    : theta_M(0.0)
    , theta_P(0.0)
    , theta_D(0.0)  {
    }
  FingerJoints_(const ContainerAllocator& _alloc)
    : theta_M(0.0)
    , theta_P(0.0)
    , theta_D(0.0)  {
  (void)_alloc;
    }



   typedef double _theta_M_type;
  _theta_M_type theta_M;

   typedef double _theta_P_type;
  _theta_P_type theta_P;

   typedef double _theta_D_type;
  _theta_D_type theta_D;





  typedef boost::shared_ptr< ::bionic_hand::FingerJoints_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bionic_hand::FingerJoints_<ContainerAllocator> const> ConstPtr;

}; // struct FingerJoints_

typedef ::bionic_hand::FingerJoints_<std::allocator<void> > FingerJoints;

typedef boost::shared_ptr< ::bionic_hand::FingerJoints > FingerJointsPtr;
typedef boost::shared_ptr< ::bionic_hand::FingerJoints const> FingerJointsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bionic_hand::FingerJoints_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bionic_hand::FingerJoints_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::bionic_hand::FingerJoints_<ContainerAllocator1> & lhs, const ::bionic_hand::FingerJoints_<ContainerAllocator2> & rhs)
{
  return lhs.theta_M == rhs.theta_M &&
    lhs.theta_P == rhs.theta_P &&
    lhs.theta_D == rhs.theta_D;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::bionic_hand::FingerJoints_<ContainerAllocator1> & lhs, const ::bionic_hand::FingerJoints_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace bionic_hand

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::bionic_hand::FingerJoints_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bionic_hand::FingerJoints_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bionic_hand::FingerJoints_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bionic_hand::FingerJoints_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bionic_hand::FingerJoints_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bionic_hand::FingerJoints_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bionic_hand::FingerJoints_<ContainerAllocator> >
{
  static const char* value()
  {
    return "87bf9b04d1a94d2eda566ee32685c210";
  }

  static const char* value(const ::bionic_hand::FingerJoints_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x87bf9b04d1a94d2eULL;
  static const uint64_t static_value2 = 0xda566ee32685c210ULL;
};

template<class ContainerAllocator>
struct DataType< ::bionic_hand::FingerJoints_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bionic_hand/FingerJoints";
  }

  static const char* value(const ::bionic_hand::FingerJoints_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bionic_hand::FingerJoints_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 theta_M\n"
"float64 theta_P\n"
"float64 theta_D\n"
;
  }

  static const char* value(const ::bionic_hand::FingerJoints_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bionic_hand::FingerJoints_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.theta_M);
      stream.next(m.theta_P);
      stream.next(m.theta_D);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FingerJoints_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bionic_hand::FingerJoints_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bionic_hand::FingerJoints_<ContainerAllocator>& v)
  {
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

#endif // BIONIC_HAND_MESSAGE_FINGERJOINTS_H
