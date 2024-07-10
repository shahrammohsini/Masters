// Generated by gencpp from file bionic_hand/ControlCommand.msg
// DO NOT EDIT!


#ifndef BIONIC_HAND_MESSAGE_CONTROLCOMMAND_H
#define BIONIC_HAND_MESSAGE_CONTROLCOMMAND_H


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
struct ControlCommand_
{
  typedef ControlCommand_<ContainerAllocator> Type;

  ControlCommand_()
    : PWM(0.0)  {
    }
  ControlCommand_(const ContainerAllocator& _alloc)
    : PWM(0.0)  {
  (void)_alloc;
    }



   typedef double _PWM_type;
  _PWM_type PWM;





  typedef boost::shared_ptr< ::bionic_hand::ControlCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bionic_hand::ControlCommand_<ContainerAllocator> const> ConstPtr;

}; // struct ControlCommand_

typedef ::bionic_hand::ControlCommand_<std::allocator<void> > ControlCommand;

typedef boost::shared_ptr< ::bionic_hand::ControlCommand > ControlCommandPtr;
typedef boost::shared_ptr< ::bionic_hand::ControlCommand const> ControlCommandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bionic_hand::ControlCommand_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bionic_hand::ControlCommand_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::bionic_hand::ControlCommand_<ContainerAllocator1> & lhs, const ::bionic_hand::ControlCommand_<ContainerAllocator2> & rhs)
{
  return lhs.PWM == rhs.PWM;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::bionic_hand::ControlCommand_<ContainerAllocator1> & lhs, const ::bionic_hand::ControlCommand_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace bionic_hand

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::bionic_hand::ControlCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bionic_hand::ControlCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bionic_hand::ControlCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bionic_hand::ControlCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bionic_hand::ControlCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bionic_hand::ControlCommand_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bionic_hand::ControlCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "630d1348e66951f61746659ef3574616";
  }

  static const char* value(const ::bionic_hand::ControlCommand_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x630d1348e66951f6ULL;
  static const uint64_t static_value2 = 0x1746659ef3574616ULL;
};

template<class ContainerAllocator>
struct DataType< ::bionic_hand::ControlCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bionic_hand/ControlCommand";
  }

  static const char* value(const ::bionic_hand::ControlCommand_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bionic_hand::ControlCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 PWM\n"
;
  }

  static const char* value(const ::bionic_hand::ControlCommand_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bionic_hand::ControlCommand_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.PWM);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ControlCommand_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bionic_hand::ControlCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bionic_hand::ControlCommand_<ContainerAllocator>& v)
  {
    s << indent << "PWM: ";
    Printer<double>::stream(s, indent + "  ", v.PWM);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BIONIC_HAND_MESSAGE_CONTROLCOMMAND_H