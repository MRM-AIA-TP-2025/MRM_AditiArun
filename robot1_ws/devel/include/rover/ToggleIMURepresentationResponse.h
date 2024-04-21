// Generated by gencpp from file rover/ToggleIMURepresentationResponse.msg
// DO NOT EDIT!


#ifndef ROVER_MESSAGE_TOGGLEIMUREPRESENTATIONRESPONSE_H
#define ROVER_MESSAGE_TOGGLEIMUREPRESENTATIONRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rover
{
template <class ContainerAllocator>
struct ToggleIMURepresentationResponse_
{
  typedef ToggleIMURepresentationResponse_<ContainerAllocator> Type;

  ToggleIMURepresentationResponse_()
    : roll(0.0)
    , pitch(0.0)
    , yaw(0.0)  {
    }
  ToggleIMURepresentationResponse_(const ContainerAllocator& _alloc)
    : roll(0.0)
    , pitch(0.0)
    , yaw(0.0)  {
  (void)_alloc;
    }



   typedef double _roll_type;
  _roll_type roll;

   typedef double _pitch_type;
  _pitch_type pitch;

   typedef double _yaw_type;
  _yaw_type yaw;





  typedef boost::shared_ptr< ::rover::ToggleIMURepresentationResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rover::ToggleIMURepresentationResponse_<ContainerAllocator> const> ConstPtr;

}; // struct ToggleIMURepresentationResponse_

typedef ::rover::ToggleIMURepresentationResponse_<std::allocator<void> > ToggleIMURepresentationResponse;

typedef boost::shared_ptr< ::rover::ToggleIMURepresentationResponse > ToggleIMURepresentationResponsePtr;
typedef boost::shared_ptr< ::rover::ToggleIMURepresentationResponse const> ToggleIMURepresentationResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rover::ToggleIMURepresentationResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rover::ToggleIMURepresentationResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::rover::ToggleIMURepresentationResponse_<ContainerAllocator1> & lhs, const ::rover::ToggleIMURepresentationResponse_<ContainerAllocator2> & rhs)
{
  return lhs.roll == rhs.roll &&
    lhs.pitch == rhs.pitch &&
    lhs.yaw == rhs.yaw;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::rover::ToggleIMURepresentationResponse_<ContainerAllocator1> & lhs, const ::rover::ToggleIMURepresentationResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace rover

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::rover::ToggleIMURepresentationResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rover::ToggleIMURepresentationResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rover::ToggleIMURepresentationResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rover::ToggleIMURepresentationResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rover::ToggleIMURepresentationResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rover::ToggleIMURepresentationResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rover::ToggleIMURepresentationResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "eeec8b25a660789a89540dedcb2b06d6";
  }

  static const char* value(const ::rover::ToggleIMURepresentationResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xeeec8b25a660789aULL;
  static const uint64_t static_value2 = 0x89540dedcb2b06d6ULL;
};

template<class ContainerAllocator>
struct DataType< ::rover::ToggleIMURepresentationResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rover/ToggleIMURepresentationResponse";
  }

  static const char* value(const ::rover::ToggleIMURepresentationResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rover::ToggleIMURepresentationResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 roll\n"
"float64 pitch\n"
"float64 yaw\n"
"\n"
;
  }

  static const char* value(const ::rover::ToggleIMURepresentationResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rover::ToggleIMURepresentationResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.roll);
      stream.next(m.pitch);
      stream.next(m.yaw);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ToggleIMURepresentationResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rover::ToggleIMURepresentationResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rover::ToggleIMURepresentationResponse_<ContainerAllocator>& v)
  {
    s << indent << "roll: ";
    Printer<double>::stream(s, indent + "  ", v.roll);
    s << indent << "pitch: ";
    Printer<double>::stream(s, indent + "  ", v.pitch);
    s << indent << "yaw: ";
    Printer<double>::stream(s, indent + "  ", v.yaw);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROVER_MESSAGE_TOGGLEIMUREPRESENTATIONRESPONSE_H
