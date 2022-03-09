// Generated by gencpp from file vector_map_server/PositionStateResponse.msg
// DO NOT EDIT!


#ifndef VECTOR_MAP_SERVER_MESSAGE_POSITIONSTATERESPONSE_H
#define VECTOR_MAP_SERVER_MESSAGE_POSITIONSTATERESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace vector_map_server
{
template <class ContainerAllocator>
struct PositionStateResponse_
{
  typedef PositionStateResponse_<ContainerAllocator> Type;

  PositionStateResponse_()
    : state(false)  {
    }
  PositionStateResponse_(const ContainerAllocator& _alloc)
    : state(false)  {
  (void)_alloc;
    }



   typedef uint8_t _state_type;
  _state_type state;





  typedef boost::shared_ptr< ::vector_map_server::PositionStateResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vector_map_server::PositionStateResponse_<ContainerAllocator> const> ConstPtr;

}; // struct PositionStateResponse_

typedef ::vector_map_server::PositionStateResponse_<std::allocator<void> > PositionStateResponse;

typedef boost::shared_ptr< ::vector_map_server::PositionStateResponse > PositionStateResponsePtr;
typedef boost::shared_ptr< ::vector_map_server::PositionStateResponse const> PositionStateResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vector_map_server::PositionStateResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vector_map_server::PositionStateResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::vector_map_server::PositionStateResponse_<ContainerAllocator1> & lhs, const ::vector_map_server::PositionStateResponse_<ContainerAllocator2> & rhs)
{
  return lhs.state == rhs.state;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::vector_map_server::PositionStateResponse_<ContainerAllocator1> & lhs, const ::vector_map_server::PositionStateResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace vector_map_server

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::vector_map_server::PositionStateResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vector_map_server::PositionStateResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vector_map_server::PositionStateResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vector_map_server::PositionStateResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vector_map_server::PositionStateResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vector_map_server::PositionStateResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vector_map_server::PositionStateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "001fde3cab9e313a150416ff09c08ee4";
  }

  static const char* value(const ::vector_map_server::PositionStateResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x001fde3cab9e313aULL;
  static const uint64_t static_value2 = 0x150416ff09c08ee4ULL;
};

template<class ContainerAllocator>
struct DataType< ::vector_map_server::PositionStateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vector_map_server/PositionStateResponse";
  }

  static const char* value(const ::vector_map_server::PositionStateResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vector_map_server::PositionStateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool state\n"
"\n"
;
  }

  static const char* value(const ::vector_map_server::PositionStateResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vector_map_server::PositionStateResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PositionStateResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vector_map_server::PositionStateResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vector_map_server::PositionStateResponse_<ContainerAllocator>& v)
  {
    s << indent << "state: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VECTOR_MAP_SERVER_MESSAGE_POSITIONSTATERESPONSE_H