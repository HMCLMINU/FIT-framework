// Generated by gencpp from file carla_msgs/CarlaTrafficLightInfo.msg
// DO NOT EDIT!


#ifndef CARLA_MSGS_MESSAGE_CARLATRAFFICLIGHTINFO_H
#define CARLA_MSGS_MESSAGE_CARLATRAFFICLIGHTINFO_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose.h>
#include <carla_msgs/CarlaBoundingBox.h>

namespace carla_msgs
{
template <class ContainerAllocator>
struct CarlaTrafficLightInfo_
{
  typedef CarlaTrafficLightInfo_<ContainerAllocator> Type;

  CarlaTrafficLightInfo_()
    : id(0)
    , transform()
    , trigger_volume()  {
    }
  CarlaTrafficLightInfo_(const ContainerAllocator& _alloc)
    : id(0)
    , transform(_alloc)
    , trigger_volume(_alloc)  {
  (void)_alloc;
    }



   typedef uint32_t _id_type;
  _id_type id;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _transform_type;
  _transform_type transform;

   typedef  ::carla_msgs::CarlaBoundingBox_<ContainerAllocator>  _trigger_volume_type;
  _trigger_volume_type trigger_volume;





  typedef boost::shared_ptr< ::carla_msgs::CarlaTrafficLightInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::carla_msgs::CarlaTrafficLightInfo_<ContainerAllocator> const> ConstPtr;

}; // struct CarlaTrafficLightInfo_

typedef ::carla_msgs::CarlaTrafficLightInfo_<std::allocator<void> > CarlaTrafficLightInfo;

typedef boost::shared_ptr< ::carla_msgs::CarlaTrafficLightInfo > CarlaTrafficLightInfoPtr;
typedef boost::shared_ptr< ::carla_msgs::CarlaTrafficLightInfo const> CarlaTrafficLightInfoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::carla_msgs::CarlaTrafficLightInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::carla_msgs::CarlaTrafficLightInfo_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::carla_msgs::CarlaTrafficLightInfo_<ContainerAllocator1> & lhs, const ::carla_msgs::CarlaTrafficLightInfo_<ContainerAllocator2> & rhs)
{
  return lhs.id == rhs.id &&
    lhs.transform == rhs.transform &&
    lhs.trigger_volume == rhs.trigger_volume;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::carla_msgs::CarlaTrafficLightInfo_<ContainerAllocator1> & lhs, const ::carla_msgs::CarlaTrafficLightInfo_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace carla_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::carla_msgs::CarlaTrafficLightInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::carla_msgs::CarlaTrafficLightInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::carla_msgs::CarlaTrafficLightInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::carla_msgs::CarlaTrafficLightInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::carla_msgs::CarlaTrafficLightInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::carla_msgs::CarlaTrafficLightInfo_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::carla_msgs::CarlaTrafficLightInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c303b00d6ff9db591d60b1662aec9d48";
  }

  static const char* value(const ::carla_msgs::CarlaTrafficLightInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc303b00d6ff9db59ULL;
  static const uint64_t static_value2 = 0x1d60b1662aec9d48ULL;
};

template<class ContainerAllocator>
struct DataType< ::carla_msgs::CarlaTrafficLightInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "carla_msgs/CarlaTrafficLightInfo";
  }

  static const char* value(const ::carla_msgs::CarlaTrafficLightInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::carla_msgs::CarlaTrafficLightInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#\n"
"# Copyright (c) 2020 Intel Corporation.\n"
"#\n"
"# This work is licensed under the terms of the MIT license.\n"
"# For a copy, see <https://opensource.org/licenses/MIT>.\n"
"#\n"
"uint32 id\n"
"geometry_msgs/Pose transform\n"
"CarlaBoundingBox trigger_volume # position is relative to transform\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
"\n"
"================================================================================\n"
"MSG: carla_msgs/CarlaBoundingBox\n"
"#\n"
"# Copyright (c) 2020 Intel Corporation.\n"
"#\n"
"# This work is licensed under the terms of the MIT license.\n"
"# For a copy, see <https://opensource.org/licenses/MIT>.\n"
"#\n"
"geometry_msgs/Vector3 center\n"
"\n"
"geometry_msgs/Vector3 size\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::carla_msgs::CarlaTrafficLightInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::carla_msgs::CarlaTrafficLightInfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.transform);
      stream.next(m.trigger_volume);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CarlaTrafficLightInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::carla_msgs::CarlaTrafficLightInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::carla_msgs::CarlaTrafficLightInfo_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.id);
    s << indent << "transform: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.transform);
    s << indent << "trigger_volume: ";
    s << std::endl;
    Printer< ::carla_msgs::CarlaBoundingBox_<ContainerAllocator> >::stream(s, indent + "  ", v.trigger_volume);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CARLA_MSGS_MESSAGE_CARLATRAFFICLIGHTINFO_H
