// Generated by gencpp from file readgps/imu.msg
// DO NOT EDIT!


#ifndef READGPS_MESSAGE_IMU_H
#define READGPS_MESSAGE_IMU_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include "modules/drivers/gnss/parser/include/header.h"
#include "modules/drivers/gnss/parser/include/Point3D.h"

namespace readgps
{
template <class ContainerAllocator>
struct imu_
{
  typedef imu_<ContainerAllocator> Type;

  imu_()
    : header()
    , measurement_time(0.0)
    , measurement_span(0.0)
    , linear_acceleration()
    , angular_velocity()  {
    }
  imu_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , measurement_time(0.0)
    , measurement_span(0.0)
    , linear_acceleration(_alloc)
    , angular_velocity(_alloc)  {
  (void)_alloc;
    }



   typedef  ::readgps::header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _measurement_time_type;
  _measurement_time_type measurement_time;

   typedef float _measurement_span_type;
  _measurement_span_type measurement_span;

   typedef  ::readgps::Point3D_<ContainerAllocator>  _linear_acceleration_type;
  _linear_acceleration_type linear_acceleration;

   typedef  ::readgps::Point3D_<ContainerAllocator>  _angular_velocity_type;
  _angular_velocity_type angular_velocity;





  typedef boost::shared_ptr< ::readgps::imu_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::readgps::imu_<ContainerAllocator> const> ConstPtr;

}; // struct imu_

typedef ::readgps::imu_<std::allocator<void> > imu;

typedef boost::shared_ptr< ::readgps::imu > imuPtr;
typedef boost::shared_ptr< ::readgps::imu const> imuConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::readgps::imu_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::readgps::imu_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace readgps

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'readgps': ['/home/ubuwgb/catkin_ws/src/readgps/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::readgps::imu_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::readgps::imu_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::readgps::imu_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::readgps::imu_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::readgps::imu_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::readgps::imu_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::readgps::imu_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0995bb4c564070feb8f9a169789016b5";
  }

  static const char* value(const ::readgps::imu_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0995bb4c564070feULL;
  static const uint64_t static_value2 = 0xb8f9a169789016b5ULL;
};

template<class ContainerAllocator>
struct DataType< ::readgps::imu_<ContainerAllocator> >
{
  static const char* value()
  {
    return "readgps/imu";
  }

  static const char* value(const ::readgps::imu_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::readgps::imu_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
float64 measurement_time\n\
float32 measurement_span\n\
Point3D linear_acceleration\n\
Point3D angular_velocity\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: readgps/Point3D\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::readgps::imu_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::readgps::imu_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.measurement_time);
      stream.next(m.measurement_span);
      stream.next(m.linear_acceleration);
      stream.next(m.angular_velocity);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct imu_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::readgps::imu_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::readgps::imu_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::readgps::header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "measurement_time: ";
    Printer<double>::stream(s, indent + "  ", v.measurement_time);
    s << indent << "measurement_span: ";
    Printer<float>::stream(s, indent + "  ", v.measurement_span);
    s << indent << "linear_acceleration: ";
    s << std::endl;
    Printer< ::readgps::Point3D_<ContainerAllocator> >::stream(s, indent + "  ", v.linear_acceleration);
    s << indent << "angular_velocity: ";
    s << std::endl;
    Printer< ::readgps::Point3D_<ContainerAllocator> >::stream(s, indent + "  ", v.angular_velocity);
  }
};

} // namespace message_operations
} // namespace ros

#endif // READGPS_MESSAGE_IMU_H
