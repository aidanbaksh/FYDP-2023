// Generated by gencpp from file lidar_msgs/box.msg
// DO NOT EDIT!


#ifndef LIDAR_MSGS_MESSAGE_BOX_H
#define LIDAR_MSGS_MESSAGE_BOX_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace lidar_msgs
{
template <class ContainerAllocator>
struct box_
{
  typedef box_<ContainerAllocator> Type;

  box_()
    : x_min(0.0)
    , y_min(0.0)
    , z_min(0.0)
    , x_max(0.0)
    , y_max(0.0)
    , z_max(0.0)  {
    }
  box_(const ContainerAllocator& _alloc)
    : x_min(0.0)
    , y_min(0.0)
    , z_min(0.0)
    , x_max(0.0)
    , y_max(0.0)
    , z_max(0.0)  {
  (void)_alloc;
    }



   typedef float _x_min_type;
  _x_min_type x_min;

   typedef float _y_min_type;
  _y_min_type y_min;

   typedef float _z_min_type;
  _z_min_type z_min;

   typedef float _x_max_type;
  _x_max_type x_max;

   typedef float _y_max_type;
  _y_max_type y_max;

   typedef float _z_max_type;
  _z_max_type z_max;





  typedef boost::shared_ptr< ::lidar_msgs::box_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lidar_msgs::box_<ContainerAllocator> const> ConstPtr;

}; // struct box_

typedef ::lidar_msgs::box_<std::allocator<void> > box;

typedef boost::shared_ptr< ::lidar_msgs::box > boxPtr;
typedef boost::shared_ptr< ::lidar_msgs::box const> boxConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lidar_msgs::box_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lidar_msgs::box_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::lidar_msgs::box_<ContainerAllocator1> & lhs, const ::lidar_msgs::box_<ContainerAllocator2> & rhs)
{
  return lhs.x_min == rhs.x_min &&
    lhs.y_min == rhs.y_min &&
    lhs.z_min == rhs.z_min &&
    lhs.x_max == rhs.x_max &&
    lhs.y_max == rhs.y_max &&
    lhs.z_max == rhs.z_max;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::lidar_msgs::box_<ContainerAllocator1> & lhs, const ::lidar_msgs::box_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace lidar_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::lidar_msgs::box_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lidar_msgs::box_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lidar_msgs::box_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lidar_msgs::box_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lidar_msgs::box_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lidar_msgs::box_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lidar_msgs::box_<ContainerAllocator> >
{
  static const char* value()
  {
    return "15f96afef8ea13b641fa2f30638908e0";
  }

  static const char* value(const ::lidar_msgs::box_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x15f96afef8ea13b6ULL;
  static const uint64_t static_value2 = 0x41fa2f30638908e0ULL;
};

template<class ContainerAllocator>
struct DataType< ::lidar_msgs::box_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lidar_msgs/box";
  }

  static const char* value(const ::lidar_msgs::box_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lidar_msgs::box_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 x_min\n"
"float32 y_min\n"
"float32 z_min\n"
"float32 x_max\n"
"float32 y_max\n"
"float32 z_max\n"
;
  }

  static const char* value(const ::lidar_msgs::box_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lidar_msgs::box_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x_min);
      stream.next(m.y_min);
      stream.next(m.z_min);
      stream.next(m.x_max);
      stream.next(m.y_max);
      stream.next(m.z_max);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct box_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lidar_msgs::box_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lidar_msgs::box_<ContainerAllocator>& v)
  {
    s << indent << "x_min: ";
    Printer<float>::stream(s, indent + "  ", v.x_min);
    s << indent << "y_min: ";
    Printer<float>::stream(s, indent + "  ", v.y_min);
    s << indent << "z_min: ";
    Printer<float>::stream(s, indent + "  ", v.z_min);
    s << indent << "x_max: ";
    Printer<float>::stream(s, indent + "  ", v.x_max);
    s << indent << "y_max: ";
    Printer<float>::stream(s, indent + "  ", v.y_max);
    s << indent << "z_max: ";
    Printer<float>::stream(s, indent + "  ", v.z_max);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LIDAR_MSGS_MESSAGE_BOX_H
