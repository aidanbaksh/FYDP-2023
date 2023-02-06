// Generated by gencpp from file lidar_msgs/BoxInfo.msg
// DO NOT EDIT!


#ifndef LIDAR_MSGS_MESSAGE_BOXINFO_H
#define LIDAR_MSGS_MESSAGE_BOXINFO_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <lidar_msgs/box.h>

namespace lidar_msgs
{
template <class ContainerAllocator>
struct BoxInfo_
{
  typedef BoxInfo_<ContainerAllocator> Type;

  BoxInfo_()
    : box()  {
    }
  BoxInfo_(const ContainerAllocator& _alloc)
    : box(_alloc)  {
  (void)_alloc;
    }



   typedef  ::lidar_msgs::box_<ContainerAllocator>  _box_type;
  _box_type box;





  typedef boost::shared_ptr< ::lidar_msgs::BoxInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lidar_msgs::BoxInfo_<ContainerAllocator> const> ConstPtr;

}; // struct BoxInfo_

typedef ::lidar_msgs::BoxInfo_<std::allocator<void> > BoxInfo;

typedef boost::shared_ptr< ::lidar_msgs::BoxInfo > BoxInfoPtr;
typedef boost::shared_ptr< ::lidar_msgs::BoxInfo const> BoxInfoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lidar_msgs::BoxInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lidar_msgs::BoxInfo_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::lidar_msgs::BoxInfo_<ContainerAllocator1> & lhs, const ::lidar_msgs::BoxInfo_<ContainerAllocator2> & rhs)
{
  return lhs.box == rhs.box;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::lidar_msgs::BoxInfo_<ContainerAllocator1> & lhs, const ::lidar_msgs::BoxInfo_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace lidar_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::lidar_msgs::BoxInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lidar_msgs::BoxInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lidar_msgs::BoxInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lidar_msgs::BoxInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lidar_msgs::BoxInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lidar_msgs::BoxInfo_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lidar_msgs::BoxInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7f621b8f09e614f07fb08a2b7f9a6583";
  }

  static const char* value(const ::lidar_msgs::BoxInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7f621b8f09e614f0ULL;
  static const uint64_t static_value2 = 0x7fb08a2b7f9a6583ULL;
};

template<class ContainerAllocator>
struct DataType< ::lidar_msgs::BoxInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lidar_msgs/BoxInfo";
  }

  static const char* value(const ::lidar_msgs::BoxInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lidar_msgs::BoxInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "box box\n"
"\n"
"================================================================================\n"
"MSG: lidar_msgs/box\n"
"float32 x_min\n"
"float32 y_min\n"
"float32 z_min\n"
"float32 x_max\n"
"float32 y_max\n"
"float32 z_max\n"
;
  }

  static const char* value(const ::lidar_msgs::BoxInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lidar_msgs::BoxInfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.box);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BoxInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lidar_msgs::BoxInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lidar_msgs::BoxInfo_<ContainerAllocator>& v)
  {
    s << indent << "box: ";
    s << std::endl;
    Printer< ::lidar_msgs::box_<ContainerAllocator> >::stream(s, indent + "  ", v.box);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LIDAR_MSGS_MESSAGE_BOXINFO_H
