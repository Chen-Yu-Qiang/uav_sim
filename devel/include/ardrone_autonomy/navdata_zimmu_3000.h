// Generated by gencpp from file ardrone_autonomy/navdata_zimmu_3000.msg
// DO NOT EDIT!


#ifndef ARDRONE_AUTONOMY_MESSAGE_NAVDATA_ZIMMU_3000_H
#define ARDRONE_AUTONOMY_MESSAGE_NAVDATA_ZIMMU_3000_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace ardrone_autonomy
{
template <class ContainerAllocator>
struct navdata_zimmu_3000_
{
  typedef navdata_zimmu_3000_<ContainerAllocator> Type;

  navdata_zimmu_3000_()
    : header()
    , drone_time(0.0)
    , tag(0)
    , size(0)
    , vzimmuLSB(0)
    , vzfind(0.0)  {
    }
  navdata_zimmu_3000_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , drone_time(0.0)
    , tag(0)
    , size(0)
    , vzimmuLSB(0)
    , vzfind(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _drone_time_type;
  _drone_time_type drone_time;

   typedef uint16_t _tag_type;
  _tag_type tag;

   typedef uint16_t _size_type;
  _size_type size;

   typedef int32_t _vzimmuLSB_type;
  _vzimmuLSB_type vzimmuLSB;

   typedef float _vzfind_type;
  _vzfind_type vzfind;





  typedef boost::shared_ptr< ::ardrone_autonomy::navdata_zimmu_3000_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ardrone_autonomy::navdata_zimmu_3000_<ContainerAllocator> const> ConstPtr;

}; // struct navdata_zimmu_3000_

typedef ::ardrone_autonomy::navdata_zimmu_3000_<std::allocator<void> > navdata_zimmu_3000;

typedef boost::shared_ptr< ::ardrone_autonomy::navdata_zimmu_3000 > navdata_zimmu_3000Ptr;
typedef boost::shared_ptr< ::ardrone_autonomy::navdata_zimmu_3000 const> navdata_zimmu_3000ConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ardrone_autonomy::navdata_zimmu_3000_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ardrone_autonomy::navdata_zimmu_3000_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ardrone_autonomy::navdata_zimmu_3000_<ContainerAllocator1> & lhs, const ::ardrone_autonomy::navdata_zimmu_3000_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.drone_time == rhs.drone_time &&
    lhs.tag == rhs.tag &&
    lhs.size == rhs.size &&
    lhs.vzimmuLSB == rhs.vzimmuLSB &&
    lhs.vzfind == rhs.vzfind;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ardrone_autonomy::navdata_zimmu_3000_<ContainerAllocator1> & lhs, const ::ardrone_autonomy::navdata_zimmu_3000_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ardrone_autonomy

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::ardrone_autonomy::navdata_zimmu_3000_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ardrone_autonomy::navdata_zimmu_3000_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ardrone_autonomy::navdata_zimmu_3000_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ardrone_autonomy::navdata_zimmu_3000_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ardrone_autonomy::navdata_zimmu_3000_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ardrone_autonomy::navdata_zimmu_3000_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ardrone_autonomy::navdata_zimmu_3000_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ae43ce2a4ef6cf0002f177f8265000bc";
  }

  static const char* value(const ::ardrone_autonomy::navdata_zimmu_3000_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xae43ce2a4ef6cf00ULL;
  static const uint64_t static_value2 = 0x02f177f8265000bcULL;
};

template<class ContainerAllocator>
struct DataType< ::ardrone_autonomy::navdata_zimmu_3000_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ardrone_autonomy/navdata_zimmu_3000";
  }

  static const char* value(const ::ardrone_autonomy::navdata_zimmu_3000_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ardrone_autonomy::navdata_zimmu_3000_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"float64 drone_time\n"
"uint16 tag\n"
"uint16 size\n"
"int32 vzimmuLSB\n"
"float32 vzfind\n"
"\n"
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

  static const char* value(const ::ardrone_autonomy::navdata_zimmu_3000_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ardrone_autonomy::navdata_zimmu_3000_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.drone_time);
      stream.next(m.tag);
      stream.next(m.size);
      stream.next(m.vzimmuLSB);
      stream.next(m.vzfind);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct navdata_zimmu_3000_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ardrone_autonomy::navdata_zimmu_3000_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ardrone_autonomy::navdata_zimmu_3000_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "drone_time: ";
    Printer<double>::stream(s, indent + "  ", v.drone_time);
    s << indent << "tag: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.tag);
    s << indent << "size: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.size);
    s << indent << "vzimmuLSB: ";
    Printer<int32_t>::stream(s, indent + "  ", v.vzimmuLSB);
    s << indent << "vzfind: ";
    Printer<float>::stream(s, indent + "  ", v.vzfind);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ARDRONE_AUTONOMY_MESSAGE_NAVDATA_ZIMMU_3000_H
