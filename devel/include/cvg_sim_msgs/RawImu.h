// Generated by gencpp from file cvg_sim_msgs/RawImu.msg
// DO NOT EDIT!


#ifndef CVG_SIM_MSGS_MESSAGE_RAWIMU_H
#define CVG_SIM_MSGS_MESSAGE_RAWIMU_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace cvg_sim_msgs
{
template <class ContainerAllocator>
struct RawImu_
{
  typedef RawImu_<ContainerAllocator> Type;

  RawImu_()
    : header()
    , angular_velocity()
    , linear_acceleration()  {
      angular_velocity.assign(0);

      linear_acceleration.assign(0);
  }
  RawImu_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , angular_velocity()
    , linear_acceleration()  {
  (void)_alloc;
      angular_velocity.assign(0);

      linear_acceleration.assign(0);
  }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef boost::array<uint16_t, 3>  _angular_velocity_type;
  _angular_velocity_type angular_velocity;

   typedef boost::array<uint16_t, 3>  _linear_acceleration_type;
  _linear_acceleration_type linear_acceleration;





  typedef boost::shared_ptr< ::cvg_sim_msgs::RawImu_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cvg_sim_msgs::RawImu_<ContainerAllocator> const> ConstPtr;

}; // struct RawImu_

typedef ::cvg_sim_msgs::RawImu_<std::allocator<void> > RawImu;

typedef boost::shared_ptr< ::cvg_sim_msgs::RawImu > RawImuPtr;
typedef boost::shared_ptr< ::cvg_sim_msgs::RawImu const> RawImuConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cvg_sim_msgs::RawImu_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cvg_sim_msgs::RawImu_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cvg_sim_msgs::RawImu_<ContainerAllocator1> & lhs, const ::cvg_sim_msgs::RawImu_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.angular_velocity == rhs.angular_velocity &&
    lhs.linear_acceleration == rhs.linear_acceleration;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cvg_sim_msgs::RawImu_<ContainerAllocator1> & lhs, const ::cvg_sim_msgs::RawImu_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cvg_sim_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::cvg_sim_msgs::RawImu_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cvg_sim_msgs::RawImu_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cvg_sim_msgs::RawImu_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cvg_sim_msgs::RawImu_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cvg_sim_msgs::RawImu_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cvg_sim_msgs::RawImu_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cvg_sim_msgs::RawImu_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0879a838e899792bcf72ccfe7b5595ef";
  }

  static const char* value(const ::cvg_sim_msgs::RawImu_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0879a838e899792bULL;
  static const uint64_t static_value2 = 0xcf72ccfe7b5595efULL;
};

template<class ContainerAllocator>
struct DataType< ::cvg_sim_msgs::RawImu_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cvg_sim_msgs/RawImu";
  }

  static const char* value(const ::cvg_sim_msgs::RawImu_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cvg_sim_msgs::RawImu_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"uint16[3] angular_velocity\n"
"uint16[3] linear_acceleration\n"
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

  static const char* value(const ::cvg_sim_msgs::RawImu_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cvg_sim_msgs::RawImu_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.angular_velocity);
      stream.next(m.linear_acceleration);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RawImu_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cvg_sim_msgs::RawImu_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cvg_sim_msgs::RawImu_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "angular_velocity[]" << std::endl;
    for (size_t i = 0; i < v.angular_velocity.size(); ++i)
    {
      s << indent << "  angular_velocity[" << i << "]: ";
      Printer<uint16_t>::stream(s, indent + "  ", v.angular_velocity[i]);
    }
    s << indent << "linear_acceleration[]" << std::endl;
    for (size_t i = 0; i < v.linear_acceleration.size(); ++i)
    {
      s << indent << "  linear_acceleration[" << i << "]: ";
      Printer<uint16_t>::stream(s, indent + "  ", v.linear_acceleration[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // CVG_SIM_MSGS_MESSAGE_RAWIMU_H
