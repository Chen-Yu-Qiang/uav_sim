// Generated by gencpp from file cvg_sim_gazebo_plugins/SetBiasResponse.msg
// DO NOT EDIT!


#ifndef CVG_SIM_GAZEBO_PLUGINS_MESSAGE_SETBIASRESPONSE_H
#define CVG_SIM_GAZEBO_PLUGINS_MESSAGE_SETBIASRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace cvg_sim_gazebo_plugins
{
template <class ContainerAllocator>
struct SetBiasResponse_
{
  typedef SetBiasResponse_<ContainerAllocator> Type;

  SetBiasResponse_()
    {
    }
  SetBiasResponse_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::cvg_sim_gazebo_plugins::SetBiasResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cvg_sim_gazebo_plugins::SetBiasResponse_<ContainerAllocator> const> ConstPtr;

}; // struct SetBiasResponse_

typedef ::cvg_sim_gazebo_plugins::SetBiasResponse_<std::allocator<void> > SetBiasResponse;

typedef boost::shared_ptr< ::cvg_sim_gazebo_plugins::SetBiasResponse > SetBiasResponsePtr;
typedef boost::shared_ptr< ::cvg_sim_gazebo_plugins::SetBiasResponse const> SetBiasResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cvg_sim_gazebo_plugins::SetBiasResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cvg_sim_gazebo_plugins::SetBiasResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace cvg_sim_gazebo_plugins

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::cvg_sim_gazebo_plugins::SetBiasResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cvg_sim_gazebo_plugins::SetBiasResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cvg_sim_gazebo_plugins::SetBiasResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cvg_sim_gazebo_plugins::SetBiasResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cvg_sim_gazebo_plugins::SetBiasResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cvg_sim_gazebo_plugins::SetBiasResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cvg_sim_gazebo_plugins::SetBiasResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::cvg_sim_gazebo_plugins::SetBiasResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::cvg_sim_gazebo_plugins::SetBiasResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cvg_sim_gazebo_plugins/SetBiasResponse";
  }

  static const char* value(const ::cvg_sim_gazebo_plugins::SetBiasResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cvg_sim_gazebo_plugins::SetBiasResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::cvg_sim_gazebo_plugins::SetBiasResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cvg_sim_gazebo_plugins::SetBiasResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetBiasResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cvg_sim_gazebo_plugins::SetBiasResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::cvg_sim_gazebo_plugins::SetBiasResponse_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // CVG_SIM_GAZEBO_PLUGINS_MESSAGE_SETBIASRESPONSE_H
