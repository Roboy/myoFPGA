// Generated by gencpp from file std_msgs/Char.msg
// DO NOT EDIT!


#ifndef STD_MSGS_MESSAGE_CHAR_H
#define STD_MSGS_MESSAGE_CHAR_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace std_msgs
{
template <class ContainerAllocator>
struct Char_
{
  typedef Char_<ContainerAllocator> Type;

  Char_()
    : data(0)  {
    }
  Char_(const ContainerAllocator& _alloc)
    : data(0)  {
  (void)_alloc;
    }



   typedef uint8_t _data_type;
  _data_type data;




  typedef boost::shared_ptr< ::std_msgs::Char_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::std_msgs::Char_<ContainerAllocator> const> ConstPtr;

}; // struct Char_

typedef ::std_msgs::Char_<std::allocator<void> > Char;

typedef boost::shared_ptr< ::std_msgs::Char > CharPtr;
typedef boost::shared_ptr< ::std_msgs::Char const> CharConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::std_msgs::Char_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::std_msgs::Char_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace std_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/root/ros_catkin_ws/src/std_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::std_msgs::Char_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::std_msgs::Char_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::std_msgs::Char_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::std_msgs::Char_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::std_msgs::Char_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::std_msgs::Char_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::std_msgs::Char_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1bf77f25acecdedba0e224b162199717";
  }

  static const char* value(const ::std_msgs::Char_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1bf77f25acecdedbULL;
  static const uint64_t static_value2 = 0xa0e224b162199717ULL;
};

template<class ContainerAllocator>
struct DataType< ::std_msgs::Char_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Char";
  }

  static const char* value(const ::std_msgs::Char_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::std_msgs::Char_<ContainerAllocator> >
{
  static const char* value()
  {
    return "char data\n\
";
  }

  static const char* value(const ::std_msgs::Char_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::std_msgs::Char_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Char_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::std_msgs::Char_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::std_msgs::Char_<ContainerAllocator>& v)
  {
    s << indent << "data: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.data);
  }
};

} // namespace message_operations
} // namespace ros

#endif // STD_MSGS_MESSAGE_CHAR_H