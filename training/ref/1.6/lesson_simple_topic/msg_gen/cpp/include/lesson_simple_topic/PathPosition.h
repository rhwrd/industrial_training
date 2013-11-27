/* Auto-generated by genmsg_cpp for file /home/imeinzen/training/work/1.6/lesson_simple_topic/msg/PathPosition.msg */
#ifndef LESSON_SIMPLE_TOPIC_MESSAGE_PATHPOSITION_H
#define LESSON_SIMPLE_TOPIC_MESSAGE_PATHPOSITION_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace lesson_simple_topic
{
template <class ContainerAllocator>
struct PathPosition_ {
  typedef PathPosition_<ContainerAllocator> Type;

  PathPosition_()
  : x(0.0)
  , y(0.0)
  , angle(0.0)
  {
  }

  PathPosition_(const ContainerAllocator& _alloc)
  : x(0.0)
  , y(0.0)
  , angle(0.0)
  {
  }

  typedef double _x_type;
  double x;

  typedef double _y_type;
  double y;

  typedef double _angle_type;
  double angle;


  typedef boost::shared_ptr< ::lesson_simple_topic::PathPosition_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lesson_simple_topic::PathPosition_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PathPosition
typedef  ::lesson_simple_topic::PathPosition_<std::allocator<void> > PathPosition;

typedef boost::shared_ptr< ::lesson_simple_topic::PathPosition> PathPositionPtr;
typedef boost::shared_ptr< ::lesson_simple_topic::PathPosition const> PathPositionConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::lesson_simple_topic::PathPosition_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::lesson_simple_topic::PathPosition_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace lesson_simple_topic

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::lesson_simple_topic::PathPosition_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::lesson_simple_topic::PathPosition_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::lesson_simple_topic::PathPosition_<ContainerAllocator> > {
  static const char* value() 
  {
    return "57832a67d2f8a00310788a06c92c59b2";
  }

  static const char* value(const  ::lesson_simple_topic::PathPosition_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x57832a67d2f8a003ULL;
  static const uint64_t static_value2 = 0x10788a06c92c59b2ULL;
};

template<class ContainerAllocator>
struct DataType< ::lesson_simple_topic::PathPosition_<ContainerAllocator> > {
  static const char* value() 
  {
    return "lesson_simple_topic/PathPosition";
  }

  static const char* value(const  ::lesson_simple_topic::PathPosition_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::lesson_simple_topic::PathPosition_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# A 2D position: X, Y, and angle\n\
float64 x\n\
float64 y\n\
float64 angle\n\
\n\
";
  }

  static const char* value(const  ::lesson_simple_topic::PathPosition_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::lesson_simple_topic::PathPosition_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::lesson_simple_topic::PathPosition_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.angle);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PathPosition_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lesson_simple_topic::PathPosition_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::lesson_simple_topic::PathPosition_<ContainerAllocator> & v) 
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "angle: ";
    Printer<double>::stream(s, indent + "  ", v.angle);
  }
};


} // namespace message_operations
} // namespace ros

#endif // LESSON_SIMPLE_TOPIC_MESSAGE_PATHPOSITION_H

