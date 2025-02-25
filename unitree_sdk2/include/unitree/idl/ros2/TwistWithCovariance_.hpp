/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to CXX Translator
  File name: TwistWithCovariance_.idl
  Source: TwistWithCovariance_.hpp
  Cyclone DDS: v0.10.2

*****************************************************************/
#ifndef DDSCXX_UNITREE_IDL_ROS2_TWISTWITHCOVARIANCE__HPP
#define DDSCXX_UNITREE_IDL_ROS2_TWISTWITHCOVARIANCE__HPP

#include "unitree/idl/ros2/Twist_.hpp"

#include <array>

namespace geometry_msgs
{
namespace msg
{
namespace dds_
{
class TwistWithCovariance_
{
private:
 ::geometry_msgs::msg::dds_::Twist_ twist_;
 std::array<double, 36> covariance_ = { };

public:
  TwistWithCovariance_() = default;

  explicit TwistWithCovariance_(
    const ::geometry_msgs::msg::dds_::Twist_& twist,
    const std::array<double, 36>& covariance) :
    twist_(twist),
    covariance_(covariance) { }

  const ::geometry_msgs::msg::dds_::Twist_& twist() const { return this->twist_; }
  ::geometry_msgs::msg::dds_::Twist_& twist() { return this->twist_; }
  void twist(const ::geometry_msgs::msg::dds_::Twist_& _val_) { this->twist_ = _val_; }
  void twist(::geometry_msgs::msg::dds_::Twist_&& _val_) { this->twist_ = _val_; }
  const std::array<double, 36>& covariance() const { return this->covariance_; }
  std::array<double, 36>& covariance() { return this->covariance_; }
  void covariance(const std::array<double, 36>& _val_) { this->covariance_ = _val_; }
  void covariance(std::array<double, 36>&& _val_) { this->covariance_ = _val_; }

  bool operator==(const TwistWithCovariance_& _other) const
  {
    (void) _other;
    return twist_ == _other.twist_ &&
      covariance_ == _other.covariance_;
  }

  bool operator!=(const TwistWithCovariance_& _other) const
  {
    return !(*this == _other);
  }

};

}

}

}

#include "dds/topic/TopicTraits.hpp"
#include "org/eclipse/cyclonedds/topic/datatopic.hpp"

namespace org {
namespace eclipse {
namespace cyclonedds {
namespace topic {

template <> constexpr const char* TopicTraits<::geometry_msgs::msg::dds_::TwistWithCovariance_>::getTypeName()
{
  return "geometry_msgs::msg::dds_::TwistWithCovariance_";
}

template <> constexpr bool TopicTraits<::geometry_msgs::msg::dds_::TwistWithCovariance_>::isKeyless()
{
  return true;
}

#ifdef DDSCXX_HAS_TYPE_DISCOVERY
template<> constexpr unsigned int TopicTraits<::geometry_msgs::msg::dds_::TwistWithCovariance_>::type_map_blob_sz() { return 882; }
template<> constexpr unsigned int TopicTraits<::geometry_msgs::msg::dds_::TwistWithCovariance_>::type_info_blob_sz() { return 196; }
template<> inline const uint8_t * TopicTraits<::geometry_msgs::msg::dds_::TwistWithCovariance_>::type_map_blob() {
  static const uint8_t blob[] = {
 0x23,  0x01,  0x00,  0x00,  0x03,  0x00,  0x00,  0x00,  0xf1,  0x40,  0xaa,  0x70,  0xe1,  0xf7,  0x44,  0x45, 
 0x66,  0x8e,  0xcd,  0x3e,  0x4a,  0x4b,  0xa6,  0x00,  0x4e,  0x00,  0x00,  0x00,  0xf1,  0x51,  0x01,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x3e,  0x00,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00, 
 0x19,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0xf1,  0x28,  0xcf,  0x21,  0x56,  0x4e, 
 0xbe,  0xe5,  0x84,  0x6f,  0x1b,  0x3c,  0xfb,  0x2b,  0x52,  0xeb,  0x3b,  0xac,  0x16,  0x00,  0x00,  0x00, 
 0x16,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x90,  0xf3,  0x01,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x24,  0x0a,  0xac,  0xdb,  0x06,  0x55,  0xf1,  0x28,  0xcf,  0x21,  0x56,  0x4e, 
 0xbe,  0xe5,  0x84,  0x6f,  0x1b,  0x3c,  0xfb,  0x2b,  0x52,  0x00,  0x00,  0x00,  0x51,  0x00,  0x00,  0x00, 
 0xf1,  0x51,  0x01,  0x00,  0x01,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x41,  0x00,  0x00,  0x00, 
 0x02,  0x00,  0x00,  0x00,  0x19,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0xf1,  0x5e, 
 0x73,  0x97,  0xe7,  0xe8,  0x64,  0x40,  0xdf,  0x64,  0xaf,  0x76,  0xcd,  0x4c,  0xbc,  0x9a,  0x93,  0x2b, 
 0x3c,  0x00,  0x00,  0x00,  0x19,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0xf1,  0x5e, 
 0x73,  0x97,  0xe7,  0xe8,  0x64,  0x40,  0xdf,  0x64,  0xaf,  0x76,  0xcd,  0x4c,  0xbc,  0xd1,  0x8b,  0x86, 
 0x24,  0xf1,  0x5e,  0x73,  0x97,  0xe7,  0xe8,  0x64,  0x40,  0xdf,  0x64,  0xaf,  0x76,  0xcd,  0x4c,  0xbc, 
 0x43,  0x00,  0x00,  0x00,  0xf1,  0x51,  0x01,  0x00,  0x01,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00, 
 0x33,  0x00,  0x00,  0x00,  0x03,  0x00,  0x00,  0x00,  0x0b,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x0a,  0x9d,  0xd4,  0xe4,  0x61,  0x00,  0x0b,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x0a,  0x41,  0x52,  0x90,  0x76,  0x00,  0x0b,  0x00,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x0a,  0xfb,  0xad,  0xe9,  0xe3,  0x00,  0xe4,  0x01,  0x00,  0x00,  0x03,  0x00,  0x00,  0x00, 
 0xf2,  0xf5,  0xc6,  0x39,  0xf6,  0x33,  0xf1,  0x5a,  0x6d,  0x7b,  0xe3,  0xa4,  0x7b,  0x7e,  0x15,  0x00, 
 0x99,  0x00,  0x00,  0x00,  0xf2,  0x51,  0x01,  0x00,  0x37,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00, 
 0x2f,  0x00,  0x00,  0x00,  0x67,  0x65,  0x6f,  0x6d,  0x65,  0x74,  0x72,  0x79,  0x5f,  0x6d,  0x73,  0x67, 
 0x73,  0x3a,  0x3a,  0x6d,  0x73,  0x67,  0x3a,  0x3a,  0x64,  0x64,  0x73,  0x5f,  0x3a,  0x3a,  0x54,  0x77, 
 0x69,  0x73,  0x74,  0x57,  0x69,  0x74,  0x68,  0x43,  0x6f,  0x76,  0x61,  0x72,  0x69,  0x61,  0x6e,  0x63, 
 0x65,  0x5f,  0x00,  0x00,  0x55,  0x00,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00,  0x24,  0x00,  0x00,  0x00, 
 0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0xf2,  0x66,  0x9d,  0xb6,  0x2b,  0xd5,  0x53,  0x1f,  0x0b,  0xe5, 
 0x2c,  0x2a,  0x17,  0x6d,  0x6f,  0x00,  0x00,  0x00,  0x06,  0x00,  0x00,  0x00,  0x74,  0x77,  0x69,  0x73, 
 0x74,  0x00,  0x00,  0x00,  0x25,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x90,  0xf3, 
 0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x24,  0x0a,  0x00,  0x00,  0x0b,  0x00,  0x00,  0x00, 
 0x63,  0x6f,  0x76,  0x61,  0x72,  0x69,  0x61,  0x6e,  0x63,  0x65,  0x00,  0x00,  0x00,  0xf2,  0x66,  0x9d, 
 0xb6,  0x2b,  0xd5,  0x53,  0x1f,  0x0b,  0xe5,  0x2c,  0x2a,  0x17,  0x6d,  0x6f,  0x92,  0x00,  0x00,  0x00, 
 0xf2,  0x51,  0x01,  0x00,  0x29,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x21,  0x00,  0x00,  0x00, 
 0x67,  0x65,  0x6f,  0x6d,  0x65,  0x74,  0x72,  0x79,  0x5f,  0x6d,  0x73,  0x67,  0x73,  0x3a,  0x3a,  0x6d, 
 0x73,  0x67,  0x3a,  0x3a,  0x64,  0x64,  0x73,  0x5f,  0x3a,  0x3a,  0x54,  0x77,  0x69,  0x73,  0x74,  0x5f, 
 0x00,  0x00,  0x00,  0x00,  0x5a,  0x00,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00,  0x25,  0x00,  0x00,  0x00, 
 0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0xf2,  0xac,  0xe2,  0x5e,  0x59,  0xb0,  0x1a,  0x7a,  0x3a,  0x5c, 
 0xda,  0xb3,  0x78,  0xfd,  0x32,  0x00,  0x00,  0x00,  0x07,  0x00,  0x00,  0x00,  0x6c,  0x69,  0x6e,  0x65, 
 0x61,  0x72,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x26,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0xf2,  0xac,  0xe2,  0x5e,  0x59,  0xb0,  0x1a,  0x7a,  0x3a,  0x5c,  0xda,  0xb3,  0x78,  0xfd, 
 0x32,  0x00,  0x00,  0x00,  0x08,  0x00,  0x00,  0x00,  0x61,  0x6e,  0x67,  0x75,  0x6c,  0x61,  0x72,  0x00, 
 0x00,  0x00,  0xf2,  0xac,  0xe2,  0x5e,  0x59,  0xb0,  0x1a,  0x7a,  0x3a,  0x5c,  0xda,  0xb3,  0x78,  0xfd, 
 0x32,  0x00,  0x00,  0x00,  0x78,  0x00,  0x00,  0x00,  0xf2,  0x51,  0x01,  0x00,  0x2b,  0x00,  0x00,  0x00, 
 0x00,  0x00,  0x00,  0x00,  0x23,  0x00,  0x00,  0x00,  0x67,  0x65,  0x6f,  0x6d,  0x65,  0x74,  0x72,  0x79, 
 0x5f,  0x6d,  0x73,  0x67,  0x73,  0x3a,  0x3a,  0x6d,  0x73,  0x67,  0x3a,  0x3a,  0x64,  0x64,  0x73,  0x5f, 
 0x3a,  0x3a,  0x56,  0x65,  0x63,  0x74,  0x6f,  0x72,  0x33,  0x5f,  0x00,  0x00,  0x40,  0x00,  0x00,  0x00, 
 0x03,  0x00,  0x00,  0x00,  0x10,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0x0a,  0x00, 
 0x02,  0x00,  0x00,  0x00,  0x78,  0x00,  0x00,  0x00,  0x10,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x0a,  0x00,  0x02,  0x00,  0x00,  0x00,  0x79,  0x00,  0x00,  0x00,  0x10,  0x00,  0x00,  0x00, 
 0x02,  0x00,  0x00,  0x00,  0x01,  0x00,  0x0a,  0x00,  0x02,  0x00,  0x00,  0x00,  0x7a,  0x00,  0x00,  0x00, 
 0x5e,  0x00,  0x00,  0x00,  0x03,  0x00,  0x00,  0x00,  0xf2,  0xf5,  0xc6,  0x39,  0xf6,  0x33,  0xf1,  0x5a, 
 0x6d,  0x7b,  0xe3,  0xa4,  0x7b,  0x7e,  0x15,  0xf1,  0x40,  0xaa,  0x70,  0xe1,  0xf7,  0x44,  0x45,  0x66, 
 0x8e,  0xcd,  0x3e,  0x4a,  0x4b,  0xa6,  0xf2,  0x66,  0x9d,  0xb6,  0x2b,  0xd5,  0x53,  0x1f,  0x0b,  0xe5, 
 0x2c,  0x2a,  0x17,  0x6d,  0x6f,  0xf1,  0x28,  0xcf,  0x21,  0x56,  0x4e,  0xbe,  0xe5,  0x84,  0x6f,  0x1b, 
 0x3c,  0xfb,  0x2b,  0x52,  0xf2,  0xac,  0xe2,  0x5e,  0x59,  0xb0,  0x1a,  0x7a,  0x3a,  0x5c,  0xda,  0xb3, 
 0x78,  0xfd,  0x32,  0xf1,  0x5e,  0x73,  0x97,  0xe7,  0xe8,  0x64,  0x40,  0xdf,  0x64,  0xaf,  0x76,  0xcd, 
 0x4c,  0xbc, };
  return blob;
}
template<> inline const uint8_t * TopicTraits<::geometry_msgs::msg::dds_::TwistWithCovariance_>::type_info_blob() {
  static const uint8_t blob[] = {
 0xc0,  0x00,  0x00,  0x00,  0x01,  0x10,  0x00,  0x40,  0x58,  0x00,  0x00,  0x00,  0x54,  0x00,  0x00,  0x00, 
 0x14,  0x00,  0x00,  0x00,  0xf1,  0x40,  0xaa,  0x70,  0xe1,  0xf7,  0x44,  0x45,  0x66,  0x8e,  0xcd,  0x3e, 
 0x4a,  0x4b,  0xa6,  0x00,  0x52,  0x00,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00,  0x34,  0x00,  0x00,  0x00, 
 0x02,  0x00,  0x00,  0x00,  0x14,  0x00,  0x00,  0x00,  0xf1,  0x28,  0xcf,  0x21,  0x56,  0x4e,  0xbe,  0xe5, 
 0x84,  0x6f,  0x1b,  0x3c,  0xfb,  0x2b,  0x52,  0x00,  0x55,  0x00,  0x00,  0x00,  0x14,  0x00,  0x00,  0x00, 
 0xf1,  0x5e,  0x73,  0x97,  0xe7,  0xe8,  0x64,  0x40,  0xdf,  0x64,  0xaf,  0x76,  0xcd,  0x4c,  0xbc,  0x00, 
 0x47,  0x00,  0x00,  0x00,  0x02,  0x10,  0x00,  0x40,  0x58,  0x00,  0x00,  0x00,  0x54,  0x00,  0x00,  0x00, 
 0x14,  0x00,  0x00,  0x00,  0xf2,  0xf5,  0xc6,  0x39,  0xf6,  0x33,  0xf1,  0x5a,  0x6d,  0x7b,  0xe3,  0xa4, 
 0x7b,  0x7e,  0x15,  0x00,  0x9d,  0x00,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00,  0x34,  0x00,  0x00,  0x00, 
 0x02,  0x00,  0x00,  0x00,  0x14,  0x00,  0x00,  0x00,  0xf2,  0x66,  0x9d,  0xb6,  0x2b,  0xd5,  0x53,  0x1f, 
 0x0b,  0xe5,  0x2c,  0x2a,  0x17,  0x6d,  0x6f,  0x00,  0x96,  0x00,  0x00,  0x00,  0x14,  0x00,  0x00,  0x00, 
 0xf2,  0xac,  0xe2,  0x5e,  0x59,  0xb0,  0x1a,  0x7a,  0x3a,  0x5c,  0xda,  0xb3,  0x78,  0xfd,  0x32,  0x00, 
 0x7c,  0x00,  0x00,  0x00, };
  return blob;
}
#endif //DDSCXX_HAS_TYPE_DISCOVERY

} //namespace topic
} //namespace cyclonedds
} //namespace eclipse
} //namespace org

namespace dds {
namespace topic {

template <>
struct topic_type_name<::geometry_msgs::msg::dds_::TwistWithCovariance_>
{
    static std::string value()
    {
      return org::eclipse::cyclonedds::topic::TopicTraits<::geometry_msgs::msg::dds_::TwistWithCovariance_>::getTypeName();
    }
};

}
}

REGISTER_TOPIC_TYPE(::geometry_msgs::msg::dds_::TwistWithCovariance_)

namespace org{
namespace eclipse{
namespace cyclonedds{
namespace core{
namespace cdr{

template<>
propvec &get_type_props<::geometry_msgs::msg::dds_::TwistWithCovariance_>();

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool write(T& streamer, const ::geometry_msgs::msg::dds_::TwistWithCovariance_& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!write(streamer, instance.twist(), prop))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!write(streamer, instance.covariance()[0], instance.covariance().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool write(S& str, const ::geometry_msgs::msg::dds_::TwistWithCovariance_& instance, bool as_key) {
  auto &props = get_type_props<::geometry_msgs::msg::dds_::TwistWithCovariance_>();
  str.set_mode(cdr_stream::stream_mode::write, as_key);
  return write(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool read(T& streamer, ::geometry_msgs::msg::dds_::TwistWithCovariance_& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!read(streamer, instance.twist(), prop))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!read(streamer, instance.covariance()[0], instance.covariance().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool read(S& str, ::geometry_msgs::msg::dds_::TwistWithCovariance_& instance, bool as_key) {
  auto &props = get_type_props<::geometry_msgs::msg::dds_::TwistWithCovariance_>();
  str.set_mode(cdr_stream::stream_mode::read, as_key);
  return read(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool move(T& streamer, const ::geometry_msgs::msg::dds_::TwistWithCovariance_& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!move(streamer, instance.twist(), prop))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!move(streamer, instance.covariance()[0], instance.covariance().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool move(S& str, const ::geometry_msgs::msg::dds_::TwistWithCovariance_& instance, bool as_key) {
  auto &props = get_type_props<::geometry_msgs::msg::dds_::TwistWithCovariance_>();
  str.set_mode(cdr_stream::stream_mode::move, as_key);
  return move(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool max(T& streamer, const ::geometry_msgs::msg::dds_::TwistWithCovariance_& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!max(streamer, instance.twist(), prop))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!max(streamer, instance.covariance()[0], instance.covariance().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool max(S& str, const ::geometry_msgs::msg::dds_::TwistWithCovariance_& instance, bool as_key) {
  auto &props = get_type_props<::geometry_msgs::msg::dds_::TwistWithCovariance_>();
  str.set_mode(cdr_stream::stream_mode::max, as_key);
  return max(str, instance, props.data()); 
}

} //namespace cdr
} //namespace core
} //namespace cyclonedds
} //namespace eclipse
} //namespace org

#endif // DDSCXX_UNITREE_IDL_ROS2_TWISTWITHCOVARIANCE__HPP
