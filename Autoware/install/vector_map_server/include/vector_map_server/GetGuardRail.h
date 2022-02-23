// Generated by gencpp from file vector_map_server/GetGuardRail.msg
// DO NOT EDIT!


#ifndef VECTOR_MAP_SERVER_MESSAGE_GETGUARDRAIL_H
#define VECTOR_MAP_SERVER_MESSAGE_GETGUARDRAIL_H

#include <ros/service_traits.h>


#include <vector_map_server/GetGuardRailRequest.h>
#include <vector_map_server/GetGuardRailResponse.h>


namespace vector_map_server
{

struct GetGuardRail
{

typedef GetGuardRailRequest Request;
typedef GetGuardRailResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetGuardRail
} // namespace vector_map_server


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::vector_map_server::GetGuardRail > {
  static const char* value()
  {
    return "9722667ca9b8d40b70c4d7e1b16e9ea0";
  }

  static const char* value(const ::vector_map_server::GetGuardRail&) { return value(); }
};

template<>
struct DataType< ::vector_map_server::GetGuardRail > {
  static const char* value()
  {
    return "vector_map_server/GetGuardRail";
  }

  static const char* value(const ::vector_map_server::GetGuardRail&) { return value(); }
};


// service_traits::MD5Sum< ::vector_map_server::GetGuardRailRequest> should match
// service_traits::MD5Sum< ::vector_map_server::GetGuardRail >
template<>
struct MD5Sum< ::vector_map_server::GetGuardRailRequest>
{
  static const char* value()
  {
    return MD5Sum< ::vector_map_server::GetGuardRail >::value();
  }
  static const char* value(const ::vector_map_server::GetGuardRailRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::vector_map_server::GetGuardRailRequest> should match
// service_traits::DataType< ::vector_map_server::GetGuardRail >
template<>
struct DataType< ::vector_map_server::GetGuardRailRequest>
{
  static const char* value()
  {
    return DataType< ::vector_map_server::GetGuardRail >::value();
  }
  static const char* value(const ::vector_map_server::GetGuardRailRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::vector_map_server::GetGuardRailResponse> should match
// service_traits::MD5Sum< ::vector_map_server::GetGuardRail >
template<>
struct MD5Sum< ::vector_map_server::GetGuardRailResponse>
{
  static const char* value()
  {
    return MD5Sum< ::vector_map_server::GetGuardRail >::value();
  }
  static const char* value(const ::vector_map_server::GetGuardRailResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::vector_map_server::GetGuardRailResponse> should match
// service_traits::DataType< ::vector_map_server::GetGuardRail >
template<>
struct DataType< ::vector_map_server::GetGuardRailResponse>
{
  static const char* value()
  {
    return DataType< ::vector_map_server::GetGuardRail >::value();
  }
  static const char* value(const ::vector_map_server::GetGuardRailResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // VECTOR_MAP_SERVER_MESSAGE_GETGUARDRAIL_H
