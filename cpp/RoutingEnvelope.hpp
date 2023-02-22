#pragma once

#include "MsgPackSupport.hpp"

#include <string_view>

namespace tc {

/**
 * @brief A wrapper around a message that includes a route.
 *
 * @tparam MsgT The type of the wrapped message.
 */
template <typename MsgT>
struct RoutingEnvelope {
  std::string_view route;
  MsgT msg;
};

}  // namespace tc

template <typename MsgT>
struct MsgPackSerializer<tc::RoutingEnvelope<MsgT>> {
  MsgPackSerializerFor<std::string_view> route;
  MsgPackSerializerFor<MsgT> msg;

  MsgPackSerializer(tc::RoutingEnvelope<MsgT> const &envelope) : route{envelope.route}, msg{envelope.msg} {}

  KNL_MSGPACK_SERIALIZABLE(route, msg)
};

template <typename MsgT>
struct MsgPackDeserializer<tc::RoutingEnvelope<MsgT>> {
  MsgPackDeserializerFor<std::string_view> route;
  MsgPackDeserializerFor<MsgT> msg;

  MsgPackDeserializer(tc::RoutingEnvelope<MsgT> &envelope) : route{envelope.route}, msg{envelope.msg} {}

  KNL_MSGPACK_DESERIALIZABLE(route, msg)
};