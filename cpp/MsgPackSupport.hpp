#pragma once

#include <msgpack.hpp>

#include <experimental/type_traits>

#define KNL_MSGPACK_DESERIALIZABLE(...)                                                            \
  void msgpack_unpack(msgpack::object const &msgpack_o) {                                          \
    msgpack::type::make_define_map MSGPACK_DEFINE_MAP_IMPL(__VA_ARGS__).msgpack_unpack(msgpack_o); \
  }

#define KNL_MSGPACK_SERIALIZABLE(...)                                                                         \
  template <typename PackerT>                                                                                 \
  void msgpack_pack(PackerT &msgpack_pk) const {                                                              \
    msgpack::type::make_define_map MSGPACK_DEFINE_MAP_IMPL(__VA_ARGS__).msgpack_pack(msgpack_pk);             \
  }                                                                                                           \
  template <typename ObjectT>                                                                                 \
  void msgpack_object(ObjectT *msgpack_o, msgpack::zone &msgpack_z) const {                                   \
    msgpack::type::make_define_map MSGPACK_DEFINE_MAP_IMPL(__VA_ARGS__).msgpack_object(msgpack_o, msgpack_z); \
  }

template <typename T>
struct MsgPackSerializer;

template <typename T>
struct MsgPackDeserializer;

template <typename T>
using MsgPackRemoveCR = std::remove_const_t<std::remove_reference_t<T>>;

template <typename T>
using IsMsgPackSerializableHelper = decltype(std::declval<MsgPackSerializer<T> const &>().msgpack_pack(
    std::declval<msgpack::packer<msgpack::sbuffer> &>()));

template <typename T>
constexpr bool IsMsgPackSerializableV = std::experimental::is_detected_v<IsMsgPackSerializableHelper, T>;

template <typename T>
using IsMsgPackDeserializableHelper =
    decltype(std::declval<MsgPackDeserializer<T> &>().msgpack_unpack(std::declval<msgpack::object const &>()));

template <typename T>
constexpr bool IsMsgPackDeserializableV = std::experimental::is_detected_v<IsMsgPackDeserializableHelper, T>;

template <typename T>
using MsgPackSerializerFor =
    std::conditional_t<IsMsgPackSerializableV<MsgPackRemoveCR<T>>, MsgPackSerializer<MsgPackRemoveCR<T>>, T const &>;

template <typename T>
using MsgPackDeserializerFor =
    std::conditional_t<IsMsgPackDeserializableV<MsgPackRemoveCR<T>>, MsgPackDeserializer<MsgPackRemoveCR<T>>, T &>;