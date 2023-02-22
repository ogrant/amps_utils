#include "MsgPackSupport.hpp"

#include <iostream>

/**
 * @brief Dummy market data message.
 *
 */
struct MarketDataMessage {
  int id{};
  double price{};
  int quantity{};

  friend std::ostream &operator<<(std::ostream &os, MarketDataMessage const &msg) {
    os << "id: " << msg.id << ", price: " << msg.price << ", quantity: " << msg.quantity;
    return os;
  }
};

template <>
struct MsgPackSerializer<MarketDataMessage> {
  MsgPackSerializerFor<int> id;
  MsgPackSerializerFor<double> price;
  MsgPackSerializerFor<int> quantity;

  MsgPackSerializer(MarketDataMessage const &msg) : id{msg.id}, price{msg.price}, quantity{msg.quantity} {}

  KNL_MSGPACK_SERIALIZABLE(id, price, quantity)
};

static_assert(IsMsgPackSerializableV<MarketDataMessage>);

template <>
struct MsgPackDeserializer<MarketDataMessage> {
  MsgPackDeserializerFor<int> id;
  MsgPackDeserializerFor<double> price;
  MsgPackDeserializerFor<int> quantity;

  MsgPackDeserializer(MarketDataMessage &msg) : id{msg.id}, price{msg.price}, quantity{msg.quantity} {}

  KNL_MSGPACK_DESERIALIZABLE(id, price, quantity)
};

static_assert(IsMsgPackDeserializableV<MarketDataMessage>);