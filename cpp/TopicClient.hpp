#include "AMPSStubs.hpp"
#include "FnTraits.hpp"

#include <experimental/type_traits>
#include <regex>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace tc {

/**
 * @brief Indicates the beginning of a state-of-world.
 *
 */
struct SowBegin {};

/**
 * @brief Indicates the end of a state-of-world.
 *
 */
struct SowEnd {};

/**
 * @brief Indicates an entry part of the state-of-world.
 *
 */
struct SowEntry {};

/**
 * @brief Indicates an entry that was removed from the state-of-world.
 *
 */
struct SowDelete {};

/**
 * @brief Indicates a message that was received in real-time (outside of the state-of-world).
 *
 */
struct Realtime {};

/**
 * @brief Bookmark information.
 *
 */
class Bookmark {
  friend class TopicClient;
  friend class TopicClientMock;
  std::string value_;

 public:
  explicit operator bool() const noexcept { return not value_.empty(); }
};

/**
 * @brief Subscription id.
 *
 */
class SubscriptionId {
  friend class TopicClient;
  friend class TopicClientMock;
  std::string value_;

 public:
  explicit operator bool() const noexcept { return not value_.empty(); }
};

/**
 * @brief Helpers to determine the type of function arguments.
 * This is used to determine the message type present in the arguments of a function.
 */
namespace detail {

/**
 * @brief Deterime if a type is a control message.
 *
 * @tparam T The type to check.
 */
// clang-format off
template <typename T>
constexpr bool IsControlV =
    std::is_same_v<T, SowBegin> ||
    std::is_same_v<T, SowEnd> ||
    std::is_same_v<T, SowEntry> ||
    std::is_same_v<T, SowDelete> ||
    std::is_same_v<T, Realtime> ||
    std::is_same_v<T, Bookmark>;
// clang-format on

/**
 * @brief Operator traits.
 *
 */
template <typename OpT>
struct OpTraits {
  template <typename ArgT>
  static constexpr inline bool IsValidArgV =
      not(std::is_same_v<ArgT, void> || IsControlV<std::remove_const_t<std::remove_reference_t<ArgT>>>);

  using Helper = FnTraits<decltype(&OpT::operator())>;
  using Arg0 = typename Helper::template Arg<0>;
  using Arg1 = typename Helper::template Arg<1>;
  using Arg2 = typename Helper::template Arg<2>;

  // clang-format off
  using Type = std::remove_const_t<std::remove_reference_t<
    std::conditional_t<
      IsValidArgV<Arg2>,
      Arg2,
      std::conditional_t<
        IsValidArgV<Arg1>,
        Arg1,
        std::conditional_t<
          IsValidArgV<Arg0>,
          Arg0,
          void
        >
      >
    >
  >>;
  // clang-format on
};

namespace unit_test {

template <typename MsgT, typename OpT>
constexpr bool test_op_traits(OpT &&) {
  using Type = typename OpTraits<OpT>::Type;
  return std::is_same_v<Type, MsgT>;
};

struct MyMessage1 {};
struct MyMessage2 {};

static_assert(test_op_traits<MyMessage1>([](MyMessage1 const &msg) {}));
static_assert(test_op_traits<MyMessage1>([](Bookmark const &, MyMessage1 const &msg) {}));
static_assert(test_op_traits<MyMessage2>([](MyMessage2 const &msg) {}));
static_assert(test_op_traits<void>([](SowBegin) {}));
static_assert(test_op_traits<void>([](SowEnd) {}));

}  // namespace unit_test

}  // namespace detail

/**
 * @brief Simple type list implementation.
 *
 * @tpa$ram Ts Types of the list.
 */
template <typename... Ts>
struct TypeList {
  static inline constexpr auto kSize = sizeof...(Ts);

  template <std::size_t IdxNT>
  using At = std::tuple_element_t<IdxNT, std::tuple<Ts...>>;
};

/**
 * @brief Apply a template to a type list.
 *
 */
namespace detail {

template <template <typename...> class CallT, typename ListT>
struct ApplyImpl;

template <template <typename...> class CallT, typename... Ts>
struct ApplyImpl<CallT, TypeList<Ts...>> {
  using Type = CallT<Ts...>;
};

}  // namespace detail

template <template <typename...> class CallT, typename ListT>
using Apply = typename detail::ApplyImpl<CallT, ListT>::Type;

/**
 * @brief Remove void from a type list.
 *
 */
namespace detail {

template <typename ListT, typename ResultT = TypeList<>>
struct RemoveVoidImpl;

template <typename T, typename... Ts, typename... Rs>
struct RemoveVoidImpl<TypeList<T, Ts...>, TypeList<Rs...>> {
  using Type = typename RemoveVoidImpl<
      TypeList<Ts...>,
      std::conditional_t<std::is_same_v<T, void>, TypeList<Rs...>, TypeList<Rs..., T>>>::Type;
};

template <typename... Rs>
struct RemoveVoidImpl<TypeList<>, TypeList<Rs...>> {
  using Type = TypeList<Rs...>;
};

}  // namespace detail

template <typename... Ts>
using RemoveVoid = typename detail::RemoveVoidImpl<TypeList<Ts...>>::Type;

static_assert(std::is_same_v<RemoveVoid<int, double>, TypeList<int, double>>);
static_assert(std::is_same_v<RemoveVoid<int, void, double, void, char>, TypeList<int, double, char>>);
static_assert(std::is_same_v<RemoveVoid<void>, TypeList<>>);

/**
 * @brief Remove duplicate types from a type list.
 *
 */
namespace detail {

template <typename ListT, typename ResultT = TypeList<>>
struct RemoveDuplicatesImpl;

template <typename T, typename... Ts, typename... Rs>
struct RemoveDuplicatesImpl<TypeList<T, Ts...>, TypeList<Rs...>> {
  using Type = typename RemoveDuplicatesImpl<
      TypeList<Ts...>,
      std::conditional_t<(std::is_same_v<T, Rs> || ...), TypeList<Rs...>, TypeList<Rs..., T>>>::Type;
};

template <typename... Rs>
struct RemoveDuplicatesImpl<TypeList<>, TypeList<Rs...>> {
  using Type = TypeList<Rs...>;
};

}  // namespace detail

template <typename... Ts>
using RemoveDuplicates = typename detail::RemoveDuplicatesImpl<TypeList<Ts...>>::Type;

static_assert(std::is_same_v<RemoveDuplicates<>, TypeList<>>);
static_assert(std::is_same_v<RemoveDuplicates<int, double>, TypeList<int, double>>);
static_assert(std::is_same_v<RemoveDuplicates<int, double, int, double>, TypeList<int, double>>);

/**
 * @brief Indicates that no message type was defined.
 * This indicates that none of the lambdas passed as callbacks expect a message.
 */
struct NoMsgType {};

/**
 * @brief Indicates that multiple message types were defined.
 * This indicates that multiple messages types were defined by the lamdas passed as callbacks.
 */
struct MultipleMsgTypes {};

/**
 * @brief Determine the message type from a list of lambdas.
 *
 */
namespace detail {

template <typename ListT>
struct MessageTypeResult;

template <typename... Ts>
struct MessageTypeResult<TypeList<Ts...>> {
  using Type = MultipleMsgTypes;
};

template <>
struct MessageTypeResult<TypeList<>> {
  using Type = NoMsgType;
};

template <typename T>
struct MessageTypeResult<TypeList<T>> {
  using Type = T;
};

template <typename... OpVT>
struct MessageTypeImpl {
  using Step0 = TypeList<typename tc::detail::OpTraits<OpVT>::Type...>;
  using Step1 = Apply<RemoveVoid, Step0>;
  using Step2 = Apply<RemoveDuplicates, Step1>;
  using Type = typename MessageTypeResult<Step2>::Type;
};

}  // namespace detail

template <typename... OpVT>
using MessageType = typename detail::MessageTypeImpl<OpVT...>::Type;

namespace detail {

namespace unit_test {

template <typename ResultT, typename... OpVT>
constexpr bool test_message_type(OpVT &&...) {
  using Res = MessageType<OpVT...>;
  return std::is_same_v<ResultT, Res>;
}

// clang-format off
static_assert(test_message_type<NoMsgType>(
   [](SowBegin) {},
   [](SowEnd) {}));

static_assert(test_message_type<MultipleMsgTypes>(
   [](SowEnd) {},
   [](MyMessage1 const &) {},
   [](Bookmark const &, MyMessage2 const &) {}));

static_assert(test_message_type<MultipleMsgTypes>(
   [](SowEnd) {},
   [](SowEntry, MyMessage1 const &) {},
   [](Realtime, MyMessage2 const &) {}));

static_assert(test_message_type<MyMessage1>(
   [](SowEnd) {},
   [](MyMessage1 const &) {},
   [](Bookmark const &, MyMessage1 const &) {}));

static_assert(test_message_type<MyMessage2>(
   [](SowEnd) {},
   [](MyMessage2 const &) {},
   [](Bookmark const &, MyMessage2 const &) {}));

static_assert(test_message_type<MyMessage1>(
   [](SowBegin) {},
   [](SowEnd) {},
   [](SowEntry, MyMessage1 const &) {},
   [](Realtime, Bookmark const &, MyMessage1 const &) {}));
// clang-format on

}  // namespace unit_test

}  // namespace detail

/**
 * @brief Trait to determine if a function object supports a set of arguments.
 *
 */
namespace detail {

template <typename T, typename... As>
using SupportsCallHelper = decltype(std::declval<T>()(std::declval<As>()...));

}  // namespace detail

template <typename T, typename... As>
constexpr bool SupportsCallV = std::experimental::is_detected_v<detail::SupportsCallHelper, T, As...>;

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

/**
 * @brief A routing envelope used for sending a message to avoid copies.
 * This is only used by the #TopicClient when publishing messages using a route.
 * @tparam MsgT The type of the wrapped message.
 */
template <typename MsgT>
using OutboundRoutingEnvelope = RoutingEnvelope<MsgT const &>;

/**
 * @brief Trait to determine if a type is a rounting envelope.
 *
 */
namespace detail {

template <typename T>
struct IsRoutingEnvelope : public std::false_type {};

template <typename MsgT>
struct IsRoutingEnvelope<RoutingEnvelope<MsgT>> : public std::true_type {};

}  // namespace detail

template <typename T>
constexpr bool IsRoutingEnvelopeV = detail::IsRoutingEnvelope<T>{};

namespace unit_test {

static_assert(not IsRoutingEnvelopeV<int>);
static_assert(IsRoutingEnvelopeV<RoutingEnvelope<int>>);
static_assert(IsRoutingEnvelopeV<RoutingEnvelope<std::string const &>>);

}  // namespace unit_test

/**
 * @brief Create a functor from multiple lambdas.
 *
 * @tparam OpVT The types of the lambdas.
 */
template <typename... OpVT>
struct Overloaded : public OpVT... {
  using OpVT::operator()...;
};

template <typename... OpVT>
Overloaded(OpVT...) -> Overloaded<std::decay_t<OpVT>...>;

/**
 * @brief AMPS callback wrapper.
 *
 */
template <typename MsgT, bool UseEnvelopeNT, typename... OpVT>
class TopicClientCallback : public Overloaded<OpVT...> {
  /**
   * @brief Attempt to dispatch a message to the callback.
   *
   * @tparam UsedMsgT The used message type (either MsgT or RoutingEnvelope<MsgT>).
   * @tparam CbT The callback type.
   * @tparam TagT The tag type.
   * @param cb The callback object.
   * @param msgIn The AMPS message to dispatch.
   */
  template <typename UsedMsgT, typename CbT, typename TagT>
  static void doDispatchMessage(CbT &cb, TagT, AMPS::Message const &msgIn) {
    static_assert(std::is_same_v<UsedMsgT, MsgT> || std::is_same_v<UsedMsgT, RoutingEnvelope<MsgT>>);
#if 0
    // TODO: Implement proper deserialization
    UsedMsgT msgOut;
    MsgT const &payload = [&msgOut]{ if constexpr (IsRoutingEnvelopeV<UsedMsgT>) { return msgOut.msg; } else { return msgOut; } }();
    MsgPackDeserializer<UsedMsgT> unpacker{msgOut};
    auto const handle = msgpack::unpack(msgIn.getData().data(), msgIn.getData().len());
    handle.get().convert(unpacker);
#endif
    MsgT const payload;
    if constexpr (SupportsCallV<CbT, TagT, tc::Bookmark const &, MsgT const &>) {
      tc::Bookmark bm;
      // TODO: bm.value_ = msgIn.getBookmark();
      cb(TagT{}, bm, payload);
    }
    else if constexpr (SupportsCallV<CbT, TagT, MsgT const &>) {
      cb(TagT{}, payload);
    }
    else if constexpr (SupportsCallV<CbT, tc::Bookmark const &, MsgT const &>) {
      tc::Bookmark bm;
      // TODO: bm.value_ = msgIn.getBookmark();
      cb(bm, payload);
    }
    else if constexpr (SupportsCallV<CbT, MsgT const &>) {
      cb(payload);
    }
  }

 public:
  /**
   * @brief Construct a new TopicClientCallback from the defined callbacks.
   *
   * @param ops The callbacks.
   */
  explicit TopicClientCallback(OpVT &&...ops) : Overloaded<OpVT...>{std::forward<OpVT>(ops)...} {}

  /**
   * @brief Dispatch a received AMPS message to any compatible callback.
   *
   * @param msg The AMPS message.
   */
  void dispatch(AMPS::Message const &msg) {
    using UsedMsgType = std::conditional_t<UseEnvelopeNT, RoutingEnvelope<MsgT>, MsgT>;
    switch (msg.getCommandEnum()) {
    case AMPS::Message::Command::GroupBegin:
      if constexpr (SupportsCallV<TopicClientCallback, tc::SowBegin>) this->operator()(tc::SowBegin{});
      break;
    case AMPS::Message::Command::GroupEnd:
      if constexpr (SupportsCallV<TopicClientCallback, tc::SowEnd>) this->operator()(tc::SowEnd{});
      break;
    case AMPS::Message::Command::SOW:
      doDispatchMessage<UsedMsgType>(*this, tc::SowEntry{}, msg);
      break;
    case AMPS::Message::Command::OOF:
      doDispatchMessage<UsedMsgType>(*this, tc::SowDelete{}, msg);
      break;
    case AMPS::Message::Command::Publish:
      doDispatchMessage<UsedMsgType>(*this, tc::Realtime{}, msg);
      break;
    default:
      break;
    }
  }
};

/**
 * @brief AMPS based topic client.
 *
 * The TopicClient is a wrapper around the AMPS HAClient class and provides the following extra features:
 * - Dispatching of AMPS::Message to corresponding callbacks if defined.
 * - Support for using routing envelopes.
 *
 * Supported callbacks:
 * - (SowBegin): Called just before receiving the first snapshot entry.
 * - (SowEnd): Called once al snapshot entries have been received.
 * - (SowEntry, MsgT const &): Called for each snapshot entry.
 * - (SowEntry, Bookmark const &, MsgT const &): Called for each snapshot entry (with bookmark).
 * - (SowDelete, MsgT const &): Called for each entry removed from the state of world.
 * - (SowDelete, Bookmark const &, MsgT const &): Called for each entry removed from the state of world (with bookmark).
 * - (Realtime, MsgT const &): Called for each realtime update.
 * - (Realtime, Bookmark const &, MsgT const &): Called for each realtime update (with bookmark).
 * - (MsgT const &): Called for both snapshot entries and realtime updates.
 * - (Bookmark const &, MsgT const &): Called for both snapshot entries and realtime updates (with bookmark).
 *
 * Note that the callbacks are called in the context of the AMPS thread.
 *
 * Example code:
 *
 *     TopicClient client;
 *     // Get a snapshot of all entries in topic "MyTopic".
 *     auto const subId0 = client
 *        .sow("MyTopic")
 *        .withCallbacks(
 *           [](tc::SowBegin) { std::cout << "Snapshot begin" << std::endl; },
 *           [](tc::SowEnd) { std::cout << "Snapshot end" << std::endl; },
 *           [](MyMessage const &msg) { std::cout << "Snapshot entry" << std::endl; }
 *        );
 *
 *     // Subscribe to realtime updates to the topic "MyTopic".
 *     auto const subId0 = client
 *        .subscribe("MyTopic")
 *        .withCallbacks(
 *           [](MyMessage const &msg) { std::cout << "Realtime entry" << std::endl; }
 *        );
 *
 *     // Get a snapshot of all entries in topic "MyTopic" and any subsequent realtime update.
 *     // The single callback defined will receive both snapshot and realtime entries.
 *     auto const subId0 = client
 *        .sowAndSubscribe("MyTopic")
 *        .withCallbacks(
 *           [](MyMessage const &msg) { std::cout << "Snapshot or realtime entry" << std::endl; }
 *        );
 *
 *     // Get a snapshot of all entries in topic "MyTopic" and any subsequent realtime update.
 *     // Two different callbacks are defined to handle snapshot and realtime entries.
 *     auto const subId0 = client
 *        .sowAndSubscribe("MyTopic")
 *        .withCallbacks(
 *           [](tc::SowEntry, MyMessage const &msg) { std::cout << "Snapshot entry" << std::endl; },
 *           [](tc::Realtime, MyMessage const &msg) { std::cout << "Realtime entry" << std::endl; }
 *        );
 */
class TopicClient {
  AMPS::HAClient client_;

  /**
   * @brief Helper class to build a subscription.
   *
   */
  class SubscriptionBuilder {
    friend class TopicClient;

    TopicClient &client_;
    AMPS::Message::Command::Type type_{};
    std::string topic_{};
    std::string route_{};
    std::string filter_{};
    std::string order_{};

    /**
     * @brief Construct a new SubscriptionBuilder object
     *
     * @param client The topic client this builder is associated with.
     */
    explicit SubscriptionBuilder(TopicClient &client) : client_(client) {}

    /**
     * @brief Create the filter string based on the route and filter options.
     * This function will adjust the filter specified depending on if route is defined.
     * @param route The route to use, or empty if no route should be used.
     * @param filter The filter to use.
     * @return The adjusted filter string.
     */
    std::string doCreateFilter(std::string const &route, std::string const &filter) {
      if (route.empty()) return filter;
      if (filter.empty()) return "/route=\"" + route + "\"";
      std::regex const r(R"!(((?:\/\w+)+))!");
      auto const updatedFilter = std::regex_replace(filter, r, "/msg$1");
      return route != "*" ? "/route=\"" + route + "\" AND (" + updatedFilter + ")" : updatedFilter;
    }

    /**
     * @brief Configure the given command.
     * The function will set the topic, filter and order.
     * @param cmd The command object to configure.
     */
    void doConfigureCommand(AMPS::Command &cmd) {
      cmd.setTopic(topic_);
      auto const filter = doCreateFilter(route_, filter_);
      if (not filter.empty()) cmd.setFilter(filter);
      if (not order_.empty()) cmd.setOrder(order_);
    }

   public:
    /**
     * @brief Set the subscription filter.
     * Filtering is done on the server side and is based on the fields of the message.
     * @param filter The string representing the filter to apply.
     * @return The subscription builder.
     */
    SubscriptionBuilder &withFilter(std::string const &filter) {
      filter_ = filter;
      return *this;
    }

    /**
     * @brief Set the subscription route.
     * Routing is an extra layer of filtering that is used in conjuction with a topic using routing envelopes.
     * @param route The string representing the route to filter on.
     * @return The subscription builder.
     * @note This only works for topics supporting routing.
     */
    SubscriptionBuilder &withRoute(std::string const &route) {
      route_ = route;
      return *this;
    }

    /**
     * @brief Set the order in which entries of the subscription should be returned.
     * Ordering is done on the server side and is based on the fields of the message.
     * @param order The string representing the order to apply.
     * @return The subscription builder.
     * @note This is only supported for sow subscriptions.
     */
    SubscriptionBuilder &withOrder(std::string const &order) {
      order_ = order;
      return *this;
    }

    /**
     * @brief Define the callbacks to be called on the various subscription events.
     * At least one callback must define the expected message type.
     * @tparam OpVT The callback types.
     * @param ops The callbacks.
     * @return The subscription id.
     */
    template <typename... OpVT>
    SubscriptionId withCallbacks(OpVT &&...ops) {
      using MessageType = tc::MessageType<OpVT...>;
      static_assert(not std::is_same_v<MessageType, tc::MultipleMsgTypes>, "Multiple message types");
      static_assert(not std::is_same_v<MessageType, tc::NoMsgType>, "No message type");
      AMPS::Command cmd{type_};
      doConfigureCommand(cmd);
      SubscriptionId subId{};
      if (route_.empty()) {
        subId.value_ = client_.nativeClient().executeAsync(
            cmd, TopicClientCallback<MessageType, false, OpVT...>(std::forward<OpVT>(ops)...));
      }
      else {
        subId.value_ = client_.nativeClient().executeAsync(
            cmd, TopicClientCallback<MessageType, true, OpVT...>(std::forward<OpVT>(ops)...));
      }
      return subId;
    }
  };

 public:
  /**
   * @brief Access the native AMPS client.
   * This should be avoided as much as possible.
   * @return The native AMPS client.
   */
  AMPS::HAClient &nativeClient() { return client_; }

  /**
   * @brief Create a subscription to receive a topic snapshot.
   * This subscription will receive a snapshot of all topic entries. No realtime updates will be received.
   * @param topic The topic to subscribe to.
   * @return A subscription builder.
   */
  auto sow(std::string const &topic) {
    SubscriptionBuilder builder(*this);
    builder.type_ = AMPS::Message::Command::SOW;
    builder.topic_ = topic;
    return builder;
  }

  /**
   * @brief Create a subscription for realtime updates.
   * This subscription will only receive realtime updates to the topic. No initial snapshot will be received.
   * @param topic The topic to subscribe to.
   * @return A subscription builder.
   */
  auto subscribe(std::string const &topic) {
    SubscriptionBuilder builder(*this);
    builder.type_ = AMPS::Message::Command::Subscribe;
    builder.topic_ = topic;
    return builder;
  }

  /**
   * @brief Create a subscription for snapshot and realtime updates.
   * This subscription will receive a snapshot of all topic entries and then receive realtime updates.
   * @param topic
   * @return auto
   */
  auto sowAndSubscribe(std::string const &topic) {
    SubscriptionBuilder builder(*this);
    builder.type_ = AMPS::Message::Command::SOWAndSubscribe;
    builder.topic_ = topic;
    return builder;
  }

  /**
   * @brief Stop the specified subscription.
   *
   * @param subId The subscription id.
   */
  void unsubscribe(SubscriptionId const &subId) { client_.unsubscribe(subId.value_); }

  /**
   * @brief Stop all subscriptions.
   *
   */
  void unsubscribeAll() { client_.unsubscribe(); }
};

/**
 * @brief Presents the same interface as TopicClient, allowing to the send messages manually.
 *
 * @note This is to be used for unit testing.
 */
class TopicClientMock {
  struct Callback {
    std::string callbackId_;
    AMPS::Message::Command::Type type_;
    std::function<void(AMPS::Message const &)> cb_;
  };

  class TopicInfo {
    friend class TopicClientMock;

    TopicClientMock *client_{nullptr};
    std::string topic_;
    std::uint64_t nextCallbackId_ = 1;
    std::vector<Callback> callbacks_;

    template <typename MsgT>
    void doEncode(MsgT const &msg, AMPS::Message &msgOut) const {
      msgOut.bookmark_ = client_->doGetBookmark();
      // TODO: Encode msg into msgOut.buffer_.
    }

    void doDispatch(AMPS::Message const &msg) const {
      for (auto const &cb : callbacks_) {
        switch (msg.getCommandEnum()) {
        case AMPS::Message::Command::GroupBegin:
          if (AMPS::Message::Command::Subscribe == cb.type_) continue;
          break;
        case AMPS::Message::Command::GroupEnd:
          if (AMPS::Message::Command::Subscribe == cb.type_) continue;
          break;
        case AMPS::Message::Command::SOW:
          if (AMPS::Message::Command::Subscribe == cb.type_) continue;
          break;
        case AMPS::Message::Command::Publish:
          if (AMPS::Message::Command::SOW == cb.type_) continue;
          break;
        case AMPS::Message::Command::OOF:
          if (AMPS::Message::Command::SOW == cb.type_) continue;
          break;
        default:
          break;
        }
        cb.cb_(msg);
      }
    }

    void doAddCallback(Callback const &cb) {
      callbacks_.push_back(cb);
      callbacks_.back().callbackId_ = topic_ + ":" + std::to_string(nextCallbackId_++);
    }

    void doRemoveCallback(std::string const &callbackId) {
      auto it = std::find_if(
          callbacks_.begin(), callbacks_.end(), [&callbackId](auto const &cb) { return cb.callbackId_ == callbackId; });
      if (it != callbacks_.end()) {
        callbacks_.erase(it);
      }
    }

   public:
    void sowBegin() const {
      AMPS::Message msgOut;
      msgOut.cmd_ = AMPS::Message::Command::GroupBegin;
      doDispatch(msgOut);
    }

    void sowEnd() const {
      AMPS::Message msgOut;
      msgOut.cmd_ = AMPS::Message::Command::GroupEnd;
      doDispatch(msgOut);
    }

    template <typename MsgT>
    void sowEntry(MsgT const &msg) const {
      AMPS::Message msgOut;
      msgOut.cmd_ = AMPS::Message::Command::SOW;
      doEncode(msg, msgOut);
      doDispatch(msgOut);
    }

    template <typename MsgT>
    void sowDelete(MsgT const &msg) const {
      AMPS::Message msgOut;
      msgOut.cmd_ = AMPS::Message::Command::OOF;
      doEncode(msg, msgOut);
      doDispatch(msgOut);
    }

    template <typename MsgT>
    void realtime(MsgT const &msg) const {
      AMPS::Message msgOut;
      msgOut.cmd_ = AMPS::Message::Command::Publish;
      doEncode(msg, msgOut);
      doDispatch(msgOut);
    }
  };

  std::uint64_t mutable bookmark_;
  std::unordered_map<std::string, TopicInfo> topics_;

  class SubscriptionBuilder {
    friend class TopicClientMock;

    TopicClientMock &client_;
    AMPS::Message::Command::Type type_{};
    std::string topic_{};
    std::string route_{};
    std::string filter_{};
    std::string order_{};

    explicit SubscriptionBuilder(TopicClientMock &client) : client_(client) {}

   public:
    SubscriptionBuilder &withFilter(std::string const &filter) {
      filter_ = filter;
      return *this;
    }

    SubscriptionBuilder &withRoute(std::string const &route) {
      route_ = route;
      return *this;
    }

    SubscriptionBuilder &withOrder(std::string const &order) {
      order_ = order;
      return *this;
    }

    template <typename... OpVT>
    tc::SubscriptionId withCallbacks(OpVT &&...ops) {
      using MessageType = tc::MessageType<OpVT...>;
      static_assert(not std::is_same_v<MessageType, tc::MultipleMsgTypes>, "Multiple message types");
      static_assert(not std::is_same_v<MessageType, tc::NoMsgType>, "No message type");
      Callback cb{.type_ = type_, .cb_ = nullptr};
      if (route_.empty()) {
        cb.cb_ = [tcc = std::make_shared<tc::TopicClientCallback<MessageType, false, OpVT...>>(
                      std::forward<OpVT>(ops)...)](AMPS::Message const &msg) { tcc->dispatch(msg); };
      }
      else {
        cb.cb_ = [tcc = std::make_shared<tc::TopicClientCallback<MessageType, true, OpVT...>>(
                      std::forward<OpVT>(ops)...)](AMPS::Message const &msg) { tcc->dispatch(msg); };
      }
      auto &topic = client_.topics_[topic_];
      topic.client_ = &client_;
      topic.topic_ = topic_;
      topic.doAddCallback(cb);
      SubscriptionId subId;
      subId.value_ = topic.callbacks_.back().callbackId_;
      return subId;
    }
  };

  std::string doGetBookmark() const { return std::to_string(++bookmark_); }

 public:
  auto sow(std::string const &topic) {
    SubscriptionBuilder builder(*this);
    builder.type_ = AMPS::Message::Command::SOW;
    builder.topic_ = topic;
    return builder;
  }

  auto subscribe(std::string const &topic) {
    SubscriptionBuilder builder(*this);
    builder.type_ = AMPS::Message::Command::Subscribe;
    builder.topic_ = topic;
    return builder;
  }

  auto sowAndSubscribe(std::string const &topic) {
    SubscriptionBuilder builder(*this);
    builder.type_ = AMPS::Message::Command::SOWAndSubscribe;
    builder.topic_ = topic;
    return builder;
  }

  template <typename MsgT>
  void publish(std::string const &topic, MsgT const &msg) {
    getTopicEntryPoint(topic).realtime(msg);
  }

  void unsubscribe(tc::SubscriptionId const &subId) {
    auto const idx = subId.value_.find(':');
    if (std::string::npos == idx) {
      throw std::runtime_error("Invalid subscription subId: " + subId.value_);
    }
    auto topic = subId.value_.substr(0, idx);
    auto const iter = topics_.find(topic);
    if (topics_.end() == iter) {
      throw std::runtime_error("Topic '" + topic + "' not found");
    }
    iter->second.doRemoveCallback(subId.value_);
  }

  TopicInfo const &getTopicEntryPoint(std::string const &topic) const {
    auto const iter = topics_.find(topic);
    if (topics_.end() == iter) {
      throw std::runtime_error("Topic '" + topic + "'not found");
    }
    return iter->second;
  }
};

}  // namespace tc
