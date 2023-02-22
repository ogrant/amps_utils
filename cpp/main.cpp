#include "MarketDataMessage.hpp"
#include "TopicClient.hpp"

#include <iostream>

using Test = MsgPackSerializerFor<MarketDataMessage>;
using Test2 = MsgPackSerializerFor<MarketDataMessage const &>;

static constexpr bool kTest = IsMsgPackSerializableV<MarketDataMessage>;
static constexpr bool kTest2 = IsMsgPackSerializableV<MarketDataMessage const &>;
static constexpr bool kTest3 = IsMsgPackSerializableV<MsgPackRemoveCR<MarketDataMessage const &>>;

/**
 * @brief Example component that generically uses a data source.
 *
 */
class MyComponent {
  tc::SubscriptionId subId_{};

 public:
  template <typename DataSourceT>
  void setupWithDataSource(DataSourceT &source) {
    // clang-format off
    subId_ = source.sowAndSubscribe("Foo")
      .withRoute("A")
      .withFilter("/id=123")
      .withCallbacks(
        [](tc::SowBegin) { std::cout << "SowBegin" << std::endl; },
        [](tc::SowEnd) { std::cout << "SowEnd" << std::endl; },
        [](tc::SowEntry, MarketDataMessage const &msg) { std::cout << "SowEntry " << msg << std::endl; },
        [](tc::Realtime, MarketDataMessage const &msg) { std::cout << "Realtime " << msg << std::endl; });
    // clang-format on
  }

  template <typename DataSourceT>
  void unsubscribe(DataSourceT &source) {
    source.unsubscribe(subId_);
  }
};

/**
 * @brief Example of how services would be implemented for both the production and unit testing.
 *
 */
class Service {
 protected:
  MyComponent *component_{nullptr};

 public:
  explicit Service(MyComponent &component) : component_{&component} {}
  virtual ~Service() = default;

  virtual void start() = 0;
};

void startDataSource(Service &service) {
  service.start();
}

class DataSourceService : public Service {
  tc::TopicClient client_;

 public:
  using Service::Service;

  auto &client() { return client_; }
  void start() override { component_->setupWithDataSource(client_); }
};

class DataSourceMockService : public Service {
  tc::TopicClientMock client_;

 public:
  using Service::Service;

  auto &client() { return client_; }
  void start() override { component_->setupWithDataSource(client_); }
};

/**
 * @brief Example usage of the topic client.
 *
 * @return int Return code of the program.
 */
void test_topic_client() {
  MyComponent component;
  DataSourceService dataSource{component};
  startDataSource(dataSource);
}

void test_topic_client_mock() {
  MyComponent component;
  DataSourceMockService dataSource{component};
  startDataSource(dataSource);
  // Example test code:
  auto &client = dataSource.client();
  auto &topic = client.getTopicEntryPoint("Foo");
  std::string const route = "A";
  MarketDataMessage msg;
  msg.id = 1;
  topic.sowBegin();
  topic.sowEntry(msg, route);
  topic.sowEnd();
  msg.id = 2;
  topic.realtime(msg, route);
  msg.id = 3;
  client.publish("Foo", route, msg);
}

int main() {
  std::cout << "kTest=" << kTest << std::endl;
  std::cout << "kTest2=" << kTest2 << std::endl;
  std::cout << "kTest3=" << kTest3 << std::endl;
  test_topic_client();
  test_topic_client_mock();
  return 0;
}
