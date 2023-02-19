#pragma once

#include <iostream>
#include <string>

/**
 * @brief AMPS Stub.
 *
 */
namespace AMPS {

struct Message {
  struct Command {
    enum Type {
      GroupBegin,
      GroupEnd,
      OOF,
      Publish,
      SOW,
      Subscribe,
      SOWAndSubscribe,
    };
  };

  Command::Type cmd_ = Command::Type::SOW;
  std::string bookmark_{};
  std::string buffer_{};

  struct Data {
    char const *ptr_ = nullptr;
    std::size_t size_ = 0;

    char const *data() const { return ptr_; }
    std::size_t len() const { return size_; }
  };

  auto getCommandEnum() const { return cmd_; }
  auto getBookmark() const { return bookmark_; }
  auto getData() const { return Data{buffer_.data(), buffer_.size()}; }
};

class Command {
 public:
  explicit Command(Message::Command::Type const) {}
  void setTopic(std::string const &s) { std::cout << "setTopic: " << s << std::endl; }
  void setFilter(std::string const &s) { std::cout << "setFilter: " << s << std::endl; }
  void setOrder(std::string const &s) { std::cout << "setOrder: " << s << std::endl; }
};

class HAClient {
 public:
  template <typename OpT>
  std::string executeAsync(Command const &cmd, OpT &&) {
    return "dummySubId";
  }

  void unsubscribe(std::string const &subId) { std::cout << "unsubscribe: " << subId << std::endl; }
  void unsubscribe() { std::cout << "unsubscribe: all" << std::endl; }
};

}  // namespace AMPS