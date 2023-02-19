#pragma once

#include <tuple>
#include <type_traits>

/**
 * @brief Function traits.
 *
 * @tparam FnT The function type.
 */
template <typename FnT>
struct FnTraits;

template <typename R, typename... Args>
struct FnTraits<R(Args...)> {
  using ReturnType = R;
  using ArgsAsTuple = std::tuple<Args...>;

  static constexpr inline std::size_t kArity = sizeof...(Args);

  template <std::size_t I, bool = (I < kArity)>
  struct ArgHelper {
    using Type = std::tuple_element_t<I, ArgsAsTuple>;
  };

  template <std::size_t I>
  struct ArgHelper<I, false> {
    using Type = void;
  };

  template <std::size_t I>
  using Arg = typename ArgHelper<I>::Type;
};

template <typename R, typename... ArgsVT>
struct FnTraits<R (*)(ArgsVT...)> : FnTraits<R(ArgsVT...)> {};

template <typename R, typename C, typename... ArgsVT>
struct FnTraits<R (C::*)(ArgsVT...)> : FnTraits<R(ArgsVT...)> {};

template <typename R, typename C, typename... ArgsVT>
struct FnTraits<R (C::*)(ArgsVT...) const> : FnTraits<R(ArgsVT...)> {};

namespace unit_tests {

/**
 * @addtogroup Unit tests
 * @{
 */
static_assert(std::is_same_v<FnTraits<void()>::ReturnType, void>);
static_assert(std::is_same_v<FnTraits<void()>::ArgsAsTuple, std::tuple<>>);
static_assert(FnTraits<void()>::kArity == 0);
static_assert(std::is_same_v<FnTraits<void()>::Arg<0>, void>);

static_assert(std::is_same_v<FnTraits<void(int)>::ReturnType, void>);
static_assert(std::is_same_v<FnTraits<void(int)>::ArgsAsTuple, std::tuple<int>>);
static_assert(FnTraits<void(int)>::kArity == 1);
static_assert(std::is_same_v<FnTraits<void(int)>::Arg<0>, int>);
static_assert(std::is_same_v<FnTraits<void(int)>::Arg<1>, void>);

static_assert(std::is_same_v<FnTraits<std::string const &(int)>::ReturnType, std::string const &>);
static_assert(std::is_same_v<FnTraits<std::string const &(int)>::ArgsAsTuple, std::tuple<int>>);
static_assert(FnTraits<std::string const &(int)>::kArity == 1);
static_assert(std::is_same_v<FnTraits<std::string const &(int)>::Arg<0>, int>);
static_assert(std::is_same_v<FnTraits<std::string const &(int)>::Arg<1>, void>);
/**
 * @}
 */

}  // namespace unit_tests