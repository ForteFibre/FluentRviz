#pragma once

#include <algorithm>
#include <cstddef>
#include <initializer_list>
#include <iterator>
#include <type_traits>
#include <utility>
#include <vector>

namespace flrv::traits
{
template <typename Into, typename From, typename Enabler = void>
struct convertor;

namespace detail
{
  template <typename Into, typename From>
  using detect_convertor = std::void_t<decltype(convertor<Into, From>{ })>;
}  // namespace detail

template <typename Into, typename From, typename Enable = void>
inline constexpr bool is_convertible = false;

template <typename Into, typename From>
inline constexpr bool is_convertible<Into, From, detail::detect_convertor<Into, From>> = true;

template <typename T>
struct convertor<T, T>
{
  static auto do_convert(const T &value) -> T
  { return value; }

  static auto do_convert(T &&value) -> T
  { return std::move(value); }
};

template <typename Into, typename From>
struct convertor<std::vector<Into>, std::vector<From>, std::enable_if_t<is_convertible<Into, From>>>
{
  static auto do_convert(const std::vector<From> &from) -> std::vector<Into>
  {
    auto into = std::vector<Into>{ };
    into.reserve(from.size());
    for (const auto &e : from) {
      into.emplace_back(convertor<Into, From>::do_convert(e));
    }
    return into;
  }
};

template <typename Into, typename From>
struct convertor<std::vector<Into>, std::initializer_list<From>, std::enable_if_t<is_convertible<Into, From>>>
{
  static auto do_convert(std::initializer_list<From> from) -> std::vector<Into>
  {
    auto into = std::vector<Into>{ };
    into.reserve(from.size());
    for (const auto &e : from) {
      into.emplace_back(convertor<Into, From>::do_convert(e));
    }
    return into;
  }
};

template <typename Into, typename From>
[[nodiscard]]
auto convert(const From &from) -> decltype(auto)
{ return convertor<Into, From>::do_convert(from); }

template <typename Into, typename From>
[[nodiscard]]
auto convert(From &&from) -> decltype(auto)
{ return convertor<Into, From>::do_convert(std::move(from)); }
}  // namespace flrv::traits
