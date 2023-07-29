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
template <typename ...Constraint>
using Require = std::nullptr_t;

template <typename Into, typename From, typename Enabler = void>
struct Convert;

template <typename Into, typename From>
using ConversionDefined =
  decltype(Convert<Into, std::remove_cv_t<std::remove_reference_t<From>>>::do_convert(std::declval<From>()));

template <typename T>
struct Convert<T, T>
{
  static auto do_convert(const T &value) -> T
  { return value; }

  static auto do_convert(T &&value) -> T
  { return std::move(value); }
};

template <typename Into, typename From>
struct Convert<std::vector<Into>, std::vector<From>, std::void_t<ConversionDefined<Into, From>>>
{
  static auto do_convert(const std::vector<From> &from) -> std::vector<Into>
  {
    auto into = std::vector<Into>{ };
    into.reserve(from.size());
    for (const auto &e : from) {
      into.emplace_back(Convert<Into, From>::do_convert(e));
    }
    return into;
  }

  template<
    traits::Require<
      std::enable_if_t<std::is_same_v<From, Into>>> = nullptr>
  static auto do_convert(std::vector<From> &&from) -> std::vector<Into>
  { return std::move(from); }
};

template <typename Into, typename From>
struct Convert<std::vector<Into>, std::initializer_list<From>, std::void_t<ConversionDefined<Into, From>>>
{
  static auto do_convert(std::initializer_list<From> from) -> std::vector<Into>
  {
    auto into = std::vector<Into>{ };
    into.reserve(from.size());
    for (const auto &e : from) {
      into.emplace_back(Convert<Into, From>::do_convert(e));
    }
    return into;
  }
};

template <typename Into, typename From>
[[nodiscard]]
auto convert(const From &from) -> decltype(auto)
{ return Convert<Into, From>::do_convert(from); }

template <typename Into, typename From>
[[nodiscard]]
auto convert(From &&from) -> decltype(auto)
{ return Convert<Into, From>::do_convert(std::move(from)); }
}  // namespace flrv::traits
