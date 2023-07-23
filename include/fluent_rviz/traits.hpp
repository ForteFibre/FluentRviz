#pragma once

#include <cstddef>
#include <utility>

namespace flrv::traits
{
template <typename ...Constraint>
using Require = std::nullptr_t;

template <typename Into, typename From, typename Enabler = void>
struct Convert;

template <typename T>
struct Convert<T, T>
{
  static auto do_convert(const T &value) -> T
  { return value; }
};

template <typename Into, typename From>
using ConversionDefined = decltype(Convert<Into, From>::do_convert(std::declval<From>()));

template <typename Into, typename From>
[[nodiscard]]
auto convert(From &&from) -> decltype(auto)
{ return Convert<Into, From>::do_convert(std::forward<From>(from)); }
}  // namespace flrv::traits
