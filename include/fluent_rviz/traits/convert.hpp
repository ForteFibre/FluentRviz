#pragma once

namespace flrv::traits
{
template <typename Into, typename From, typename Enabler = void>
struct Convert;

template <typename T>
struct Convert<T, T>
{
  static auto do_convert(const T &value) -> T
  { return value; }
};

template <typename Into, typename From>
[[nodiscard]]
auto convert(const From &from) -> decltype(auto)
{ return Convert<Into, From>::do_convert(from); }
}  // namespace flrv::traits
