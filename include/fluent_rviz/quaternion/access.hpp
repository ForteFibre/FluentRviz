#pragma once

#include <cstddef>
#include <type_traits>
#include <utility>

namespace flrv::quaternion
{
template <typename QuaternionLike, typename Enabler = void>
struct Access;

template <std::size_t Index, typename QuaternionLike>
[[nodiscard]]
auto get(const QuaternionLike &quaternion) -> double
{ return Access<QuaternionLike>::template get<Index>(quaternion); }

template <std::size_t Index, typename QuaternionLike>
auto set(QuaternionLike &quaternion, double value) -> void
{ Access<QuaternionLike>::template set<Index>(quaternion, value); }

template <typename QuaternionLike>
[[nodiscard]]
auto x(const QuaternionLike &quaternion) -> double
{ return get<0>(quaternion); }

template <typename QuaternionLike>
auto x(QuaternionLike &quaternion, double value) -> void
{ set<0>(quaternion, value); }

template <typename QuaternionLike>
[[nodiscard]]
auto y(const QuaternionLike &quaternion) -> double
{ return get<1>(quaternion); }

template <typename QuaternionLike>
auto y(QuaternionLike &quaternion, double value) -> void
{ set<1>(quaternion, value); }

template <typename QuaternionLike>
[[nodiscard]]
auto z(const QuaternionLike &quaternion) -> double
{ return get<2>(quaternion); }

template <typename QuaternionLike>
auto z(QuaternionLike &quaternion, double value) -> void
{ set<2>(quaternion, value); }

template <typename QuaternionLike>
[[nodiscard]]
auto w(const QuaternionLike &quaternion) -> double
{ return get<3>(quaternion); }

template <typename QuaternionLike>
auto w(QuaternionLike &quaternion, double value) -> void
{ set<3>(quaternion, value); }
}  // namespace flrv::quaternion
