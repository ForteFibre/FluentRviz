#pragma once

#include <cstddef>
#include <type_traits>
#include <utility>

namespace flrv::point
{
template <typename PointLike, typename Enabler = void>
struct Access;

template <std::size_t Index, typename PointLike>
[[nodiscard]]
auto get(const PointLike &point) -> double
{ return Access<PointLike>::template get<Index>(point); }

template <std::size_t Index, typename PointLike>
auto set(PointLike &point, double value) -> void
{ Access<PointLike>::template set<Index>(point, value); }

template <typename PointLike>
[[nodiscard]]
auto x(const PointLike &point) -> double
{ return get<0>(point); }

template <typename PointLike>
auto x(PointLike &point, double value) -> void
{ set<0>(point, value); }

template <typename PointLike>
[[nodiscard]]
auto y(const PointLike &point) -> double
{ return get<1>(point); }

template <typename PointLike>
auto y(PointLike &point, double value) -> void
{ set<1>(point, value); }

template <typename PointLike>
[[nodiscard]]
auto z(const PointLike &point) -> double
{ return get<2>(point); }

template <typename PointLike>
auto z(PointLike &point, double value) -> void
{ set<2>(point, value); }
}  // namespace flrv::point
