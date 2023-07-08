#pragma once

#include <utility>

namespace flrv::marker
{
namespace detail
{
  template <
    template <typename Derived> typename MarkerProperty,
    typename MarkerToken>
  struct MarkerComposition;
}

template <
  template <typename Derived> typename MarkerProperty,
  typename MarkerToken>
auto compose_marker(MarkerToken && token)
{
  return detail::MarkerComposition<MarkerProperty, MarkerToken>::get(std::forward<MarkerToken>(token));
}
}  // namespace flrv::marker
