#pragma once

namespace flrv::traits
{
template <typename Lhs, typename Rhs, typename Enabler = void>
struct Assign;

template <typename Lhs, typename Rhs>
auto assign(Lhs &lhs, const Rhs &rhs) -> decltype(auto)
{ return Assign<Lhs, Rhs>::do_assign(lhs, rhs); }
}  // namespace flrv::traits
