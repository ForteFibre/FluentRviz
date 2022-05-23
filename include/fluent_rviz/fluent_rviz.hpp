// Copyright (c) 2022 ForteFibre
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FLUENT_RVIZ__FLUENT_RVIZ_HPP_
#define FLUENT_RVIZ__FLUENT_RVIZ_HPP_

#include <array>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <type_traits>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

namespace flrv
{

namespace traits
{

template <class From, class To, class Enabler = void>
struct converter;

template <class From, class To>
struct converter<From, To, std::enable_if_t<std::is_convertible_v<From, To>>>
{
  static constexpr To convert(const From & value) { return value; }
};

}  // namespace traits

namespace util
{

namespace detail
{
template <class Derived, class Base, template <class, class> class... Features>
struct chain_impl
{
  using type = Base;
};

template <
  class Derived, class Base, template <class, class> class Feature,
  template <class, class> class... Features>
struct chain_impl<Derived, Base, Feature, Features...>
: chain_impl<Derived, Feature<Derived, Base>, Features...>
{
};
}  // namespace detail

template <class Derived, class Base, template <class, class> class... Features>
using chain = typename detail::chain_impl<Derived, Base, Features...>::type;

namespace detail
{
template <class AlwaysVoid, template <class...> class Op, class... Args>
constexpr inline bool is_detected_impl_v = false;

template <template <class...> class Op, class... Args>
constexpr inline bool is_detected_impl_v<std::void_t<Op<Args...>>, Op, Args...> = true;
}  // namespace detail

template <template <class...> class Op, class... Args>
constexpr inline bool is_detected_v = detail::is_detected_impl_v<void, Op, Args...>;

template <class From, class To>
using convert_type = decltype(traits::converter<From, To>::convert(std::declval<From>()));

template <class From, class To>
constexpr inline bool is_convertible_v = is_detected_v<convert_type, From, To>;

template <class To, class From>
constexpr auto convert(const From & from)
  -> std::enable_if_t<is_convertible_v<From, To>, convert_type<From, To>>
{
  return traits::converter<From, To>::convert(from);
}

template <class T>
using size_type = decltype(std::declval<T>().size());

template <class T>
constexpr inline bool has_size_v = is_detected_v<size_type, T>;

}  // namespace util

namespace param
{

namespace detail
{
template <class T, class Enabler = void>
struct access
{
  static constexpr size_t size = 0;
};
}  // namespace detail

template <class T, size_t I>
using elem_type = std::remove_const_t<
  std::remove_reference_t<decltype(detail::access<T>::template get<I>(std::declval<T &>()))>>;

template <class T>
constexpr inline bool is_accessible_v = detail::access<T>::size > 0;

template <class T>
constexpr inline bool is_vec3_compat_v = detail::access<T>::size == 3;

template <class T>
constexpr inline bool is_quat_compat_v = detail::access<T>::size == 4;

template <class T>
constexpr inline bool is_rgba_compat_v = detail::access<T>::size == 4;

template <class S, class T>
constexpr inline bool is_same_size_v =
  is_accessible_v<S> && is_accessible_v<T> && detail::access<S>::size == detail::access<T>::size;

template <size_t I, class T>
auto get(const T & t) -> std::enable_if_t<is_accessible_v<T>, elem_type<T, I>>
{
  return detail::access<T>::template get<I>(t);
}

template <size_t I, class T>
auto set(T & t, const elem_type<T, I> & value) -> std::enable_if_t<is_accessible_v<T>>
{
  return detail::access<T>::template set<I>(t, value);
}

namespace detail
{
template <class T, class Enabler = void>
struct construct;

template <class T, class Seq>
struct construct_accessible;

template <class T, size_t... Is>
struct construct_accessible<T, std::index_sequence<Is...>>
{
  static T from(const elem_type<T, Is> &... args)
  {
    T ret;
    (set<Is>(ret, args), ...);
    return ret;
  }

  template <class From>
  static auto from(const From & arg) -> std::enable_if_t<is_same_size_v<T, From>, T>
  {
    return from(get<Is>(arg)...);
  }
};

template <class T>
struct construct<T, std::enable_if_t<is_accessible_v<T> && std::is_default_constructible_v<T>>>
: construct_accessible<T, std::make_index_sequence<detail::access<T>::size>>
{
};
}  // namespace detail

struct Vector3 : geometry_msgs::msg::Vector3
{
  using geometry_msgs::msg::Vector3::Vector3;

  Vector3(const double x, const double y, const double z) noexcept
  {
    this->x = x, this->y = y, this->z = z;
  }

  template <class Return = Vector3>
  static auto UnitX() noexcept -> std::enable_if_t<is_vec3_compat_v<Return>, Return>
  {
    return detail::construct<Return>::from(1, 0, 0);
  }

  template <class Return = Vector3>
  static auto UnitY() noexcept -> std::enable_if_t<is_vec3_compat_v<Return>, Return>
  {
    return detail::construct<Return>::from(0, 1, 0);
  }

  template <class Return = Vector3>
  static auto UnitZ() noexcept -> std::enable_if_t<is_vec3_compat_v<Return>, Return>
  {
    return detail::construct<Return>::from(0, 0, 1);
  }

  template <class T>
  operator T() const
  {
    return util::convert<T, geometry_msgs::msg::Vector3>(*this);
  }
};

namespace detail
{
template <class T>
struct access<T, std::enable_if_t<std::is_base_of_v<geometry_msgs::msg::Vector3, T>>>
{
  static constexpr size_t size = 3;

  template <size_t I>
  static double get(const geometry_msgs::msg::Vector3 & t) noexcept
  {
    if constexpr (I == 0) return t.x;
    if constexpr (I == 1) return t.y;
    if constexpr (I == 2) return t.z;
  }
  template <size_t I>
  static void set(geometry_msgs::msg::Vector3 & t, double value) noexcept
  {
    if constexpr (I == 0) t.x = value;
    if constexpr (I == 1) t.y = value;
    if constexpr (I == 2) t.z = value;
  }
};
}  // namespace detail

struct Point : geometry_msgs::msg::Point
{
  using geometry_msgs::msg::Point::Point;

  Point(const double x, const double y, const double z) noexcept
  {
    this->x = x, this->y = y, this->z = z;
  }

  template <class T>
  operator T() const
  {
    return util::convert<T, geometry_msgs::msg::Point>(*this);
  }
};

namespace detail
{
template <class T>
struct access<T, std::enable_if_t<std::is_base_of_v<geometry_msgs::msg::Point, T>>>
{
  static constexpr size_t size = 3;

  template <size_t I>
  static double get(const geometry_msgs::msg::Point & t) noexcept
  {
    if constexpr (I == 0) return t.x;
    if constexpr (I == 1) return t.y;
    if constexpr (I == 2) return t.z;
  }
  template <size_t I>
  static void set(geometry_msgs::msg::Point & t, double value) noexcept
  {
    if constexpr (I == 0) t.x = value;
    if constexpr (I == 1) t.y = value;
    if constexpr (I == 2) t.z = value;
  }
};
}  // namespace detail

namespace detail
{
template <class S, class T, class F, size_t... Is>
auto apply(S & lhs, const T & rhs, const F & f, std::index_sequence<Is...>)
  -> std::enable_if_t<is_same_size_v<S, T>, S &>
{
  return (set<Is>(lhs, f(get<Is>(lhs), get<Is>(rhs))), ..., lhs);
}

template <class S, class T, class F, size_t... Is>
auto apply(S & lhs, T rhs, const F & f, std::index_sequence<Is...>)
  -> std::enable_if_t<is_accessible_v<S> && std::is_scalar_v<T>, S &>
{
  return (set<Is>(lhs, f(get<Is>(lhs), rhs)), ..., lhs);
}
}  // namespace detail

template <class S, class T, class F>
auto apply(S & lhs, const T & rhs, const F & f) noexcept
  -> std::enable_if_t<is_same_size_v<S, T>, S &>
{
  return detail::apply(lhs, rhs, f, std::make_index_sequence<detail::access<S>::size>());
}

template <class S, class T, class F>
auto apply(S & lhs, T rhs, const F & f) noexcept
  -> std::enable_if_t<is_accessible_v<S> && std::is_scalar_v<T>, S &>
{
  return detail::apply(lhs, rhs, f, std::make_index_sequence<detail::access<S>::size>());
}

template <class S, class T>
auto operator+=(S & lhs, const T & rhs) noexcept -> std::enable_if_t<is_same_size_v<S, T>, S &>
{
  return apply(lhs, rhs, std::plus<>());
}

template <class S, class T>
auto operator-=(S & lhs, const T & rhs) noexcept -> std::enable_if_t<is_same_size_v<S, T>, S &>
{
  return apply(lhs, rhs, std::minus<>());
}

template <class S, class T>
auto operator*=(S & lhs, T rhs) noexcept
  -> std::enable_if_t<is_accessible_v<S> && std::is_scalar_v<T>, S &>
{
  return apply(lhs, rhs, std::multiplies<>());
}

template <class S, class T>
auto operator/=(S & lhs, T rhs) noexcept
  -> std::enable_if_t<is_accessible_v<S> && std::is_scalar_v<T>, S &>
{
  return apply(lhs, rhs, std::divides<>());
}

template <class S, class T>
auto operator+(S lhs, const T & rhs) noexcept -> std::enable_if_t<is_same_size_v<S, T>, S>
{
  return lhs += rhs;
}

template <class S, class T>
auto operator-(S lhs, const T & rhs) noexcept -> std::enable_if_t<is_same_size_v<S, T>, S>
{
  return lhs -= rhs;
}

template <class S, class T>
auto operator*(S lhs, T rhs) noexcept
  -> std::enable_if_t<is_accessible_v<S> && std::is_scalar_v<T>, S>
{
  return lhs *= rhs;
}

template <class S, class T>
auto operator*(S lhs, T rhs) noexcept
  -> std::enable_if_t<std::is_scalar_v<S> && is_accessible_v<T>, T>
{
  return rhs *= lhs;
}

template <class S, class T>
auto operator/(S lhs, T rhs) noexcept
  -> std::enable_if_t<is_accessible_v<S> && std::is_scalar_v<T>, S>
{
  return lhs /= rhs;
}

template <class T>
auto operator+(const T & s) noexcept -> std::enable_if_t<is_accessible_v<T>, T>
{
  return s;
}

template <class T>
auto operator-(const T & s) noexcept -> std::enable_if_t<is_accessible_v<T>, T>
{
  return -1 * s;
}

namespace detail
{
template <class S, class T, size_t... Is>
auto dot(const S & lhs, const T & rhs, std::index_sequence<Is...>) noexcept
  -> std::enable_if_t<is_same_size_v<S, T>, double>
{
  return ((get<Is>(lhs) * get<Is>(rhs)) + ...);
}
}  // namespace detail

template <class S, class T>
auto dot(const S & lhs, const T & rhs) noexcept -> std::enable_if_t<is_same_size_v<S, T>, double>
{
  return detail::dot(lhs, rhs, std::make_index_sequence<detail::access<S>::size>());
}

template <class T>
auto squared_norm(const T & s) noexcept -> std::enable_if_t<is_accessible_v<T>, double>
{
  return dot(s, s);
}

template <class T>
auto norm(const T & s) noexcept -> std::enable_if_t<is_accessible_v<T>, double>
{
  return std::sqrt(squared_norm(s));
}

template <class S, class T>
auto cross(const S & lhs, const T & rhs) noexcept
  -> std::enable_if_t<is_vec3_compat_v<S> && is_vec3_compat_v<T>, S>
{
  return detail::construct<S>::from(
    get<1>(lhs) * get<2>(rhs) - get<2>(lhs) * get<1>(rhs),
    get<2>(lhs) * get<0>(rhs) - get<0>(lhs) * get<2>(rhs),
    get<0>(lhs) * get<1>(rhs) - get<1>(lhs) * get<0>(rhs));
}

namespace detail
{
template <class T, size_t... Is>
auto print_impl(std::ostream & os, const T & s, std::index_sequence<Is...>) noexcept
  -> std::enable_if_t<is_accessible_v<T>, std::ostream &>
{
  return ((os << (Is != 0 ? ", " : "") << get<Is>(s)), ...);
}
}  // namespace detail

template <class T>
auto operator<<(std::ostream & os, const T & s) noexcept
  -> std::enable_if_t<is_accessible_v<T>, std::ostream &>
{
  return detail::print_impl(os, s, std::make_index_sequence<detail::access<T>::size>());
}

struct Quaternion : geometry_msgs::msg::Quaternion
{
  using geometry_msgs::msg::Quaternion::Quaternion;

  Quaternion(const double w, const double x, const double y, const double z) noexcept
  {
    this->w = w, this->x = x, this->y = y, this->z = z;
  }

  template <class Return = Quaternion, class T>
  static auto ScalarVector(const double scalar, const T & vector) noexcept
    -> std::enable_if_t<is_vec3_compat_v<T> && is_quat_compat_v<Return>, Return>
  {
    return detail::construct<Return>::from(scalar, get<0>(vector), get<1>(vector), get<2>(vector));
  }

  template <class Return = Quaternion, class T = Vector3>
  static auto AngleAxis(const double angle, const T & axis = Vector3::UnitZ<T>()) noexcept
    -> std::enable_if_t<is_vec3_compat_v<T> && is_quat_compat_v<Return>, Return>
  {
    return ScalarVector<Return>(std::cos(angle / 2.0), axis * std::sin(angle / 2.0));
  }

  template <class T>
  operator T() const
  {
    return util::convert<T, geometry_msgs::msg::Quaternion>(*this);
  }
};

namespace detail
{
template <class T>
struct access<T, std::enable_if_t<std::is_base_of_v<geometry_msgs::msg::Quaternion, T>>>
{
  static constexpr size_t size = 4;

  template <size_t I>
  static double get(const geometry_msgs::msg::Quaternion & t) noexcept
  {
    if constexpr (I == 0) return t.w;
    if constexpr (I == 1) return t.x;
    if constexpr (I == 2) return t.y;
    if constexpr (I == 3) return t.z;
  }
  template <size_t I>
  static void set(geometry_msgs::msg::Quaternion & t, double value) noexcept
  {
    if constexpr (I == 0) t.w = value;
    if constexpr (I == 1) t.x = value;
    if constexpr (I == 2) t.y = value;
    if constexpr (I == 3) t.z = value;
  }
};
}  // namespace detail

template <class T>
auto scalar(const T & t) noexcept -> std::enable_if_t<is_quat_compat_v<T>, elem_type<T, 0>>
{
  return get<0>(t);
}

template <class Return = Vector3, class T>
auto vector(const T & t) noexcept
  -> std::enable_if_t<is_quat_compat_v<T> && is_vec3_compat_v<Return>, Return>
{
  return detail::construct<Return>::from(get<1>(t), get<2>(t), get<3>(t));
}

template <class T>
auto conjugate(const T & t) noexcept -> std::enable_if_t<is_quat_compat_v<T>, T>
{
  return Quaternion::ScalarVector<T>(scalar(t), -vector(t));
}

template <class T>
auto inverse(const T & t) noexcept -> std::enable_if_t<is_quat_compat_v<T>, T>
{
  return conjugate(t) / squared_norm(t);
}

template <class S, class T>
auto operator*(const S & lhs, const T & rhs) noexcept
  -> std::enable_if_t<is_quat_compat_v<S> && is_quat_compat_v<T>, S>
{
  return Quaternion::ScalarVector<T>(
    scalar(lhs) * scalar(rhs) - dot(vector(lhs), vector(rhs)),
    scalar(lhs) * vector(rhs) + scalar(rhs) * vector(lhs) + cross(vector(lhs), vector(rhs)));
}

template <class S, class T>
auto operator*(const S & lhs, const T & rhs) noexcept
  -> std::enable_if_t<is_quat_compat_v<S> && is_vec3_compat_v<T>, T>
{
  return vector<T>(lhs * Quaternion::ScalarVector(0, rhs) * inverse(lhs));
}

template <class S, class T>
auto lerp(const S & a, const T & b, double t) noexcept -> std::enable_if_t<is_same_size_v<S, T>, S>
{
  return a + t * (b - a);
}

template <class S, class T>
auto slerp(const S & a, const T & b, double t) noexcept
  -> std::enable_if_t<is_quat_compat_v<S> && is_quat_compat_v<T>, S>
{
  double dot = dot(a, b);
  double theta = std::acos(dot);
  return std::sin((1 - t) * theta) / std::sin(theta) * a +
         std::sin(t * theta) / std::sin(theta) * (dot < 0 ? -b : b);
}

struct Color : std_msgs::msg::ColorRGBA
{
  using std_msgs::msg::ColorRGBA::ColorRGBA;

  Color(const float r, const float g, const float b, const float a = 1.0f) noexcept
  {
    this->r = r, this->g = g, this->b = b, this->a = a;
  }

  template <class Return = Color>
  static auto White(const float alpha = 1.0f) noexcept
    -> std::enable_if_t<is_rgba_compat_v<Return>, Return>
  {
    return detail::construct<Return>::from(1.00, 1.00, 1.00, alpha);
  }

  template <class Return = Color>
  static auto Silver(const float alpha = 1.0f) noexcept
    -> std::enable_if_t<is_rgba_compat_v<Return>, Return>
  {
    return detail::construct<Return>::from(0.75, 0.75, 0.75, alpha);
  }

  template <class Return = Color>
  static auto Gray(const float alpha = 1.0f) noexcept
    -> std::enable_if_t<is_rgba_compat_v<Return>, Return>
  {
    return detail::construct<Return>::from(0.50, 0.50, 0.50, alpha);
  }

  template <class Return = Color>
  static auto Black(const float alpha = 1.0f) noexcept
    -> std::enable_if_t<is_rgba_compat_v<Return>, Return>
  {
    return detail::construct<Return>::from(0.00, 0.00, 0.00, alpha);
  }

  template <class Return = Color>
  static auto Red(const float alpha = 1.0f) noexcept
    -> std::enable_if_t<is_rgba_compat_v<Return>, Return>
  {
    return detail::construct<Return>::from(1.00, 0.00, 0.00, alpha);
  }

  template <class Return = Color>
  static auto Maroon(const float alpha = 1.0f) noexcept
    -> std::enable_if_t<is_rgba_compat_v<Return>, Return>
  {
    return detail::construct<Return>::from(0.50, 0.00, 0.00, alpha);
  }

  template <class Return = Color>
  static auto Yellow(const float alpha = 1.0f) noexcept
    -> std::enable_if_t<is_rgba_compat_v<Return>, Return>
  {
    return detail::construct<Return>::from(1.00, 1.00, 0.00, alpha);
  }

  template <class Return = Color>
  static auto Olive(const float alpha = 1.0f) noexcept
    -> std::enable_if_t<is_rgba_compat_v<Return>, Return>
  {
    return detail::construct<Return>::from(0.50, 0.50, 0.00, alpha);
  }

  template <class Return = Color>
  static auto Lime(const float alpha = 1.0f) noexcept
    -> std::enable_if_t<is_rgba_compat_v<Return>, Return>
  {
    return detail::construct<Return>::from(0.00, 1.00, 0.00, alpha);
  }

  template <class Return = Color>
  static auto Green(const float alpha = 1.0f) noexcept
    -> std::enable_if_t<is_rgba_compat_v<Return>, Return>
  {
    return detail::construct<Return>::from(0.00, 0.50, 0.00, alpha);
  }

  template <class Return = Color>
  static auto Aqua(const float alpha = 1.0f) noexcept
    -> std::enable_if_t<is_rgba_compat_v<Return>, Return>
  {
    return detail::construct<Return>::from(0.00, 1.00, 1.00, alpha);
  }

  template <class Return = Color>
  static auto Teal(const float alpha = 1.0f) noexcept
    -> std::enable_if_t<is_rgba_compat_v<Return>, Return>
  {
    return detail::construct<Return>::from(0.00, 0.50, 0.50, alpha);
  }

  template <class Return = Color>
  static auto Blue(const float alpha = 1.0f) noexcept
    -> std::enable_if_t<is_rgba_compat_v<Return>, Return>
  {
    return detail::construct<Return>::from(0.00, 0.00, 1.00, alpha);
  }

  template <class Return = Color>
  static auto Navy(const float alpha = 1.0f) noexcept
    -> std::enable_if_t<is_rgba_compat_v<Return>, Return>
  {
    return detail::construct<Return>::from(0.00, 0.00, 0.50, alpha);
  }

  template <class Return = Color>
  static auto Fuchsia(const float alpha = 1.0f) noexcept
    -> std::enable_if_t<is_rgba_compat_v<Return>, Return>
  {
    return detail::construct<Return>::from(1.00, 0.00, 1.00, alpha);
  }

  template <class Return = Color>
  static auto Purple(const float alpha = 1.0f) noexcept
    -> std::enable_if_t<is_rgba_compat_v<Return>, Return>
  {
    return detail::construct<Return>::from(0.50, 0.00, 0.50, alpha);
  }

  template <class T>
  operator T() const
  {
    return util::convert<T, std_msgs::msg::ColorRGBA>(*this);
  }
};

namespace detail
{
template <class T>
struct access<T, std::enable_if_t<std::is_base_of_v<std_msgs::msg::ColorRGBA, T>>>
{
  static constexpr size_t size = 4;

  template <size_t I>
  static float get(const std_msgs::msg::ColorRGBA & t) noexcept
  {
    if constexpr (I == 0) return t.r;
    if constexpr (I == 1) return t.g;
    if constexpr (I == 2) return t.b;
    if constexpr (I == 3) return t.a;
  }
  template <size_t I>
  static void set(std_msgs::msg::ColorRGBA & t, float value) noexcept
  {
    if constexpr (I == 0) t.r = value;
    if constexpr (I == 1) t.g = value;
    if constexpr (I == 2) t.b = value;
    if constexpr (I == 3) t.a = value;
  }
};
}  // namespace detail

}  // namespace param

namespace traits
{
template <class From, class To>
struct converter<
  From, To, std::enable_if_t<!std::is_convertible_v<From, To> && param::is_same_size_v<From, To>>>
{
  static constexpr To convert(const From & arg) { return param::detail::construct<To>::from(arg); }
};
}  // namespace traits

namespace marker
{

namespace detail
{
template <class Dest, class Src, class Enabler = void>
struct merger;
}

namespace attr
{
template <class Derived, class Base>
struct CRTP : Base
{
protected:
  Derived & derived() noexcept { return static_cast<Derived &>(*this); }
  const Derived & derived() const noexcept { return static_cast<const Derived &>(*this); }
};

template <class Derived, class Base>
struct Position : Base
{
  template <class T>
  Derived & position(const T & position) noexcept
  {
    this->message.pose.position = util::convert<geometry_msgs::msg::Point>(position);
    return this->derived();
  }

  Derived & position(const double x, const double y, const double z) noexcept
  {
    this->message.pose.position.x = x;
    this->message.pose.position.y = y;
    this->message.pose.position.z = z;
    return this->derived();
  }
};

template <class Derived, class Base>
struct Orientation : Base
{
  Orientation() noexcept { orientation(1, 0, 0, 0); }

  template <class T>
  Derived & orientation(const T & orientation) noexcept
  {
    this->message.pose.orientation = util::convert<geometry_msgs::msg::Quaternion>(orientation);
    return this->derived();
  }

  Derived & orientation(const double w, const double x, const double y, const double z) noexcept
  {
    this->message.pose.orientation.w = w;
    this->message.pose.orientation.x = x;
    this->message.pose.orientation.y = y;
    this->message.pose.orientation.z = z;
    return this->derived();
  }
};

template <class Derived, class Base>
struct Scale : Base
{
  Scale() noexcept { scale(1, 1, 1); }

  Derived & scale(const double x, const double y, const double z) noexcept
  {
    this->message.scale.x = x;
    this->message.scale.y = y;
    this->message.scale.z = z;
    return this->derived();
  }
};

template <class Derived, class Base>
struct PoseArrowScale : Base
{
  PoseArrowScale() noexcept { scale(0.2, 0.2, 1); }

  Derived & scale(const double length, const double width, const double height) noexcept
  {
    this->message.scale.x = length;
    this->message.scale.y = width;
    this->message.scale.z = height;
    return this->derived();
  }
};

template <class Derived, class Base>
struct VectorArrowScale : Base
{
  VectorArrowScale() noexcept { scale(0.2, 0.4, 0.4); }

  Derived & scale(
    const double shaft_diameter, const double head_diameter, const double head_length) noexcept
  {
    this->message.scale.x = shaft_diameter;
    this->message.scale.y = head_diameter;
    this->message.scale.z = head_length;
    return this->derived();
  }
};

template <class Derived, class Base>
struct PointScale : Base
{
  PointScale() noexcept { scale(0.05); }

  Derived & scale(const double width) noexcept { return scale(width, width); }

  Derived & scale(const double width, const double height) noexcept
  {
    this->message.scale.x = width;
    this->message.scale.y = height;
    return this->derived();
  }
};

template <class Derived, class Base>
struct LineScale : Base
{
  LineScale() noexcept { scale(0.05); }

  Derived & scale(const double width) noexcept
  {
    this->message.scale.x = width;
    return this->derived();
  }
};

template <class Derived, class Base>
struct TextScale : Base
{
  TextScale() noexcept { scale(0.05); }

  Derived & scale(const double height) noexcept
  {
    this->message.scale.z = height;
    return this->derived();
  }
};

template <class Derived, class Base>
struct Color : Base
{
  Color() noexcept { color(1, 1, 1, 1); }

  template <class T>
  Derived & color(const T & color) noexcept
  {
    this->message.color = util::convert<std_msgs::msg::ColorRGBA>(color);
    return this->derived();
  }

  Derived & color(const double r, const double g, const double b, const double a = 1) noexcept
  {
    this->message.color.r = r;
    this->message.color.g = g;
    this->message.color.b = b;
    this->message.color.a = a;
    return this->derived();
  }
};

namespace internal
{
template <class To>
struct convert
{
  template <class From>
  decltype(auto) operator()(const From & from) const noexcept
  {
    return util::convert<To>(from);
  }
};
}  // namespace internal

template <class Derived, class Base>
struct Colors : Color<Derived, Base>
{
  Colors() noexcept { color(1, 1, 1, 1); }

  template <class T>
  Derived & color(const T & color) noexcept
  {
    this->message.colors.clear();
    return Color<Derived, Base>::color(color);
  }

  Derived & color(const double r, const double g, const double b, const double a = 1) noexcept
  {
    this->message.colors.clear();
    return Color<Derived, Base>::color(r, g, b, a);
  }

  template <
    class Iterable = std::initializer_list<param::Color>,
    class Func = internal::convert<std_msgs::msg::ColorRGBA>>
  Derived & colors(const Iterable & iterable, const Func & func = Func()) noexcept
  {
    if constexpr (util::has_size_v<Iterable>) {
      this->message.colors.reserve(iterable.size());
    }
    for (const auto & e : iterable) {
      this->message.colors.push_back(func(e));
    }
    return this->derived();
  }
};

template <class Derived, class Base>
struct Points : Base
{
  template <
    class Iterable = std::initializer_list<param::Point>,
    class Func = internal::convert<geometry_msgs::msg::Point>>
  Derived & points(const Iterable & iterable, const Func & func = Func()) noexcept
  {
    if constexpr (util::has_size_v<Iterable>) {
      this->message.points.reserve(iterable.size());
    }
    for (const auto & e : iterable) {
      this->message.points.push_back(func(e));
    }
    return this->derived();
  }
};

template <class Derived, class Base>
struct LinePoints : Base
{
  LinePoints() noexcept
  {
    this->message.points.resize(2);
    end(1, 0, 0);
  }

  template <class T>
  Derived & start(const T & point) noexcept
  {
    return this->point(0, point);
  }
  Derived & start(const double x, const double y, const double z) noexcept
  {
    return this->point(0, x, y, z);
  }

  template <class T>
  Derived & end(const T & point) noexcept
  {
    return this->point(1, point);
  }
  Derived & end(const double x, const double y, const double z) noexcept
  {
    return this->point(1, x, y, z);
  }

private:
  template <class T>
  Derived & point(size_t index, const T & point) noexcept
  {
    this->message.points[index] = util::convert<geometry_msgs::msg::Point>(point);
    return this->derived();
  }

  Derived & point(size_t index, const double x, const double y, const double z) noexcept
  {
    this->message.points[index].x = x;
    this->message.points[index].y = y;
    this->message.points[index].z = z;
    return this->derived();
  }
};

template <class Derived, class Base>
struct Text : Base
{
  Text() noexcept { text("visualization_msgs::msg::Marker"); }

  Derived & text(const std::string & text) noexcept
  {
    this->message.text = text;
    return this->derived();
  }
};

template <class Derived, class Base>
struct MeshResource : Base
{
  Derived & mesh_resource(const std::string & mesh_resource) noexcept
  {
    this->message.mesh_resource = mesh_resource;
    return this->derived();
  }

  Derived & mesh_use_embedded_materials(const uint8_t mesh_use_embedded_materials) noexcept
  {
    this->message.mesh_use_embedded_materials = mesh_use_embedded_materials;
    return this->derived();
  }
};

}  // namespace attr

struct MarkerWrapper
{
protected:
  visualization_msgs::msg::Marker message;

public:
  operator visualization_msgs::msg::Marker &() noexcept { return message; }
  operator const visualization_msgs::msg::Marker &() const noexcept { return message; }
};

struct DeleteAll : MarkerWrapper
{
  DeleteAll(const std::string & ns = "") noexcept
  {
    this->message.action = visualization_msgs::msg::Marker::DELETEALL;
    this->message.ns = ns;
  }
};

struct Delete : MarkerWrapper
{
  Delete(const int32_t id = 0, const std::string & ns = "") noexcept
  {
    this->message.action = visualization_msgs::msg::Marker::DELETE;
    this->message.id = id;
    this->message.ns = ns;
  }
};

template <int32_t Type, template <class, class> class... Features>
struct Add : util::chain<Add<Type, Features...>, MarkerWrapper, attr::CRTP, Features...>
{
  Add(const int32_t id = 0, const std::string & ns = "") noexcept
  {
    this->message.action = visualization_msgs::msg::Marker::ADD;
    this->message.type = Type;
    this->message.id = id;
    this->message.ns = ns;
  }

  Add & lifetime(const double lifetime) noexcept
  {
    this->message.lifetime = lifetime;
    return *this;
  }

  Add & frame_locked(const uint8_t frame_locked) noexcept
  {
    this->message.frame_locked = frame_locked;
    return *this;
  }
};

using PoseArrow = Add<
  visualization_msgs::msg::Marker::ARROW, attr::Position, attr::Orientation, attr::PoseArrowScale,
  attr::Color>;

using LineArrow = Add<
  visualization_msgs::msg::Marker::ARROW, attr::VectorArrowScale, attr::Color, attr::LinePoints>;

using Cube = Add<
  visualization_msgs::msg::Marker::CUBE, attr::Position, attr::Orientation, attr::Scale,
  attr::Color>;

using Sphere = Add<
  visualization_msgs::msg::Marker::SPHERE, attr::Position, attr::Orientation, attr::Scale,
  attr::Color>;

using Cylinder = Add<
  visualization_msgs::msg::Marker::CYLINDER, attr::Position, attr::Orientation, attr::Scale,
  attr::Color>;

using LineStrip = Add<
  visualization_msgs::msg::Marker::LINE_STRIP, attr::Position, attr::Orientation, attr::LineScale,
  attr::Colors, attr::Points>;

using LineList = Add<
  visualization_msgs::msg::Marker::LINE_LIST, attr::Position, attr::Orientation, attr::LineScale,
  attr::Colors, attr::Points>;

using CubeList = Add<
  visualization_msgs::msg::Marker::CUBE_LIST, attr::Position, attr::Orientation, attr::Scale,
  attr::Colors, attr::Points>;

using SphereList = Add<
  visualization_msgs::msg::Marker::SPHERE_LIST, attr::Position, attr::Orientation, attr::Scale,
  attr::Colors, attr::Points>;

using Points = Add<
  visualization_msgs::msg::Marker::POINTS, attr::Position, attr::Orientation, attr::PointScale,
  attr::Colors, attr::Points>;

using TextViewFacing = Add<
  visualization_msgs::msg::Marker::TEXT_VIEW_FACING, attr::Position, attr::TextScale, attr::Color,
  attr::Text>;

using MeshResource = Add<
  visualization_msgs::msg::Marker::MESH_RESOURCE, attr::Position, attr::Orientation, attr::Scale,
  attr::Color, attr::MeshResource>;

using TriangleList = Add<
  visualization_msgs::msg::Marker::TRIANGLE_LIST, attr::Position, attr::Orientation, attr::Scale,
  attr::Colors, attr::Points>;

}  // namespace marker

class Rviz
{
private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  rclcpp::Clock::SharedPtr clock_;
  std::string frame_id_;

public:
  Rviz() {}
  void init(
    rclcpp::Node & node, const std::string & frame_id = "map",
    const std::string & topic = "visualization_marker")
  {
    frame_id_ = frame_id;
    publisher_ = node.create_publisher<visualization_msgs::msg::Marker>(topic, 1);
    clock_ = node.get_clock();
  }

  inline const Rviz & operator<<(const visualization_msgs::msg::Marker & marker) const
  {
    publish(marker);
    return *this;
  }

  inline const Rviz & operator<<(visualization_msgs::msg::Marker && marker) const
  {
    publish(marker);
    return *this;
  }

  void publish(const visualization_msgs::msg::Marker & marker) const
  {
    publish(visualization_msgs::msg::Marker(marker));
  }

  void publish(visualization_msgs::msg::Marker && marker) const
  {
    marker.header.frame_id = frame_id_;
    marker.header.stamp = clock_->now();
    publisher_->publish(marker);
  }
};

}  // namespace flrv

#endif  // FLUENT_RVIZ__FLUENT_RVIZ_HPP_
