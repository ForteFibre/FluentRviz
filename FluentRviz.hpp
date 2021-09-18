#pragma once

#include <array>
#include <numeric>
#include <string>
#include <vector>
#include <type_traits>

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>

namespace flrv {

namespace traits {

    template<class From, class To, class Enabler = void>
    struct converter;

    template<class T>
    struct converter<T, T> {
        static constexpr const T &convert(const T &value) { return value; };
    };

    template<class From, class To>
    struct converter<From, To, std::enable_if_t<std::is_convertible_v<From, To>>> {
        static constexpr const To &convert(const To &value) { return value; }
    };

}

namespace util {

    template<class To, class From>
    constexpr decltype(auto) convert(const From &from)
    { return traits::converter<From, To>::convert(from); }

    template<
        class Derived,
        class Base,
        template<class, class> class ...Features>
    struct chain { using type = Base; };

    template<
        class Derived,
        class Base,
        template<class, class> class Feature,
        template<class, class> class ...Features>
    struct chain<Derived, Base, Feature, Features...>
        : chain<Derived, Feature<Derived, Base>, Features...> { };

    template<
        class Derived,
        class Base,
        template<class, class> class ...Features>
    using chained = typename chain<Derived, Base, Features...>::type;

    namespace detail {
        template<class AlwaysVoid, template<class...> class Op, class... Args>
        constexpr inline bool is_detected_impl_v = false;

        template<template<class...> class Op, class... Args>
        constexpr inline bool is_detected_impl_v<std::void_t<Op<Args...>>, Op, Args...> = true;
    } // namespace detail

    template<template<class...> class Op, class... Args>
    constexpr inline bool is_detected_v = detail::is_detected_impl_v<void, Op, Args...>;

    template<class From, class To>
    using convert_type = decltype(traits::converter<From, To>::convert(std::declval<From>()));

    template<class From, class To>
    constexpr inline bool is_convertible_v = is_detected_v<convert_type, From, To>;

    template<class T>
    using size_type = decltype(std::declval<T>().size());

    template<class T>
    constexpr inline bool has_size_v = is_detected_v<size_type, T>;
}

namespace param {

    namespace detail {
        template<class T, class Enabler = void>
        struct access {
            static constexpr size_t size = 0;
        };

        template<class T, size_t I>
        using ref_type = decltype(access<T>::template ref<I>(std::declval<T &>()));

        template<class T, size_t I>
        using elem_type = std::remove_reference_t<ref_type<T, I>>;

        template<class T, size_t I>
        using const_ref_type = const elem_type<T, I> &;
    }

    template<class T>
    constexpr inline bool is_accessible_v = detail::access<T>::size > 0;

    template<class T>
    constexpr inline bool is_vec3_compat_v = detail::access<T>::size == 3;

    template<class T>
    constexpr inline bool is_quat_compat_v = detail::access<T>::size == 4;

    template<class S, class T>
    constexpr inline bool is_same_size_v = is_accessible_v<S> && is_accessible_v<T>
        && detail::access<S>::size == detail::access<T>::size;

    template<size_t I, class T>
    auto ref(T &t)
    -> std::enable_if_t<is_accessible_v<T>, detail::ref_type<T, I>>
    { return detail::access<T>::template ref<I>(t); }

    template<size_t I, class T>
    auto ref(const T &t)
    -> std::enable_if_t<is_accessible_v<T>, detail::const_ref_type<T, I>>
    { return detail::access<T>::template ref<I>(t); }

    namespace detail {
        template<class T, class Enabler = void>
        struct construct;

        template<class T, class Seq>
        struct construct_accessible;

        template<class T, size_t ...Is>
        struct construct_accessible<T, std::index_sequence<Is...>> {
            static T from(detail::const_ref_type<T, Is> ...args) {
                T ret;
                ((ref<Is>(ret) = args), ...);
                return ret;
            }
        };

        template<class T>
        struct construct<T, std::enable_if_t<is_accessible_v<T> && std::is_default_constructible_v<T>>>
            : construct_accessible<T, std::make_index_sequence<detail::access<T>::size>> { };
    }

    struct Vector3 : geometry_msgs::Vector3 {
        Vector3() = default;

        Vector3(const double x, const double y, const double z) noexcept
        { this->x = x, this->y = y, this->z = z; }

        Vector3(const geometry_msgs::Vector3 &vector3) noexcept
            : geometry_msgs::Vector3(vector3) { }

        template<class Return = Vector3>
        static Vector3 UnitX() noexcept { return detail::construct<Return>::from(1, 0, 0); }

        template<class Return = Vector3>
        static Vector3 UnitY() noexcept { return detail::construct<Return>::from(0, 1, 0); }

        template<class Return = Vector3>
        static Vector3 UnitZ() noexcept { return detail::construct<Return>::from(0, 0, 1); }

        template<class T>
        operator T() { return util::convert<T>(*this); }
    };

    namespace detail {
        template<class T>
        struct access<T, std::enable_if_t<std::is_base_of_v<geometry_msgs::Vector3, T>>> {
            static constexpr size_t size = 3;

            template<size_t I>
            static double &ref(geometry_msgs::Vector3 &t) noexcept
            {
                if constexpr (I == 0) return t.x;
                if constexpr (I == 1) return t.y;
                if constexpr (I == 2) return t.z;
            }
            template<size_t I>
            static const double &ref(const geometry_msgs::Vector3 &t) noexcept
            {
                if constexpr (I == 0) return t.x;
                if constexpr (I == 1) return t.y;
                if constexpr (I == 2) return t.z;
            }
        };
    }

    struct Point : geometry_msgs::Point {
        Point() = default;

        Point(const double x, const double y, const double z) noexcept
        { this->x = x, this->y = y, this->z = z; }

        Point(const geometry_msgs::Point &point)
            : geometry_msgs::Point(point) { }

        template<class T>
        operator T() { return util::convert<T>(*this); }
    };

    namespace detail {
        template<class T>
        struct access<T, std::enable_if_t<std::is_base_of_v<geometry_msgs::Point, T>>> {
            static constexpr size_t size = 3;

            template<size_t I>
            static double &ref(geometry_msgs::Point &t) noexcept
            {
                if constexpr (I == 0) return t.x;
                if constexpr (I == 1) return t.y;
                if constexpr (I == 2) return t.z;
            }
            template<size_t I>
            static const double &ref(const geometry_msgs::Point &t) noexcept
            {
                if constexpr (I == 0) return t.x;
                if constexpr (I == 1) return t.y;
                if constexpr (I == 2) return t.z;
            }
        };
    }

    namespace detail {
        template<class S, class T, class F, size_t ...Is>
        auto apply_impl(S &lhs, const T &rhs, const F &f, std::index_sequence<Is...>)
        -> std::enable_if_t<is_same_size_v<S, T>>
        { ((ref<Is>(lhs) = f(ref<Is>(lhs), ref<Is>(rhs))), ...); }

        template<class S, class T, class F, size_t ...Is>
        auto apply_impl(S &lhs, T rhs, const F &f, std::index_sequence<Is...>)
        -> std::enable_if_t<is_accessible_v<S> && std::is_scalar_v<T>>
        { ((ref<Is>(lhs) = f(ref<Is>(lhs), rhs)), ...); }
    }

    template<class S, class T, class F>
    auto apply(S &lhs, const T &rhs, const F &f) noexcept
    -> std::enable_if_t<is_same_size_v<S, T>, S &>
    {
        detail::apply_impl(lhs, rhs, f, std::make_index_sequence<detail::access<S>::size>());
        return lhs;
    }

    template<class S, class T, class F>
    auto apply(S &lhs, T rhs, const F &f) noexcept
    -> std::enable_if_t<is_accessible_v<S> && std::is_scalar_v<T>, S &>
    {
        detail::apply_impl(lhs, rhs, f, std::make_index_sequence<detail::access<S>::size>());
        return lhs;
    }

    template<class S, class T>
    auto operator+=(S &lhs, const T &rhs) noexcept
    -> std::enable_if_t<is_same_size_v<S, T>, S &>
    { return apply(lhs, rhs, std::plus<>()); }

    template<class S, class T>
    auto operator-=(S &lhs, const T &rhs) noexcept
    -> std::enable_if_t<is_same_size_v<S, T>, S &>
    { return apply(lhs, rhs, std::minus<>()); }

    template<class S, class T>
    auto operator*=(S &lhs, T rhs) noexcept
    -> std::enable_if_t<is_accessible_v<S> && std::is_scalar_v<T>, S &>
    { return apply(lhs, rhs, std::multiplies<>()); }

    template<class S, class T>
    auto operator/=(S &lhs, T rhs) noexcept
    -> std::enable_if_t<is_accessible_v<S> && std::is_scalar_v<T>, S &>
    { return apply(lhs, rhs, std::divides<>()); }

    template<class S, class T>
    auto operator+(S lhs, const T &rhs) noexcept
    -> std::enable_if_t<is_same_size_v<S, T>, S>
    { return lhs += rhs; }

    template<class S, class T>
    auto operator-(S lhs, const T &rhs) noexcept
    -> std::enable_if_t<is_same_size_v<S, T>, S>
    { return lhs -= rhs; }

    template<class S, class T>
    auto operator*(S lhs, T rhs) noexcept
    -> std::enable_if_t<is_accessible_v<S> && std::is_scalar_v<T>, S>
    { return lhs *= rhs; }

    template<class S, class T>
    auto operator*(S lhs, T rhs) noexcept
    -> std::enable_if_t<std::is_scalar_v<S> && is_accessible_v<T>, T>
    { return rhs *= lhs; }

    template<class S, class T>
    auto operator/(S lhs, T rhs) noexcept
    -> std::enable_if_t<is_accessible_v<S> && std::is_scalar_v<T>, S>
    { return lhs /= rhs; }

    template<class T>
    auto operator+(const T &s) noexcept
    -> std::enable_if_t<is_accessible_v<T>, T>
    { return s; }

    template<class T>
    auto operator-(const T &s) noexcept
    -> std::enable_if_t<is_accessible_v<T>, T>
    { return -1 * s; }

    namespace detail {
        template<class S, class T, size_t ...Is>
        auto dot_impl(const S &lhs, const T &rhs, std::index_sequence<Is...>) noexcept
        -> std::enable_if_t<is_same_size_v<S, T>, double>
        { return ((ref<Is>(lhs) * ref<Is>(rhs)) + ...); }
    }

    template<class S, class T>
    auto dot(const S &lhs, const T &rhs) noexcept
    -> std::enable_if_t<is_same_size_v<S, T>, double>
    { return detail::dot_impl(lhs, rhs, std::make_index_sequence<detail::access<S>::size>()); }

    template<class T>
    auto squared_norm(const T &s) noexcept
    -> std::enable_if_t<is_accessible_v<T>, double>
    { return dot(s, s); }

    template<class T>
    auto norm(const T &s) noexcept
    -> std::enable_if_t<is_accessible_v<T>, double>
    { return std::sqrt(squared_norm(s)); }

    template<class S, class T>
    auto cross(const S &lhs, const T &rhs) noexcept
    -> std::enable_if_t<is_vec3_compat_v<S> && is_vec3_compat_v<T>, S>
    {
        return detail::construct<S>::from(
            ref<1>(lhs) * ref<2>(rhs) - ref<2>(lhs) * ref<1>(rhs),
            ref<2>(lhs) * ref<0>(rhs) - ref<0>(lhs) * ref<2>(rhs),
            ref<0>(lhs) * ref<1>(rhs) - ref<1>(lhs) * ref<0>(rhs));
    }

    namespace detail {
        template<class T, size_t ...Is>
        auto print_impl(std::ostream &os, const T &s, std::index_sequence<Is...>) noexcept
        -> std::enable_if_t<is_accessible_v<T>, std::ostream &>
        { return ((os << (Is != 0 ? ", " : "") << ref<Is>(s)), ...); }
    }

    template<class T>
    auto operator<<(std::ostream &os, const T &s) noexcept
    -> std::enable_if_t<is_accessible_v<T>, std::ostream &>
    { return detail::print_impl(os, s, std::make_index_sequence<detail::access<T>::size>()); }

    struct Quaternion : geometry_msgs::Quaternion {
        Quaternion() = default;

        Quaternion(const double w, const double x, const double y, const double z) noexcept
        { this->w = w, this->x = x, this->y = y, this->z = z; }

        Quaternion(const geometry_msgs::Quaternion &quaternion) noexcept
            : geometry_msgs::Quaternion(quaternion) { }

        template<class Return = Quaternion, class T>
        static auto ScalarVector(const double scalar, const T &vector) noexcept
        -> std::enable_if_t<is_vec3_compat_v<T>, Return>
        { return detail::construct<Return>::from(scalar, ref<0>(vector), ref<1>(vector), ref<2>(vector)); }

        template<class Return = Quaternion, class T = Vector3>
        static auto AngleAxis(const double angle, const T &axis = Vector3::UnitZ<T>()) noexcept
        -> std::enable_if_t<is_vec3_compat_v<T>, Return>
        { return ScalarVector<Return>(std::cos(angle / 2.0), axis * std::sin(angle / 2.0)); }

        template<class T>
        operator T() { return util::convert<T>(*this); }
    };

    namespace detail {
        template<class T>
        struct access<T, std::enable_if_t<std::is_base_of_v<geometry_msgs::Quaternion, T>>> {
            static constexpr size_t size = 4;

            template<size_t I>
            static double &ref(geometry_msgs::Quaternion &t) noexcept
            {
                if constexpr (I == 0) return t.w;
                if constexpr (I == 1) return t.x;
                if constexpr (I == 2) return t.y;
                if constexpr (I == 3) return t.z;
            }
            template<size_t I>
            static const double &ref(const geometry_msgs::Quaternion &t) noexcept
            {
                if constexpr (I == 0) return t.w;
                if constexpr (I == 1) return t.x;
                if constexpr (I == 2) return t.y;
                if constexpr (I == 3) return t.z;
            }
        };
    }

    template<class T>
    auto scalar(const T &t) noexcept
    -> std::enable_if_t<is_quat_compat_v<T>, std::remove_reference_t<decltype(ref<0>(t))>>
    { return ref<0>(t); }

    template<class Return = Vector3, class T>
    auto vector(const T &t) noexcept
    -> std::enable_if_t<is_quat_compat_v<T>, Return>
    { return detail::construct<Return>::from(ref<1>(t), ref<2>(t), ref<3>(t)); }

    template<class T>
    auto conjugate(const T &t) noexcept
    -> std::enable_if_t<is_quat_compat_v<T>, T>
    { return Quaternion::ScalarVector<T>(scalar(t), -vector(t)); }

    template<class T>
    auto inverse(const T &t) noexcept
    -> std::enable_if_t<is_quat_compat_v<T>, T>
    { return conjugate(t) / squared_norm(t); }

    template<class S, class T>
    auto operator*(const S &lhs, const T &rhs) noexcept
    -> std::enable_if_t<is_quat_compat_v<S> && is_quat_compat_v<T>, S>
    {
        return Quaternion::ScalarVector<T>(
            scalar(lhs) * scalar(rhs) - dot(vector(lhs), vector(rhs)),
            scalar(lhs) * vector(rhs) + scalar(rhs) * vector(lhs) + cross(vector(lhs), vector(rhs))
        );
    }

    template<class S, class T>
    auto operator*(const S &lhs, const T &rhs) noexcept
    -> std::enable_if_t<is_quat_compat_v<S> && is_vec3_compat_v<T>, T>
    { return vector<T>(lhs * Quaternion::ScalarVector(0, rhs) * inverse(lhs)); }

    template<class S, class T>
    auto lerp(const S &a, const T &b, double t) noexcept
    -> std::enable_if_t<is_same_size_v<S, T>, S>
    { return a + t * (b - a); }

    template<class S, class T>
    auto slerp(const S &a, const T &b, double t) noexcept
    -> std::enable_if_t<is_quat_compat_v<S> && is_quat_compat_v<T>, S>
    {
        double dot = dot(a, b);
        double theta = std::acos(dot);
        return std::sin((1 - t) * theta) / std::sin(theta) * a
            + std::sin(t * theta) / std::sin(theta) * (dot < 0 ? -b : b);
    }

    struct Color : std_msgs::ColorRGBA {
        Color() = default;

        Color(const float r, const float g, const float b, const float a = 1.0f) noexcept
        { this->r = r, this->g = g, this->b = b, this->a = a; }

        template<class Return = Color>
        static Color White(const float alpha = 1.0f) noexcept { return detail::construct<Return>::from(1.00, 1.00, 1.00, alpha); }

        template<class Return = Color>
        static Color Silver(const float alpha = 1.0f) noexcept { return detail::construct<Return>::from(0.75, 0.75, 0.75, alpha); }

        template<class Return = Color>
        static Color Gray(const float alpha = 1.0f) noexcept { return detail::construct<Return>::from(0.50, 0.50, 0.50, alpha); }

        template<class Return = Color>
        static Color Black(const float alpha = 1.0f) noexcept { return detail::construct<Return>::from(0.00, 0.00, 0.00, alpha); }

        template<class Return = Color>
        static Color Red(const float alpha = 1.0f) noexcept { return detail::construct<Return>::from(1.00, 0.00, 0.00, alpha); }

        template<class Return = Color>
        static Color Maroon(const float alpha = 1.0f) noexcept { return detail::construct<Return>::from(0.50, 0.00, 0.00, alpha); }

        template<class Return = Color>
        static Color Yellow(const float alpha = 1.0f) noexcept { return detail::construct<Return>::from(1.00, 1.00, 0.00, alpha); }

        template<class Return = Color>
        static Color Olive(const float alpha = 1.0f) noexcept { return detail::construct<Return>::from(0.50, 0.50, 0.00, alpha); }

        template<class Return = Color>
        static Color Lime(const float alpha = 1.0f) noexcept { return detail::construct<Return>::from(0.00, 1.00, 0.00, alpha); }

        template<class Return = Color>
        static Color Green(const float alpha = 1.0f) noexcept { return detail::construct<Return>::from(0.00, 0.50, 0.00, alpha); }

        template<class Return = Color>
        static Color Aqua(const float alpha = 1.0f) noexcept { return detail::construct<Return>::from(0.00, 1.00, 1.00, alpha); }

        template<class Return = Color>
        static Color Teal(const float alpha = 1.0f) noexcept { return detail::construct<Return>::from(0.00, 0.50, 0.50, alpha); }

        template<class Return = Color>
        static Color Blue(const float alpha = 1.0f) noexcept { return detail::construct<Return>::from(0.00, 0.00, 1.00, alpha); }

        template<class Return = Color>
        static Color Navy(const float alpha = 1.0f) noexcept { return detail::construct<Return>::from(0.00, 0.00, 0.50, alpha); }

        template<class Return = Color>
        static Color Fuchsia(const float alpha = 1.0f) noexcept { return detail::construct<Return>::from(1.00, 0.00, 1.00, alpha); }

        template<class Return = Color>
        static Color Purple(const float alpha = 1.0f) noexcept { return detail::construct<Return>::from(0.50, 0.00, 0.50, alpha); }

        template<class T>
        operator T() { return util::convert<T>(*this); }
    };

    namespace detail {
        template<class T>
        struct access<T, std::enable_if_t<std::is_base_of_v<std_msgs::ColorRGBA, T>>> {
            static constexpr size_t size = 4;

            template<size_t I>
            static float &ref(std_msgs::ColorRGBA &t) noexcept
            {
                if constexpr (I == 0) return t.r;
                if constexpr (I == 1) return t.g;
                if constexpr (I == 2) return t.b;
                if constexpr (I == 3) return t.a;
            }
            template<size_t I>
            static const float &ref(const std_msgs::ColorRGBA &t) noexcept
            {
                if constexpr (I == 0) return t.r;
                if constexpr (I == 1) return t.g;
                if constexpr (I == 2) return t.b;
                if constexpr (I == 3) return t.a;
            }
        };
    }

}

namespace marker {

    namespace attr {
        template<class Derived, class Base>
        struct CRTP : Base {
        protected:
            Derived &derived() noexcept { return static_cast<Derived &>(*this); }
            const Derived &derived() const noexcept { return static_cast<const Derived &>(*this); }
        };

        template<class Derived, class Base>
        struct Position : Base {
            template<class T>
            Derived &position(const T &position) noexcept
            {
                this->message.pose.position = util::convert<geometry_msgs::Point>(position);
                return this->derived();
            }

            Derived &position(const double x, const double y, const double z) noexcept
            {
                this->message.pose.position.x = x;
                this->message.pose.position.y = y;
                this->message.pose.position.z = z;
                return this->derived();
            }
        };

        template<class Derived, class Base>
        struct Orientation : Base {
            Orientation() noexcept { orientation(1, 0, 0, 0); }

            template<class T>
            Derived &orientation(const T &orientation) noexcept
            {
                this->message.pose.orientation = util::convert<geometry_msgs::Quaternion>(orientation);
                return this->derived();
            }

            Derived &orientation(const double w, const double x, const double y, const double z) noexcept
            {
                this->message.pose.orientation.w = w;
                this->message.pose.orientation.x = x;
                this->message.pose.orientation.y = y;
                this->message.pose.orientation.z = z;
                return this->derived();
            }
        };

        template<class Derived, class Base>
        struct Scale : Base {
            Scale() noexcept { scale(1, 1, 1); }

            Derived &scale(const double x, const double y, const double z) noexcept
            {
                this->message.scale.x = x;
                this->message.scale.y = y;
                this->message.scale.z = z;
                return this->derived();
            }
        };

        template<class Derived, class Base>
        struct PoseArrowScale : Base {
            PoseArrowScale() noexcept { scale(0.2, 0.2, 1); }

            Derived &scale(const double length, const double width, const double height) noexcept
            {
                this->message.scale.x = length;
                this->message.scale.y = width;
                this->message.scale.z = height;
                return this->derived();
            }
        };

        template<class Derived, class Base>
        struct VectorArrowScale : Base {
            VectorArrowScale() noexcept { scale(0.2, 0.4, 0.4); }

            Derived &scale(const double shaft_diameter, const double head_diameter, const double head_length) noexcept
            {
                this->message.scale.x = shaft_diameter;
                this->message.scale.y = head_diameter;
                this->message.scale.z = head_length;
                return this->derived();
            }
        };

        template<class Derived, class Base>
        struct PointScale : Base {
            PointScale() noexcept { scale(0.05); }

            Derived &scale(const double width) noexcept
            { return scale(width, width); }

            Derived &scale(const double width, const double height) noexcept
            {
                this->message.scale.x = width;
                this->message.scale.y = height;
                return this->derived();
            }
        };

        template<class Derived, class Base>
        struct LineScale : Base {
            LineScale() noexcept { scale(0.05); }

            Derived &scale(const double width) noexcept
            {
                this->message.scale.x = width;
                return this->derived();
            }
        };

        template<class Derived, class Base>
        struct TextScale : Base {
            TextScale() noexcept { scale(0.05); }

            Derived &scale(const double height) noexcept
            {
                this->message.scale.z = height;
                return this->derived();
            }
        };

        template<class Derived, class Base>
        struct Color : Base {
            Color() noexcept { color(1, 1, 1, 1); }

            template<class T>
            Derived &color(const T &color) noexcept
            {
                this->message.color = util::convert<std_msgs::ColorRGBA>(color);
                return this->derived();
            }

            Derived &color(const double r, const double g, const double b, const double a = 1) noexcept
            {
                this->message.color.r = r;
                this->message.color.g = g;
                this->message.color.b = b;
                this->message.color.a = a;
                return this->derived();
            }
        };

        template<class Derived, class Base>
        struct Colors : Color<Derived, Base> {
            Colors() noexcept { color(1, 1, 1, 1); }

            template<class T>
            Derived &color(const T &color) noexcept
            {
                this->message.colors.clear();
                return Color<Derived, Base>::color(color);
            }

            Derived &color(const double r, const double g, const double b, const double a = 1) noexcept
            {
                this->message.colors.clear();
                return Color<Derived, Base>::color(r, g, b, a);
            }

            template<class Iterable>
            Derived &colors(const Iterable &iterable) noexcept
            {
                if constexpr (util::has_size_v<Iterable>) {
                    this->message.colors.reserve(iterable.size());
                }
                for (const auto &e : iterable) {
                    this->message.colors.push_back(util::convert<std_msgs::ColorRGBA>(e));
                }
                return this->derived();
            }
        };

        template<class Derived, class Base>
        struct Points : Base {
            template<class Iterable>
            Derived &points(const Iterable &iterable) noexcept
            {
                if constexpr (util::has_size_v<Iterable>) {
                    this->message.points.reserve(iterable.size());
                }
                for (const auto &e : iterable) {
                    this->message.points.push_back(util::convert<geometry_msgs::Point>(e));
                }
                return this->derived();
            }
        };

        template<class Derived, class Base>
        struct LinePoints : Base {
            LinePoints() noexcept
            {
                this->message.points.resize(2);
                end(1, 0, 0);
            }

            template<class T>
            Derived &start(const T &point) noexcept { return this->point(0, point); }
            Derived &start(const double x, const double y, const double z) noexcept { return this->point(0, x, y, z); }

            template<class T>
            Derived &end(const T &point) noexcept { return this->point(1, point); }
            Derived &end(const double x, const double y, const double z) noexcept { return this->point(1, x, y, z); }

        private:
            template<class T>
            Derived &point(size_t index, const T &point) noexcept
            {
                this->message.points[index] = util::convert<geometry_msgs::Point>(point);
                return this->derived();
            }

            Derived &point(size_t index, const double x, const double y, const double z) noexcept
            {
                this->message.points[index].x = x;
                this->message.points[index].y = y;
                this->message.points[index].z = z;
                return this->derived();
            }
        };

        template<class Derived, class Base>
        struct Text : Base {
            Text() noexcept { text("visualization_msgs::Marker"); }

            Derived &text(const std::string &text) noexcept
            {
                this->message.text = text;
                return this->derived();
            }
        };

        template<class Derived, class Base>
        struct MeshResource : Base {
            Derived &mesh_resource(const std::string &mesh_resource) noexcept
            {
                this->message.mesh_resouce = mesh_resource;
                return this->derived();
            }

            Derived &mesh_use_embedded_materials(const uint8_t mesh_use_embedded_materials) noexcept
            {
                this->message.mesh_use_embedded_materials = mesh_use_embedded_materials;
                return this->derived();
            }
        };

    }

    struct MarkerWrapper {
        operator const visualization_msgs::Marker &() { return this->message; }

    protected:
        visualization_msgs::Marker message;
    };

    struct DeleteAll : MarkerWrapper {
        DeleteAll(const std::string &ns = "")
        {
            this->message.action = visualization_msgs::Marker::DELETEALL;
            this->message.ns = ns;
        }
    };

    struct Delete : MarkerWrapper {
        Delete(const int32_t id, const std::string &ns = "")
        {
            this->message.action = visualization_msgs::Marker::DELETE;
            this->message.id = id;
            this->message.ns = ns;
        }
    };

    template<
        int32_t Type,
        template<class, class> class ...Features>
    struct Add : util::chained<Add<Type, Features...>, MarkerWrapper, attr::CRTP, Features...> {
        Add(const int32_t id, const std::string &ns = "")
        {
            this->message.action = visualization_msgs::Marker::DELETEALL;
            this->message.type = Type;
            this->message.id = id;
            this->message.ns = ns;
        }

        Add &lifetime(const double lifetime) noexcept
        {
            this->message.lifetime = lifetime;
            return *this;
        }

        Add &frame_locked(const uint8_t frame_locked) noexcept
        {
            this->message.frame_locked = frame_locked;
            return *this;
        }
    };

    using PoseArrow = Add<
        visualization_msgs::Marker::ARROW,
        attr::Position, attr::Orientation, attr::PoseArrowScale, attr::Color>;

    using LineArrow = Add<
        visualization_msgs::Marker::ARROW,
        attr::VectorArrowScale, attr::Color, attr::LinePoints>;

    using Cube = Add<
        visualization_msgs::Marker::CUBE,
        attr::Position, attr::Orientation, attr::Scale, attr::Color>;

    using Sphere = Add<
        visualization_msgs::Marker::SPHERE,
        attr::Position, attr::Orientation, attr::Scale, attr::Color>;

    using Cylinder = Add<
        visualization_msgs::Marker::CYLINDER,
        attr::Position, attr::Orientation, attr::Scale, attr::Color>;

    using LineStrip = Add<
        visualization_msgs::Marker::LINE_STRIP,
        attr::Position, attr::Orientation, attr::LineScale, attr::Color, attr::Points>;

    using LineList = Add<
        visualization_msgs::Marker::LINE_LIST,
        attr::Position, attr::Orientation, attr::LineScale, attr::Colors, attr::Points>;

    using CubeList = Add<
        visualization_msgs::Marker::CUBE_LIST,
        attr::Position, attr::Orientation, attr::Scale, attr::Colors, attr::Points>;

    using SphereList = Add<
        visualization_msgs::Marker::SPHERE_LIST,
        attr::Position, attr::Orientation, attr::Scale, attr::Colors, attr::Points>;

    using Points = Add<
        visualization_msgs::Marker::POINTS,
        attr::Position, attr::Orientation, attr::PointScale, attr::Colors, attr::Points>;

    using TextViewFacing = Add<
        visualization_msgs::Marker::TEXT_VIEW_FACING,
        attr::Position, attr::TextScale, attr::Color, attr::Text>;

    using MeshResource = Add<
        visualization_msgs::Marker::MESH_RESOURCE,
        attr::Position, attr::Orientation, attr::Scale, attr::Color, attr::MeshResource>;

    using TriangleList = Add<
        visualization_msgs::Marker::TRIANGLE_LIST,
        attr::Position, attr::Orientation, attr::Scale, attr::Colors, attr::Points>;

}

class Rviz {
    ros::NodeHandle _node_handle;
    ros::Publisher _publisher;

    std::string _frame_id;

public:
    Rviz(const std::string &frame_id = "map", const std::string &topic = "visualization_marker")
        : _publisher(_node_handle.advertise<visualization_msgs::Marker>(topic, 1))
        , _frame_id(frame_id) { }

    const Rviz &operator<<(const visualization_msgs::Marker &marker) const
    {
        publish(marker);
        return *this;
    }

    void publish(visualization_msgs::Marker marker) const
    {
        marker.header.frame_id = _frame_id;
        marker.header.stamp = ros::Time::now();
        _publisher.publish(marker);
    }
};

}
