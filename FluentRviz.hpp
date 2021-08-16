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
    struct converter { };

    template<class Type>
    struct converter<Type, Type> {
        static constexpr const Type &convert(const Type &value) { return value; };
    };

    template<class T, size_t I>
    struct accessor { };

}

namespace util {

    template<class To, class From>
    constexpr decltype(auto) convert(const From &from) { return traits::converter<From, To>::convert(from); }

    template<size_t I, class T>
    constexpr decltype(auto) get(const T &t) { return traits::accessor<T, I>::get(t); }

    template<size_t I, class T, class Arg>
    constexpr void set(T &t, Arg &&arg) { return traits::accessor<T, I>::set(t, std::forward<Arg>(arg)); };

    template<
        class Derived,
        class Base,
        template<class, class> class ...Decorators>
    struct Decorate : Base { };

    template<
        class Derived,
        class Base,
        template<class, class> class Decorator>
    struct Decorate<Derived, Base, Decorator>
        : Decorator<Derived, Base> { };

    template<
        class Derived,
        class Base,
        template<class, class> class Decorator,
        template<class, class> class ...Decorators>
    struct Decorate<Derived, Base, Decorator, Decorators...>
        : Decorate<Derived, Decorator<Derived, Base>, Decorators...> { };

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

    template<class Derived, class ...Args>
    class ExtensionClosure;

    template<class Extension0, class Extension1>
    class ExtensionChain;

    struct Extension { };

    template<class T>
    static constexpr bool is_extension_v = std::is_base_of_v<Extension, std::remove_reference_t<T>>;

    template<
        class Self,
        class Extension,
        std::enable_if_t<!is_extension_v<Self> && is_extension_v<Extension>, int> = 0>
    constexpr decltype(auto) operator|(Self &&self, Extension &&ex)
    { return std::forward<Extension>(ex)(std::forward<Self>(self)); }

    template<
        class Extension0,
        class Extension1,
        std::enable_if_t<is_extension_v<Extension0> && is_extension_v<Extension1>, int> = 0>
    constexpr auto operator|(Extension0 &&ex0, Extension1 &&ex1)
    { return ExtensionChain(std::forward<Extension0>(ex0), std::forward<Extension1>(ex1)); }

    template<class T, class ...Args>
    using process_type = decltype(T::process(std::declval<Args>()...));

    template<class T, class ...Args>
    constexpr inline bool is_process_invokable_v = is_detected_v<process_type, T, Args...>;

    template<class T>
    struct ExtensionAdapter {
        template<
            class ...Args,
            std::enable_if_t<!is_process_invokable_v<T, Args...>, int> = 0>
        constexpr auto operator()(Args &&...args) const
        { return ExtensionClosure<T, Args...>(std::forward<Args>(args)...); }

        template<
            class ...Args,
            std::enable_if_t<is_process_invokable_v<T, Args...>, int> = 0>
        constexpr decltype(auto) operator()(Args &&...args) const
        { return T::process(std::forward<Args>(args)...); }
    };

    template<class Derived, class ...Args>
    class ExtensionClosure : Extension {
        std::tuple<Args...> _args;

    public:
        constexpr ExtensionClosure(Args ...args)
            : _args(args...)
        { }

        template<class Self>
        constexpr decltype(auto) operator()(Self &&self) const
        {
            return std::apply([&self](const auto &...args) -> decltype(auto) {
                return Derived::process(std::forward<Self>(self), args...);
            }, _args);
        }
    };

    template<class Extension0, class Extension1>
    class ExtensionChain : Extension {
        Extension0 _ex0;
        Extension1 _ex1;

    public:
        constexpr ExtensionChain(const Extension0 &ex0, const Extension1 &ex1)
            : _ex0(ex0)
            , _ex1(ex1)
        { }

        template<class Self>
        constexpr decltype(auto) operator()(Self &&self) const
        { return std::forward<Self>(self) | _ex0 | _ex1; }
    };

}

namespace param {

    template<size_t D, class Derived>
    struct VectorBase {
        std::array<double, D> storage;

        constexpr double &operator[](ssize_t i) noexcept { return storage[i]; }
        constexpr const double &operator[](ssize_t i) const noexcept { return storage[i]; }

        template<class Op>
        static constexpr Derived &apply(Derived &lhs, const Derived &rhs, const Op &op = Op()) noexcept
        {
            for (size_t i = 0; i < D; i++) lhs[i] = op(lhs[i], rhs[i]);
            return lhs;
        }

        template<class Op>
        static constexpr Derived &apply(Derived &lhs, const double rhs, const Op &op = Op()) noexcept
        {
            for (size_t i = 0; i < D; i++) lhs[i] = op(lhs[i], rhs);
            return lhs;
        }

        friend constexpr Derived &operator+=(Derived &lhs, const Derived &rhs) noexcept { return apply(lhs, rhs, std::plus<double>()); };
        friend constexpr Derived &operator+=(Derived &&lhs, const Derived &rhs) noexcept { return apply(lhs, rhs, std::plus<double>()); };
        friend constexpr Derived &operator-=(Derived &lhs, const Derived &rhs) noexcept { return apply(lhs, rhs, std::minus<double>()); };
        friend constexpr Derived &operator-=(Derived &&lhs, const Derived &rhs) noexcept { return apply(lhs, rhs, std::minus<double>()); };
        friend constexpr Derived &operator*=(Derived &lhs, const double &rhs) noexcept { return apply(lhs, rhs, std::multiplies<double>()); };
        friend constexpr Derived &operator*=(Derived &&lhs, const double &rhs) noexcept { return apply(lhs, rhs, std::multiplies<double>()); };
        friend constexpr Derived &operator/=(Derived &lhs, const double &rhs) noexcept { return apply(lhs, rhs, std::divides<double>()); };
        friend constexpr Derived &operator/=(Derived &&lhs, const double &rhs) noexcept { return apply(lhs, rhs, std::divides<double>()); };

        friend constexpr Derived operator+(const Derived &lhs, const Derived &rhs) noexcept { return Derived(lhs) += rhs; }
        friend constexpr Derived operator-(const Derived &lhs, const Derived &rhs) noexcept { return Derived(lhs) -= rhs; }
        friend constexpr Derived operator*(const Derived &lhs, const double &rhs) noexcept { return Derived(lhs) *= rhs; }
        friend constexpr Derived operator*(const double &lhs, const Derived &rhs) noexcept { return Derived(rhs) *= lhs; }
        friend constexpr Derived operator/(const Derived &lhs, const double &rhs) noexcept { return Derived(lhs) /= rhs; }

        friend constexpr Derived operator+(const Derived &s) noexcept { return s; }
        friend constexpr Derived operator-(const Derived &s) noexcept { return s * -1; }
    };

    template<size_t D, class Derived>
    constexpr double dot(const VectorBase<D, Derived> &lhs, const VectorBase<D, Derived> &rhs) noexcept
    { return std::inner_product(lhs.storage.begin(), lhs.storage.end(), rhs.storage.begin(), 0.0); }

    template<size_t D, class Derived>
    constexpr double norm(const VectorBase<D, Derived> &v) noexcept
    { return std::sqrt(dot(v, v)); }

    template<size_t D>
    struct Vector : VectorBase<D, Vector<D>> { };

    using Vector3 = Vector<3>;

    constexpr Vector3 cross(const Vector3 &lhs, const Vector3 &rhs) noexcept
    {
        auto [ lx, ly, lz ] = lhs.storage;
        auto [ rx, ry, rz ] = rhs.storage;
        return { ly * rz - lz * ry, lz * rx - lx * rz, lx * ry - ly * rx };
    }

    struct Quaternion : public VectorBase<4, Quaternion> {
        constexpr Quaternion(const double w, const double x, const double y, const double z) noexcept
            : VectorBase { w, x, y, z }
        { }

        constexpr Quaternion(const double scalar, const Vector3 &vector) noexcept
            : Quaternion { scalar, vector[0], vector[1], vector[2] }
        { }
    };

    constexpr double scalar(const Quaternion &q) noexcept { return q[0]; }
    constexpr Vector3 vector(const Quaternion &q) noexcept { return { q[1], q[2], q[3] }; }
    constexpr Quaternion conjugate(const Quaternion &q) noexcept { return { scalar(q), -vector(q) }; }
    constexpr Quaternion inverse(const Quaternion &q) noexcept { return conjugate(q) / norm(q); }

    constexpr Quaternion operator*(const Quaternion &lhs, const Quaternion &rhs) noexcept
    {
        double lsc = scalar(lhs), rsc = scalar(rhs);
        Vector3 lvec = vector(lhs), rvec = vector(rhs);
        return { lsc * rsc - dot(lvec, rvec), lsc * rvec + rsc * lvec + cross(lvec, rvec) };
    }

    struct Axis : public Vector3 {
        constexpr Axis(const Vector3 &v)
            : Vector3 { v / norm(v) }
        { }

        constexpr Axis(const double x, const double y, const double z)
            : Axis { { x, y, z } }
        { }

        static constexpr Axis X() { return { 1, 0, 0 }; }
        static constexpr Axis Y() { return { 0, 1, 0 }; }
        static constexpr Axis Z() { return { 0, 0, 1 }; }
    };

    struct Rotation : public Quaternion {
        constexpr Rotation(const double angle, const Axis &axis = Axis::Z()) noexcept
            : Quaternion { std::cos(angle / 2), (axis / norm(axis)) * std::sin(angle / 2) }
        { }

        constexpr Rotation(const Quaternion &q) noexcept
            : Quaternion { q / norm(q) }
        { }
    };

    constexpr double angle(const Rotation &r) noexcept { return std::acos(scalar(r)); }
    constexpr Axis axis(const Rotation &r) noexcept { return vector(r) / std::sin(angle(r) / 2); }

    template<class Derived>
    constexpr Derived rotate(const Vector3 &v, const Rotation &r) noexcept
    { return vector(r * Quaternion(0, v) * inverse(r)); }

    struct Color {
        float r, g, b, a;

        static constexpr Color White()   noexcept { return { 1.00, 1.00, 1.00 }; }
        static constexpr Color Silver()  noexcept { return { 0.75, 0.75, 0.75 }; }
        static constexpr Color Gray()    noexcept { return { 0.50, 0.50, 0.50 }; }
        static constexpr Color Black()   noexcept { return { 0.00, 0.00, 0.00 }; }
        static constexpr Color Red()     noexcept { return { 1.00, 0.00, 0.00 }; }
        static constexpr Color Maroon()  noexcept { return { 0.50, 0.00, 0.00 }; }
        static constexpr Color Yellow()  noexcept { return { 1.00, 1.00, 0.00 }; }
        static constexpr Color Olive()   noexcept { return { 0.50, 0.50, 0.00 }; }
        static constexpr Color Lime()    noexcept { return { 0.00, 1.00, 0.00 }; }
        static constexpr Color Green()   noexcept { return { 0.00, 0.50, 0.00 }; }
        static constexpr Color Aqua()    noexcept { return { 0.00, 1.00, 1.00 }; }
        static constexpr Color Teal()    noexcept { return { 0.00, 0.50, 0.50 }; }
        static constexpr Color Blue()    noexcept { return { 0.00, 0.00, 1.00 }; }
        static constexpr Color Navy()    noexcept { return { 0.00, 0.00, 0.50 }; }
        static constexpr Color Fuchsia() noexcept { return { 1.00, 0.00, 1.00 }; }
        static constexpr Color Purple()  noexcept { return { 0.50, 0.00, 0.50 }; }
    };

    namespace detail {
        template<class T, size_t I>
        struct ColorComponent {
            template<class S, std::enable_if_t<util::is_convertible_v<S, T>, int> = 0>
            static constexpr S process(const S &s, const double arg)
            {
                T tmp = util::convert<T>(s);
                util::set<I>(tmp, arg);
                return util::convert<S>(tmp);
            }

            template<class S, std::enable_if_t<util::is_convertible_v<S, T>, int> = 0>
            static constexpr auto process(const S &s)
            {
                T tmp = util::convert<T>(s);
                return util::get<I>(tmp);
            }
        };
    }

    namespace color {
        constexpr inline auto red   = util::ExtensionAdapter<detail::ColorComponent<Color, 0>>();
        constexpr inline auto green = util::ExtensionAdapter<detail::ColorComponent<Color, 1>>();
        constexpr inline auto blue  = util::ExtensionAdapter<detail::ColorComponent<Color, 2>>();
        constexpr inline auto alpha = util::ExtensionAdapter<detail::ColorComponent<Color, 3>>();
    }

    struct RGBA {
        float r, g, b, a;
    };

    namespace rgba {
        constexpr inline auto red   = util::ExtensionAdapter<detail::ColorComponent<RGBA, 0>>();
        constexpr inline auto blue  = util::ExtensionAdapter<detail::ColorComponent<RGBA, 1>>();
        constexpr inline auto green = util::ExtensionAdapter<detail::ColorComponent<RGBA, 2>>();
    }

    struct HSLA {
        float h, s, l, a;
    };

    namespace hsla {
        constexpr inline auto hue        = util::ExtensionAdapter<detail::ColorComponent<HSLA, 0>>();
        constexpr inline auto saturation = util::ExtensionAdapter<detail::ColorComponent<HSLA, 1>>();
        constexpr inline auto lightness  = util::ExtensionAdapter<detail::ColorComponent<HSLA, 2>>();
    }

}

namespace traits {

    template<class Vector3Like>
    struct converter<Vector3Like, param::Vector3,
        std::enable_if_t<
            std::is_same_v<Vector3Like, geometry_msgs::Vector3>
            || std::is_same_v<Vector3Like, geometry_msgs::Point>>> {

        static constexpr param::Vector3 convert(const Vector3Like &vec)
        { return { vec.x, vec.y, vec.z }; }
    };

    template<class Vector3Like>
    struct converter<param::Vector3, Vector3Like,
        std::enable_if_t<
            std::is_same_v<Vector3Like, geometry_msgs::Vector3>
            || std::is_same_v<Vector3Like, geometry_msgs::Point>>> {

        static Vector3Like convert(const param::Vector3 &vec)
        {
            Vector3Like ret;
            ret.x = vec[0], ret.y = vec[1], ret.z = vec[2];
            return ret;
        }
    };

    template<class Quaternion>
    struct converter<geometry_msgs::Quaternion, Quaternion,
        std::enable_if_t<std::is_base_of_v<param::Quaternion, Quaternion>>> {

        static constexpr Quaternion convert(const geometry_msgs::Quaternion &quat)
        { return { quat.w, quat.x, quat.y, quat.z }; }
    };

    template<class Quaternion>
    struct converter<Quaternion, geometry_msgs::Quaternion,
        std::enable_if_t<std::is_base_of_v<param::Quaternion, Quaternion>>> {

        static geometry_msgs::Quaternion convert(const Quaternion &quat)
        {
            geometry_msgs::Quaternion ret;
            ret.w = quat[0], ret.x = quat[1], ret.y = quat[2], ret.z = quat[3];
            return ret;
        }
    };

    template<class ColorA, class ColorB>
    struct converter<ColorA, ColorB,
        std::enable_if_t<
            !std::is_same_v<ColorA, ColorB>
            && util::is_convertible_v<ColorA, param::Color>
            && util::is_convertible_v<param::Color, ColorB>>> {

        static constexpr ColorB convert(const ColorA &color)
        { return converter<param::Color, ColorB>::convert(converter<ColorA, param::Color>::convert(color)); }
    };

    template<>
    struct converter<std_msgs::ColorRGBA, param::Color> {
        static constexpr param::Color convert(const std_msgs::ColorRGBA &color) noexcept
        {
            return {
                std::clamp(color.r, 0.0f, 1.0f),
                std::clamp(color.g, 0.0f, 1.0f),
                std::clamp(color.b, 0.0f, 1.0f),
                std::clamp(color.a, 0.0f, 1.0f),
            };
        }
    };

    template<>
    struct converter<param::Color, std_msgs::ColorRGBA> {
        static std_msgs::ColorRGBA convert(const param::Color &color) noexcept
        {
            std_msgs::ColorRGBA ret;
            ret.r = std::clamp(color.r, 0.0f, 1.0f);
            ret.g = std::clamp(color.g, 0.0f, 1.0f);
            ret.b = std::clamp(color.b, 0.0f, 1.0f);
            ret.a = std::clamp(color.a, 0.0f, 1.0f);
            return ret;
        }
    };

    template<>
    struct converter<param::RGBA, param::Color> {
        static constexpr param::Color convert(const param::RGBA &color) noexcept
        {
            return {
                std::clamp(color.r / 255.0f, 0.0f, 1.0f),
                std::clamp(color.g / 255.0f, 0.0f, 1.0f),
                std::clamp(color.b / 255.0f, 0.0f, 1.0f),
                std::clamp(color.a, 0.0f, 1.0f),
            };
        }
    };

    template<>
    struct converter<param::Color, param::RGBA> {
        static constexpr param::RGBA convert(const param::Color &color) noexcept
        {
            return {
                std::clamp(color.r, 0.0f, 1.0f) * 255.0f,
                std::clamp(color.g, 0.0f, 1.0f) * 255.0f,
                std::clamp(color.b, 0.0f, 1.0f) * 255.0f,
                std::clamp(color.a, 0.0f, 1.0f),
            };
        }
    };

    template<>
    struct converter<param::HSLA, param::Color> {
        static constexpr param::Color convert(const param::HSLA &hsla) noexcept
        {
            const float h = std::remainder(hsla.h / 60.0f + 6.0f, 6.0f);
            const float s = std::clamp(hsla.s / 100.0f, 0.0f, 1.0f);
            const float l = std::clamp(hsla.l / 100.0f, 0.0f, 1.0f);
            const float a = std::clamp(hsla.a, 0.0f, 1.0f);

            const float c = (1.0f - std::abs(2.0f * l - 1.0f)) * s;
            const float x = c * (1.0f - std::abs(std::remainder(h, 2.0f) - 1.0f));
            const float m = l - c / 2.0f;

            if (h < 1.0f) return { m + c, m + x, m     };
            if (h < 2.0f) return { m + x, m + c, m     };
            if (h < 3.0f) return { m    , m + c, m + x };
            if (h < 4.0f) return { m    , m + x, m + c };
            if (h < 5.0f) return { m + x, m    , m + c };
                          return { m + c, m    , m + x };
        }
    };

    template<>
    struct converter<param::Color, param::HSLA> {
        static constexpr param::HSLA convert(const param::Color &color) noexcept
        {
            const float r = std::clamp(color.r, 0.0f, 1.0f);
            const float g = std::clamp(color.g, 0.0f, 1.0f);
            const float b = std::clamp(color.b, 0.0f, 1.0f);
            const float a = std::clamp(color.a, 0.0f, 1.0f);

            const float max = std::max({ r, g, b });
            const float min = std::min({ r, g, b });
            const float c = max - min;

            const float h = 60.0f * [&] {
                if (c != 0) return 0.0f;
                if (max == g) return 2.0f + (b - r) / c;
                if (max == b) return 4.0f + (r - g) / c;
                return std::remainder(6.0f + (g - b) / c, 6.0f);
            }();
            const float l = 100.0f * ((max + min) / 2.0f);
            const float s = 100.0f * [&] {
                if (l == 0.0f || l == 1.0f) return 0.0f;
                return c / (1.0f - std::abs(2.0f * l - 1.0f));
            }();

            return { h, s, l, a };
        }
    };

    template<>
    struct accessor<param::Color, 0> {
        static constexpr float get(const param::Color &c) { return c.r; }
        static constexpr void set(param::Color &c, float arg) { c.r = arg; }
    };

    template<>
    struct accessor<param::Color, 1> {
        static constexpr float get(const param::Color &c) { return c.g; }
        static constexpr void set(param::Color &c, float arg) { c.g = arg; }
    };

    template<>
    struct accessor<param::Color, 2> {
        static constexpr float get(const param::Color &c) { return c.b; }
        static constexpr void set(param::Color &c, float arg) { c.b = arg; }
    };

    template<>
    struct accessor<param::Color, 3> {
        static constexpr float get(const param::Color &c) { return c.a; }
        static constexpr void set(param::Color &c, float arg) { c.a = arg; }
    };

    template<>
    struct accessor<param::RGBA, 0> {
        static constexpr float get(const param::RGBA &c) { return c.r; }
        static constexpr void set(param::RGBA &c, float arg) { c.r = arg; }
    };

    template<>
    struct accessor<param::RGBA, 1> {
        static constexpr float get(const param::RGBA &c) { return c.g; }
        static constexpr void set(param::RGBA &c, float arg) { c.g = arg; }
    };

    template<>
    struct accessor<param::RGBA, 2> {
        static constexpr float get(const param::RGBA &c) { return c.b; }
        static constexpr void set(param::RGBA &c, float arg) { c.b = arg; }
    };

    template<>
    struct accessor<param::RGBA, 3> {
        static constexpr float get(const param::RGBA &c) { return c.a; }
        static constexpr void set(param::RGBA &c, float arg) { c.a = arg; }
    };

    template<>
    struct accessor<param::HSLA, 0> {
        static constexpr float get(const param::HSLA &c) { return c.h; }
        static constexpr void set(param::HSLA &c, float arg) { c.h = arg; }
    };

    template<>
    struct accessor<param::HSLA, 1> {
        static constexpr float get(const param::HSLA &c) { return c.s; }
        static constexpr void set(param::HSLA &c, float arg) { c.s = arg; }
    };

    template<>
    struct accessor<param::HSLA, 2> {
        static constexpr float get(const param::HSLA &c) { return c.l; }
        static constexpr void set(param::HSLA &c, float arg) { c.l = arg; }
    };

    template<>
    struct accessor<param::HSLA, 3> {
        static constexpr float get(const param::HSLA &c) { return c.a; }
        static constexpr void set(param::HSLA &c, float arg) { c.a = arg; }
    };
}

namespace marker {

    namespace decorator {
        template<class Derived, class Base>
        struct CRTP : Base {
        protected:
            Derived &derived() noexcept { return static_cast<Derived &>(*this); }
            const Derived &derived() const noexcept { return static_cast<const Derived &>(*this); }
        };

        template<int32_t Type>
        struct ActionType {
            template<class Derived, class Base>
            struct Decorator : Base {
                Decorator() noexcept { this->message.action = Type; }
            };
        };

        template<int32_t Type>
        struct MarkerType {
            template<class Derived, class Base>
            struct Decorator : Base {
                Decorator() noexcept { this->message.type = Type; }
            };
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
            PointScale() noexcept { scale(0.05, 0.05); }

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
        struct ArrowPoints : Base {
            ArrowPoints() noexcept
            {
                this->message.points.resize(2);
                end(1, 0, 0);
            }

            template<class T>
            Derived &start(const T &point) noexcept { return set(0, point); }
            Derived &start(const double x, const double y, const double z) noexcept { return set(0, x, y, z); }

            template<class T>
            Derived &end(const T &point) noexcept { return set(1, point); }
            Derived &end(const double x, const double y, const double z) noexcept { return set(1, x, y, z); }

        private:
            template<class T>
            Derived &set(size_t index, const T &point) noexcept
            {
                this->message.points[index] = util::convert<geometry_msgs::Point>(point);
                return this->derived();
            }

            Derived &set(size_t index, const double x, const double y, const double z) noexcept
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

    template<class T>
    struct MessageBase {
        operator const T &() { return this->message; }

    protected:
        T message;
    };

    struct DeleteAll
        : util::Decorate<
            DeleteAll,
            MessageBase<visualization_msgs::Marker>,
            decorator::ActionType<visualization_msgs::Marker::DELETEALL>::Decorator
        > {

        DeleteAll(const std::string &ns = "")
        {
            this->message.ns = ns;
        }
    };

    struct Delete
        : util::Decorate<
            Delete,
            MessageBase<visualization_msgs::Marker>,
            decorator::ActionType<visualization_msgs::Marker::DELETE>::Decorator
        > {

        Delete(const int32_t id, const std::string &ns = "")
        {
            this->message.id = id;
            this->message.ns = ns;
        }
    };

    template<
        int32_t Type,
        template<class, class> class ...Decorators>
    struct Add
        : util::Decorate<
            Add<Type, Decorators...>,
            MessageBase<visualization_msgs::Marker>,
            decorator::CRTP,
            decorator::ActionType<visualization_msgs::Marker::ADD>::Decorator,
            decorator::MarkerType<Type>::template Decorator,
            Decorators...
        > {

        Add(const int32_t id, const std::string &ns = "")
        {
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
        decorator::Position, decorator::Orientation, decorator::PoseArrowScale, decorator::Color>;

    using VectorArrow = Add<
        visualization_msgs::Marker::ARROW,
        decorator::VectorArrowScale, decorator::Color, decorator::ArrowPoints>;

    using Cube = Add<
        visualization_msgs::Marker::CUBE,
        decorator::Position, decorator::Orientation, decorator::Scale, decorator::Color>;

    using Sphere = Add<
        visualization_msgs::Marker::SPHERE,
        decorator::Position, decorator::Orientation, decorator::Scale, decorator::Color>;

    using Cylinder = Add<
        visualization_msgs::Marker::CYLINDER,
        decorator::Position, decorator::Orientation, decorator::Scale, decorator::Color>;

    using LineStrip = Add<
        visualization_msgs::Marker::LINE_STRIP,
        decorator::Position, decorator::Orientation, decorator::LineScale, decorator::Color, decorator::Points>;

    using LineList = Add<
        visualization_msgs::Marker::LINE_LIST,
        decorator::Position, decorator::Orientation, decorator::LineScale, decorator::Colors, decorator::Points>;

    using CubeList = Add<
        visualization_msgs::Marker::CUBE_LIST,
        decorator::Position, decorator::Orientation, decorator::Scale, decorator::Colors, decorator::Points>;

    using SphereList = Add<
        visualization_msgs::Marker::SPHERE_LIST,
        decorator::Position, decorator::Orientation, decorator::Scale, decorator::Colors, decorator::Points>;

    using Points = Add<
        visualization_msgs::Marker::POINTS,
        decorator::Position, decorator::Orientation, decorator::PointScale, decorator::Colors, decorator::Points>;

    using TextViewFacing = Add<
        visualization_msgs::Marker::TEXT_VIEW_FACING,
        decorator::Position, decorator::TextScale, decorator::Color, decorator::Text>;

    using MeshResource = Add<
        visualization_msgs::Marker::MESH_RESOURCE,
        decorator::Position, decorator::Orientation, decorator::Scale, decorator::Color, decorator::MeshResource>;

    using TriangleList = Add<
        visualization_msgs::Marker::TRIANGLE_LIST,
        decorator::Position, decorator::Orientation, decorator::Scale, decorator::Colors, decorator::Points>;

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
