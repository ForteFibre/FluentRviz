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
#include <visualization_msgs/MarkerArray.h>

namespace flrv {

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
    template<class From, class To, class Enabler = void>
    struct converter;

    template<class Type>
    struct converter<Type, Type> {
        static const Type &convert(const Type &value) { return value; };
    };
}

template<class To, class From>
decltype(auto) convert(const From &from) { return detail::converter<From, To>::convert(from); }

namespace internal {
    namespace detail {
        template <class AlwaysVoid, template<class...> class Op, class... Args>
        struct detector : std::false_type { };

        template <template<class...> class Op, class... Args>
        struct detector<std::void_t<Op<Args...>>, Op, Args...> : std::true_type { };
    }

    template <template<class...> class Op, class... Args>
    constexpr inline bool is_detected_v = detail::detector<void, Op, Args...>::value;

    template<class From, class To>
    using convert_type = decltype(convert<To>(std::declval<From>()));

    template<class From, class To>
    constexpr inline bool is_convertible_v = is_detected_v<convert_type, From, To>;
}

template<class Derived, class Base>
struct CustomizableConversion : Base {
    template<class From>
    static Derived from(const From &from) { return convert<Derived>(from); }

    template<class To>
    operator To() { return convert<To>(this->derived()); }
};

template<class Derived, class Base>
struct CRTPDecorator : Base {
protected:
    Derived &derived() noexcept { return static_cast<Derived &>(*this); }
    const Derived &derived() const noexcept { return static_cast<const Derived &>(*this); }
};

template<size_t D>
struct VectorValues {
    static constexpr size_t dimension = D;
    std::array<double, D> storage;
};

template<class Derived, class Base>
struct VectorBase : Base {
    double &operator[](ssize_t i) noexcept { return this->storage[i]; }
    const double &operator[](ssize_t i) const noexcept { return this->storage[i]; }

private:
    template<class Op>
    Derived &apply(const Derived &rhs, const Op &op = Op()) noexcept
    {
        for (size_t i = 0; i < Base::dimension; i++) (*this)[i] = op((*this)[i], rhs[i]);
        return this->derived();
    }

    template<class Op>
    Derived &apply(const double &rhs, const Op &op = Op()) noexcept
    {
        for (size_t i = 0; i < Base::dimension; i++) (*this)[i] = op((*this)[i], rhs);
        return this->derived();
    }

public:
    Derived &operator+=(const Derived &rhs) noexcept { return apply(rhs, std::plus<double>()); };
    Derived &operator-=(const Derived &rhs) noexcept { return apply(rhs, std::minus<double>()); };
    Derived &operator*=(const double &rhs) noexcept { return apply(rhs, std::multiplies<double>()); };
    Derived &operator/=(const double &rhs) noexcept { return apply(rhs, std::divides<double>()); };

    Derived operator+(const Derived &rhs) const noexcept { return Derived(this->derived()) += rhs; }
    Derived operator-(const Derived &rhs) const noexcept { return Derived(this->derived()) -= rhs; }
    Derived operator*(const double &rhs) const noexcept { return Derived(this->derived()) *= rhs; }
    Derived operator/(const double &rhs) const noexcept { return Derived(this->derived()) /= rhs; }
    friend Derived operator*(const double &lhs, const Derived &rhs) noexcept { return rhs * lhs; }
    friend Derived operator/(const double &lhs, const Derived &rhs) noexcept { return rhs / lhs; }

    Derived operator+() const noexcept { return this->derived(); }
    Derived operator-() const noexcept { return this->derived() * -1; }

    double dot(const Derived &rhs) const noexcept
    {
        return std::inner_product(this->storage.begin(), this->storage.end(), rhs.storage.begin(), 0.0);
    }

    double norm() const noexcept { return std::sqrt(dot(this->derived())); }
    Derived normalize() const noexcept { return this->derived() / norm(); }
};

template<class Derived, class Base>
struct VectorAccessX : Base {
    double &x() noexcept { return (*this)[0]; }
    const double &x() const noexcept { return (*this)[0]; }
    Derived &x(double value) noexcept { x() = value; return this->derived(); }
};

template<class Derived, class Base>
struct VectorAccessY : Base {
    double &y() noexcept { return (*this)[1]; }
    const double &y() const noexcept { return (*this)[1]; }
    Derived &y(double value) noexcept { y() = value; return this->derived(); }
};

template<class Derived, class Base>
struct VectorAccessZ : Base {
    double &z() noexcept { return (*this)[2]; }
    const double &z() const noexcept { return (*this)[2]; }
    Derived &z(double value) noexcept { z() = value; return this->derived(); }
};

template<class Derived, class Base>
struct VectorAccessW : Base {
    double &w() noexcept { return (*this)[3]; }
    const double &w() const noexcept { return (*this)[3]; }
    Derived &w(double value) noexcept { w() = value; return this->derived(); }
};

struct Vector3
    : Decorate<
        Vector3, VectorValues<3>,
        CRTPDecorator, CustomizableConversion, VectorBase, VectorAccessX, VectorAccessY, VectorAccessZ
    > {

    Vector3() = default;

    Vector3(const double x, const double y, const double z)
        : Decorate { x, y, z } { }

    static inline Vector3 UnitX() noexcept { return { 1, 0, 0 }; }
    static inline Vector3 UnitY() noexcept { return { 0, 1, 0 }; }
    static inline Vector3 UnitZ() noexcept { return { 0, 0, 1 }; }

    Vector3 cross(const Vector3 &rhs) const noexcept
    {
        return {
            y() * rhs.z() - z() * rhs.y(),
            z() * rhs.x() - x() * rhs.z(),
            x() * rhs.y() - y() * rhs.x()
        };
    }
};

namespace detail {
    template<>
    struct converter<Vector3, geometry_msgs::Vector3> {
        static geometry_msgs::Vector3 convert(const Vector3 &vector)
        {
            geometry_msgs::Vector3 ret;
            ret.x = vector.x(), ret.y = vector.y(), ret.z = vector.z();
            return ret;
        }
    };

    template<>
    struct converter<geometry_msgs::Vector3, Vector3> {
        static Vector3 convert(const geometry_msgs::Vector3 &vector)
        {
            return { vector.x, vector.y, vector.z };
        }
    };

    template<>
    struct converter<Vector3, geometry_msgs::Point> {
        static geometry_msgs::Point convert(const Vector3 &vector)
        {
            geometry_msgs::Point ret;
            ret.x = vector.x(), ret.y = vector.y(), ret.z = vector.z();
            return ret;
        }
    };

    template<>
    struct converter<geometry_msgs::Point, Vector3> {
        static Vector3 convert(const geometry_msgs::Point &point)
        {
            return { point.x, point.y, point.z };
        }
    };
}

struct Quaternion
    : Decorate<
        Quaternion, VectorValues<4>,
        CRTPDecorator, CustomizableConversion, VectorBase, VectorAccessX, VectorAccessY, VectorAccessZ, VectorAccessW
    > {

    Quaternion() = default;

    Quaternion(const double x, const double y, const double z, const double w) noexcept
        : Decorate { x, y, z, w } { }

    Quaternion(const double scalar, const Vector3 &vector) noexcept
        : Quaternion(vector.x(), vector.y(), vector.z(), scalar) { }

    double scalar() const noexcept { return w(); }
    Vector3 vector() const noexcept { return { x(), y(), z() }; }

    Quaternion conjugate() const noexcept { return Quaternion(scalar(), -vector()); }
    Quaternion inverse() const noexcept { return conjugate() / dot(*this); }

    Quaternion operator*(const Quaternion &rhs) const noexcept
    {
        double lsc = scalar(), rsc = rhs.scalar();
        Vector3 lvec = vector(), rvec = rhs.vector();
        return Quaternion(lsc * rsc - lvec.dot(rvec), lsc * rvec + rsc * lvec + lvec.cross(rvec));
    }

    friend Quaternion operator*(const Quaternion &lhs, const Vector3 &rhs) noexcept { return lhs * Quaternion(0, rhs); }
    friend Quaternion operator*(const Vector3 &lhs, const Quaternion &rhs) noexcept { return Quaternion(0, lhs) * rhs; }
};

namespace detail {
    template<>
    struct converter<Quaternion, geometry_msgs::Quaternion> {
        static geometry_msgs::Quaternion convert(const Quaternion &quaternion)
        {
            geometry_msgs::Quaternion ret;
            ret.x = quaternion.x(), ret.y = quaternion.y(), ret.z = quaternion.z(), ret.w = quaternion.w();
            return ret;
        }
    };

    template<>
    struct converter<geometry_msgs::Quaternion, Quaternion> {
        static Quaternion convert(const geometry_msgs::Quaternion &quaternion)
        {
            return { quaternion.x, quaternion.y, quaternion.z, quaternion.w };
        }
    };
}

class Rotation {
    Quaternion quaternion;

public:
    Rotation(const double angle, const Vector3 &axis = Vector3::UnitZ()) noexcept
        : quaternion(std::cos(angle / 2), axis.normalize() * std::sin(angle / 2)) { }

    Vector3 axis() const noexcept { return quaternion.vector() / std::sin(angle() / 2); }
    double angle() const noexcept { return std::acos(quaternion.scalar()) * 2; }

    Vector3 rotate(const Vector3 &rhs) const noexcept { return (quaternion * rhs * quaternion.inverse()).vector(); }

    operator Quaternion() const noexcept { return quaternion; }
};

template<class ...Types>
struct ColorValues {
    static constexpr size_t dimension = sizeof...(Types);
    std::tuple<Types...> values;

    template<size_t I>
    auto &get() noexcept { return std::get<I>(values); }

    template<size_t I>
    const auto &get() const noexcept { return std::get<I>(values); }
};

namespace detail {
    template<class ColorA, class ColorB>
    struct converter<ColorA, ColorB,
        std::enable_if_t<
            !std::is_same_v<ColorA, ColorB>
            && internal::is_convertible_v<ColorA, std_msgs::ColorRGBA>
            && internal::is_convertible_v<std_msgs::ColorRGBA, ColorB>>> {

        static ColorB convert(const ColorA &color)
        {
            return converter<std_msgs::ColorRGBA, ColorB>::convert(converter<ColorA, std_msgs::ColorRGBA>::convert(color));
        }
    };
}

struct RGBA
    : Decorate<
        RGBA, ColorValues<double, double, double, double>,
        CRTPDecorator, CustomizableConversion
    > {

    RGBA() = default;

    RGBA(const double red, const double green, const double blue, const double alpha = 1.0)
        : Decorate { std::forward_as_tuple(red, green, blue, alpha) } { }

    double &red() noexcept { return get<0>(); }
    double &green() noexcept { return get<1>(); }
    double &blue() noexcept { return get<2>(); }
    double &alpha() noexcept { return get<3>(); }

    const double &red() const noexcept { return get<0>(); }
    const double &green() const noexcept { return get<1>(); }
    const double &blue() const noexcept { return get<2>(); }
    const double &alpha() const noexcept { return get<3>(); }

    RGBA &red(const double red) noexcept { this->red() = red; return *this; }
    RGBA &blue(const double blue) noexcept { this->blue() = blue; return *this; }
    RGBA &green(const double green) noexcept { this->green() = green; return *this; }
    RGBA &alpha(const double alpha) noexcept { this->alpha() = alpha; return *this; }
};

namespace detail {
    template<>
    struct converter<RGBA, std_msgs::ColorRGBA> {
        static std_msgs::ColorRGBA convert(const RGBA &rgba)
        {
            std_msgs::ColorRGBA ret;
            ret.r = rgba.red(), ret.g = rgba.green(), ret.b = rgba.blue(), ret.a = rgba.alpha();
            return ret;
        }
    };

    template<>
    struct converter<std_msgs::ColorRGBA, RGBA> {
        static RGBA convert(const std_msgs::ColorRGBA &color_rgba)
        {
            return { color_rgba.r, color_rgba.g, color_rgba.b, color_rgba.a };
        }
    };
}

struct HSLA
    : Decorate<
        HSLA, ColorValues<double, double, double, double>,
        CRTPDecorator, CustomizableConversion
    > {

    HSLA() = default;

    HSLA(const double hue, const double saturation = 100.0, const double lightness = 50.0, const double alpha = 1.0)
        : Decorate { std::forward_as_tuple(hue, saturation, lightness, alpha) } { }

    double &hue() noexcept { return get<0>(); }
    double &saturation() noexcept { return get<1>(); }
    double &lightness() noexcept { return get<2>(); }
    double &alpha() noexcept { return get<3>(); }

    const double &hue() const noexcept { return get<0>(); }
    const double &saturation() const noexcept { return get<1>(); }
    const double &lightness() const noexcept { return get<2>(); }
    const double &alpha() const noexcept { return get<3>(); }

    HSLA &hue(const double hue) noexcept { this->hue() = hue; return *this; }
    HSLA &saturation(const double saturation) noexcept { this->saturation() = saturation; return *this; }
    HSLA &lightness(const double lightness) noexcept { this->lightness() = lightness; return *this; }
    HSLA &alpha(const double alpha) noexcept { this->alpha() = alpha; return *this; }
};

namespace detail {
    template<>
    struct converter<HSLA, std_msgs::ColorRGBA> {
        static std_msgs::ColorRGBA convert(const HSLA &hsla)
        {
            const double h = hsla.hue() / 60, s = hsla.saturation() / 100, l = hsla.lightness() / 100, a = hsla.alpha();

            const double c = (1 - std::abs(2 * l - 1)) * s;
            const double x = c * (1 - std::abs(std::remainder(h, 2) - 1));

            const double m = l - c / 2;

            const auto f = [&](double v0, double v1, double v2, double v3, double v4, double v5) {
                if (0 <= h && h < 1) return v0;
                if (1 <= h && h < 2) return v1;
                if (2 <= h && h < 3) return v2;
                if (3 <= h && h < 4) return v3;
                if (4 <= h && h < 5) return v4;
                if (5 <= h && h < 6) return v5;
                return 0.0;
            };

            std_msgs::ColorRGBA ret;
            ret.r = m + f(c, x, 0, 0, x, c), ret.g = m + f(x, c, c, x, 0, 0), ret.b = m + f(0, 0, x, c, c, x), ret.a = a;
            return ret;
        }
    };

    template<>
    struct converter<std_msgs::ColorRGBA, HSLA> {
        static HSLA convert(const std_msgs::ColorRGBA &color_rgba)
        {
            const double r = color_rgba.r, g = color_rgba.g, b = color_rgba.b, a = color_rgba.a;

            const double max = std::max({ r, g, b });
            const double min = std::min({ r, g, b });
            const double c = max - min;

            const double h = 60 * [&] {
                if (c != 0) return 0.0;
                if (max == g) return 2 + (b - r) / c;
                if (max == b) return 4 + (r - g) / c;
                return std::remainder(6 + (g - b) / c, 6);
            }();
            const double l = 100 * ((max + min) / 2);
            const double s = 100 * [&] {
                if (l == 0 || l == 1) return 0.0;
                return c / (1 - std::abs(2 * l - 1));
            }();

            return { h, s, l, a };
        }
    };
}

namespace detail {
    template<class T, size_t ...Indices>
    T lerp_impl(const T &a, const T &b, const double t, std::index_sequence<Indices...>)
    {
        T ret;
        ((ret.template get<Indices>() = a.template get<Indices>() * (1 - t) + b.template get<Indices>() * t), ...);
        return ret;
    }
}

template<class T>
T lerp(const T &a, const T &b, const double t) { return detail::lerp_impl(a, b, t, std::make_index_sequence<T::dimension>()); }

template<class T>
auto lerp_func(const T &a, const T &b) { return [=](const double t) { lerp(a, b, t); }; }

class Indices {
    ssize_t _begin, _end, _step;

    class Cursol {
        ssize_t _value, _step;

    public:
        Cursol() noexcept = default;
        Cursol(ssize_t value, ssize_t step) noexcept : _value(value), _step(step) { }

        ssize_t &operator*() noexcept { return _value; }
        bool operator!=(const Cursol &rhs) const noexcept { return _value != rhs._value; }
        Cursol &operator++() noexcept { _value += _step; return *this; }
    };

public:
    Indices(ssize_t begin, ssize_t end, ssize_t step = 1): _begin(begin), _end(end), _step(step) { }
    Indices(ssize_t end): Indices(0, end) { }

    Cursol begin() { return Cursol(_begin, _step); }
    Cursol end() { return Cursol(_end, _step); }
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
        this->message.pose.position = convert<geometry_msgs::Point>(position);
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
        this->message.pose.orientation = convert<Quaternion>(orientation);
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
        this->message.color = convert<std_msgs::ColorRGBA>(color);
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

namespace internal {
    template<class T>
    using size_type = decltype(std::declval<T>().size());

    template<class T>
    constexpr inline bool has_size_v = is_detected_v<size_type, T>;
}

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
        if constexpr (internal::has_size_v<Iterable>) {
            this->message.colors.reserve(iterable.size());
        }

        for (const auto &e : iterable) {
            this->message.colors.push_back(convert<std_msgs::ColorRGBA>(e));
        }
    }
};

template<class Derived, class Base>
struct Points : Base {
    template<class Iterable>
    Derived &points(const Iterable &iterable) noexcept
    {
        if constexpr (internal::has_size_v<Iterable>) {
            this->message.points.reserve(iterable.size());
        }

        for (const auto &e : iterable) {
            this->message.points.push_back(convert<geometry_msgs::Point>(e));
        }
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
        this->message.points[index] = convert<geometry_msgs::Point>(point);
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

template<template<class> class Element>
struct Each {
    template<class Derived, class Base>
    struct Decorator : Base {
        using Element = Element<Derived>;

        template<class Iterable, class Func>
        Derived &each(const Iterable &iterable, const Func &func)
        {
            Element element(this->derived());
            for (const auto &e : iterable) func(e, element);
            return this->derived();
        }
    };
};

template<class Marker>
struct CommonElement {
private:
    Marker &_marker;

public:
    CommonElement(Marker &marker): _marker(marker) { }

    template<typename T>
    CommonElement &add_position(const T &position)
    {
        _marker.message.points.push_back(convert<geometry_msgs::Point>(position));
    }

    CommonElement &add_position(const double x, const double y, const double z)
    {
        geometry_msgs::Point point;
        point.x = x, point.y = y, point.z = z;
        _marker.message.points.push_back(point);
    }

    template<typename T>
    CommonElement &add_color(const T &position)
    {
        _marker.message.colors.push_back(convert<std_msgs::ColorRGBA>(position));
    }

    CommonElement &add_color(const double r, const double g, const double b, const double a = 1.0)
    {
        std_msgs::ColorRGBA color;
        color.r = r, color.g = g, color.b = b, color.a = a;
        _marker.message.colors.push_back(color);
    }
};

template<class T>
struct MessageBase {
    using message_type = T;

protected:
    T message;
};

template<class Derived, class Base>
struct MessageConversion : Base {
    operator const typename Base::message_type &() { return this->message; }
};

struct DeleteAll
    : Decorate<
        DeleteAll,
        MessageBase<visualization_msgs::Marker>,
        ActionType<visualization_msgs::Marker::DELETEALL>::Decorator
    > {

    DeleteAll(const std::string &ns = "")
    {
        this->message.ns = ns;
    }
};

struct Delete
    : Decorate<
        Delete,
        MessageBase<visualization_msgs::Marker>,
        ActionType<visualization_msgs::Marker::DELETE>::Decorator
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
    : Decorate<
        Add<Type, Decorators...>,
        MessageBase<visualization_msgs::Marker>,
        CRTPDecorator,
        ActionType<visualization_msgs::Marker::ADD>::Decorator,
        MarkerType<Type>::template Decorator,
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

using PoseArrowMarker = Add<
    visualization_msgs::Marker::ARROW,
    MessageConversion,
    Position, Orientation, PoseArrowScale, Color>;

using VectorArrowMarker = Add<
    visualization_msgs::Marker::ARROW,
    MessageConversion,
    VectorArrowScale, Color, ArrowPoints>;

using CubeMarker = Add<
    visualization_msgs::Marker::CUBE,
    MessageConversion,
    Position, Orientation, Scale, Color>;

using SphereMarker = Add<
    visualization_msgs::Marker::SPHERE,
    MessageConversion,
    Position, Orientation, Scale, Color>;

using CylinderMarker = Add<
    visualization_msgs::Marker::CYLINDER,
    MessageConversion,
    Position, Orientation, Scale, Color>;

using LineStripMarker = Add<
    visualization_msgs::Marker::LINE_STRIP,
    MessageConversion,
    Each<CommonElement>::Decorator,
    Position, Orientation, LineScale, Color, Points>;

using LineListMarker = Add<
    visualization_msgs::Marker::LINE_LIST,
    MessageConversion,
    Each<CommonElement>::Decorator,
    Position, Orientation, LineScale, Colors, Points>;

using CubeListMarker = Add<
    visualization_msgs::Marker::CUBE_LIST,
    MessageConversion,
    Each<CommonElement>::Decorator,
    Position, Orientation, Scale, Colors, Points>;

using SphereListMarker = Add<
    visualization_msgs::Marker::SPHERE_LIST,
    MessageConversion,
    Each<CommonElement>::Decorator,
    Position, Orientation, Scale, Colors, Points>;

using PointsMarker = Add<
    visualization_msgs::Marker::POINTS,
    MessageConversion,
    Each<CommonElement>::Decorator,
    Position, Orientation, PointScale, Colors, Points>;

using TextViewFacingMarker = Add<
    visualization_msgs::Marker::TEXT_VIEW_FACING,
    MessageConversion,
    Position, TextScale, Color, Text>;

using MeshResourceMarker = Add<
    visualization_msgs::Marker::MESH_RESOURCE,
    MessageConversion,
    Position, Orientation, Scale, Color, MeshResource>;

using TriangleListMarker = Add<
    visualization_msgs::Marker::TRIANGLE_LIST,
    MessageConversion,
    Each<CommonElement>::Decorator,
    Position, Orientation, Scale, Colors, Points>;

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
