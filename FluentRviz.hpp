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

namespace converter {

    template<class From, class To, class Enabler = void>
    struct impl;

    template<class Type>
    struct impl<Type, Type> {
        static const Type &convert(const Type &value) { return value; };
    };

}

namespace util {

    template<class To, class From>
    decltype(auto) convert(const From &from) { return converter::impl<From, To>::convert(from); }

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
        const Cursol begin() const { return Cursol(_begin, _step); }
        Cursol end() { return Cursol(_end, _step); }
        const Cursol end() const { return Cursol(_end, _step); }
    };

    namespace decorator {
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
    }

}

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
    using convert_type = decltype(util::convert<To>(std::declval<From>()));

    template<class From, class To>
    constexpr inline bool is_convertible_v = is_detected_v<convert_type, From, To>;

    template<class T>
    using size_type = decltype(std::declval<T>().size());

    template<class T>
    constexpr inline bool has_size_v = is_detected_v<size_type, T>;
}

namespace param {

    namespace decorator {

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

    }

    template<size_t D>
    struct VectorValues {
        static constexpr size_t dimension = D;
        std::array<double, D> storage;
    };

    struct Vector3
        : util::Decorate<
            Vector3, VectorValues<3>,
            util::decorator::CRTPDecorator, util::decorator::CustomizableConversion,
            decorator::VectorBase, decorator::VectorAccessX, decorator::VectorAccessY, decorator::VectorAccessZ
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

    struct Quaternion
        : util::Decorate<
            Quaternion, VectorValues<4>,
            util::decorator::CRTPDecorator, util::decorator::CustomizableConversion,
            decorator::VectorBase, decorator::VectorAccessX, decorator::VectorAccessY, decorator::VectorAccessZ, decorator::VectorAccessW
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

    struct Color
        : util::Decorate<
            Color, ColorValues<double, double, double, double>,
            util::decorator::CRTPDecorator, util::decorator::CustomizableConversion
        > {

        Color() = default;

        Color(const double red, const double green, const double blue, const double alpha = 1.0)
            : Decorate { std::forward_as_tuple(red, green, blue, alpha) } { }

        double &red() noexcept { return get<0>(); }
        double &green() noexcept { return get<1>(); }
        double &blue() noexcept { return get<2>(); }
        double &alpha() noexcept { return get<3>(); }

        const double &red() const noexcept { return get<0>(); }
        const double &green() const noexcept { return get<1>(); }
        const double &blue() const noexcept { return get<2>(); }
        const double &alpha() const noexcept { return get<3>(); }

        Color &red(const double red) noexcept { this->red() = red; return *this; }
        Color &blue(const double blue) noexcept { this->blue() = blue; return *this; }
        Color &green(const double green) noexcept { this->green() = green; return *this; }
        Color &alpha(const double alpha) noexcept { this->alpha() = alpha; return *this; }

        static Color White()   noexcept { return { 1.00, 1.00, 1.00 }; }
        static Color Silver()  noexcept { return { 0.75, 0.75, 0.75 }; }
        static Color Gray()    noexcept { return { 0.50, 0.50, 0.50 }; }
        static Color Black()   noexcept { return { 0.00, 0.00, 0.00 }; }
        static Color Red()     noexcept { return { 1.00, 0.00, 0.00 }; }
        static Color Maroon()  noexcept { return { 0.50, 0.00, 0.00 }; }
        static Color Yellow()  noexcept { return { 1.00, 1.00, 0.00 }; }
        static Color Olive()   noexcept { return { 0.50, 0.50, 0.00 }; }
        static Color Lime()    noexcept { return { 0.00, 1.00, 0.00 }; }
        static Color Green()   noexcept { return { 0.00, 0.50, 0.00 }; }
        static Color Aqua()    noexcept { return { 0.00, 1.00, 1.00 }; }
        static Color Teal()    noexcept { return { 0.00, 0.50, 0.50 }; }
        static Color Blue()    noexcept { return { 0.00, 0.00, 1.00 }; }
        static Color Navy()    noexcept { return { 0.00, 0.00, 0.50 }; }
        static Color Fuchsia() noexcept { return { 1.00, 0.00, 1.00 }; }
        static Color Purple()  noexcept { return { 0.50, 0.00, 0.50 }; }
    };

    struct RGBA
        : util::Decorate<
            RGBA, ColorValues<double, double, double, double>,
            util::decorator::CRTPDecorator, util::decorator::CustomizableConversion
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

        static RGBA HexRGB(const uint32_t hex, const double alpha = 1.0) noexcept
        {
            return { (hex >> 16) & 0xff, (hex >> 8) & 0xff, hex & 0xff, alpha };
        }
        static RGBA HexRGBA(const uint32_t hex) noexcept
        {
            return { (hex >> 24) & 0xff, (hex >> 16) & 0xff, (hex >> 8) & 0xff, (hex & 0xff) / 255.0 };
        }
    };

    struct HSLA
        : util::Decorate<
            HSLA, ColorValues<double, double, double, double>,
            util::decorator::CRTPDecorator, util::decorator::CustomizableConversion
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

}

namespace converter {

    template<>
    struct impl<param::Vector3, geometry_msgs::Vector3> {
        static geometry_msgs::Vector3 convert(const param::Vector3 &vector)
        {
            geometry_msgs::Vector3 ret;
            ret.x = vector.x(), ret.y = vector.y(), ret.z = vector.z();
            return ret;
        }
    };

    template<>
    struct impl<geometry_msgs::Vector3, param::Vector3> {
        static param::Vector3 convert(const geometry_msgs::Vector3 &vector)
        {
            return { vector.x, vector.y, vector.z };
        }
    };

    template<>
    struct impl<param::Vector3, geometry_msgs::Point> {
        static geometry_msgs::Point convert(const param::Vector3 &vector)
        {
            geometry_msgs::Point ret;
            ret.x = vector.x(), ret.y = vector.y(), ret.z = vector.z();
            return ret;
        }
    };

    template<>
    struct impl<geometry_msgs::Point, param::Vector3> {
        static param::Vector3 convert(const geometry_msgs::Point &point)
        {
            return { point.x, point.y, point.z };
        }
    };

    template<>
    struct impl<param::Quaternion, geometry_msgs::Quaternion> {
        static geometry_msgs::Quaternion convert(const param::Quaternion &quaternion)
        {
            geometry_msgs::Quaternion ret;
            ret.x = quaternion.x(), ret.y = quaternion.y(), ret.z = quaternion.z(), ret.w = quaternion.w();
            return ret;
        }
    };

    template<>
    struct impl<geometry_msgs::Quaternion, param::Quaternion> {
        static param::Quaternion convert(const geometry_msgs::Quaternion &quaternion)
        {
            return { quaternion.x, quaternion.y, quaternion.z, quaternion.w };
        }
    };

    template<class ColorA, class ColorB>
    struct impl<ColorA, ColorB,
        std::enable_if_t<
            !std::is_same_v<ColorA, ColorB>
            && internal::is_convertible_v<ColorA, std_msgs::ColorRGBA>
            && internal::is_convertible_v<std_msgs::ColorRGBA, ColorB>>> {

        static ColorB convert(const ColorA &color)
        {
            return impl<std_msgs::ColorRGBA, ColorB>::convert(impl<ColorA, std_msgs::ColorRGBA>::convert(color));
        }
    };

    template<>
    struct impl<param::Color, std_msgs::ColorRGBA> {
        static std_msgs::ColorRGBA convert(const param::Color &color)
        {
            std_msgs::ColorRGBA ret;
            ret.r = color.red(), ret.g = color.green(), ret.b = color.blue(), ret.a = color.alpha();
            return ret;
        }
    };

    template<>
    struct impl<std_msgs::ColorRGBA, param::Color> {
        static param::Color convert(const std_msgs::ColorRGBA &color_rgba)
        {
            return { color_rgba.r, color_rgba.g, color_rgba.b, color_rgba.a };
        }
    };

    template<>
    struct impl<param::RGBA, std_msgs::ColorRGBA> {
        static std_msgs::ColorRGBA convert(const param::RGBA &rgba)
        {
            std_msgs::ColorRGBA ret;
            ret.r = rgba.red() / 255, ret.g = rgba.green() / 255, ret.b = rgba.blue() / 255, ret.a = rgba.alpha();
            return ret;
        }
    };

    template<>
    struct impl<std_msgs::ColorRGBA, param::RGBA> {
        static param::RGBA convert(const std_msgs::ColorRGBA &color_rgba)
        {
            return {
                static_cast<uint32_t>(color_rgba.r * 255),
                static_cast<uint32_t>(color_rgba.g * 255),
                static_cast<uint32_t>(color_rgba.b * 255),
                color_rgba.a
            };
        }
    };

    template<>
    struct impl<param::HSLA, std_msgs::ColorRGBA> {
        static std_msgs::ColorRGBA convert(const param::HSLA &hsla)
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
    struct impl<std_msgs::ColorRGBA, param::HSLA> {
        static param::HSLA convert(const std_msgs::ColorRGBA &color_rgba)
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

namespace marker {

    namespace decorator {

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
                if constexpr (internal::has_size_v<Iterable>) {
                    this->message.colors.reserve(iterable.size());
                }

                for (const auto &e : iterable) {
                    this->message.colors.push_back(util::convert<std_msgs::ColorRGBA>(e));
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
                    this->message.points.push_back(util::convert<geometry_msgs::Point>(e));
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

        template<class Derived, class Base>
        struct Each : Base {
            struct Manipulator;

            template<class Iterable, class Func>
            Derived &each(const Iterable &iterable, const Func &func)
            {
                Manipulator element(this->message);
                for (const auto &e : iterable) func(e, element);
                return this->derived();
            }
        };

        template<class Derived, class Base>
        struct Each<Derived, Base>::Manipulator {
        private:
            visualization_msgs::Marker &_marker;

        public:
            Manipulator(visualization_msgs::Marker &marker): _marker(marker) { }

            template<typename T>
            Manipulator &add_position(const T &position)
            {
                _marker.points.push_back(util::convert<geometry_msgs::Point>(position));
                return *this;
            }

            Manipulator &add_position(const double x, const double y, const double z)
            {
                geometry_msgs::Point point;
                point.x = x, point.y = y, point.z = z;
                _marker.points.push_back(point);
                return *this;
            }

            template<typename T>
            Manipulator &add_color(const T &position)
            {
                _marker.colors.push_back(util::convert<std_msgs::ColorRGBA>(position));
                return *this;
            }

            Manipulator &add_color(const double r, const double g, const double b, const double a = 1.0)
            {
                std_msgs::ColorRGBA color;
                color.r = r, color.g = g, color.b = b, color.a = a;
                _marker.colors.push_back(color);
                return *this;
            }
        };

        template<class Derived, class Base>
        struct MessageConversion : Base {
            operator const typename Base::message_type &() { return this->message; }
        };

    }

    template<class T>
    struct MessageBase {
        using message_type = T;

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
            util::decorator::CRTPDecorator,
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
        decorator::MessageConversion,
        decorator::Position, decorator::Orientation, decorator::PoseArrowScale, decorator::Color>;

    using VectorArrow = Add<
        visualization_msgs::Marker::ARROW,
        decorator::MessageConversion,
        decorator::VectorArrowScale, decorator::Color, decorator::ArrowPoints>;

    using Cube = Add<
        visualization_msgs::Marker::CUBE,
        decorator::MessageConversion,
        decorator::Position, decorator::Orientation, decorator::Scale, decorator::Color>;

    using Sphere = Add<
        visualization_msgs::Marker::SPHERE,
        decorator::MessageConversion,
        decorator::Position, decorator::Orientation, decorator::Scale, decorator::Color>;

    using Cylinder = Add<
        visualization_msgs::Marker::CYLINDER,
        decorator::MessageConversion,
        decorator::Position, decorator::Orientation, decorator::Scale, decorator::Color>;

    using LineStrip = Add<
        visualization_msgs::Marker::LINE_STRIP,
        decorator::MessageConversion, decorator::Each,
        decorator::Position, decorator::Orientation, decorator::LineScale, decorator::Color, decorator::Points>;

    using LineList = Add<
        visualization_msgs::Marker::LINE_LIST,
        decorator::MessageConversion, decorator::Each,
        decorator::Position, decorator::Orientation, decorator::LineScale, decorator::Colors, decorator::Points>;

    using CubeList = Add<
        visualization_msgs::Marker::CUBE_LIST,
        decorator::MessageConversion, decorator::Each,
        decorator::Position, decorator::Orientation, decorator::Scale, decorator::Colors, decorator::Points>;

    using SphereList = Add<
        visualization_msgs::Marker::SPHERE_LIST,
        decorator::MessageConversion, decorator::Each,
        decorator::Position, decorator::Orientation, decorator::Scale, decorator::Colors, decorator::Points>;

    using Points = Add<
        visualization_msgs::Marker::POINTS,
        decorator::MessageConversion, decorator::Each,
        decorator::Position, decorator::Orientation, decorator::PointScale, decorator::Colors, decorator::Points>;

    using TextViewFacing = Add<
        visualization_msgs::Marker::TEXT_VIEW_FACING,
        decorator::MessageConversion,
        decorator::Position, decorator::TextScale, decorator::Color, decorator::Text>;

    using MeshResource = Add<
        visualization_msgs::Marker::MESH_RESOURCE,
        decorator::MessageConversion,
        decorator::Position, decorator::Orientation, decorator::Scale, decorator::Color, decorator::MeshResource>;

    using TriangleList = Add<
        visualization_msgs::Marker::TRIANGLE_LIST,
        decorator::MessageConversion, decorator::Each,
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
