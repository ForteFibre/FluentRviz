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

template<
    typename Derived,
    typename Base,
    template<typename, typename> typename ...Decorators>
struct Decorate : Base { };

template<
    typename Derived,
    typename Base,
    template<typename, typename> typename Decorator>
struct Decorate<Derived, Base, Decorator>
    : Decorator<Derived, Base> { };

template<
    typename Derived,
    typename Base,
    template<typename, typename> typename Decorator,
    template<typename, typename> typename ...Decorators>
struct Decorate<Derived, Base, Decorator, Decorators...>
    : Decorate<Derived, Decorator<Derived, Base>, Decorators...> { };

namespace detail {
    template<typename From, typename To, typename Enabler = void>
    struct converter {
        static inline To convert(const From &from) = delete;
    };

    template<typename Type>
    struct converter<Type, Type> {
        static inline Type convert(const Type &value) { return value; };
    };
}

template<typename To, typename From>
To convert(const From &from) { return detail::converter<From, To>::convert(from); }

template<typename Derived, typename Base>
struct Conversion : Base {
    template<typename To>
    operator To() { return convert<To>(this->derived()); }
};

template<typename Derived, typename Base>
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

template<typename Derived, typename Base>
struct VectorBase : Base {
    double &operator[](ssize_t i) noexcept { return this->storage[i]; }
    const double &operator[](ssize_t i) const noexcept { return this->storage[i]; }

private:
    template<typename Op>
    Derived &apply(const Derived &rhs, const Op &op = Op()) noexcept
    {
        for (size_t i = 0; i < Base::dimension; i++) (*this)[i] = op((*this)[i], rhs[i]);
        return this->derived();
    }

    template<typename Op>
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

template<typename Derived, typename Base>
struct VectorAccessX : Base {
    double &x() noexcept { return (*this)[0]; }
    const double &x() const noexcept { return (*this)[0]; }
    Derived &x(double value) noexcept { x() = value; return this->derived(); }
};

template<typename Derived, typename Base>
struct VectorAccessY : Base {
    double &y() noexcept { return (*this)[1]; }
    const double &y() const noexcept { return (*this)[1]; }
    Derived &y(double value) noexcept { y() = value; return this->derived(); }
};

template<typename Derived, typename Base>
struct VectorAccessZ : Base {
    double &z() noexcept { return (*this)[2]; }
    const double &z() const noexcept { return (*this)[2]; }
    Derived &z(double value) noexcept { z() = value; return this->derived(); }
};

template<typename Derived, typename Base>
struct VectorAccessW : Base {
    double &w() noexcept { return (*this)[3]; }
    const double &w() const noexcept { return (*this)[3]; }
    Derived &w(double value) noexcept { w() = value; return this->derived(); }
};

struct Vector3
    : Decorate<
        Vector3, VectorValues<3>,
        CRTPDecorator, Conversion, VectorBase, VectorAccessX, VectorAccessY, VectorAccessZ
    > {

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
        static inline geometry_msgs::Vector3 convert(const Vector3 &vector)
        {
            geometry_msgs::Vector3 ret;
            ret.x = vector.x(), ret.y = vector.y(), ret.z = vector.z();
            return ret;
        }
    };

    template<>
    struct converter<geometry_msgs::Vector3, Vector3> {
        static inline Vector3 convert(const geometry_msgs::Vector3 &vector)
        {
            return { vector.x, vector.y, vector.z };
        }
    };

    template<>
    struct converter<Vector3, geometry_msgs::Point> {
        static inline geometry_msgs::Point convert(const Vector3 &vector)
        {
            geometry_msgs::Point ret;
            ret.x = vector.x(), ret.y = vector.y(), ret.z = vector.z();
            return ret;
        }
    };

    template<>
    struct converter<geometry_msgs::Point, Vector3> {
        static inline Vector3 convert(const geometry_msgs::Point &point)
        {
            return { point.x, point.y, point.z };
        }
    };
}

struct Quaternion
    : Decorate<
        Quaternion, VectorValues<4>,
        CRTPDecorator, Conversion, VectorBase, VectorAccessX, VectorAccessY, VectorAccessZ, VectorAccessW
    > {

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
        static inline geometry_msgs::Quaternion convert(const Quaternion &quaternion)
        {
            geometry_msgs::Quaternion ret;
            ret.x = quaternion.x(), ret.y = quaternion.y(), ret.z = quaternion.z(), ret.w = quaternion.w();
            return ret;
        }
    };

    template<>
    struct converter<geometry_msgs::Quaternion, Quaternion> {
        static inline Quaternion convert(const geometry_msgs::Quaternion &quaternion)
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

template<typename ...Types>
struct ColorValues {
    static constexpr size_t dimension = sizeof...(Types);
    std::tuple<Types...> values;

    template<size_t I>
    auto &get() { return std::get<I>(values); }
};

namespace detail {
    template<typename ColorA, typename ColorB>
    struct converter<ColorA, ColorB,
        std::void_t<converter<ColorA, std_msgs::ColorRGBA>, converter<std_msgs::ColorRGBA, ColorB>>> {

        static ColorB convert(const ColorA &color)
        {
            return converter<std_msgs::ColorRGBA, ColorB>::convert(converter<ColorA, std_msgs::ColorRGBA>::convert(color));
        }
    };
}

struct RGBA
    : Decorate<
        RGBA, ColorValues<double, double, double, double>,
        CRTPDecorator, Conversion
    > {

    RGBA(const double red, const double green, const double blue, const double alpha = 1.0)
        : Decorate { std::forward_as_tuple(red, green, blue, alpha) } { }

    double &red() noexcept { return std::get<0>(values); }
    double &green() noexcept { return std::get<1>(values); }
    double &blue() noexcept { return std::get<2>(values); }
    double &alpha() noexcept { return std::get<3>(values); }

    const double &red() const noexcept { return std::get<0>(values); }
    const double &green() const noexcept { return std::get<1>(values); }
    const double &blue() const noexcept { return std::get<2>(values); }
    const double &alpha() const noexcept { return std::get<3>(values); }

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
            ret.r = rgba.red();
            ret.g = rgba.green();
            ret.b = rgba.blue();
            ret.a = rgba.alpha();
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
        CRTPDecorator, Conversion
    > {

public:
    HSLA(const double hue, const double saturation, const double lightness, const double alpha = 1.0)
        : Decorate { std::forward_as_tuple(hue, saturation, lightness, alpha) } { }

    double &hue() noexcept { return std::get<0>(values); }
    double &saturation() noexcept { return std::get<1>(values); }
    double &lightness() noexcept { return std::get<2>(values); }
    double &alpha() noexcept { return std::get<3>(values); }

    const double &hue() const noexcept { return std::get<0>(values); }
    const double &saturation() const noexcept { return std::get<1>(values); }
    const double &lightness() const noexcept { return std::get<2>(values); }
    const double &alpha() const noexcept { return std::get<3>(values); }

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
            const double h = hsla.hue() / 60, s = hsla.saturation(), l = hsla.lightness(), a = hsla.alpha();

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
            };

            std_msgs::ColorRGBA ret;
            ret.r = m + f(c, x, 0, 0, x, c);
            ret.g = m + f(x, c, c, x, 0, 0);
            ret.b = m + f(0, 0, x, c, c, x);
            ret.a = a;
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

            const double h = std::remainder([&] {
                if (c == 0) return 0.0;
                if (max == r) return 60 * (0 + (g - b) / c);
                if (max == g) return 60 * (2 + (b - r) / c);
                if (max == b) return 60 * (4 + (r - g) / c);
            }() + 360, 360);
            const double l = (max + min) / 2;
            const double s = (l == 0 || l == 1) ? 0 : c / (1 - std::abs(2 * l - 1));

            return { h, s, l, a };
        }
    };
}

namespace detail {
    template<typename T, size_t ...Indices>
    T lerp_impl(const T &a, const T &b, const double t, std::index_sequence<Indices...>)
    {
        T ret;
        ((ret.template get<Indices>() = a.template get<Indices>() * (1 - t) + b.template get<Indices>() * t), ...);
        return ret;
    }
}

template<typename T>
T lerp(const T &a, const T &b, const double t) { return detail::lerp_impl(a, b, t, std::make_index_sequence<T::dimension>()); }

template<typename T>
auto lerp_func(const T &a, const T &b) { return [=](const double t) { lerp(a, b, t); }; }

template<int32_t TYPE>
struct ActionType {
    template<typename Derived, typename Base>
    struct Decorator : Base {
        Decorator() noexcept { this->message.action = TYPE; }
    };
};

template<int32_t TYPE>
struct MarkerType {
    template<typename Derived, typename Base>
    struct Decorator : Base {
        Decorator() noexcept { this->message.type = TYPE; }
    };
};

template<typename Derived, typename Base>
struct Position : Base {
    Derived &position(const double x, const double y, const double z) noexcept
    {
        this->message.pose.position.x = x;
        this->message.pose.position.y = y;
        this->message.pose.position.z = z;
        return this->derived();
    }
};

template<typename Derived, typename Base>
struct Orientation : Base {
    Orientation() noexcept { orientation(1, 0, 0, 0); }

    Derived &orientation(const double w, const double x, const double y, const double z) noexcept
    {
        this->message.pose.orientation.w = w;
        this->message.pose.orientation.x = x;
        this->message.pose.orientation.y = y;
        this->message.pose.orientation.z = z;
        return this->derived();
    }
};

template<typename Derived, typename Base>
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

template<typename Derived, typename Base>
struct PoseArrowScale : Base {
    PoseArrowScale() noexcept { scale(0.2, 0.2, 1); }

    Derived &scale(const double length, const double width, const double height) noexcept
    {
        this->marker.scale.x = length;
        this->marker.scale.y = width;
        this->marker.scale.z = height;
        return this->derived();
    }
};

template<typename Derived, typename Base>
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

template<typename Derived, typename Base>
struct PointScale : Base {
    PointScale() noexcept { scale(0.05, 0.05); }

    Derived &scale(const double width, const double height) noexcept
    {
        this->marker.scale.x = width;
        this->marker.scale.y = height;
        return this->derived();
    }
};

template<typename Derived, typename Base>
struct LineScale : Base {
    LineScale() noexcept { scale(0.05); }

    Derived &scale(const double width) noexcept
    {
        this->marker.scale.x = width;
        return this->derived();
    }
};

template<typename Derived, typename Base>
struct TextScale : Base {
    TextScale() noexcept { scale(0.05); }

    Derived &scale(const double height) noexcept
    {
        this->marker.scale.z = height;
        return this->derived();
    }
};

template<typename Derived, typename Base>
struct Color : Base {
    Color() noexcept { color(1, 1, 1, 1); }

    Derived &color(const double r, const double g, const double b, const double a = 1) noexcept
    {
        this->message.color.r = r;
        this->message.color.g = g;
        this->message.color.b = b;
        this->message.color.a = a;
        return this->derived();
    }
};

template<typename Derived, typename Base>
struct Colors : Base {
    Colors() noexcept { color(1, 1, 1, 1); }

    Derived &color(const double r, const double g, const double b, const double a = 1) noexcept
    {
        this->message.color.r = r;
        this->message.color.g = g;
        this->message.color.b = b;
        this->message.color.a = a;
        this->message.colors.clear();
        return this->derived();
    }

    Derived &color(const std::vector<std_msgs::ColorRGBA> &colors) noexcept
    {
        this->message.colors = colors;
        return this->derived();
    }
};

template<typename Derived, typename Base>
struct Points : Base {
    Derived &points(const std::vector<geometry_msgs::Point> &points) noexcept
    {
        this->message.points = points;
        return this->derived();
    }
};

template<typename Derived, typename Base>
struct ArrowPoints : Base {
    ArrowPoints() noexcept
    {
        this->message.points.resize(2);
        end(1, 0, 0);
    }

    Derived &start(const double x, const double y, const double z) noexcept
    {
        return set(0, x, y, z);
    }

    Derived &end(const double x, const double y, const double z) noexcept
    {
        return set(1, x, y, z);
    }

private:
    Derived &set(size_t index, const double x, const double y, const double z) noexcept
    {
        this->message.points[index].x = x;
        this->message.points[index].y = y;
        this->message.points[index].z = z;
        return this->derived();
    }
};

template<typename Derived, typename Base>
struct Text : Base {
    Text() noexcept { text("visualization_msgs::Marker"); }

    Derived &text(const std::string &text) noexcept
    {
        this->message.text = text;
        return this->derived();
    }
};

template<typename Derived, typename Base>
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

template<typename T>
struct MessageBase {
protected:
    T message;

public:
    operator const T &() const noexcept { return message; }
};

struct DeleteAll
    : Decorate<
        DeleteAll,
        MessageBase<visualization_msgs::Marker>,
        ActionType<visualization_msgs::Marker::DELETEALL>::Decorator
    > { };

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
    int32_t MARKER_TYPE,
    template<typename, typename> typename ...Decorators>
struct Add
    : Decorate<
        Add<MARKER_TYPE, Decorators...>,
        MessageBase<visualization_msgs::Marker>,
        CRTPDecorator,
        ActionType<visualization_msgs::Marker::ADD>::Decorator,
        MarkerType<MARKER_TYPE>::template Decorator,
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
    Position, Orientation, PoseArrowScale, Color>;

using VectorArrowMarker = Add<
    visualization_msgs::Marker::ARROW,
    VectorArrowScale, Color, ArrowPoints>;

using CubeMarker = Add<
    visualization_msgs::Marker::CUBE,
    Position, Orientation, Scale, Color>;

using SphereMarker = Add<
    visualization_msgs::Marker::SPHERE,
    Position, Orientation, Scale, Color>;

using CylinderMarker = Add<
    visualization_msgs::Marker::CYLINDER,
    Position, Orientation, Scale, Color>;

using LineStripMarker = Add<
    visualization_msgs::Marker::LINE_STRIP,
    Position, Orientation, LineScale, Color, Points>;

using LineListMarker = Add<
    visualization_msgs::Marker::LINE_LIST,
    Position, Orientation, LineScale, Colors, Points>;

using CubeListMarker = Add<
    visualization_msgs::Marker::CUBE_LIST,
    Position, Orientation, Scale, Colors, Points>;

using SphereListMarker = Add<
    visualization_msgs::Marker::SPHERE_LIST,
    Position, Orientation, Scale, Colors, Points>;

using PointsMarker = Add<
    visualization_msgs::Marker::POINTS,
    Position, Orientation, PointScale, Colors, Points>;

using TextViewFacingMarker = Add<
    visualization_msgs::Marker::TEXT_VIEW_FACING,
    Position, TextScale, Color, Text>;

using MeshResourceMarker = Add<
    visualization_msgs::Marker::MESH_RESOURCE,
    Position, Orientation, Scale, Color, MeshResource>;

using TriangleListMarker = Add<
    visualization_msgs::Marker::TRIANGLE_LIST,
    Position, Orientation, Scale, Colors, Points>;

class Rviz {
    ros::NodeHandle _node_handler;
    ros::Publisher _publisher;

    std::string _frame_id;

public:
    Rviz(const std::string &frame_id = "map", const std::string &topic = "visualization_marker")
        : _publisher(_node_handler.advertise<visualization_msgs::Marker>(topic, 1))
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
