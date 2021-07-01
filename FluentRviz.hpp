#pragma once

#include <array>
#include <numeric>
#include <string>
#include <vector>
#include <type_traits>
#include <initializer_list>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

namespace flrv {

template<
    typename Derived,
    typename Base,
    template<typename, typename> typename ...Decorators>
struct Decorate {
    using Type = Base;
};

template<
    typename Derived,
    typename Base,
    template<typename, typename> typename Decorator>
struct Decorate<Derived, Base, Decorator> {
    using Type = Decorator<Derived, Base>;
};

template<
    typename Derived,
    typename Base,
    template<typename, typename> typename Decorator,
    template<typename, typename> typename ...Decorators>
struct Decorate<Derived, Base, Decorator, Decorators...> {
    using Type = typename Decorate<Derived, Decorator<Derived, Base>, Decorators...>::Type;
};

namespace detail {
    template<typename From, typename To>
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
struct CRTPDecorator : Base {
protected:
    Derived &derived() noexcept { return static_cast<Derived &>(*this); }
    const Derived &derived() const noexcept { return static_cast<const Derived &>(*this); }
};

template<typename T>
struct Storage {
    T storage;
};

template<size_t DIM, typename Derived>
struct VectorBase : CRTPDecorator<Derived, Storage<std::array<double, DIM>>> {
private:
    template<typename Op>
    Derived &apply(const Derived &rhs, const Op &op = Op()) noexcept
    {
        for (size_t i = 0; i < DIM; i++) this->storage[i] = op(this->storage[i], rhs.storage[i]);
        return this->derived();
    }

    template<typename Op>
    Derived &apply(const double &rhs, const Op &op = Op()) noexcept
    {
        for (size_t i = 0; i < DIM; i++) this->storage[i] = op(this->storage[i], rhs);
        return this->derived();
    }

public:
    Derived &operator+=(const Derived &rhs) noexcept { return apply(rhs, std::plus<double>()); };
    Derived &operator-=(const Derived &rhs) noexcept { return apply(rhs, std::minus<double>()); };
    Derived &operator*=(const double &rhs) noexcept { return apply(rhs, std::multiplies<double>()); };
    Derived &operator/=(const double &rhs) noexcept { return apply(rhs, std::divides<double>()); };

    Derived operator+(const Derived &rhs) const noexcept { return Derived(*this) += rhs; }
    Derived operator-(const Derived &rhs) const noexcept { return Derived(*this) -= rhs; }
    Derived operator*(const double &rhs) const noexcept { return Derived(*this) *= rhs; }
    Derived operator/(const double &rhs) const noexcept { return Derived(*this) /= rhs; }
    friend Derived operator*(const double &lhs, const Derived &rhs) noexcept { return rhs * lhs; }
    friend Derived operator/(const double &lhs, const Derived &rhs) noexcept { return rhs / lhs; }

    Derived operator+() const noexcept { return *this; }
    Derived operator-() const noexcept { return *this * -1; }

    double &operator[](const size_t i) noexcept { return this->storage[i]; }
    const double &operator[](const size_t i) const noexcept { return this->storage[i]; }

    double dot(const Derived &rhs) const noexcept
    {
        Derived result = *this;
        result.apply(rhs, std::multiplies<double>());
        return std::reduce(result.storage.begin(), result.storage.end());
    }

    double norm() const noexcept { return std::sqrt(dot(*this)); }
    Derived normalize() const noexcept { return *this / norm(); }

    template<typename To>
    operator To() const noexcept { return convert<To>(this->derived()); }
};

struct Vector3 : VectorBase<3, Vector3> {
    Vector3(const double x, const double y, const double z)
        : VectorBase<3, Vector3> { x, y, z }
    { }

    double &x() noexcept { return (*this)[0]; }
    double &y() noexcept { return (*this)[1]; }
    double &z() noexcept { return (*this)[2]; }

    const double &x() const noexcept { return (*this)[0]; }
    const double &y() const noexcept { return (*this)[1]; }
    const double &z() const noexcept { return (*this)[2]; }

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

struct Quaternion : VectorBase<4, Quaternion> {
    Quaternion(const double x, const double y, const double z, const double w)
        : VectorBase<4, Quaternion> { x, y, z, w }
    { }

    double &x() noexcept { return (*this)[0]; }
    double &y() noexcept { return (*this)[1]; }
    double &z() noexcept { return (*this)[2]; }
    double &w() noexcept { return (*this)[3]; }

    const double &x() const noexcept { return (*this)[0]; }
    const double &y() const noexcept { return (*this)[1]; }
    const double &z() const noexcept { return (*this)[2]; }
    const double &w() const noexcept { return (*this)[3]; }

    static Quaternion from_scalar_vector(const double scalar, const Vector3 &vector) noexcept
    {
        return (Quaternion { vector.x(), vector.y(), vector.z(), scalar }).normalize();
    }

    static Quaternion from_angle_axis(const double angle, const Vector3 &axis = Vector3::UnitZ()) noexcept
    {
        return from_scalar_vector(std::cos(angle / 2), axis.normalize() * std::sin(angle / 2));
    }

    Vector3 vector() const noexcept { return { x(), y(), z() }; }
    double scalar() const noexcept { return w(); }

    Vector3 axis() const noexcept { return vector() / std::sin(angle() / 2); }
    double angle() const noexcept { return std::acos(scalar()) * 2; }

    Quaternion conjugation() const noexcept { return from_scalar_vector(scalar(), -vector()); }
    Quaternion inverse() const noexcept { return conjugation() / (this->norm() * this->norm()); }

    Quaternion operator*(const Quaternion &rhs) const noexcept
    {
        double lsc = scalar(), rsc = rhs.scalar();
        Vector3 lvec = vector(), rvec = rhs.vector();
        return from_scalar_vector(lsc * rsc - lvec.dot(rvec), lsc * rvec + rsc * lvec + lvec.cross(rvec));
    }

    friend Quaternion operator*(const Quaternion &lhs, const Vector3 &rhs) noexcept { return lhs * from_scalar_vector(0, rhs); }
    friend Quaternion operator*(const Vector3 &lhs, const Quaternion &rhs) noexcept { return from_scalar_vector(0, lhs) * rhs; }

    Vector3 rotate(const Vector3 &rhs) const noexcept { return ((*this) * rhs * (*this).inverse()).vector(); }
};

namespace detail {
    template<>
    struct converter<Vector3, geometry_msgs::Vector3> {
        static inline geometry_msgs::Vector3 convert(const Vector3 &vector)
        {
            geometry_msgs::Vector3 ret;
            ret.x = vector.x();
            ret.y = vector.y();
            ret.z = vector.z();
            return ret;
        }
    };

    template<>
    struct converter<geometry_msgs::Vector3, Vector3> {
        static inline Vector3 convert(const geometry_msgs::Vector3 &vector)
        { return { vector.x, vector.y, vector.z }; }
    };

    template<>
    struct converter<Vector3, geometry_msgs::Point> {
        static inline geometry_msgs::Point convert(const Vector3 &vector)
        {
            geometry_msgs::Point ret;
            ret.x = vector.x();
            ret.y = vector.y();
            ret.z = vector.z();
            return ret;
        }
    };

    template<>
    struct converter<geometry_msgs::Point, Vector3> {
        static inline Vector3 convert(const geometry_msgs::Point &point)
        { return { point.x, point.y, point.z }; }
    };

    template<>
    struct converter<Quaternion, geometry_msgs::Quaternion> {
        static inline geometry_msgs::Quaternion convert(const Quaternion &quaternion)
        {
            geometry_msgs::Quaternion ret;
            ret.x = quaternion.x();
            ret.y = quaternion.y();
            ret.z = quaternion.z();
            ret.w = quaternion.w();
            return ret;
        }
    };

    template<>
    struct converter<geometry_msgs::Quaternion, Quaternion> {
        static inline Quaternion convert(const geometry_msgs::Quaternion &quaternion)
        { return { quaternion.x, quaternion.y, quaternion.z, quaternion.w }; }
    };
}

template<int32_t TYPE>
struct ActionType {
    template<typename Derived, typename Base>
    struct Decorator : Base {
        Decorator() noexcept
        {
            this->message.action = TYPE;
        }
    };
};

template<int32_t TYPE>
struct MarkerType {
    template<typename Derived, typename Base>
    struct Decorator : Base {
        Decorator() noexcept
        {
            this->message.type = TYPE;
        }
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
    Orientation() noexcept
    {
        orientation(1, 0, 0, 0);
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

template<typename Derived, typename Base>
struct Scale : Base {
    Scale() noexcept
    {
        scale(1, 1, 1);
    }

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
    PoseArrowScale() noexcept
    {
        scale(0.2, 0.2, 1);
    }

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
    VectorArrowScale() noexcept
    {
        scale(0.2, 0.4, 0.4);
    }

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
    PointScale() noexcept
    {
        scale(0.05, 0.05);
    }

    Derived &scale(const double width, const double height) noexcept
    {
        this->marker.scale.x = width;
        this->marker.scale.y = height;
        return this->derived();
    }
};

template<typename Derived, typename Base>
struct LineScale : Base {
    LineScale() noexcept
    {
        scale(0.05);
    }

    Derived &scale(const double width) noexcept
    {
        this->marker.scale.x = width;
        return this->derived();
    }
};

template<typename Derived, typename Base>
struct TextScale : Base {
    TextScale() noexcept
    {
        scale(0.05);
    }

    Derived &scale(const double height) noexcept
    {
        this->marker.scale.z = height;
        return this->derived();
    }
};

template<typename Derived, typename Base>
struct Color : Base {
    Color() noexcept
    {
        color(1, 1, 1, 1);
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

template<typename Derived, typename Base>
struct Colors : Base {
    Colors() noexcept
    {
        color(1, 1, 1, 1);
    }

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
    Text() noexcept
    {
        text("visualization_msgs::Marker");
    }

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
    operator const T &() const noexcept
    {
        return message;
    }
};

struct DeleteAll
    : Decorate<
        DeleteAll,
        MessageBase<visualization_msgs::Marker>,
        ActionType<visualization_msgs::Marker::DELETEALL>::Decorator
    >::Type { };

struct Delete
    : Decorate<
        Delete,
        MessageBase<visualization_msgs::Marker>,
        ActionType<visualization_msgs::Marker::DELETE>::Decorator
    >::Type {

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
    >::Type {

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
        , _frame_id(frame_id)
    { }

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
