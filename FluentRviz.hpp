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

    template<class Type>
    struct converter<Type, Type> {
        static constexpr const Type &convert(const Type &value) { return value; };
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

    template<size_t D, class Derived>
    struct VectorBase {
        std::array<double, D> storage;

        constexpr Derived &derived() noexcept { return static_cast<Derived &>(*this); }
        constexpr const Derived &derived() const noexcept { return static_cast<const Derived &>(*this); }

        constexpr double &operator[](const ssize_t i) noexcept { return storage[i]; }
        constexpr const double &operator[](const ssize_t i) const noexcept { return storage[i]; }

        template<class Op, class T>
        constexpr Derived &apply(const T &rhs, const Op &op = Op()) noexcept
        {
            for (size_t i = 0; i < D; i++) {
                if constexpr (std::is_same_v<T, Derived>) (*this)[i] = op((*this)[i], rhs[i]);
                else (*this)[i] = op((*this)[i], rhs);
            }
            return this->derived();
        }

        constexpr Derived &operator+=(const Derived &rhs) noexcept { return apply(rhs, std::plus<double>()); };
        constexpr Derived &operator-=(const Derived &rhs) noexcept { return apply(rhs, std::minus<double>()); };
        constexpr Derived &operator*=(const double &rhs) noexcept { return apply(rhs, std::multiplies<double>()); };
        constexpr Derived &operator/=(const double &rhs) noexcept { return apply(rhs, std::divides<double>()); };

        friend constexpr Derived operator+(const Derived &lhs, const Derived &rhs) noexcept { return Derived(lhs) += rhs; }
        friend constexpr Derived operator-(const Derived &lhs, const Derived &rhs) noexcept { return Derived(lhs) -= rhs; }
        friend constexpr Derived operator*(const Derived &lhs, const double &rhs) noexcept { return Derived(lhs) *= rhs; }
        friend constexpr Derived operator*(const double &lhs, const Derived &rhs) noexcept { return Derived(rhs) *= lhs; }
        friend constexpr Derived operator/(const Derived &lhs, const double &rhs) noexcept { return Derived(lhs) /= rhs; }

        friend constexpr Derived operator+(const Derived &s) noexcept { return s; }
        friend constexpr Derived operator-(const Derived &s) noexcept { return s * -1; }

        constexpr double dot(const Derived &rhs) const noexcept
        { return std::inner_product(storage.begin(), storage.end(), rhs.storage.begin(), 0.0); }

        constexpr double norm() const noexcept
        { return std::sqrt(dot(this->derived())); }
    };

    template<size_t D>
    struct Vector : public VectorBase<D, Vector<D>> { };

    template<>
    struct Vector<3> : public VectorBase<3, Vector<3>> {
        constexpr Vector<3> cross(const Vector<3> &rhs) const noexcept
        {
            auto [ lx, ly, lz ] = this->storage;
            auto [ rx, ry, rz ] = rhs.storage;
            return { ly * rz - lz * ry, lz * rx - lx * rz, lx * ry - ly * rx };
        }

        static constexpr Vector<3> UnitX() noexcept { return { 1, 0, 0 }; }
        static constexpr Vector<3> UnitY() noexcept { return { 0, 1, 0 }; }
        static constexpr Vector<3> UnitZ() noexcept { return { 0, 0, 1 }; }
    };

    using Vector3 = Vector<3>;

    struct Quaternion : public VectorBase<4, Quaternion> {
        constexpr Quaternion(const double w, const double x, const double y, const double z) noexcept
            : VectorBase { w, x, y, z } { }

        constexpr Quaternion(const double scalar, const Vector3 &vector) noexcept
            : VectorBase { scalar, vector[0], vector[1], vector[2] } { }

        constexpr double scalar() const noexcept { return (*this)[0]; }
        constexpr Vector3 vector() const noexcept { return { (*this)[1], (*this)[2], (*this)[3] }; }
        constexpr Quaternion conjugate() const noexcept { return { scalar(), -vector() }; }
        constexpr Quaternion inverse() const noexcept { return conjugate() / norm(); }

        constexpr Quaternion operator*(const Quaternion &rhs) const noexcept
        {
            double lsc = scalar(), rsc = rhs.scalar();
            Vector3 lvec = vector(), rvec = rhs.vector();
            return { lsc * rsc - lvec.dot(rvec), lsc * rvec + rsc * lvec + lvec.cross(rvec) };
        }

        constexpr Vector3 operator*(const Vector3 &rhs) const noexcept
        { return ((*this) * Quaternion(0, rhs) * inverse()).vector(); }

        static constexpr Quaternion AngleAxis(const double angle, const Vector3 &axis = Vector3::UnitZ()) noexcept
        { return Quaternion(std::cos(angle / 2), axis * std::sin(angle / 2)); }
    };

    struct Color {
        float r, g, b, a;

        constexpr Color(const float red, const float green, const float blue, const float alpha = 1.0f)
            : r(red), g(green), b(blue), a(alpha)
        { }

        static constexpr Color White(const float alpha = 1.0f)   noexcept { return { 1.00, 1.00, 1.00, alpha }; }
        static constexpr Color Silver(const float alpha = 1.0f)  noexcept { return { 0.75, 0.75, 0.75, alpha }; }
        static constexpr Color Gray(const float alpha = 1.0f)    noexcept { return { 0.50, 0.50, 0.50, alpha }; }
        static constexpr Color Black(const float alpha = 1.0f)   noexcept { return { 0.00, 0.00, 0.00, alpha }; }
        static constexpr Color Red(const float alpha = 1.0f)     noexcept { return { 1.00, 0.00, 0.00, alpha }; }
        static constexpr Color Maroon(const float alpha = 1.0f)  noexcept { return { 0.50, 0.00, 0.00, alpha }; }
        static constexpr Color Yellow(const float alpha = 1.0f)  noexcept { return { 1.00, 1.00, 0.00, alpha }; }
        static constexpr Color Olive(const float alpha = 1.0f)   noexcept { return { 0.50, 0.50, 0.00, alpha }; }
        static constexpr Color Lime(const float alpha = 1.0f)    noexcept { return { 0.00, 1.00, 0.00, alpha }; }
        static constexpr Color Green(const float alpha = 1.0f)   noexcept { return { 0.00, 0.50, 0.00, alpha }; }
        static constexpr Color Aqua(const float alpha = 1.0f)    noexcept { return { 0.00, 1.00, 1.00, alpha }; }
        static constexpr Color Teal(const float alpha = 1.0f)    noexcept { return { 0.00, 0.50, 0.50, alpha }; }
        static constexpr Color Blue(const float alpha = 1.0f)    noexcept { return { 0.00, 0.00, 1.00, alpha }; }
        static constexpr Color Navy(const float alpha = 1.0f)    noexcept { return { 0.00, 0.00, 0.50, alpha }; }
        static constexpr Color Fuchsia(const float alpha = 1.0f) noexcept { return { 1.00, 0.00, 1.00, alpha }; }
        static constexpr Color Purple(const float alpha = 1.0f)  noexcept { return { 0.50, 0.00, 0.50, alpha }; }
    };

}

namespace traits {
    template<>
    struct converter<geometry_msgs::Vector3, param::Vector3> {
        static constexpr param::Vector3 convert(const geometry_msgs::Vector3 &vec)
        { return { vec.x, vec.y, vec.z }; }
    };

    template<>
    struct converter<param::Vector3, geometry_msgs::Vector3> {
        static geometry_msgs::Vector3 convert(const param::Vector3 &vec)
        {
            geometry_msgs::Vector3 ret;
            ret.x = vec[0], ret.y = vec[1], ret.z = vec[2];
            return ret;
        }
    };

    template<>
    struct converter<geometry_msgs::Point, param::Vector3> {
        static constexpr param::Vector3 convert(const geometry_msgs::Point &vec)
        { return { vec.x, vec.y, vec.z }; }
    };

    template<>
    struct converter<param::Vector3, geometry_msgs::Point> {
        static geometry_msgs::Point convert(const param::Vector3 &vec)
        {
            geometry_msgs::Point ret;
            ret.x = vec[0], ret.y = vec[1], ret.z = vec[2];
            return ret;
        }
    };

    template<>
    struct converter<geometry_msgs::Quaternion, param::Quaternion> {
        static constexpr param::Quaternion convert(const geometry_msgs::Quaternion &quat)
        { return { quat.w, quat.x, quat.y, quat.z }; }
    };

    template<>
    struct converter<param::Quaternion, geometry_msgs::Quaternion> {
        static geometry_msgs::Quaternion convert(const param::Quaternion &quat)
        {
            geometry_msgs::Quaternion ret;
            ret.w = quat[0], ret.x = quat[1], ret.y = quat[2], ret.z = quat[3];
            return ret;
        }
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

}

namespace marker {

    namespace feature {
        template<class Derived, class Base>
        struct CRTP : Base {
        protected:
            Derived &derived() noexcept { return static_cast<Derived &>(*this); }
            const Derived &derived() const noexcept { return static_cast<const Derived &>(*this); }
        };

        template<int32_t Type>
        struct ActionType {
            template<class Derived, class Base>
            struct Feature : Base {
                Feature() noexcept { this->message.action = Type; }
            };
        };

        template<int32_t Type>
        struct MarkerType {
            template<class Derived, class Base>
            struct Feature : Base {
                Feature() noexcept { this->message.type = Type; }
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
        operator const auto &() { return this->message; }

    protected:
        visualization_msgs::Marker message;
    };

    struct DeleteAll
        : util::chained<
            DeleteAll, MarkerWrapper,
            feature::ActionType<visualization_msgs::Marker::DELETEALL>::Feature
        > {

        DeleteAll(const std::string &ns = "")
        {
            this->message.ns = ns;
        }
    };

    struct Delete
        : util::chained<
            Delete, MarkerWrapper,
            feature::ActionType<visualization_msgs::Marker::DELETE>::Feature
        > {

        Delete(const int32_t id, const std::string &ns = "")
        {
            this->message.id = id;
            this->message.ns = ns;
        }
    };

    template<
        int32_t Type,
        template<class, class> class ...Features>
    struct Add
        : util::chained<
            Add<Type, Features...>, MarkerWrapper,
            feature::CRTP,
            feature::ActionType<visualization_msgs::Marker::ADD>::Feature,
            feature::MarkerType<Type>::template Feature,
            Features...
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
        feature::Position, feature::Orientation, feature::PoseArrowScale, feature::Color>;

    using LineArrow = Add<
        visualization_msgs::Marker::ARROW,
        feature::VectorArrowScale, feature::Color, feature::LinePoints>;

    using Cube = Add<
        visualization_msgs::Marker::CUBE,
        feature::Position, feature::Orientation, feature::Scale, feature::Color>;

    using Sphere = Add<
        visualization_msgs::Marker::SPHERE,
        feature::Position, feature::Orientation, feature::Scale, feature::Color>;

    using Cylinder = Add<
        visualization_msgs::Marker::CYLINDER,
        feature::Position, feature::Orientation, feature::Scale, feature::Color>;

    using LineStrip = Add<
        visualization_msgs::Marker::LINE_STRIP,
        feature::Position, feature::Orientation, feature::LineScale, feature::Color, feature::Points>;

    using LineList = Add<
        visualization_msgs::Marker::LINE_LIST,
        feature::Position, feature::Orientation, feature::LineScale, feature::Colors, feature::Points>;

    using CubeList = Add<
        visualization_msgs::Marker::CUBE_LIST,
        feature::Position, feature::Orientation, feature::Scale, feature::Colors, feature::Points>;

    using SphereList = Add<
        visualization_msgs::Marker::SPHERE_LIST,
        feature::Position, feature::Orientation, feature::Scale, feature::Colors, feature::Points>;

    using Points = Add<
        visualization_msgs::Marker::POINTS,
        feature::Position, feature::Orientation, feature::PointScale, feature::Colors, feature::Points>;

    using TextViewFacing = Add<
        visualization_msgs::Marker::TEXT_VIEW_FACING,
        feature::Position, feature::TextScale, feature::Color, feature::Text>;

    using MeshResource = Add<
        visualization_msgs::Marker::MESH_RESOURCE,
        feature::Position, feature::Orientation, feature::Scale, feature::Color, feature::MeshResource>;

    using TriangleList = Add<
        visualization_msgs::Marker::TRIANGLE_LIST,
        feature::Position, feature::Orientation, feature::Scale, feature::Colors, feature::Points>;

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
