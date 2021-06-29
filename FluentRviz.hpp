#pragma once

#include <string>
#include <vector>
#include <type_traits>
#include <complex>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

namespace flrv {

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
        color(1, 1, 1);
    }

    Derived &color(const double r, const double g, const double b) noexcept
    {
        this->message.color.r = r;
        this->message.color.g = g;
        this->message.color.b = b;
        return this->derived();
    }
};

template<typename Derived, typename Base>
struct Colors : Base {
    Colors() noexcept
    {
        color(1, 1, 1);
    }

    Derived &color(const double r, const double g, const double b) noexcept
    {
        this->message.color.r = r;
        this->message.color.g = g;
        this->message.color.b = b;
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
        geometry_msgs::Point point;
        point.x = x;
        point.y = y;
        point.z = z;
        this->message.points[index] = point;
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

template<typename Derived, typename Base>
struct CRTP : Base {
    Derived &derived() noexcept
    {
        return static_cast<Derived &>(*this);
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

template<
    typename Derived,
    typename Base,
    template<typename, typename> typename ...Decorators>
struct Decorate;

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
    using Type = Decorator<Derived, typename Decorate<Derived, Base, Decorators...>::Type>;
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
        ActionType<visualization_msgs::Marker::ADD>::Decorator,
        MarkerType<MARKER_TYPE>::template Decorator,
        Decorators...,
        CRTP
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
    Scale, Color, VectorArrowScale, ArrowPoints>;

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
}
