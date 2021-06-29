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
        Decorator()
        {
            this->message.action = TYPE;
        }
    };
};

template<int32_t TYPE>
struct MarkerType {
    template<typename Derived, typename Base>
    struct Decorator : Base {
        Decorator()
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
    Orientation()
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
    Scale()
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
struct Color : Base {
    Color()
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
    Colors()
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
struct Text : Base {
    Text()
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
    Derived &derived()
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
    Position, Orientation, Scale, Color>;

using VectorArrowMarker = Add<
    visualization_msgs::Marker::ARROW,
    Scale, Color, Points>;

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
    Position, Orientation, Scale, Color, Points>;

using LineListMarker = Add<
    visualization_msgs::Marker::LINE_LIST,
    Position, Orientation, Scale, Color, Points>;

using CubeListMarker = Add<
    visualization_msgs::Marker::CUBE_LIST,
    Position, Orientation, Scale, Color, Points>;

using SphereListMarker = Add<
    visualization_msgs::Marker::SPHERE_LIST,
    Position, Orientation, Scale, Color, Points>;

using PointsMarker = Add<
    visualization_msgs::Marker::POINTS,
    Position, Orientation, Scale, Color, Points>;

using TextViewFacingMarker = Add<
    visualization_msgs::Marker::TEXT_VIEW_FACING,
    Position, Scale, Color, Text>;

using MeshResourceMarker = Add<
    visualization_msgs::Marker::MESH_RESOURCE,
    Position, Orientation, Scale, Color, MeshResource>;

using TriangleListMarker = Add<
    visualization_msgs::Marker::TRIANGLE_LIST,
    Position, Orientation, Scale, Color, Points>;
}
