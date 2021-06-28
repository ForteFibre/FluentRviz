#pragma once

#include <string>
#include <vector>
#include <type_traits>
#include <complex>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

namespace flrv {

struct Marker {
protected:
    visualization_msgs::Marker marker;
};

template<int32_t ActionType>
struct Action {
    template<typename T>
    struct Bind : T {
        Bind()
        {
            this->marker.action = ActionType;
        }
    };
};

template<int32_t MarkerType>
struct Type {
    template<typename T>
    struct Bind : T {
        Bind()
        {
            this->marker.type = MarkerType;
        }
    };
};

template<typename T>
struct Position : T {
    [[nodiscard]]
    Position &position(const double x, const double y, const double z) noexcept
    {
        this->marker.x = x;
        this->marker.y = y;
        this->marker.z = z;
        return *this;
    }
};

template<typename T>
struct Orientation : T {
    Orientation()
    {
        orientation(1, 0, 0, 0);
    }

    [[nodiscard]]
    Orientation &orientation(const double w, const double x, const double y, const double z) noexcept
    {
        this->marker.w = w;
        this->marker.x = x;
        this->marker.y = y;
        this->marker.z = z;
        return *this;
    }
};

template<typename T>
struct Scale : T {
    Scale()
    {
        scale(1, 1, 1);
    }

    [[nodiscard]]
    Scale &scale(const double x, const double y, const double z) noexcept
    {
        this->marker.x = x;
        this->marker.y = y;
        this->marker.z = z;
        return *this;
    }
};

template<typename T>
struct Color : T {
    Color()
    {
        color(1, 1, 1);
    }

    [[nodiscard]]
    Color &color(const double r, const double g, const double b) noexcept
    {
        this->marker.r = r;
        this->marker.g = g;
        this->marker.b = b;
        return *this;
    }
};

template<typename T>
struct Colors : T {
    Colors()
    {
        color(1, 1, 1);
    }

    [[nodiscard]]
    Colors &color(const double r, const double g, const double b) noexcept
    {
        this->marker.r = r;
        this->marker.g = g;
        this->marker.b = b;
        this->marker.colors.clear();
        return *this;
    }

    [[nodiscard]]
    Colors &color(const std::vector<std_msgs::ColorRGBA> &colors) noexcept
    {
        this->marker.colors = colors;
        return *this;
    }
};

template<typename T>
struct Lifetime : T {
    [[nodiscard]]
    Lifetime &lifetime(const double lifetime) noexcept
    {
        this->marker.lifetime = lifetime;
        return *this;
    }
};

template<typename T>
struct FrameLocked : T {
    [[nodiscard]]
    FrameLocked &frame_locked(const uint8_t frame_locked) noexcept
    {
        this->marker.frame_locked = frame_locked;
        return *this;
    }
};

template<typename T>
struct Points : T {
    [[nodiscard]]
    Points &points(const std::vector<geometry_msgs::Point> &points) noexcept
    {
        this->marker.points = points;
        return *this;
    }
};

template<typename T>
struct Text : T {
    Text()
    {
        text("visualization_msgs::Marker");
    }

    [[nodiscard]]
    Text &text(const std::string &text) noexcept
    {
        this->marker.text = text;
        return *this;
    }
};

template<typename T>
struct MeshResource : T {
    [[nodiscard]]
    MeshResource &mesh_resource(const std::string &mesh_resource) noexcept
    {
        this->marker.mesh_resouce = mesh_resource;
        return *this;
    }

    [[nodiscard]]
    MeshResource &mesh_use_embedded_materials(const uint8_t mesh_use_embedded_materials) noexcept
    {
        this->marker.mesh_use_embedded_materials = mesh_use_embedded_materials;
        return *this;
    }
};

template<typename T>
struct Identified : T {
    Identified(const int32_t id, const std::string &ns = "") noexcept
    {
        this->marker.id = id;
        this->marker.ns = ns;
    }
};

template<typename Element, template<typename> typename ...Decorators>
struct ApplyImpl {
private:
    template<
        typename InnerElement,
        template<typename> typename InnerDecorator,
        template<typename> typename ...InnerDecorators>
    struct ApplyExpand {
        using Type = typename ApplyImpl<InnerDecorator<InnerElement>, InnerDecorators...>::Type;
    };

public:
    using Type = typename ApplyExpand<Element, Decorators...>::Type;
};

template<typename Element>
struct ApplyImpl<Element> {
    using Type = Element;
};

template<typename Element, template<typename> typename ...Decorators>
using Apply = typename ApplyImpl<Element, Decorators...>::Type;

using Cube = Apply<Marker,
    Action<visualization_msgs::Marker::ADD>::Bind,
    Type<visualization_msgs::Marker::CUBE>::Bind,
    Position, Orientation, Scale, Color, Identified>;

}
