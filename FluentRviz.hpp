#pragma once

#include <string>
#include <vector>
#include <type_traits>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

namespace flrviz {

namespace option {
    enum class Arrow {
        POSE, VECTOR,
    };

    enum class Color {
        SEPARATE
    };
} // namespace option

namespace color {
    inline constexpr static int32_t RED = 0xF44336;
    inline constexpr static int32_t PINK = 0xFF4081;
    inline constexpr static int32_t PURPLE = 0x9C27B0;
    inline constexpr static int32_t DEEP_PURPLE = 0x7B1FA2;
    inline constexpr static int32_t INDIGO = 0x3F51B5;
    inline constexpr static int32_t BRUE = 0x448aff;
    inline constexpr static int32_t LIGHT_BRUE = 0x03A9F4;
    inline constexpr static int32_t CYAN = 0x00BCD4;
    inline constexpr static int32_t TEAL = 0x009688;
    inline constexpr static int32_t GREEN = 0x4CAF50;
    inline constexpr static int32_t LIGHT_GREEN = 0x8BC34A;
    inline constexpr static int32_t LIME = 0xCDDC39;
    inline constexpr static int32_t YELLOW = 0xFFEB3B;
    inline constexpr static int32_t AMBER = 0xFFC107;
    inline constexpr static int32_t ORANGE = 0xFF9800;
    inline constexpr static int32_t DEEP_ORANGE = 0xFF5722;
    inline constexpr static int32_t BROWN = 0x795548;
    inline constexpr static int32_t GREY = 0x9E9E9E;
    inline constexpr static int32_t BLUE_GREY = 0x607D8B;
    inline constexpr static int32_t WHITE = 0xffffff;
    inline constexpr static int32_t BLACK = 0x000000;
} // namespace color

namespace traits {
    namespace detail {
        template<typename AlwaysVoid, template<typename...> typename Op, typename... Args>
        struct detector : std::false_type { };

        template<template<typename...> typename Op, typename... Args>
        struct detector<std::void_t<Op<Args...>>, Op, Args...> : std::true_type { };
    } // namespace detail

    template<template<typename...> typename Op, typename... Args>
    constexpr inline bool is_detected_v = detail::detector<void, Op, Args...>::value;

    enum class Member : size_t {
        X = 0, Y = 1, Z = 2, W = 3,
    };

    template<typename T>
    using x_var_t = decltype(std::declval<T>().x);
    template<typename T>
    using y_var_t = decltype(std::declval<T>().y);
    template<typename T>
    using z_var_t = decltype(std::declval<T>().z);
    template<typename T>
    using w_var_t = decltype(std::declval<T>().w);

    template<typename T, auto M>
    inline constexpr bool is_var_accessible_v = false;
    template<typename T>
    inline constexpr bool is_var_accessible_v<T, Member::X> = is_detected_v<x_var_t, T>;
    template<typename T>
    inline constexpr bool is_var_accessible_v<T, Member::Y> = is_detected_v<y_var_t, T>;
    template<typename T>
    inline constexpr bool is_var_accessible_v<T, Member::Z> = is_detected_v<z_var_t, T>;
    template<typename T>
    inline constexpr bool is_var_accessible_v<T, Member::W> = is_detected_v<w_var_t, T>;

    template<typename T>
    using x_func_t = decltype(std::declval<T>().x());
    template<typename T>
    using y_func_t = decltype(std::declval<T>().y());
    template<typename T>
    using z_func_t = decltype(std::declval<T>().z());
    template<typename T>
    using w_func_t = decltype(std::declval<T>().w());

    template<typename T, auto M>
    inline constexpr bool is_func_accessible_v = false;
    template<typename T>
    inline constexpr bool is_func_accessible_v<T, Member::X> = is_detected_v<x_func_t, T>;
    template<typename T>
    inline constexpr bool is_func_accessible_v<T, Member::Y> = is_detected_v<y_func_t, T>;
    template<typename T>
    inline constexpr bool is_func_accessible_v<T, Member::Z> = is_detected_v<z_func_t, T>;
    template<typename T>
    inline constexpr bool is_func_accessible_v<T, Member::W> = is_detected_v<w_func_t, T>;

    template<typename T>
    using index_t = decltype(std::declval<T>()[std::declval<size_t>()]);
    template<typename T>
    inline constexpr bool is_index_accessible_v = is_detected_v<index_t, T>;

    template<typename T>
    using std_get_t = decltype(std::get<std::declval<size_t>()>(std::declval<T>()));
    template<typename T>
    inline constexpr bool is_std_get_defined_v = is_detected_v<std_get_t, T>;

    template<typename T>
    inline constexpr bool false_v = false;

    template<typename T, auto M, typename Enable = void>
    struct access;

    template<typename T>
    struct access<T, Member::X, std::enable_if_t<is_var_accessible_v<T, Member::X>>> {
        [[nodiscard]] static inline auto get(const T &value) noexcept
        { return value.x; }
    };
    template<typename T>
    struct access<T, Member::Y, std::enable_if_t<is_var_accessible_v<T, Member::Y>>> {
        [[nodiscard]] static inline auto get(const T &value) noexcept
        { return value.y; }
    };
    template<typename T>
    struct access<T, Member::Z, std::enable_if_t<is_var_accessible_v<T, Member::Z>>> {
        [[nodiscard]] static inline auto get(const T &value) noexcept
        { return value.z; }
    };
    template<typename T>
    struct access<T, Member::W, std::enable_if_t<is_var_accessible_v<T, Member::W>>> {
        [[nodiscard]] static inline auto get(const T &value) noexcept
        { return value.w; }
    };

    template<typename T>
    struct access<T, Member::X, std::enable_if_t<is_func_accessible_v<T, Member::X>>> {
        [[nodiscard]] static inline auto get(const T &value)
        { return value.x(); }
    };
    template<typename T>
    struct access<T, Member::Y, std::enable_if_t<is_func_accessible_v<T, Member::Y>>> {
        [[nodiscard]] static inline auto get(const T &value)
        { return value.y(); }
    };
    template<typename T>
    struct access<T, Member::Z, std::enable_if_t<is_func_accessible_v<T, Member::Z>>> {
        [[nodiscard]] static inline auto get(const T &value)
        { return value.z(); }
    };
    template<typename T>
    struct access<T, Member::W, std::enable_if_t<is_func_accessible_v<T, Member::W>>> {
        [[nodiscard]] static inline auto get(const T &value)
        { return value.w(); }
    };

    template<typename T, auto M>
    struct access<T, M, std::enable_if_t<!is_var_accessible_v<T, M> && !is_func_accessible_v<T, M> && is_index_accessible_v<T>>> {
        [[nodiscard]] static inline auto get(const T &value)
        { return value[static_cast<size_t>(M)]; }
    };

    template<typename T, auto M>
    struct access<T, M, std::enable_if_t<is_std_get_defined_v<T>>> {
        [[nodiscard]] static inline auto get(const T &value)
        { return std::get<static_cast<size_t>(M)>(value); }
    };

    template<typename T>
    struct access<std::complex<T>, Member::X> {
        [[nodiscard]] static inline auto get(const std::complex<T> &value)
        { return value.real(); }
    };
    template<typename T>
    struct access<std::complex<T>, Member::Y> {
        [[nodiscard]] static inline auto get(const std::complex<T> &value)
        { return value.imag(); }
    };

    template<auto M, typename T>
    [[nodiscard]] inline auto get(const T &value)
    { return access<T, M>::get(value); }
} // namespace internal

namespace param {
    class Vector3 {
        geometry_msgs::Vector3 vector3;

    public:
        Vector3(const geometry_msgs::Vector3 arg) noexcept
            : vector3(arg)
        { }

        Vector3(const double x, const double y, const double z) noexcept
        {
            vector3.x = x;
            vector3.y = y;
            vector3.z = z;
        }

        template<typename T>
        Vector3(const T &value) noexcept
            : Vector3(
                traits::get<traits::Member::X>(value),
                traits::get<traits::Member::Y>(value),
                traits::get<traits::Member::Z>(value))
        { }

        [[nodiscard]] static inline Vector3 UnitX() noexcept
        { return { 1, 0, 0 }; }
        [[nodiscard]] static inline Vector3 UnitY() noexcept
        { return { 0, 1, 0 }; }
        [[nodiscard]] static inline Vector3 UnitZ() noexcept
        { return { 0, 0, 1 }; }

        template<typename T>
        [[nodiscard]] static Vector3 from_2d(const T &point, const double z = 0.0) noexcept
        {
            return {
                traits::get<traits::Member::X>(point),
                traits::get<traits::Member::Y>(point),
                z
            };
        }

        operator const geometry_msgs::Vector3 &() const noexcept
        { return vector3; }
    };

    class Quaternion {
        geometry_msgs::Quaternion quaternion;

    public:
        Quaternion(const geometry_msgs::Quaternion arg) noexcept
            : quaternion(arg)
        { }

        Quaternion(const double x, const double y, const double z, const double w) noexcept
        {
            quaternion.x = x;
            quaternion.y = y;
            quaternion.z = z;
            quaternion.w = w;
        }

        template<typename T>
        Quaternion(const T &value) noexcept
            : Quaternion(
                traits::get<traits::Member::X>(value),
                traits::get<traits::Member::Y>(value),
                traits::get<traits::Member::Z>(value),
                traits::get<traits::Member::W>(value))
        { }

        [[nodiscard]] static Quaternion from_angle_axis(const double theta, const Vector3 axis = Vector3::UnitZ()) noexcept
        {
            geometry_msgs::Vector3 vector3 = axis;
            return {
                vector3.x * std::sin(theta / 2),
                vector3.y * std::sin(theta / 2),
                vector3.z * std::sin(theta / 2),
                std::cos(theta / 2),
            };
        }

        operator const geometry_msgs::Quaternion &() const noexcept
        { return quaternion; }
    };

    class Point {
        geometry_msgs::Point point;

    public:
        Point(const geometry_msgs::Point arg) noexcept
            : point(arg)
        { }

        Point(const double x, const double y, const double z = 0.0) noexcept
        {
            point.x = x;
            point.y = y;
            point.z = z;
        }

        template<typename T>
        Point(const T &value)
            : Point(
                traits::get<traits::Member::X>(value),
                traits::get<traits::Member::Y>(value),
                traits::get<traits::Member::Z>(value))
        { }

        template<typename T>
        [[nodiscard]] static Point from_2d(const T &point, const double z = 0.0) noexcept
        {
            return {
                traits::get<traits::Member::X>(point),
                traits::get<traits::Member::Y>(point),
                z
            };
        }

        operator const geometry_msgs::Point &() const noexcept
        { return point; }
    };

    class Color {
        std_msgs::ColorRGBA color;

    public:
        Color(const std_msgs::ColorRGBA arg) noexcept
            : color(arg)
        { }

        Color(const float r, const float g, const float b, const float a = 1.0) noexcept
        {
            color.r = r;
            color.g = g;
            color.b = b;
            color.a = a;
        }

        [[nodiscard]] static Color from_hex(const int32_t hex) noexcept
        {
            return {
                ((hex >> 16) & 0xff) / 255.0f,
                ((hex >> 8) & 0xff) / 255.0f,
                (hex & 0xff) / 255.0f,
            };
        }

        [[nodiscard]] Color &alpha(const float a) noexcept
        {
            color.a = a;
            return *this;
        }

        [[nodiscard]] static inline Color Red() noexcept
        { return from_hex(color::RED); }
        [[nodiscard]] static inline Color Pink() noexcept
        { return from_hex(color::PINK); }
        [[nodiscard]] static inline Color Purple() noexcept
        { return from_hex(color::PURPLE); }
        [[nodiscard]] static inline Color DeepPurple() noexcept
        { return from_hex(color::DEEP_PURPLE); }
        [[nodiscard]] static inline Color Indigo() noexcept
        { return from_hex(color::INDIGO); }
        [[nodiscard]] static inline Color Blue() noexcept
        { return from_hex(color::BRUE); }
        [[nodiscard]] static inline Color LightBlue() noexcept
        { return from_hex(color::LIGHT_BRUE); }
        [[nodiscard]] static inline Color Cyan() noexcept
        { return from_hex(color::CYAN); }
        [[nodiscard]] static inline Color Teal() noexcept
        { return from_hex(color::TEAL); }
        [[nodiscard]] static inline Color Green() noexcept
        { return from_hex(color::GREEN); }
        [[nodiscard]] static inline Color LightGreen() noexcept
        { return from_hex(color::LIGHT_GREEN); }
        [[nodiscard]] static inline Color Lime() noexcept
        { return from_hex(color::LIME); }
        [[nodiscard]] static inline Color Yellow() noexcept
        { return from_hex(color::YELLOW); }
        [[nodiscard]] static inline Color Amber() noexcept
        { return from_hex(color::AMBER); }
        [[nodiscard]] static inline Color Orange() noexcept
        { return from_hex(color::ORANGE); }
        [[nodiscard]] static inline Color DeepOrange() noexcept
        { return from_hex(color::DEEP_ORANGE); }
        [[nodiscard]] static inline Color Brown() noexcept
        { return from_hex(color::BROWN); }
        [[nodiscard]] static inline Color Grey() noexcept
        { return from_hex(color::GREY); }
        [[nodiscard]] static inline Color BlueGrey() noexcept
        { return from_hex(color::BLUE_GREY); }
        [[nodiscard]] static inline Color White() noexcept
        { return from_hex(color::WHITE); }
        [[nodiscard]] static inline Color Black() noexcept
        { return from_hex(color::BLACK); }

        operator const std_msgs::ColorRGBA &() const noexcept
        { return color; }
    };
} // namespace param

namespace internal {
    template<auto Val1, auto Val2>
    inline constexpr bool equals_v = false;

    template<auto Val>
    inline constexpr bool equals_v<Val, Val> = true;

    template<auto Val, auto... Vals>
    inline constexpr bool is_contained_v = (equals_v<Val, Vals> || ...);

    template<int32_t MarkerType>
    inline constexpr bool is_arrow_marker_v = MarkerType == visualization_msgs::Marker::ARROW;
    template<int32_t MarkerType>
    inline constexpr bool is_cube_marker_v = MarkerType == visualization_msgs::Marker::CUBE;
    template<int32_t MarkerType>
    inline constexpr bool is_sphere_marker_v = MarkerType == visualization_msgs::Marker::SPHERE;
    template<int32_t MarkerType>
    inline constexpr bool is_cylinder_marker_v = MarkerType == visualization_msgs::Marker::CYLINDER;
    template<int32_t MarkerType>
    inline constexpr bool is_line_strip_marker_v = MarkerType == visualization_msgs::Marker::LINE_STRIP;
    template<int32_t MarkerType>
    inline constexpr bool is_line_list_marker_v = MarkerType == visualization_msgs::Marker::LINE_LIST;
    template<int32_t MarkerType>
    inline constexpr bool is_cube_list_marker_v = MarkerType == visualization_msgs::Marker::CUBE_LIST;
    template<int32_t MarkerType>
    inline constexpr bool is_sphere_list_marker_v = MarkerType == visualization_msgs::Marker::SPHERE_LIST;
    template<int32_t MarkerType>
    inline constexpr bool is_points_marker_v = MarkerType == visualization_msgs::Marker::POINTS;
    template<int32_t MarkerType>
    inline constexpr bool is_text_view_facing_marker_v = MarkerType == visualization_msgs::Marker::TEXT_VIEW_FACING;
    template<int32_t MarkerType>
    inline constexpr bool is_mesh_resource_marker_v = MarkerType == visualization_msgs::Marker::MESH_RESOURCE;
    template<int32_t MarkerType>
    inline constexpr bool is_triangle_list_marker_v = MarkerType == visualization_msgs::Marker::TRIANGLE_LIST;

    template<int32_t MarkerType>
    inline constexpr bool is_common_scale_available_v =
        is_cube_marker_v<MarkerType>
        || is_sphere_marker_v<MarkerType>
        || is_cylinder_marker_v<MarkerType>
        || is_cube_list_marker_v<MarkerType>
        || is_sphere_list_marker_v<MarkerType>
        || is_mesh_resource_marker_v<MarkerType>
        || is_triangle_list_marker_v<MarkerType>;

    template<int32_t MarkerType>
    inline constexpr bool is_line_marker_v =
        is_line_strip_marker_v<MarkerType>
        || is_line_list_marker_v<MarkerType>;

    template<int32_t MarkerType>
    inline constexpr bool is_points_available_v =
        is_line_strip_marker_v<MarkerType>
        || is_line_list_marker_v<MarkerType>
        || is_cube_list_marker_v<MarkerType>
        || is_sphere_list_marker_v<MarkerType>
        || is_points_marker_v<MarkerType>
        || is_triangle_list_marker_v<MarkerType>;

    template<int32_t MarkerType>
    inline constexpr bool is_colors_available_v = is_points_available_v<MarkerType>;

    template<auto... Options>
    struct OptionPack { };

    template<typename T>
    struct PositionHelper {
        [[nodiscard]] T &&position(const double x, const double y, const double z = 0.0) && noexcept
        { return std::move(*this).position({ x, y, z }); }

        [[nodiscard]] T &&position(const param::Point position) && noexcept
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.pose.position = position;
            return std::move(self);
        }
    };

    template<typename Marker, size_t MarkerType, typename Option, typename Enable = void>
    struct PositionEnabler : PositionHelper<Marker> { };

    template<typename T>
    struct OrientationHelper {
        [[nodiscard]] T &&orientation(const double x, const double y, const double z, const double w) && noexcept
        { return orientation({ x, y, z, w }); }

        [[nodiscard]] T &&orientation(const param::Quaternion orientation) && noexcept
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.pose.orientation = orientation;
            return std::move(self);
        }
    };

    template<typename Marker, size_t MarkerType, typename Option, typename Enable = void>
    struct OrientationEnabler : OrientationHelper<Marker> { };

    template<typename T>
    struct ScaleHelper {
        [[nodiscard]] T &&scale(const double x, const double y, const double z) && noexcept
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.scale = param::Vector3(x, y, z);
            return std::move(self);
        }
    };

    template<typename T>
    struct PoseArrowScaleHelper : ScaleHelper<T> {
        [[nodiscard]] T &&scale(const double length, const double width, const double height) && noexcept
        { return std::move(*this).ScaleHelper<T>::scale(length, width, height); }
    };

    template<typename T>
    struct VectorArrowScaleHelper : ScaleHelper<T> {
        [[nodiscard]] T &&scale(const double shaft_diameter, const double head_diameter, const double head_length) && noexcept
        { return std::move(*this).ScaleHelper<T>::scale(shaft_diameter, head_diameter, head_length); }
    };

    template<typename T>
    struct PointScaleHelper : ScaleHelper<T> {
        [[nodiscard]] T &&scale(const double width, const double height) && noexcept
        { return std::move(*this).ScaleHelper<T>::scale(width, height, 0); }
    };

    template<typename T>
    struct LineScaleHelper : ScaleHelper<T> {
        [[nodiscard]] T &&scale(const double width) && noexcept
        { return std::move(*this).ScaleHelper<T>::scale(width, 0, 0); }
    };

    template<typename T>
    struct TextScaleHelper : ScaleHelper<T> {
        [[nodiscard]] T &&scale(const double height) && noexcept
        { return std::move(*this).ScaleHelper<T>::scale(0, 0, height); }
    };

    template<typename Marker, size_t MarkerType, typename Option, typename Enable = void>
    struct ScaleEnabler { };

    template<typename Marker, size_t MarkerType, auto... Options>
    struct ScaleEnabler<Marker, MarkerType, OptionPack<Options...>,
        std::enable_if_t<is_common_scale_available_v<MarkerType>>>
        : ScaleHelper<Marker> { };

    template<typename Marker, size_t MarkerType, auto... Options>
    struct ScaleEnabler<Marker, MarkerType, OptionPack<Options...>,
        std::enable_if_t<is_arrow_marker_v<MarkerType> && is_contained_v<option::Arrow::POSE, Options...>>>
        : PoseArrowScaleHelper<Marker> { };

    template<typename Marker, size_t MarkerType, auto... Options>
    struct ScaleEnabler<Marker, MarkerType, OptionPack<Options...>,
        std::enable_if_t<is_arrow_marker_v<MarkerType> && is_contained_v<option::Arrow::VECTOR, Options...>>>
        : VectorArrowScaleHelper<Marker> { };

    template<typename Marker, size_t MarkerType, auto... Options>
    struct ScaleEnabler<Marker, MarkerType, OptionPack<Options...>,
        std::enable_if_t<is_points_marker_v<MarkerType>>>
        : PointScaleHelper<Marker> { };

    template<typename Marker, size_t MarkerType, auto... Options>
    struct ScaleEnabler<Marker, MarkerType, OptionPack<Options...>,
        std::enable_if_t<is_line_marker_v<MarkerType>>>
        : LineScaleHelper<Marker> { };

    template<typename Marker, size_t MarkerType, auto... Options>
    struct ScaleEnabler<Marker, MarkerType, OptionPack<Options...>,
        std::enable_if_t<is_text_view_facing_marker_v<MarkerType>>>
        : TextScaleHelper<Marker> { };

    template<typename T>
    struct ColorHelper {
        [[nodiscard]] T &&color(const float r, const float g, const float b, const float a = 1.0) && noexcept
        { return std::move(*this).color({ r, g, b, a }); }

        [[nodiscard]] T &&color(const param::Color color) && noexcept
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.color = color;
            return std::move(self);
        }
    };

    template<typename Marker, size_t MarkerType, typename Option, typename Enable = void>
    struct ColorEnabler { };

    template<typename Marker, size_t MarkerType, auto... Options>
    struct ColorEnabler<Marker, MarkerType, OptionPack<Options...>,
        std::enable_if_t<!is_colors_available_v<MarkerType> || !is_contained_v<option::Color::SEPARATE, Options...>>>
        : ColorHelper<Marker> { };

    template<typename T>
    struct LifetimeHelper {
        [[nodiscard]] T &&lifetime(const double lifetime) && noexcept
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &Marker = self.msg();
            Marker.lifetime = ros::Duration(lifetime);
            return std::move(self);
        }
    };

    template<typename T>
    struct FrameLockedHelper {
        [[nodiscard]] T &&frame_locked(const bool frame_locked) && noexcept
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.frame_locked = frame_locked;
            return std::move(self);
        }
    };

    template<typename T>
    struct PointsHelper {
        [[nodiscard]] T &&points(std::vector<geometry_msgs::Point> points) && noexcept
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.points = std::move(points);
            return std::move(self);
        }

        template<typename PointLike>
        [[nodiscard]] T &&points(std::vector<PointLike> points) && noexcept
        {
            std::vector<geometry_msgs::Point> converted(points.size());
            std::transform(std::begin(points), std::end(points), std::begin(converted),
                [](auto &e) { return param::Point(e); });
            return std::move(*this).points(std::move(converted));
        }
    };

    template<typename T>
    struct ArrowPointsHelper {
        [[nodiscard]] T &&start(const double x, const double y, const double z = 0.0) && noexcept
        { return std::move(*this).start({ x, y, z }); }

        [[nodiscard]] T &&start(const param::Point point) && noexcept
        {
            T &self = static_cast<T &>(*this);
            return set_point(0, point);
        }

        [[nodiscard]] T &&end(const double x, const double y, const double z = 0.0) && noexcept
        { return std::move(*this).start({ x, y, z }); }

        [[nodiscard]] T &&end(const param::Point point) && noexcept
        {
            T &self = static_cast<T &>(*this);
            return set_point(1, point);
        }

    private:
        [[nodiscard]] T &&set_point(const size_t idx, const param::Point &point) noexcept
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.points[idx] = point;
            return std::move(self);
        }
    };

    template<typename Marker, size_t MarkerType, typename Option, typename Enable = void>
    struct PointsEnabler { };

    template<typename Marker, size_t MarkerType, auto... Options>
    struct PointsEnabler<Marker, MarkerType, OptionPack<Options...>,
        std::enable_if_t<is_points_available_v<MarkerType>>>
        : PointsHelper<Marker> { };

    template<typename Marker, size_t MarkerType, auto... Options>
    struct PointsEnabler<Marker, MarkerType, OptionPack<Options...>,
        std::enable_if_t<is_arrow_marker_v<MarkerType> && is_contained_v<option::Arrow::VECTOR, Options...>>>
        : ArrowPointsHelper<Marker> { };

    template<typename T>
    struct ColorsHelper {
        [[nodiscard]] T &&colors(std::vector<std_msgs::ColorRGBA> colors) && noexcept
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.colors = std::move(colors);
            return std::move(self);
        }

        template<typename S>
        [[nodiscard]] T &&colors(std::vector<param::Color> colors) && noexcept
        {
            std::vector<geometry_msgs::Point> converted(std::begin(colors), std::end(colors));
            return std::move(*this).points(std::move(converted));
        }
    };

    template<typename Marker, size_t MarkerType, typename Option, typename Enable = void>
    struct ColorsEnabler { };

    template<typename Marker, size_t MarkerType, auto... Options>
    struct ColorsEnabler<Marker, MarkerType, OptionPack<Options...>,
        std::enable_if_t<is_colors_available_v<MarkerType> && is_contained_v<option::Color::SEPARATE, Options...>>>
        : ColorsHelper<Marker> { };

    template<typename T>
    struct TextHelper {
        [[nodiscard]] T &&text(std::string text) && noexcept
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.text = std::move(text);
            return std::move(self);
        }
    };

    template<typename Marker, size_t MarkerType, typename Option, typename Enable = void>
    struct TextEnabler { };

    template<typename Marker, size_t MarkerType, auto... Options>
    struct TextEnabler<Marker, MarkerType, OptionPack<Options...>,
        std::enable_if_t<is_text_view_facing_marker_v<MarkerType>>>
        : TextHelper<Marker> { };

    template<typename T>
    struct MeshResourceHelper {
        [[nodiscard]] T &&mesh_resource(std::string mesh_resource) && noexcept
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.mesh_resource = std::move(mesh_resource);
            return std::move(self);
        }

        [[nodiscard]] T &&mesh_use_embedded_materials(const bool mesh_use_embedded_materials) && noexcept
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.mesh_use_embedded_materials = mesh_use_embedded_materials;
            return std::move(self);
        }
    };

    template<typename Marker, size_t MarkerType, typename Option, typename Enable = void>
    struct MeshResourceEnabler { };

    template<typename Marker, size_t MarkerType, auto... Options>
    struct MeshResourceEnabler<Marker, MarkerType, OptionPack<Options...>,
        std::enable_if_t<is_mesh_resource_marker_v<MarkerType>>>
        : MeshResourceHelper<Marker> { };
} // namespace internal

namespace marker {
    class MarkerBase {
        visualization_msgs::Marker marker;

    public:
        MarkerBase() = default;
        MarkerBase(const MarkerBase &) = delete;
        MarkerBase(MarkerBase &&) = default;

        [[nodiscard]] visualization_msgs::Marker &msg() noexcept
        { return marker; }
    };

    template<int32_t MarkerType, auto... Options>
    class Marker
        : public MarkerBase
        , public internal::PositionEnabler<Marker<MarkerType, Options...>, MarkerType, internal::OptionPack<Options...>>
        , public internal::OrientationEnabler<Marker<MarkerType, Options...>, MarkerType, internal::OptionPack<Options...>>
        , public internal::ScaleEnabler<Marker<MarkerType, Options...>, MarkerType, internal::OptionPack<Options...>>
        , public internal::ColorEnabler<Marker<MarkerType, Options...>, MarkerType, internal::OptionPack<Options...>>
        , public internal::LifetimeHelper<Marker<MarkerType, Options...>>
        , public internal::FrameLockedHelper<Marker<MarkerType, Options...>>
        , public internal::PointsEnabler<Marker<MarkerType, Options...>, MarkerType, internal::OptionPack<Options...>>
        , public internal::ColorsEnabler<Marker<MarkerType, Options...>, MarkerType, internal::OptionPack<Options...>>
        , public internal::TextEnabler<Marker<MarkerType, Options...>, MarkerType, internal::OptionPack<Options...>>
        , public internal::MeshResourceEnabler<Marker<MarkerType, Options...>, MarkerType, internal::OptionPack<Options...>> {

    public:
        Marker(int32_t id, std::string ns = "") noexcept
        {
            auto& marker = msg();

            marker.id = id;
            marker.ns = std::move(ns);
            marker.type = MarkerType;
            marker.pose.orientation.w = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;

            if constexpr (internal::is_common_scale_available_v<MarkerType>) {
                marker.scale.x = 1;
                marker.scale.y = 1;
                marker.scale.z = 1;
            }

            if constexpr (internal::is_points_marker_v<MarkerType>) {
                marker.scale.x = 0.05;
                marker.scale.y = 0.05;
            }

            if constexpr (internal::is_line_marker_v<MarkerType>) {
                marker.scale.x = 0.05;
            }

            if constexpr (internal::is_arrow_marker_v<MarkerType>) {
                if constexpr (internal::is_contained_v<option::Arrow::POSE, Options...>) {
                    marker.scale.x = 1;
                    marker.scale.y = 1;
                    marker.scale.z = 1;
                }
                if constexpr (internal::is_contained_v<option::Arrow::VECTOR, Options...>) {
                    marker.scale.x = .2;
                    marker.scale.y = .4;
                    marker.scale.z = .4;
                }
            }

            if constexpr (
                internal::is_arrow_marker_v<MarkerType>
                && internal::is_contained_v<option::Arrow::VECTOR, Options...>) {
                marker.points.resize(2);
            }

            if constexpr (internal::is_text_view_facing_marker_v<MarkerType>) {
                marker.scale.z = 1;
                marker.text = "visualization_msgs::Marker::TEXT_VIEW_FACING";
            }
        }
    };

    using PoseArrow = Marker<visualization_msgs::Marker::ARROW, option::Arrow::POSE>;
    using VectorArrow = Marker<visualization_msgs::Marker::ARROW, option::Arrow::VECTOR>;
    using Arrow = VectorArrow;
    using Cube = Marker<visualization_msgs::Marker::CUBE>;
    using Sphere = Marker<visualization_msgs::Marker::SPHERE>;
    using Cylinder = Marker<visualization_msgs::Marker::CYLINDER>;
    using LineStrip = Marker<visualization_msgs::Marker::LINE_STRIP>;
    using ColorfulLineStrip = Marker<visualization_msgs::Marker::LINE_STRIP, option::Color::SEPARATE>;
    using LineList = Marker<visualization_msgs::Marker::LINE_LIST>;
    using ColorfulLineList = Marker<visualization_msgs::Marker::LINE_LIST, option::Color::SEPARATE>;
    using CubeList = Marker<visualization_msgs::Marker::CUBE_LIST>;
    using ColorfulCubeList = Marker<visualization_msgs::Marker::CUBE_LIST, option::Color::SEPARATE>;
    using SphereList = Marker<visualization_msgs::Marker::SPHERE_LIST>;
    using ColorfulSphereList = Marker<visualization_msgs::Marker::SPHERE_LIST, option::Color::SEPARATE>;
    using Points = Marker<visualization_msgs::Marker::POINTS>;
    using ColorfulPoints = Marker<visualization_msgs::Marker::POINTS, option::Color::SEPARATE>;
    using TextViewFacing = Marker<visualization_msgs::Marker::TEXT_VIEW_FACING>;
    using MeshResource = Marker<visualization_msgs::Marker::MESH_RESOURCE>;
    using TriangleList = Marker<visualization_msgs::Marker::TRIANGLE_LIST>;
    using ColorfulTriangleList = Marker<visualization_msgs::Marker::TRIANGLE_LIST, option::Color::SEPARATE>;

    class Delete : public MarkerBase {
    public:
        Delete(int32_t id, std::string ns = "")
        {
            auto& marker = msg();
            marker.id = id;
            marker.ns = std::move(ns);
            marker.action = visualization_msgs::Marker::DELETE;
        }
    };

    class DeleteAll : public MarkerBase {
    public:
        DeleteAll()
        {
            auto& marker = msg();
            marker.action = visualization_msgs::Marker::DELETEALL;
        }
    };
} // namespace marker

class Rviz {
    ros::NodeHandle nh;
    ros::Publisher pub;

    std::string frame_id_;

public:
    Rviz(const std::string frame_id = "map", const std::string topic = "visualization_marker")
        : pub(nh.advertise<visualization_msgs::Marker>(topic, 1))
        , frame_id_(frame_id)
    { }

    template<typename T>
    const Rviz &operator<<(T &&marker) const
    {
        publish(marker.msg());
        return *this;
    }

    void publish(visualization_msgs::Marker &marker) const
    {
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        pub.publish(marker);
    }
};

} // namespace flrviz
