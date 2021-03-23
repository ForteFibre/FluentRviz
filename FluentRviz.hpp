#pragma once

#include <string>
#include <vector>
#include <type_traits>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#define FLRV_SUPPRESS(X) static_cast<void>(X)
#define FLRV_DERIVED(T) static_cast<T &>(*this)

namespace flrv {

namespace stream {

    template<typename Source>
    using iterator_t = decltype(std::begin(std::declval<Source &>()));

    template<typename Source>
    using element_t = decltype(*std::declval<iterator_t<Source>>());

    namespace internal {

        template<typename Source>
        struct ReferenceStream {
            Source *source;

            ReferenceStream() noexcept = default;
            ReferenceStream(Source &s) noexcept : source(std::addressof(s))
            { }

            auto begin()
            { return std::begin(*source); }

            auto end()
            { return std::end(*source); }
        };

        template<typename T>
        constexpr auto wrap(T && t)
        {
            if constexpr (std::is_lvalue_reference_v<T &&>) {
                return ReferenceStream(t);
            } else {
                return std::move(t);
            }
        }

        template<typename Iterator>
        struct IteratorStream {
            Iterator begin_;
            Iterator end_;

        public:
            IteratorStream() = default;

            IteratorStream(Iterator begin, Iterator end): begin_(begin), end_(end)
            { }

            Iterator begin() const noexcept
            { return begin_; }

            Iterator end() const noexcept
            { return end_; }
        };

        template<typename Source, typename Func>
        struct MapStream {
            Source source;
            Func func;

            struct cursol {
                MapStream *parent;
                iterator_t<Source> itr;

                cursol() = default;

                cursol(MapStream *p, iterator_t<Source> i): parent(p), itr(i)
                { }

                decltype(auto) operator*()
                { return parent->func(*itr); }

                cursol &operator++()
                { ++itr; return *this; }

                bool operator!=(const cursol &rhs)
                { return itr != rhs.itr; }
            };

            MapStream(Source s, Func f): source(s), func(std::move(f))
            { }

            auto begin()
            { return cursol(this, std::begin(source)); }

            auto end()
            { return cursol(this, std::end(source)); }
        };

        template<typename Source, typename Pred>
        struct FilterStream {
            Source source;
            Pred pred;

            struct cursol {
                FilterStream *parent;
                iterator_t<Source> itr;

                void satisfy()
                { for (auto end = std::end(parent->source); itr != end && !parent->pred(*itr); ++itr); }

                cursol() = default;

                cursol(FilterStream *fs, iterator_t<Source> i): parent(fs), itr(i)
                { satisfy(); }

                decltype(auto) operator*()
                { return *itr; }

                cursol &operator++()
                { ++itr, satisfy(); return *this; }

                bool operator!=(const cursol &rhs)
                { return itr != rhs.itr; }
            };

            FilterStream(Source s, Pred p): source(std::move(s)), pred(std::move(p))
            { }

            auto begin()
            { return cursol(this, std::begin(source)); }

            auto end()
            { return cursol(this, std::end(source)); }
        };

        template<typename Payload>
        struct StoreInternal {
            Payload payload;

            Payload &update(Payload &&new_payload)
            { return payload = std::move(new_payload); }

            template<typename Ignore>
            Payload &get(Ignore)
            { return payload; }
        };

        struct PassThroughInternal {
            template<typename Payload>
            Payload &update(Payload &&new_payload)
            { return new_payload; }

            template<typename Iterator>
            decltype(auto) get(Iterator itr)
            { return *itr; }
        };

        template<typename Source>
        struct FlattenStream : std::conditional_t<std::is_reference_v<element_t<Source>>, PassThroughInternal, StoreInternal<element_t<Source>>> {
            Source source;

            struct cursol {
                FlattenStream *parent;
                iterator_t<Source> parent_itr;
                iterator_t<element_t<Source>> child_itr;

                void satisfy()
                {
                    for(auto end = std::end(parent->source); parent_itr != end; ++parent_itr) {
                        auto &child = parent->update(*parent_itr);
                        child_itr = std::begin(child);
                        if(child_itr != std::end(child)) return;
                    }
                }

                cursol() = default;

                cursol(FlattenStream *par, iterator_t<Source> edge)
                    : parent(par), parent_itr(edge)
                { satisfy(); }

                decltype(auto) operator*()
                { return *child_itr; }

                cursol &operator++()
                {
                    if (!(++child_itr != std::end(parent->get(parent_itr)))) ++parent_itr, satisfy();
                    return *this;
                }

                bool operator!=(const cursol &rhs)
                { return parent_itr != rhs.parent_itr; }
            };

            FlattenStream(Source s): source(std::move(s))
            { }

            auto begin()
            { return cursol(this, std::begin(source)); }

            auto end()
            { return cursol(this, std::end(source)); }
        };

        template<typename Source>
        struct GroupedStream {
            Source source;
            size_t size;

            struct cursol {
                GroupedStream *parent;
                iterator_t<Source> left, right;
                size_t size;

                iterator_t<Source> skip(iterator_t<Source> itr, size_t x)
                {
                    iterator_t<Source> res = itr;
                    for (size_t i = x; i > 0 && res != std::end(parent->source); --i) ++res;
                    return res;
                }

                cursol() = default;

                cursol(GroupedStream *p, iterator_t<Source> i, size_t n): parent(p), left(i), right(skip(left, n)), size(n)
                { }

                decltype(auto) operator*()
                { return IteratorStream(left, right); }

                cursol &operator++()
                { left = skip(right, 1), right = skip(left, size); return *this; }

                bool operator!=(const cursol &rhs)
                { return left != rhs.left; }
            };

            GroupedStream(Source s, size_t n): source(std::move(s)), size(n)
            { }

            auto begin()
            { return cursol(this, std::begin(source), size); }

            auto end()
            { return cursol(this, std::end(source), size); }
        };

        struct IndicesStream {
            int64_t begin_, end_;

            struct cursol {
                int64_t value;

                cursol() = default;

                cursol(int64_t v): value(v)
                { }

                int64_t operator*()
                { return value; }

                cursol operator++()
                { ++value; return *this; }

                bool operator!=(const cursol &rhs)
                { return value != rhs.value; }
            };

            IndicesStream(int64_t begin, int64_t end): begin_(begin), end_(end)
            { }

            IndicesStream(int64_t end): IndicesStream(0, end)
            { }

            auto begin()
            { return cursol(begin_); }

            auto end()
            { return cursol(end_); }
        };
    }

    template<typename Func>
    struct map {
        Func func;

        map(Func &&f): func(f)
        { }

        template<typename Source>
        friend auto operator|(Source &&s, map<Func> &&m)
        { return internal::MapStream(internal::wrap(std::forward<Source>(s)), m.func); }
    };

    template<typename Pred>
    struct filter {
        Pred pred;

        filter(Pred p): pred(p)
        { }

        template<typename Source>
        friend auto operator|(Source &&s, filter<Pred> &&f)
        { return internal::FilterStream(internal::wrap(std::forward<Source>(s)), f.pred); }
    };

    struct flatten {
        template<typename Source>
        friend auto operator|(Source &&s, flatten)
        { return internal::FlattenStream(internal::wrap(std::forward<Source>(s))); }
    };

    struct grouped {
        size_t size;

        grouped(size_t n): size(n)
        { }

        template<typename Source>
        friend auto operator|(Source &&s, grouped &&g)
        { return internal::GroupedStream(internal::wrap(std::forward<Source>(s)), g.size); }
    };

    struct indices : public internal::IndicesStream {
        using IndicesStream::IndicesStream;
    };

    struct to_vector {
        template<typename Source>
        friend auto operator|(Source &&s, to_vector)
        {
            std::vector<element_t<Source>> res;
            for (auto &&e : s) res.push_back(e);
            return res;
        }
    };
}

namespace option {
    enum class Arrow {
        POSE, VECTOR,
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
    using traits::get;
    using traits::Member;

    struct Vector3 {
        geometry_msgs::Vector3 value;

        Vector3() = default;

        Vector3(const geometry_msgs::Vector3 arg) noexcept : value(arg)
        { }

        Vector3(const double x, const double y, const double z = 0.0) noexcept
        { value.x = x, value.y = y, value.z = z; }

        template<typename T>
        Vector3(const T &arg) noexcept : Vector3(get<Member::X>(arg), get<Member::Y>(arg), get<Member::Z>(arg))
        { }

        [[nodiscard]] static inline Vector3 UnitX() noexcept
        { return { 1, 0, 0 }; }
        [[nodiscard]] static inline Vector3 UnitY() noexcept
        { return { 0, 1, 0 }; }
        [[nodiscard]] static inline Vector3 UnitZ() noexcept
        { return { 0, 0, 1 }; }

        template<typename T>
        [[nodiscard]] static Vector3 from_2d(const T &point, const double z = 0.0) noexcept
        { return { get<Member::X>(point), get<Member::Y>(point), z }; }

        const geometry_msgs::Vector3 &to_vector3_msg() const noexcept
        { return value; }

        geometry_msgs::Point to_point_msg() const noexcept
        {
            geometry_msgs::Point res;
            res.x = x(), res.y = y(), res.z = z();
            return res;
        }

        double &x()
        { return value.x; }
        const double &x() const
        { return value.x; }

        double &y()
        { return value.y; }
        const double &y() const
        { return value.y; }

        double &z()
        { return value.z; }
        const double &z() const
        { return value.z; }

        Vector3 operator+(const Vector3 rhs) const noexcept
        { return { x() + rhs.x(), y() + rhs.y(), z() + rhs.z() }; }

        Vector3 operator-() const noexcept
        { return { -x(), -y(), -z() }; }

        Vector3 operator-(const Vector3 rhs) const noexcept
        { return { x() - rhs.x(), y() - rhs.y(), z() - rhs.z() }; }

        Vector3 operator*(const double rhs) const noexcept
        { return { rhs * x(), rhs * y(), rhs * z() }; }

        friend Vector3 operator*(const double lhs, const Vector3 rhs) noexcept
        { return rhs * lhs; }

        Vector3 operator/(const double rhs) const noexcept
        { return { x() / rhs, y() / rhs, z() / rhs }; }

        double dot(const Vector3 rhs) const noexcept
        { return x() * rhs.x() + y() * rhs.y() + z() * rhs.z(); }

        Vector3 cross(const Vector3 rhs) const noexcept
        { return { y() * rhs.z() - z() * rhs.y(), z() * rhs.x() - x() * rhs.z(), x() * rhs.y() - y() * rhs.x() }; }

        Vector3 hadamard_prod(const Vector3 rhs) const noexcept
        { return { x() * rhs.x(), y() * rhs.y(), z() * rhs.z() }; }
    };

    struct Quaternion {
        geometry_msgs::Quaternion value;

        Quaternion() = default;

        Quaternion(const geometry_msgs::Quaternion arg) noexcept : value(arg)
        { }

        Quaternion(const double x, const double y, const double z, const double w) noexcept
        { value.x = x, value.y = y, value.z = z, value.z = w; }

        template<typename T>
        Quaternion(const T &arg) noexcept : Quaternion(get<Member::X>(arg), get<Member::Y>(arg), get<Member::Z>(arg), get<Member::W>(arg)) { }

        [[nodiscard]] static Quaternion from_angle_axis(const double theta, const Vector3 axis = Vector3::UnitZ()) noexcept
        { return { axis.x() * std::sin(theta / 2), axis.y() * std::sin(theta / 2), axis.z() * std::sin(theta / 2), std::cos(theta / 2) }; }

        [[nodiscard]] static Quaternion from_vector_scalar(const Vector3 vector, const double scalar) noexcept
        { return { vector.x(), vector.y(), vector.z(), scalar }; }

        const geometry_msgs::Quaternion &to_quaternion_msg() const noexcept
        { return value; }

        double &x()
        { return value.x; }
        const double &x() const
        { return value.x; }

        double &y()
        { return value.y; }
        const double &y() const
        { return value.y; }

        double &z()
        { return value.z; }
        const double &z() const
        { return value.z; }

        double &w()
        { return value.w; }
        const double &w() const
        { return value.w; }

        Vector3 vector() const noexcept
        { return { x(), y(), z() }; }

        double scalar() const noexcept
        { return w(); }

        double theta() const noexcept
        { return std::acos(scalar()) * 2; }

        Vector3 axis() const noexcept
        { return vector() / std::sin(theta() / 2); }

        Quaternion conjugate() const noexcept
        { return { -x(), -y(), -z(), w() }; }

        double norm() const noexcept
        { return std::sqrt(x() * x() + y() * y() + z() * z() + w() * w()); }

        Quaternion inverse() const noexcept
        { return conjugate() / (norm() * norm()); }

        Vector3 rotate_vector(const Vector3 vector) const noexcept
        { return ((*this) * vector * (*this).inverse()).vector(); }

        Quaternion operator+(const Quaternion rhs) const noexcept
        { return { x() + rhs.x(), y() + rhs.y(), z() + rhs.z(), w() + rhs.w() }; }

        Quaternion operator-() const noexcept
        { return { -x(), -y(), -z(), -w() }; }

        Quaternion operator-(const Quaternion rhs) const noexcept
        { return { x() - rhs.x(), y() - rhs.y(), z() - rhs.z(), w() - rhs.w() }; }

        Quaternion operator*(const Quaternion rhs) const noexcept
        {
            Vector3 vec = scalar() * rhs.vector() + rhs.scalar() * vector() + vector().cross(rhs.vector());
            return { vec.x(), vec.y(), vec.z(), scalar() * rhs.scalar() - vector().dot(rhs.vector()) };
        }

        Quaternion operator*(const Vector3 rhs) const noexcept
        { return *this * Quaternion::from_vector_scalar(rhs, 0); }

        friend Quaternion operator*(const Vector3 lhs, const Quaternion rhs) noexcept
        { return rhs * lhs; }

        Quaternion operator*(const double rhs) const noexcept
        { return { rhs * x(), rhs * y(), rhs * z(), rhs * w() }; }

        friend Quaternion operator*(const double lhs, const Quaternion rhs) noexcept
        { return rhs * lhs; }

        Quaternion operator/(const double rhs) const noexcept
        { return { x() / rhs, y() / rhs, z() / rhs, w() / rhs }; }
    };

    struct Color {
        std_msgs::ColorRGBA value;

        Color() = default;

        Color(const std_msgs::ColorRGBA arg) noexcept : value(arg)
        { }

        Color(const float r, const float g, const float b, const float a = 1.0) noexcept
        { value.r = r, value.g = g, value.b = b, value.a = a; }

        [[nodiscard]] static Color from_hex(const int32_t hex) noexcept
        { return { ((hex >> 16) & 0xff) / 255.0f, ((hex >> 8) & 0xff) / 255.0f, (hex & 0xff) / 255.0f, }; }

        [[nodiscard]] Color &alpha(const float a) noexcept
        {
            value.a = a;
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

        const std_msgs::ColorRGBA &to_color_msg() const noexcept
        { return value; }
    };

    template<typename Source>
    class PointsFragment {
        Source &source;
        Quaternion rotation;
        Vector3 offset;
        Vector3 extent;

        class cursol {
            PointsFragment *parent;
            stream::iterator_t<Source> itr;

        public:
            cursol(PointsFragment *p, iterator_t<Source> i): parent(p), itr(i)
            { }

            Vector3 operator*()
            { return parent->rotation.rotate_vector(Vector3(*itr).hadamard_prod(extent)) + parent->offset; }

            cursol &operator++()
            { ++itr; return *this; }

            bool operator!=(const cursol &rhs)
            { return itr != rhs.itr; }
        };

    public:
        PointsFragment(Source &s): source(s), rotation(0, 0, 0, 1), extent(1, 1, 1)
        { }

        PointsFragment &&orientaion(const Quaternion orientaion) && noexcept
        { rotation = orientaion; return std::move(*this); }

        PointsFragment &&orientaion(double x, double y, double z, double w) && noexcept
        { return std::move(*this).orientaion({ x, y, z, w }); }

        PointsFragment &&position(const Vector3 position) && noexcept
        { offset = position; return std::move(*this); }

        PointsFragment &&position(double x, double y, double z) && noexcept
        { return std::move(*this).position({ x, y, z }); }

        PointsFragment &&scale(double x, double y, double z) && noexcept
        { extent = { x, y, z }; return *this; }

        auto begin()
        { return cursol(this, std::begin(source)); }

        auto end()
        { return cursol(this, std::end(source)); }
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

        [[nodiscard]] T &&position(const param::Vector3 position) && noexcept
        {
            FLRV_DERIVED(T).msg().pose.position = position.to_point_msg();
            return std::move(FLRV_DERIVED(T));
        }
    };

    template<typename Marker, int32_t MarkerType, typename Option, typename Enable = void>
    struct PositionEnabler : PositionHelper<Marker> { };

    template<typename T>
    struct OrientationHelper {
        OrientationHelper()
        { FLRV_SUPPRESS(std::move(*this).orientation(0, 0, 0, 1)); }

        [[nodiscard]] T &&orientation(const double x, const double y, const double z, const double w) && noexcept
        { return std::move(*this).orientation({ x, y, z, w }); }

        [[nodiscard]] T &&orientation(const param::Quaternion orientation) && noexcept
        {
            FLRV_DERIVED(T).msg().pose.orientation = orientation.to_quaternion_msg();
            return std::move(FLRV_DERIVED(T));
        }
    };

    template<typename Marker, int32_t MarkerType, typename Option, typename Enable = void>
    struct OrientationEnabler : OrientationHelper<Marker> { };

    template<typename Marker, int32_t MarkerType, typename Option>
    struct OrientationEnabler<Marker, MarkerType, Option,
        std::enable_if_t<is_text_view_facing_marker_v<MarkerType>>> { };

    template<typename T>
    struct ScaleHelper {
        ScaleHelper()
        { FLRV_SUPPRESS(std::move(*this).scale(1, 1, 1)); }

        [[nodiscard]] T &&scale(const double x, const double y, const double z) && noexcept
        {
            FLRV_DERIVED(T).msg().scale = param::Vector3(x, y, z).to_vector3_msg();
            return std::move(FLRV_DERIVED(T));
        }
    };

    template<typename T>
    struct PoseArrowScaleHelper : ScaleHelper<T> {
        PoseArrowScaleHelper()
        { FLRV_SUPPRESS(std::move(*this).scale(0.05, 0.05)); }

        [[nodiscard]] T &&scale(const double length, const double width, const double height) && noexcept
        { return std::move(*this).ScaleHelper<T>::scale(length, width, height); }
    };

    template<typename T>
    struct VectorArrowScaleHelper : ScaleHelper<T> {
        VectorArrowScaleHelper()
        { FLRV_SUPPRESS(std::move(*this).scale(0.2, 0.4, 0.4)); }

        [[nodiscard]] T &&scale(const double shaft_diameter, const double head_diameter, const double head_length) && noexcept
        { return std::move(*this).ScaleHelper<T>::scale(shaft_diameter, head_diameter, head_length); }
    };

    template<typename T>
    struct PointScaleHelper : ScaleHelper<T> {
        PointScaleHelper()
        { FLRV_SUPPRESS(std::move(*this).scale(0.05, 0.05)); }

        [[nodiscard]] T &&scale(const double width, const double height) && noexcept
        { return std::move(*this).ScaleHelper<T>::scale(width, height, 0); }
    };

    template<typename T>
    struct LineScaleHelper : ScaleHelper<T> {
        LineScaleHelper()
        { FLRV_SUPPRESS(std::move(*this).scale(0.05)); }

        [[nodiscard]] T &&scale(const double width) && noexcept
        { return std::move(*this).ScaleHelper<T>::scale(width, 0, 0); }
    };

    template<typename T>
    struct TextScaleHelper : ScaleHelper<T> {
        TextScaleHelper()
        { FLRV_SUPPRESS(std::move(*this).scale(1)); }

        [[nodiscard]] T &&scale(const double height) && noexcept
        { return std::move(*this).ScaleHelper<T>::scale(0, 0, height); }
    };

    template<typename Marker, int32_t MarkerType, typename Option, typename Enable = void>
    struct ScaleEnabler { };

    template<typename Marker, int32_t MarkerType, typename Option>
    struct ScaleEnabler<Marker, MarkerType, Option,
        std::enable_if_t<is_common_scale_available_v<MarkerType>>>
        : ScaleHelper<Marker> { };

    template<typename Marker, int32_t MarkerType, auto... Options>
    struct ScaleEnabler<Marker, MarkerType, OptionPack<Options...>,
        std::enable_if_t<is_arrow_marker_v<MarkerType> && is_contained_v<option::Arrow::POSE, Options...>>>
        : PoseArrowScaleHelper<Marker> { };

    template<typename Marker, int32_t MarkerType, auto... Options>
    struct ScaleEnabler<Marker, MarkerType, OptionPack<Options...>,
        std::enable_if_t<is_arrow_marker_v<MarkerType> && is_contained_v<option::Arrow::VECTOR, Options...>>>
        : VectorArrowScaleHelper<Marker> { };

    template<typename Marker, int32_t MarkerType, typename Option>
    struct ScaleEnabler<Marker, MarkerType, Option,
        std::enable_if_t<is_points_marker_v<MarkerType>>>
        : PointScaleHelper<Marker> { };

    template<typename Marker, int32_t MarkerType, typename Option>
    struct ScaleEnabler<Marker, MarkerType, Option,
        std::enable_if_t<is_line_marker_v<MarkerType>>>
        : LineScaleHelper<Marker> { };

    template<typename Marker, int32_t MarkerType, typename Option>
    struct ScaleEnabler<Marker, MarkerType, Option,
        std::enable_if_t<is_text_view_facing_marker_v<MarkerType>>>
        : TextScaleHelper<Marker> { };

    template<typename T>
    struct ColorHelper {
        ColorHelper()
        { FLRV_SUPPRESS(std::move(*this).color(1, 1, 1)); }

        [[nodiscard]] T &&color(const float r, const float g, const float b, const float a = 1.0) && noexcept
        { return std::move(*this).color({ r, g, b, a }); }

        [[nodiscard]] T &&color(const param::Color color) && noexcept
        {
            FLRV_DERIVED(T).msg().color = color.to_color_msg();
            return std::move(FLRV_DERIVED(T));
        }
    };

    template<typename T>
    struct ColorsHelper {
        ColorsHelper()
        { FLRV_SUPPRESS(std::move(*this).color(1, 1, 1)); }

        [[nodiscard]] T &&color(const float r, const float g, const float b, const float a = 1.0) && noexcept
        { return std::move(*this).color({ r, g, b, a }); }

        [[nodiscard]] T &&color(const param::Color color) && noexcept
        {
            FLRV_DERIVED(T).msg().color = color.to_color_msg();
            FLRV_DERIVED(T).msg().colors.clear();
            return std::move(FLRV_DERIVED(T));
        }

        [[nodiscard]] T &&color(std::vector<std_msgs::ColorRGBA> colors) && noexcept
        {
            FLRV_DERIVED(T).msg().colors = std::move(colors);
            return std::move(FLRV_DERIVED(T));
        }

        [[nodiscard]] T &&color(std::vector<param::Color> colors) && noexcept
        {
            return std::move(*this).color(colors | stream::map([](auto &e) { return e.to_color_msg(); }) | stream::to_vector());
        }
    };

    template<typename Marker, int32_t MarkerType, typename Option, typename Enable = void>
    struct ColorEnabler : ColorHelper<Marker> { };

    template<typename Marker, int32_t MarkerType, typename Option>
    struct ColorEnabler<Marker, MarkerType, Option,
        std::enable_if_t<is_colors_available_v<MarkerType>>>
        : ColorsHelper<Marker> { };

    template<typename T>
    struct LifetimeHelper {
        [[nodiscard]] T &&lifetime(const double lifetime) && noexcept
        {
            FLRV_DERIVED(T).msg().lifetime = ros::Duration(lifetime);
            return std::move(FLRV_DERIVED(T));
        }
    };

    template<typename T>
    struct FrameLockedHelper {
        [[nodiscard]] T &&frame_locked(const bool frame_locked) && noexcept
        {
            FLRV_DERIVED(T).msg().frame_locked = frame_locked;
            return std::move(FLRV_DERIVED(T));
        }
    };

    template<typename T>
    struct PointsHelper {
        [[nodiscard]] T &&points(std::vector<geometry_msgs::Point> points) && noexcept
        {
            FLRV_DERIVED(T).msg().points = std::move(points);
            return std::move(FLRV_DERIVED(T));
        }

        template<typename PointsLike>
        [[nodiscard]] T &&points(PointsLike points) && noexcept
        {
            return std::move(*this).points(points
                | stream::map([](auto &&e) { return param::Vector3(e).to_point_msg(); })
                | stream::to_vector());
        }
    };

    template<typename T>
    struct ArrowPointsHelper {
        ArrowPointsHelper()
        {
            FLRV_DERIVED(T).msg().points.resize(2);
            std::move(*this).end(1, 0, 0);
        }

        [[nodiscard]] T &&start(const double x, const double y, const double z = 0.0) && noexcept
        { return std::move(*this).start({ x, y, z }); }

        [[nodiscard]] T &&start(const param::Vector3 point) && noexcept
        { return set<0>(point); }

        [[nodiscard]] T &&end(const double x, const double y, const double z = 0.0) && noexcept
        { return std::move(*this).end({ x, y, z }); }

        [[nodiscard]] T &&end(const param::Vector3 point) && noexcept
        { return set<1>(point); }

    private:
        template<size_t Index>
        [[nodiscard]] T &&set(const param::Vector3 &point) noexcept
        {
            FLRV_DERIVED(T).msg().points[Index] = point.to_point_msg();
            return std::move(FLRV_DERIVED(T));
        }
    };

    template<typename Marker, int32_t MarkerType, typename Option, typename Enable = void>
    struct PointsEnabler { };

    template<typename Marker, int32_t MarkerType, auto... Options>
    struct PointsEnabler<Marker, MarkerType, OptionPack<Options...>,
        std::enable_if_t<is_points_available_v<MarkerType>>>
        : PointsHelper<Marker> { };

    template<typename Marker, int32_t MarkerType, auto... Options>
    struct PointsEnabler<Marker, MarkerType, OptionPack<Options...>,
        std::enable_if_t<is_arrow_marker_v<MarkerType> && is_contained_v<option::Arrow::VECTOR, Options...>>>
        : ArrowPointsHelper<Marker> { };

    template<typename T>
    struct TextHelper {
        TextHelper()
        { FLRV_SUPPRESS(std::move(*this).text("visualization_msgs::Marker::TEXT_VIEW_FACING")); }

        [[nodiscard]] T &&text(std::string text) && noexcept
        {
            FLRV_DERIVED(T).msg().text = std::move(text);
            return std::move(FLRV_DERIVED(T));
        }
    };

    template<typename Marker, int32_t MarkerType, typename Option, typename Enable = void>
    struct TextEnabler { };

    template<typename Marker, int32_t MarkerType, typename Option>
    struct TextEnabler<Marker, MarkerType, Option,
        std::enable_if_t<is_text_view_facing_marker_v<MarkerType>>>
        : TextHelper<Marker> { };

    template<typename T>
    struct MeshResourceHelper {
        [[nodiscard]] T &&mesh_resource(std::string mesh_resource) && noexcept
        {
            FLRV_DERIVED(T).msg().mesh_resource = std::move(mesh_resource);
            return std::move(FLRV_DERIVED(T));
        }

        [[nodiscard]] T &&mesh_use_embedded_materials(const bool mesh_use_embedded_materials) && noexcept
        {
            FLRV_DERIVED(T).msg().mesh_use_embedded_materials = mesh_use_embedded_materials;
            return std::move(FLRV_DERIVED(T));
        }
    };

    template<typename Marker, int32_t MarkerType, typename Option, typename Enable = void>
    struct MeshResourceEnabler { };

    template<typename Marker, int32_t MarkerType, auto... Options>
    struct MeshResourceEnabler<Marker, MarkerType, OptionPack<Options...>,
        std::enable_if_t<is_mesh_resource_marker_v<MarkerType>>>
        : MeshResourceHelper<Marker> { };
} // namespace internal

namespace marker {
    template<int32_t Action>
    class MarkerBase {
        visualization_msgs::Marker marker;

    public:
        MarkerBase()
        { marker.action = Action; }

        MarkerBase(const MarkerBase &) = delete;
        MarkerBase(MarkerBase &&) = delete;

        [[nodiscard]] visualization_msgs::Marker &msg() noexcept
        { return marker; }
    };

    template<int32_t MarkerType, auto... Options>
    class Marker
        : public MarkerBase<visualization_msgs::Marker::ADD>
        , public internal::PositionEnabler<Marker<MarkerType, Options...>, MarkerType, internal::OptionPack<Options...>>
        , public internal::OrientationEnabler<Marker<MarkerType, Options...>, MarkerType, internal::OptionPack<Options...>>
        , public internal::ScaleEnabler<Marker<MarkerType, Options...>, MarkerType, internal::OptionPack<Options...>>
        , public internal::ColorEnabler<Marker<MarkerType, Options...>, MarkerType, internal::OptionPack<Options...>>
        , public internal::LifetimeHelper<Marker<MarkerType, Options...>>
        , public internal::FrameLockedHelper<Marker<MarkerType, Options...>>
        , public internal::PointsEnabler<Marker<MarkerType, Options...>, MarkerType, internal::OptionPack<Options...>>
        , public internal::TextEnabler<Marker<MarkerType, Options...>, MarkerType, internal::OptionPack<Options...>>
        , public internal::MeshResourceEnabler<Marker<MarkerType, Options...>, MarkerType, internal::OptionPack<Options...>> {

    public:
        Marker(int32_t id, std::string ns = "") noexcept
        { msg().id = id, msg().ns = std::move(ns), msg().type = MarkerType; }
    };

    using PoseArrow = Marker<visualization_msgs::Marker::ARROW, option::Arrow::POSE>;
    using VectorArrow = Marker<visualization_msgs::Marker::ARROW, option::Arrow::VECTOR>;
    using Arrow = VectorArrow;
    using Cube = Marker<visualization_msgs::Marker::CUBE>;
    using Sphere = Marker<visualization_msgs::Marker::SPHERE>;
    using Cylinder = Marker<visualization_msgs::Marker::CYLINDER>;
    using LineStrip = Marker<visualization_msgs::Marker::LINE_STRIP>;
    using LineList = Marker<visualization_msgs::Marker::LINE_LIST>;
    using CubeList = Marker<visualization_msgs::Marker::CUBE_LIST>;
    using SphereList = Marker<visualization_msgs::Marker::SPHERE_LIST>;
    using Points = Marker<visualization_msgs::Marker::POINTS>;
    using TextViewFacing = Marker<visualization_msgs::Marker::TEXT_VIEW_FACING>;
    using MeshResource = Marker<visualization_msgs::Marker::MESH_RESOURCE>;
    using TriangleList = Marker<visualization_msgs::Marker::TRIANGLE_LIST>;

    class Delete : public MarkerBase<visualization_msgs::Marker::DELETE> {
    public:
        Delete(int32_t id, std::string ns = "")
        { msg().id = id, msg().ns = std::move(ns); }
    };

    class DeleteAll : public MarkerBase<visualization_msgs::Marker::DELETEALL> { };
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

} // namespace flrv
