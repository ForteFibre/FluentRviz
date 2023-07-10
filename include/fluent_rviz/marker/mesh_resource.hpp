#pragma once

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_composition.hpp"
#include "fluent_rviz/marker/marker_property_base.hpp"
#include "fluent_rviz/marker/temporal_marker.hpp"

namespace flrv::marker
{
template <typename Derived>
struct MeshResourceProperty : public MarkerPropertyBase<Derived>
{
private:
  using Base = MarkerPropertyBase<Derived>;

public:
  MeshResourceProperty()
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::ADD)
      .type(visualization_msgs::msg::Marker::MESH_RESOURCE);
  }

  using Base::ns;
  using Base::id;
  using Base::pose;
  using Base::scale;
  using Base::color;
  using Base::lifetime;
  using Base::frame_locked;
  using Base::mesh_resource;
  using Base::mesh_file;
  using Base::mesh_use_embedded_materials;
};

template <typename MarkerToken = UseTemporal>
auto MeshResource(MarkerToken && token = MarkerToken{})
{
  return compose_marker<MeshResourceProperty>(std::forward<MarkerToken>(token));
}
}  // namespace flrv::marker
