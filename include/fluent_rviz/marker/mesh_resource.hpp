#pragma once

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_base.hpp"
#include "fluent_rviz/marker/marker_wrapper.hpp"

namespace flrv::marker
{
template <typename MarkerToken = MarkerWrapper>
struct MeshResource : public MarkerBase<MarkerToken, MeshResource<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, MeshResource<MarkerToken>>;

public:
  MeshResource()
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
}  // namespace flrv::marker
