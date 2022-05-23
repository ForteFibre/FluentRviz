// Copyright (c) 2022 ForteFibre
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

#ifdef __GNUC__
#define COMPOSITE_EXPORT __attribute__((dllexport))
#define COMPOSITE_IMPORT __attribute__((dllimport))
#else
#define COMPOSITE_EXPORT __declspec(dllexport)
#define COMPOSITE_IMPORT __declspec(dllimport)
#endif

#ifdef COMPOSITE_DLL
#define COMPOSITE_PUBLIC COMPOSITE_EXPORT
#else
#define COMPOSITE_PUBLIC COMPOSITE_IMPORT
#endif

#define COMPOSITE_PUBLIC_TYPE COMPOSITE_PUBLIC

#define COMPOSITE_LOCAL

#else

#define COMPOSITE_EXPORT __attribute__((visibility("default")))
#define COMPOSITE_IMPORT

#if __GNUC__ >= 4
#define COMPOSITE_PUBLIC __attribute__((visibility("default")))
#define COMPOSITE_LOCAL __attribute__((visibility("hidden")))
#else
#define COMPOSITE_PUBLIC
#define COMPOSITE_LOCAL
#endif

#define COMPOSITE_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#include <fluent_rviz/fluent_rviz.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

class FlrvExample : public rclcpp::Node
{
public:
  COMPOSITE_PUBLIC explicit FlrvExample(const rclcpp::NodeOptions options)
  : rclcpp::Node("flrv_example_node", options)
  {
    rviz_.init(*this);
    using std::literals::chrono_literals::operator""ms;
    timer_ = create_wall_timer(500ms, std::bind(&FlrvExample::timerCallback, this));
  }
  void timerCallback()
  {
    using namespace flrv;
    using namespace flrv::param;
    // clang-format off
    rviz_ << marker::Cube(0)
               .position(0, 0, 0)
               .orientation(Quaternion::AngleAxis(M_PI / 2))
               .color(Color::Aqua());

    rviz_ << marker::LineList(2)
               .color(0, 0, 1)
               .scale(0.05)
               .points({{1, 0, 0}, {0, 1, 0}});
    // clang-format on
    RCLCPP_INFO(get_logger(), "publish!");
  }

private:
  flrv::Rviz rviz_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlrvExample>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
