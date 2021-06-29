#include "FluentRviz.hpp"

int main()
{
    auto cube = flrv::CubeMarker(0)
        .position(0, 0, 0)
        .orientation(0, 0, 0, 0)
        .color(0, 0, 0);
    return 0;
}
