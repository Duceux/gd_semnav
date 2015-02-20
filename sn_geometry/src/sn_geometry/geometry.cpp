#include <sn_geometry/geometry.h>

namespace sn {

point_t compute_center(const vector_pts_t& points)
{
    point_t center;
    for(point_t p: points)
        center+=p;
    center/=points.size();
    return center;
}

}
