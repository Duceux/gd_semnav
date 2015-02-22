#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <sn_geometry/point.h>
#include <sn_msgs/BoundingBox.h>

namespace sn {

point_t compute_center(const vector_pts_t& points);

}

#endif // GEOMETRY_H
