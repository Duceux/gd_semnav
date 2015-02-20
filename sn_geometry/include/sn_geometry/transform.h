#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <sn_geometry/point.h>
#include <tf/tf.h>

namespace sn {

    point_t transform(const point_t& pt, const tf::StampedTransform& tf);
    vector_pts_t transform(const vector_pts_t& pts, const tf::StampedTransform& tf);
    vector_pts_t transform(const vector_pts_t& pts, const pose_t& tf);

}

#endif // TRANSFORM_H
