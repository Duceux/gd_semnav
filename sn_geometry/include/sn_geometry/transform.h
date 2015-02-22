#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <sn_geometry/point.h>
#include <tf/tf.h>
#include <sn_msgs/BoundingBox.h>
#include <sensor_msgs/PointCloud2.h>

namespace sn {

    point_t transform(const point_t& pt, const tf::StampedTransform& tf);
    vector_pts_t transform(const vector_pts_t& pts, const tf::StampedTransform& tf);
    vector_pts_t transform(const vector_pts_t& pts, const pose_t& tf);

    void transform(sn_msgs::BoundingBox const& in, sn_msgs::BoundingBox & out,
              tf::StampedTransform const& tf);

    void transform(sensor_msgs::PointCloud2 const& in, sensor_msgs::PointCloud2 & out,
                   tf::StampedTransform const& tf);
}

#endif // TRANSFORM_H
