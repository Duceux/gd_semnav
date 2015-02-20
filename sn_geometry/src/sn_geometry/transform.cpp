#include <sn_geometry/transform.h>

namespace sn {

point_t transform(const point_t &pt, const tf::StampedTransform &tf)
{
    tf::Vector3 tmp = tf*tf::Vector3(pt.x, pt.y, pt.z);
    point_t res;
    res.x = tmp[0];
    res.y = tmp[1];
    res.z = tmp[2];
    return res;
}

vector_pts_t transform(const vector_pts_t &pts, const tf::StampedTransform &tf)
{
    vector_pts_t res;
    res.reserve(pts.size());
    for(point_t p: pts)
        res.push_back(transform(p, tf));
    return res;
}

vector_pts_t transform(const vector_pts_t& pts, const pose_t& tf){
    vector_pts_t res;
    res.reserve(pts.size());
    for(point_t p: pts)
        res.push_back(ominus(tf, p));
    return res;
}

}
