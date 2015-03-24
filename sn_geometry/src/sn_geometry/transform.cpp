#include <sn_geometry/transform.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/common/transforms.h>

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

void transform(const sn_msgs::BoundingBox &in, sn_msgs::BoundingBox &out, const tf::StampedTransform &tf)
{
    out.center = transform(in.center, tf);
    out.min = transform(in.min, tf);
    out.max = transform(in.max, tf);
}

void transform(const sensor_msgs::PointCloud2 &in, sensor_msgs::PointCloud2 &out, const tf::StampedTransform &tf)
{
    if(in.data.size() == 0)return;
    pcl::PointCloud<pcl::PointXYZRGB> in_cloud, out_cloud;
    pcl::fromROSMsg(in, in_cloud);
    Eigen::Affine3d etf;
    tf::transformTFToEigen(tf, etf);
    pcl::transformPointCloud(in_cloud, out_cloud, etf);
    pcl::toROSMsg(out_cloud, out);
}

pose_6d_t poseToPose6D(const pose_t &pose)
{
  pose_6d_t result;
  result.position.x = pose.x;
  result.position.y = pose.y;
  result.position.z = 0.0;
  tf::Quaternion tmp(tf::Vector3(0,0,1), pose.theta);
  result.orientation.x = tmp.x();
  result.orientation.y = tmp.y();
  result.orientation.z = tmp.z();
  result.orientation.w = tmp.w();
  return result;
}

}
