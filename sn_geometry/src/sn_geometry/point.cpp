#include <sn_geometry/point.h>

namespace sn {

double l2_norm(const point_t& pt){
  return std::sqrt(pt.x*pt.x+pt.y*pt.y+pt.z*pt.z);
}

template<typename F>
point_t generic_operator(point_t const& a, double s, F func){
  return create(func(a.x,s), func(a.y,s), func(a.z,s));
}

template<typename F>
point_t generic_operator(const point_t &a, const point_t &b, F func)
{
  return create(func(a.x, b.x), func(a.y, b.y), func(a.z, b.z));
}

point_t create(double x, double y, double z)
{
  point_t r; r.x=x; r.y=y; r.z=z;
  return r;
}


pose_t create_pose(double x, double y, double z)
{
  pose_t res; res.x=x; res.y=y; res.theta=z;
  return res;
}

pose_t ominus(const pose_t &pose)
{
  double ctheta = cos(pose.theta), stheta = sin(pose.theta);
  return create_pose(-pose.x * ctheta - pose.y * stheta,
                pose.x * stheta - pose.y * ctheta,
                -pose.theta);
}

pose_t ominus(const pose_t &pose, const pose_t& point)
{
  double ctheta = cos(pose.theta), stheta = sin(pose.theta);
  return create_pose((point.x - pose.x) * ctheta + (point.y - pose.y) * stheta,
                -(point.x - pose.x) * stheta + (point.y - pose.y) * ctheta,
                point.theta - pose.theta);

}

point_t ominus(const pose_t &pose, const point_t &point)
{
  double ctheta = cos(pose.theta), stheta = sin(pose.theta);
  return create((point.x - pose.x) * ctheta + (point.y - pose.y) * stheta,
                -(point.x - pose.x) * stheta + (point.y - pose.y) * ctheta,
                point.z);
}


pose_t oplus(const pose_t &pose, const pose_t& point)
{
  double ctheta = cos(pose.theta), stheta = sin(pose.theta);
  return create_pose(pose.x + point.x * ctheta - point.y * stheta,
                pose.y + point.x * stheta + point.y * ctheta,
                pose.theta + point.theta);
}

point_t oplus(const pose_t &pose, const point_t& point)
{
  double ctheta = cos(pose.theta), stheta = sin(pose.theta);
  return create(pose.x + point.x * ctheta - point.y * stheta,
                pose.y + point.x * stheta + point.y * ctheta,
                point.z);
}

}//namespace sn

sn::point_t operator -(const sn::point_t &a, const sn::point_t &b){
  return sn::generic_operator(a, b, [](double x, double y){return x-y;});
}

sn::point_t operator +(const sn::point_t &a, const sn::point_t &b){
  return sn::generic_operator(a, b, [](double x, double y){return x+y;});
}

sn::point_t operator /(sn::point_t const& a, double b){
  return sn::generic_operator(a, b, [](double x, double y){return x/y;});
}

sn::point_t operator *(sn::point_t const& a, double b){
  return sn::generic_operator(a, b, [](double x, double y){return x*y;});
}


sn::point_t &operator +=(sn::point_t &a, const sn::point_t &b){
  a.x+=b.x; a.y+=b.y; a.z+=b.z;
  return a;
}

sn::point_t &operator /=(sn::point_t &a, double b){
  a.x/=b; a.y/=b; a.z/=b;
  return a;
}

sn::point_t operator -(sn::point_t const& a){
  return sn::create(-a.x, -a.y, -a.z);
}


sn::point_t operator -(const sn::pose_t &a, const sn::pose_t &b)
{
  return sn::create(a.x-b.x, a.y-b.y, 0.0);
}
