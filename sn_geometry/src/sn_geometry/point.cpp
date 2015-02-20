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

point_t ominus(const pose_t &pose, const point_t &point)
{
    double ctheta = cos(pose.theta), stheta = sin(pose.theta);
    return create((point.x - pose.x) * ctheta + (point.y - pose.y) * stheta,
                  -(point.x - pose.x) * stheta + (point.y - pose.y) * ctheta, point.z);
}

pose_t create_pose(double x, double y, double z)
{
    pose_t res; res.x=x; res.y=y; res.theta=z;
    return res;
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
