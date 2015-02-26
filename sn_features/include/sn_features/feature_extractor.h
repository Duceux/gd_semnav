#ifndef FEATURE_EXTRACTOR_H
#define FEATURE_EXTRACTOR_H

#include <sn_features/types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sn_geometry/sn_geometry.h>
#include <sn_features/polar_histogram.h>

namespace sn {

void l1_normalize(feature_t & data_);

void l2_normalize(feature_t & data_);

void inf_normalize(feature_t & data_);

void normalize(feature_t & data_, double norm);

struct Extractor{
  std::string type;
  virtual void set_params(std::map<std::string, std::string> const& params){
    type = params.at("type");
  }
  virtual descriptor_t operator ()(detection_t const& det){
    descriptor_t res;
    res.robot = det.robot;
    res.header = det.header;
    res.type = type;
    return res;
  }

  virtual bool is_valid(detection_t const& det)const{return false;}

  typedef std::shared_ptr<Extractor> Ptr;
};

struct SizeExtractor: public Extractor{
  descriptor_t operator ()(detection_t const& det){
    descriptor_t res = Extractor::operator ()(det);
    res.data.push_back(l2_norm(det.points.front()-det.points.back()));
    return res;
  }

  bool is_valid(detection_t const& det)const{return det.points.size()>0;}

};

struct ColorExtractor: public Extractor{
  descriptor_t operator ()(detection_t const& det){
    descriptor_t res = Extractor::operator ()(det);

    pcl::PointCloud<pcl::RGB> cloud;
    pcl::fromROSMsg(det.cloud, cloud);

    double r = 0.0;
    double g = 0.0;
    double b = 0.0;
    for(auto color: cloud){
      r+=color.r;
      g+=color.g;
      b+=color.b;
    }
    r/=cloud.size();
    g/=cloud.size();
    b/=cloud.size();
    res.data.push_back(r);
    res.data.push_back(g);
    res.data.push_back(b);
    l2_normalize(res.data);
    return res;
  }

  bool is_valid(detection_t const& det)const{return det.cloud.data.size()>0;}

};

struct ColorTriangleExtractor: public Extractor{
  unsigned int nb_triangles;
  double theta_bin_size;
  double rho_bin_size;
  Histogram2D histogram;


  point_t fromRGB(pcl::RGB const& point){
    const float r = point.r;
    const float g = point.g;
    const float b = point.b;
    const float f1 = ((r - g) / M_SQRT2 + 127.f);
    const float f2 = ((r+g - 2.f*b) / sqrt(6.f) + 127.f);
    if(f1 < -100.0 || f1 > 300.0 || f2 < -100.0 || f2 > 300.0)
      std::cout << f1 << " " << f2 << std::endl;
    return create(f1, f2, 0.0);
  }

  descriptor_t operator ()(detection_t const& det){
    descriptor_t res = Extractor::operator ()(det);

    pcl::PointCloud<pcl::RGB> cloud;
    pcl::fromROSMsg(det.cloud, cloud);

    static std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0,cloud.size()-1);


    std::vector<point_t> triangles;
    for(auto p: cloud){
      point_t pi = fromRGB(p);
      triangles.push_back(pi);
    }

    sn::Histogram2D descriptor(5, 5, -100.0, 300.0);
    for(auto p: triangles)
      descriptor.add(p);
//    descriptor.normalize(triangles.size());
    descriptor.inf_normalize();
    res.data = descriptor.get_feature();
    histogram = descriptor;
    return res;
  }

  bool is_valid(detection_t const& det)const{return det.cloud.data.size()>0;}

  Histogram2D get_histogram(){return histogram;}

};
}

#endif // FEATURE_EXTRACTOR_H
