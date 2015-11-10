#include <sn_features/laser_descriptor.h>
#include <sn_features/histogram.h>
#include <sn_features/polar_histogram.h>

namespace sn {
#define PSEUDOZERO 0.0001

void up_sampling(vector_pts_t const& input, vector_pts_t& output, double resolution){
    output.clear();
    output.reserve(input.size()*10);
    int size = input.size()-1;
    for(int i=0; i<size;++i){
        point_t p = input[i+1]-input[i];
        int nb = std::floor(l2_norm(p)/resolution);
        p = p/l2_norm(p);
        if(nb == 0)
            output.push_back(input[i]);
        else
            for(int j=0; j<nb; ++j){
                point_t np = input[i] + p*j*resolution;
                output.push_back(np);
            }
    }
    output.push_back(input.back());
}

void down_sampling_step(vector_pts_t const& input, vector_pts_t& output, int step)
{
    output.clear();
    for(int i=0; i<input.size(); i+=step)
        output.push_back(input[i]);
}

void down_sampling_nb(vector_pts_t const& input, vector_pts_t& output, int nb)
{
    output.clear();
    output.reserve(nb);
    int step = std::ceil(input.size()/(double)nb);
    for(int i=0; i<input.size(); i+=step)
        output.push_back(input[i]);
    assert(output.size() <= nb);
}


point_t toPolar(point_t const& p){
    return create(l2_norm(p), std::atan2(p.y, p.x), 0.0);
}

void pair_of_points(vector_pts_t const& input, vector_pts_t& output){
    output.clear();
    output.reserve(input.size()*(input.size()-1));
    for(uint i=0; i<input.size(); ++i)
        for(uint j=0; j<input.size(); ++j)
        {
            if(i==j)continue;
            point_t p = input[j]-input[i];
            if(std::isnan(p.x) || std::isnan(p.y))continue;
            output.push_back(p);
            output.push_back(create(-p.y,p.x,0.0));
        }
}

void triangle_points(vector_pts_t const& input, vector_pts_t& output)
{
    output.clear();
    //    output.reserve(input.size()*(input.size()-1));
    for(uint i=0; i<input.size()-2; ++i)
        for(uint j=i+1; j<input.size()-1; ++j)
            for(uint k=j+1; k<input.size(); ++k)
            {
                double rho = l2_norm(input[k]-input[i]);
                point_t p1 = input[i]-input[j];
                point_t p2 = input[j]-input[k];
                double angle = atan2(p2.y,p2.x) - atan2(p1.y,p1.x);
                output.push_back(create(rho*cos(angle), rho*sin(angle), 0.0));
            }
    /*
    for(uint i=1; i<input.size()-1; ++i)
    {
        double rho = l2_norm(input[i-1]-input[i+1]);
        point_t p1 = input[i-1]-input[i];
        point_t p2 = input[i+1]-input[i];
        double angle = atan2(p2.y,p2.x) - atan2(p1.y,p1.x);
        output.push_back(create(rho*cos(angle), rho*sin(angle), 0.0));
    }
    */
}

double compute_angle_ref(vector_pts_t const& input){
    // compute angle and histogram
    // compute histogram
    int nbins = 128;
    float ranges[] = {-M_PI, M_PI};
    Histogram<int, float> angle_histo(nbins, CIRCULAR, ranges[0], ranges[1]);
    size_t size = input.size();
    for(size_t i=0; i<size; ++i){
        float angle = std::atan2(std::abs(input[i].y), std::abs(input[i].x));
        angle_histo.add(angle);
    }
    double theta = angle_histo.valMax();
    return theta;
}

void compute_centered_oriented(vector_pts_t const& input, vector_pts_t& output)
{
    auto center = compute_center(input);
    vector_pts_t tmp = transform(input, create_pose(center.x, center.y, 0.0));
    double ref = compute_angle_ref(tmp);
    output = transform(tmp, create_pose(0,0,ref));
}

void smooth(vector_pts_t const& input, vector_pts_t& output, int neigh){
    output.clear();
    output.reserve(input.size());
    if(input.size() > 2*neigh+1)
        for(int i=neigh; i<input.size()-neigh; ++i){
            point_t center = create(0,0,0);
            for(int j=-neigh; j<=neigh; ++j)
            {
                center+=input[i+j];
            }
            center/=(2*neigh+1);
            output.push_back(center);
        }
    else
        output = input;
}


std::vector<double> TriangleLaserExtractor::operator ()(vector_pts_t const& data){
    sn::vector_pts_t resampled, smoothed, downsampled, triangles;
    sn::up_sampling(data, resampled, sampling_resolution);
    sn::smooth(resampled, smoothed, smoothing_factor);
    sn::down_sampling_nb(smoothed, downsampled, downsampling_factor);
    sn::triangle_points(downsampled, triangles);
    sn::PolarHistogram descriptor(theta_bin_size, rho_bin_size);
    for(auto p: triangles)
        descriptor.add(p);
    //    descriptor.l1_normalize();
    descriptor.normalize(triangles.size());
    histogram = descriptor;
    return descriptor.get_feature();
}

descriptor_t TriangleLaserExtractor::operator ()(const detection_t &data)
{
    descriptor_t res = Extractor::operator ()(data);
    res.data = (*this)(data.points);
    return res;
}

void TriangleLaserExtractor::set_params(const std::map<std::string, std::string> &params)
{
    try {
        type = params.at("type");
        sampling_resolution = boost::lexical_cast<double>(params.at("sampling_resolution"));
        smoothing_factor = boost::lexical_cast<int>(params.at("smoothing_factor"));
        downsampling_factor = boost::lexical_cast<int>(params.at("downsampling_factor"));
        theta_bin_size = M_PI*2.0/boost::lexical_cast<double>(params.at("theta_bin_nb"));
        rho_bin_size = boost::lexical_cast<double>(params.at("rho_bin_size"));
    } catch( boost::bad_lexical_cast const& e) {
        std::cout << e.what() << std::endl;
  }
}

void PairOfPointsLaserExtractor::set_params(const std::map<std::string, std::string> &params)
{
  try {
      type = params.at("type");
      sampling_resolution = boost::lexical_cast<double>(params.at("sampling_resolution"));
      theta_bin_size = M_PI*2.0/boost::lexical_cast<double>(params.at("theta_bin_nb"));
      rho_bin_size = boost::lexical_cast<double>(params.at("rho_bin_size"));
  } catch( boost::bad_lexical_cast const& e) {
      std::cout << e.what() << std::endl;
}
}

descriptor_t PairOfPointsLaserExtractor::operator ()(const detection_t &data)
{
    descriptor_t res = Extractor::operator ()(data);
    res.data = (*this)(data.points);
    return res;
}

std::vector<double> PairOfPointsLaserExtractor::operator ()(vector_pts_t const& data){
    vector_pts_t resampled, ppts, ppts_reoriented;
    up_sampling(data, resampled, sampling_resolution);
    pair_of_points(resampled, ppts);
    double ref = compute_angle_ref(ppts);
    ppts_reoriented = sn::transform(ppts, create_pose(0,0,ref));
    PolarHistogram descriptor(theta_bin_size, rho_bin_size);
    for(auto p: ppts_reoriented)
        descriptor.add(p);
    descriptor.normalize(ppts_reoriented.size());
    histogram = descriptor;
    return descriptor.get_feature();
}

}
