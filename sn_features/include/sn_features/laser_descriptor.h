#ifndef LASER_DESCRIPTOR_H
#define LASER_DESCRIPTOR_H

#include <sn_msgs/Detection.h>
#include <sn_features/polar_histogram.h>
#include <sn_features/types.h>
#include <sn_features/feature_extractor.h>

namespace sn {

void up_sampling(vector_pts_t const& input, vector_pts_t& output, double resolution);

void down_sampling_nb(vector_pts_t const& input, vector_pts_t& output, int nb);

void down_sampling_step(vector_pts_t const& input, vector_pts_t& output, int step);

void pair_of_points(vector_pts_t const& input, vector_pts_t& output);

double compute_angle_ref(vector_pts_t const& input);

void compute_centered_oriented(vector_pts_t const& input, vector_pts_t& output);

void smooth(vector_pts_t const& input, vector_pts_t& output, int neigh);

void triangle_points(vector_pts_t const& input, vector_pts_t& output);

struct TriangleLaserExtractor: public Extractor{
    double sampling_resolution;
    int smoothing_factor;
    int downsampling_factor;
    double theta_bin_size;
    double rho_bin_size;
    PolarHistogram histogram;

    feature_t operator ()(vector_pts_t const& data);
    descriptor_t operator ()(detection_t const& data);
    PolarHistogram get_histogram(){return histogram;}

    void set_params(std::map<std::string, std::string> const& params);
    bool is_valid(detection_t const& det)const{return det.points.size()>0;}

};

}

#endif // LASER_DESCRIPTOR_H
