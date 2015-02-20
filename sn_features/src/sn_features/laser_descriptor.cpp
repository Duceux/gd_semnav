#include <sn_features/laser_descriptor.h>
#include <sn_features/histogram.h>

namespace sn {
#define PSEUDOZERO 0.0001

void up_sampling(vector_pts_t const& input, vector_pts_t& output, double resolution){
    output.clear();
    output.reserve(input.size()*10);
    for(uint i=0; i<input.size()-1;++i){
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
    int step = input.size()/nb;
    if(step==0)step=1;
    for(int i=0; i<input.size(); i+=step)
        output.push_back(input[i]);
}


point_t toPolar(point_t const& p){
    return create(l2_norm(p), std::atan2(p.y, p.x), 0.0);
}

void pair_of_points(vector_pts_t const& input, vector_pts_t& output){
    output.clear();
    output.reserve(input.size()*(input.size()-1));
    for(uint i=0; i<input.size()-1; ++i)
        for(uint j=i; j<input.size(); ++j)
        {
            if(i==j)continue;
            point_t p = input[j]-input[i];
            if(std::isnan(p.x) || std::isnan(p.y))continue;
            output.push_back(p);
            //            output.push_back(create(-p.y,p.x,0.0));
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
    int nbins = 100;
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
    if(input.size() > neigh)
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

}