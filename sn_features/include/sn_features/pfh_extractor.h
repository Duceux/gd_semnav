#ifndef PFH_EXTRACTOR_H
#define PFH_EXTRACTOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sn_features/feature_extractor.h>

namespace sn {

struct PFHExtractor: public Extractor{
    typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudInPtr;
    typedef pcl::PointCloud<pcl::Normal>::Ptr NormalPtr;
    typedef pcl::PointXYZRGB cld_point_t;

    // Nbr of point pairs used to build histograms
    int nbrPointsPairInHist;
    // Nbr of bins for angle features (f1,f2,f3) and distance feature (f4)
    int nbrBinsAngles, nbrBinsDist;
    // Factors to discretize the features
    double fAngleFactor, fDistFactor;
    // Size of the histogram
    size_t sizeHist;
    // Multipliers used to move in the histogram (based on number of bins)
    unsigned int base1, base2, base3, base4;

    void set_params();

    void set_params(std::map<std::string, std::string> const& params);


    descriptor_t operator ()(detection_t const& det);

    feature_t compute_pfh(CloudInPtr cloudPtr, NormalPtr normalsCloudPtr);
    double calc_max_dist(CloudInPtr cloudPtr);
    feature_t compute_global_model(CloudInPtr cloudPtr, NormalPtr normalsCloudPtr, double maxDist);

    bool is_valid(detection_t const& det)const{return det.cloud.data.size()>0;}

};

}


#endif // PFH_EXTRACTOR_H
