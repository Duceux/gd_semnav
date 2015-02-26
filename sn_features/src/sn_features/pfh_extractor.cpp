#include <sn_features/pfh_extractor.h>
#include <pcl/features/pfh.h>
#include <pcl_conversions/pcl_conversions.h>

namespace sn {

void PFHExtractor::set_params()
{
    fAngleFactor = static_cast<double>(nbrBinsAngles) / M_PI;
    fDistFactor = static_cast<double>(nbrBinsDist);
    sizeHist = nbrBinsAngles*nbrBinsAngles*nbrBinsAngles*nbrBinsDist;
    base1 = 1;
    base2 = base1 * nbrBinsAngles;
    base3 = base2 * nbrBinsAngles;
    base4 = base3 * nbrBinsDist;
}

void PFHExtractor::set_params(const std::map<std::string, std::string> &params)
{
    try {
        type = params.at("type");
        nbrPointsPairInHist = boost::lexical_cast<int>(params.at("nbrPointsPairInHist"));
        nbrBinsAngles = boost::lexical_cast<int>(params.at("nbrBinsAngles"));
        nbrBinsDist = boost::lexical_cast<int>(params.at("nbrBinsDist"));
        set_params();
    } catch( boost::bad_lexical_cast const& e) {
        std::cout << e.what() << std::endl;
    }
}

descriptor_t PFHExtractor::operator ()(const detection_t &det)
{
    CloudInPtr cloud(new CloudInPtr::element_type);
    NormalPtr nomals(new NormalPtr::element_type);
    pcl::fromROSMsg(det.cloud, *cloud);
    pcl::fromROSMsg(det.normals, *nomals);

    descriptor_t res = Extractor::operator ()(det);
    res.data = compute_pfh(cloud, nomals);
    return res;
}

feature_t PFHExtractor::compute_pfh(PFHExtractor::CloudInPtr cloudPtr, PFHExtractor::NormalPtr normalsCloudPtr)
{

    if (cloudPtr->size() != normalsCloudPtr->size())
    {
        std::cerr << "[SetInputClouds] Warning: Point cloud and normals cloud sizes differ" << std::endl;
    }

    // Find the maximum distance between two points of the point cloud (not exact, non-deterministic)
    double mdist = calc_max_dist(cloudPtr);
    feature_t feature = compute_global_model(cloudPtr, normalsCloudPtr, mdist);
    return feature;
}

double PFHExtractor::calc_max_dist(PFHExtractor::CloudInPtr cloudPtr)
{
    double maxDist = 0.0 ;

    if (cloudPtr->size()*cloudPtr->size() < nbrPointsPairInHist)
    {
        for (std::size_t i=0; i < cloudPtr->size(); ++i)
        {
            for (std::size_t j=i+1; j < cloudPtr->size(); ++j)
            {
                cld_point_t &pt1 = cloudPtr->points[i], &pt2 = cloudPtr->points[j];
                Eigen::Vector4f p1(pt1.x, pt1.y, pt1.z , 0.);
                Eigen::Vector4f p2(pt2.x, pt2.y, pt2.z , 0.);

                // Compute squared distance between points and save the largest value
                Eigen::Vector4f dist = p1-p2;
                float d = dist.dot(dist);
                if (d > maxDist)
                {
                    maxDist = d;
                }
            }
        }
    }
    else
    {
        for(unsigned int i= 0; i < nbrPointsPairInHist; i++)
        {
            // Pick two points at random
            size_t n1, n2;
            n1 = (int) ( ((float)rand())/((float)RAND_MAX) * ((float)cloudPtr->size())) ;
            do
            {
                n2 = (int) ( ((float)rand())/((float)RAND_MAX) * ((float)cloudPtr->size())) ;
            }
            while (n1 == n2);
            cld_point_t &pt1 = cloudPtr->points[n1], &pt2 = cloudPtr->points[n2];
            Eigen::Vector4f p1(pt1.x, pt1.y, pt1.z , 0.);
            Eigen::Vector4f p2(pt2.x, pt2.y, pt2.z , 0.);

            // Compute squared distance between points and save the largest value
            Eigen::Vector4f dist = p1-p2;
            float d = dist.dot(dist);
            if (d > maxDist)
            {
                maxDist = d;
            }
        }
    }
    // Take square root of saved value
    maxDist = sqrt(maxDist);
    return maxDist;
}

feature_t PFHExtractor::compute_global_model(CloudInPtr cloudPtr, NormalPtr normalsCloudPtr, double maxDist)
{
    // Clear histogram
    feature_t objectModel;
    objectModel.assign(sizeHist, 0.0);

    if (cloudPtr->size()*cloudPtr->size() < nbrPointsPairInHist)
    {
        for (std::size_t i=0; i < cloudPtr->size(); ++i)
        {
            for (std::size_t j=i+1; j < cloudPtr->size(); ++j)
            {
                cld_point_t& pt1 = cloudPtr->points[i];
                cld_point_t& pt2 = cloudPtr->points[j];
                pcl::Normal& npt1 = normalsCloudPtr->points[i];
                pcl::Normal& npt2 = normalsCloudPtr->points[j];
                Eigen::Vector4f p1(pt1.x, pt1.y, pt1.z , 0.);
                Eigen::Vector4f p2(pt2.x, pt2.y, pt2.z , 0.);
                Eigen::Vector4f np1(npt1.normal_x, npt1.normal_y, npt1.normal_z, 0.);
                Eigen::Vector4f np2(npt2.normal_x, npt2.normal_y, npt2.normal_z, 0.);

                // Compute features
                float f1, f2, f3, f4;
                pcl::computePairFeatures(p1, np1, p2, np2, f1, f2, f3, f4);

                // Normalize the features and find what histogram bin they will be added to
                int df4 = std::min(std::max(0, static_cast<int>(f4 / maxDist * fDistFactor)), nbrBinsDist-1);
                int df3 = std::min(std::max(0, static_cast<int>((f3+M_PI_2) * fAngleFactor)), nbrBinsAngles-1);
                int df2 = std::min(std::max(0, static_cast<int>((f2+M_PI_2) * fAngleFactor)), nbrBinsAngles-1);
                int df1 = std::min(std::max(0, static_cast<int>((f1+M_PI_2) * fAngleFactor)), nbrBinsAngles-1);
                size_t histInd = base1*df1+base2*df2+base3*df3+base4*df4;

                // Add the point in the histogram
                objectModel[histInd] += 1.0;
            }
        }
    }
    else
    {
        for(size_t i = 0; i < static_cast<size_t>(nbrPointsPairInHist); i++)
        {
            // Pick two points at random
            size_t n1, n2;
            n1 = (size_t) ( ((float)rand())/((float)RAND_MAX) * ((float)cloudPtr->size()-1)) ;
            do
            {
                n2 = (size_t) ( ((float)rand())/((float)RAND_MAX) * ((float)cloudPtr->size()-1)) ;
            }
            while (n1 == n2);
            assert(n1 < cloudPtr->size() && n2 < cloudPtr->size());
            cld_point_t& pt1 = cloudPtr->points[n1];
            cld_point_t& pt2 = cloudPtr->points[n2];
            pcl::Normal& npt1 = normalsCloudPtr->points[n1];
            pcl::Normal& npt2 = normalsCloudPtr->points[n2];
            Eigen::Vector4f p1(pt1.x, pt1.y, pt1.z , 0.);
            Eigen::Vector4f p2(pt2.x, pt2.y, pt2.z , 0.);
            Eigen::Vector4f np1(npt1.normal_x, npt1.normal_y, npt1.normal_z, 0.);
            Eigen::Vector4f np2(npt2.normal_x, npt2.normal_y, npt2.normal_z, 0.);

            // Compute features
            float f1, f2, f3, f4;
            pcl::computePairFeatures(p1, np1, p2, np2, f1, f2, f3, f4);

            // Normalize the features and find what histogram bin they will be added to
            int df4 = std::min(std::max(0, static_cast<int>(f4 / maxDist * fDistFactor)), nbrBinsDist-1);
            int df3 = std::min(std::max(0, static_cast<int>((f3+M_PI_2) * fAngleFactor)), nbrBinsAngles-1);
            int df2 = std::min(std::max(0, static_cast<int>((f2+M_PI_2) * fAngleFactor)), nbrBinsAngles-1);
            int df1 = std::min(std::max(0, static_cast<int>((f1+M_PI_2) * fAngleFactor)), nbrBinsAngles-1);
            size_t histInd = base1*df1+base2*df2+base3*df3+base4*df4;

            // Add the point in the histogram
            objectModel[histInd] += 1.0;
        }
    }

    // Compute sum of histogram's bins
    float sumHist = 0.0;
    for(size_t i = 0; i < sizeHist; i++)
    {
        sumHist += objectModel[i];
    }
    // Normalize histogram
    for(size_t i = 0; i < sizeHist; i++)
    {
        objectModel[i] /= sumHist;
    }
    return objectModel;
}




}
