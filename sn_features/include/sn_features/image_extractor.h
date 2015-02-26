#ifndef IMAGE_EXTRACTOR_H
#define IMAGE_EXTRACTOR_H

#include <sn_features/feature_extractor.h>
#include <sn_features/bowtools.h>
#include <cv_bridge/cv_bridge.h>



namespace sn{

struct ImageDescriptorExtractor: public Extractor{


    // For bag of words features
    opencvBOWForSmallImgs _bow;
    std::string _strVocabularyFilePath;
    bool _bVocLoaded;

    void set_params(std::map<std::string, std::string> const& params);

    descriptor_t operator ()(detection_t const& det);

    void toDescriptorMsg(cv_bridge::CvImagePtr const& imgptr, sn_msgs::Descriptor& des);

    bool is_valid(detection_t const& det)const{return det.img.image.data.size()>0;}

};

}


#endif // IMAGE_EXTRACTOR_H
