#include <sn_features/image_extractor.h>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>


namespace sn{

void ImageDescriptorExtractor::set_params(const std::map<std::string, std::string> &params)
{
    type = params.at("type");
    auto copy = params;
    _strVocabularyFilePath = copy["str_vocabulary_file_path"];
    if (type == "bow_sift")
    {
      _bVocLoaded = _bow.loadVocabulary(const_cast<char*>(_strVocabularyFilePath.c_str()));
      if (_bVocLoaded)
      {
        ROS_INFO_STREAM("Vocabulary successfully loaded.");
      }
      else
      {
        ROS_INFO_STREAM("Vocabulary file: " << _strVocabularyFilePath << " not found.");
      }
    }
    else
    {
      _bVocLoaded = false;
      _bow.hbins = boost::lexical_cast<int>(params.at("int_feature_vector_size"));
      ROS_INFO_STREAM("Image feator vector size: " << _bow.hbins << ".");
    }
}

descriptor_t ImageDescriptorExtractor::operator ()(detection_t const& det){
    descriptor_t des = Extractor::operator ()(det);


    // Get image and mask
    cv_bridge::CvImagePtr imagePtr = cv_bridge::toCvCopy(det.img.image);
    cv_bridge::CvImagePtr maskPtr = cv_bridge::toCvCopy(det.img.mask);

    cv_bridge::CvImagePtr cvPtr(new cv_bridge::CvImage);
    cvPtr->header = imagePtr->header;
    cvPtr->encoding = "32FC1";

    if (type == "bow_sift")
    {
      // Bow sift features
      _bow.compute(imagePtr->image, cvPtr->image, maskPtr->image);
      if (cvPtr->image.rows == 0)
        if (_bVocLoaded)
          cvPtr->image = cv::Mat::zeros(1, _bow.getvocabrows(), CV_32FC1);
        else
          ROS_ERROR_STREAM("Vocabulary file " << _strVocabularyFilePath << " not loaded for feature " << type << ".");
    }
    else if (type == "sift")
    {
      // Sift features
      _bow.compute_feature(imagePtr->image, cvPtr->image, maskPtr->image);
      if (cvPtr->image.rows == 0)
        cvPtr->image = cv::Mat::zeros(1, 128, CV_32FC1);
    }
    else if (type == "tbgr")
    {
      // tbgr features
      _bow.compute_tbgr_feature(imagePtr->image, cvPtr->image, maskPtr->image);
    }
    else if (type == "hovers")
    {
      // HOverS features
      _bow.compute_hovers_feature(imagePtr->image, cvPtr->image, maskPtr->image);
    }
    else if (type == "hue")
    {
      // Hue features
      _bow.compute_hsv_feature(imagePtr->image, cvPtr->image, maskPtr->image);
      if (cvPtr->image.rows == 0)
        cvPtr->image = cv::Mat::zeros(1, _bow.hbins, CV_32FC1);
    }
//    cvPtr->toImageMsg(msg_v_features.v_features[i]);
    toDescriptorMsg(cvPtr, des);
    return des;
}

void ImageDescriptorExtractor::toDescriptorMsg(const cv_bridge::CvImagePtr &imgptr, sn_msgs::Descriptor &des)
{
    des.data.clear();
    for(int i=0; i<imgptr->image.cols; ++i)
        des.data.push_back(imgptr->image.at<float>(0,i));
}

}
