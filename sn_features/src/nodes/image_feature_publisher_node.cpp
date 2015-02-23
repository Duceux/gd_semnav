#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <ros/package.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>

#include <sn_msgs/DetectionArray.h>
#include <sn_msgs/DescriptorArray.h>

#include "sn_features/bowtools.h"

class ImageFeaturePublisherNode
{
public:
  ImageFeaturePublisherNode(ros::NodeHandle n);
  virtual ~ImageFeaturePublisherNode();

  void LoadROSParams();
  bool srv_LoadRosParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  void cb_VImagesAndMasks(const sn_msgs::DetectionArrayConstPtr& msg_ptr_v_images_and_masks);

    void toDescriptorMsg(cv_bridge::CvImagePtr const& imgptr, sn_msgs::Descriptor& dess);

private:
  ros::Publisher pubFeature;
  ros::Subscriber subVImagesSegObjects;
  ros::ServiceServer srvLoadRosParams;

  // For bag of words features
  opencvBOWForSmallImgs _bow;
  std::string _strVocabularyFilePath;
  bool _bVocLoaded;

  std::string _feature_type;

};

void ImageFeaturePublisherNode::toDescriptorMsg(cv_bridge::CvImagePtr const& imgptr, sn_msgs::Descriptor& dess){
    dess.timestamp = imgptr->header.stamp;
    //TODO
}


ImageFeaturePublisherNode::ImageFeaturePublisherNode(ros::NodeHandle n)
{
  pubFeature = n.advertise<sn_msgs::DetectionArray>("descriptors", 1);
  subVImagesSegObjects = n.subscribe<sn_msgs::DetectionArray>("/pt_cld_segmentation_node/v_objects_image_and_mask", 1, &ImageFeaturePublisherNode::cb_VImagesAndMasks, this);

  srvLoadRosParams = n.advertiseService("load_ros_params", &ImageFeaturePublisherNode::srv_LoadRosParams, this);

  LoadROSParams();
  ROS_INFO_STREAM(n.getNamespace() << " spin!");
}

ImageFeaturePublisherNode::~ImageFeaturePublisherNode()
{
}

void ImageFeaturePublisherNode::LoadROSParams()
{
  ros::param::get("~str_feature_type", _feature_type);

  std::stringstream ss;
  ros::param::param<std::string>("~str_vocabulary_file_path", _strVocabularyFilePath, _strVocabularyFilePath);
  ss << ros::package::getPath("features") << "/" << _strVocabularyFilePath;
  _strVocabularyFilePath = ss.str();

  if (_feature_type == "bow_sift")
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
    ros::param::param<int>("~int_feature_vector_size", _bow.hbins, _bow.hbins);
    ROS_INFO_STREAM("Image feator vector size: " << _bow.hbins << ".");
  }
}

bool ImageFeaturePublisherNode::srv_LoadRosParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  LoadROSParams();
  return true;
}

void ImageFeaturePublisherNode::cb_VImagesAndMasks(const sn_msgs::DetectionArrayConstPtr& msg_ptr_v_images_and_masks)
{
  size_t nbObjects = msg_ptr_v_images_and_masks->detections.size();

  sn_msgs::DescriptorArray msg_v_features;
  msg_v_features.header = msg_ptr_v_images_and_masks->header;
  msg_v_features.descriptors.resize(nbObjects);

  for (size_t i = 0; i < nbObjects; ++i)
  {
    // Get image and mask
    cv_bridge::CvImagePtr imagePtr = cv_bridge::toCvCopy(msg_ptr_v_images_and_masks->detections[i].img.image);
    cv_bridge::CvImagePtr maskPtr = cv_bridge::toCvCopy(msg_ptr_v_images_and_masks->detections[i].img.mask);

    cv_bridge::CvImagePtr cvPtr(new cv_bridge::CvImage);
    cvPtr->header = imagePtr->header;
    cvPtr->encoding = "32FC1";

    if (_feature_type == "bow_sift")
    {
      // Bow sift features
      _bow.compute(imagePtr->image, cvPtr->image, maskPtr->image);
      if (cvPtr->image.rows == 0)
        if (_bVocLoaded)
          cvPtr->image = cv::Mat::zeros(1, _bow.getvocabrows(), CV_32FC1);
        else
          ROS_ERROR_STREAM("Vocabulary file " << _strVocabularyFilePath << " not loaded for feature " << _feature_type << ".");
    }
    else if (_feature_type == "sift")
    {
      // Sift features
      _bow.compute_feature(imagePtr->image, cvPtr->image, maskPtr->image);
      if (cvPtr->image.rows == 0)
        cvPtr->image = cv::Mat::zeros(1, 128, CV_32FC1);
    }
    else if (_feature_type == "tbgr")
    {
      // tbgr features
      _bow.compute_tbgr_feature(imagePtr->image, cvPtr->image, maskPtr->image);
    }
    else if (_feature_type == "hovers")
    {
      // HOverS features
      _bow.compute_hovers_feature(imagePtr->image, cvPtr->image, maskPtr->image);
    }
    else if (_feature_type == "hue")
    {
      // Hue features
      _bow.compute_hsv_feature(imagePtr->image, cvPtr->image, maskPtr->image);
      if (cvPtr->image.rows == 0)
        cvPtr->image = cv::Mat::zeros(1, _bow.hbins, CV_32FC1);
    }
//    cvPtr->toImageMsg(msg_v_features.v_features[i]);
    toDescriptorMsg(cvPtr, msg_v_features.descriptors[i]);
   }

  if (nbObjects > 0)
  {
    pubFeature.publish(msg_v_features);
  }
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "image_feature_publisher_node");
  ros::NodeHandle n(std::string("~"));

  ImageFeaturePublisherNode node(n);

  ros::spin();

  return 0;
}
