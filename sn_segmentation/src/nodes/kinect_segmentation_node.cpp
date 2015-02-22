#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <ros/package.h>
#include <sn_msgs/DetectionArray.h>
#include <sn_msgs/ImageWithMask.h>
#include <sn_msgs/BoundingBox.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "sn_segmentation/pt_cld_segmentation.h"

template <typename T>
class PtCldSegmentationNode : public PtCldSegmentation<T>
{
  typedef boost::shared_ptr<pcl::PointCloud<T> > CloudPtr;
  typedef boost::shared_ptr<const pcl::PointCloud<T> > ConstCloudPtr;

public:
  PtCldSegmentationNode(ros::NodeHandle n);
  virtual ~PtCldSegmentationNode();

  void LoadROSParams();
  bool srv_EstimateMainPlaneCoefs(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool srv_RedoMainPlaneEstimation(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool srv_LoadRosParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void cb_PtCld(const ConstCloudPtr& msg_ptr_point_cloud);

private:
  ros::Publisher pubPtCldDebug;
  ros::Publisher pubPtCldSegObjects;
  ros::Publisher pubImageAndMaskSegObjects;
  ros::Publisher pubPosAndBBoxSegObjects;
  ros::Subscriber subPtCld;
  ros::ServiceServer srvEstimateMainPlaneCoefs;
  ros::ServiceServer srvRedoMainPlaneEstimation;
  ros::ServiceServer srvLoadRosParams;
};

template <typename T>
PtCldSegmentationNode<T>::PtCldSegmentationNode(ros::NodeHandle n)
{
  pubPtCldDebug = n.advertise<pcl::PointCloud<T> >("debug_cloud", 1);
//  pubPtCldSegObjects = n.advertise<common_msgs_srvs::v_point_and_normal_clds>("v_objects_clouds", 1);
  pubPtCldSegObjects = n.advertise<sn_msgs::DetectionArray>("/detections", 1);
//  pubImageAndMaskSegObjects = n.advertise<common_msgs_srvs::v_images_and_masks>("v_objects_image_and_mask", 1);
//  pubPosAndBBoxSegObjects = n.advertise<common_msgs_srvs::v_positions_and_bounding_boxes>("v_objects_pos_and_bbox", 1);
  std::string kinect_input;
  ros::param::param<std::string>("~input", kinect_input, "/camera/depth_registered/points");
  subPtCld = n.subscribe<pcl::PointCloud<T> >(kinect_input, 1, &PtCldSegmentationNode<T>::cb_PtCld, this);

  srvRedoMainPlaneEstimation = n.advertiseService("redo_main_plane_estimation", &PtCldSegmentationNode<T>::srv_RedoMainPlaneEstimation, this);
  srvEstimateMainPlaneCoefs = n.advertiseService("estimate_main_plane_coefs", &PtCldSegmentationNode<T>::srv_EstimateMainPlaneCoefs, this);
  srvLoadRosParams = n.advertiseService("load_ros_params", &PtCldSegmentationNode<T>::srv_LoadRosParams, this);

  LoadROSParams();
}

template <typename T>
PtCldSegmentationNode<T>::~PtCldSegmentationNode()
{
}

template <typename T>
void PtCldSegmentationNode<T>::LoadROSParams()
{
  // Debug
  ros::param::get("~intDebug", this->debug_);
  std::stringstream ss;
  ros::param::param<std::string>("~strFileFloorPlane", this->strFileFloorPlane_, this->strFileFloorPlane_);
  ss << ros::package::getPath("sn_segmentation") << "/" << this->strFileFloorPlane_;
  this->strFileFloorPlane_ = ss.str();
  ros::param::param<double>("~fltRANSACFloorDistThresh", this->fltRANSACFloorDistThresh_, this->fltRANSACFloorDistThresh_);
  ros::param::param<double>("~fltVGLeafSize", this->fltVGLeafSize_, this->fltVGLeafSize_);
  ros::param::param<double>("~fltNEMaxDepth", this->fltNEMaxDepth_, this->fltNEMaxDepth_);
  ros::param::param<double>("~fltNESmoothing", this->fltNESmoothing_, this->fltNESmoothing_);
  ros::param::param<int>("~intFloorMinNbrPoints", this->intFloorMinNbrPoints_, this->intFloorMinNbrPoints_);
  ros::param::param<double>("~fltCBFloorMaxPlaneDist", this->fltCBFloorMaxPlaneDist_, this->fltCBFloorMaxPlaneDist_);
  ros::param::param<double>("~fltCBFloorMinPlaneDist", this->fltCBFloorMinPlaneDist_, this->fltCBFloorMinPlaneDist_);
  ros::param::param<double>("~fltMaxDistFromKinect", this->fltMaxDistFromKinect_, this->fltMaxDistFromKinect_);
  ros::param::param<double>("~fltFloorAngleThresh", this->fltFloorAngleThresh_, this->fltFloorAngleThresh_);
  ros::param::param<double>("~fltFloorDistThresh", this->fltFloorDistThresh_, this->fltFloorDistThresh_);
  ros::param::param<double>("~fltRANSACWallsDistThresh", this->fltRANSACWallsDistThresh_, this->fltRANSACWallsDistThresh_);
  ros::param::param<int>("~intMinNbrPointInWall", this->intMinNbrPointInWall_, this->intMinNbrPointInWall_);
  ros::param::param<double>("~fltWallAngleThresh", this->fltWallAngleThresh_, this->fltWallAngleThresh_);
  ros::param::param<double>("~fltECDistance", this->fltECDistance_, this->fltECDistance_);
  ros::param::param<int>("~intECMinSize", this->intECMinSize_, this->intECMinSize_);
  ros::param::param<int>("~intECMaxSize", this->intECMaxSize_, this->intECMaxSize_);
  ros::param::param<int>("~int2dBorderTolerance", this->int2dBorderTolerance_, this->int2dBorderTolerance_);
  ros::param::param<double>("~flt3dBorderTolerance", this->flt3dBorderTolerance_, this->flt3dBorderTolerance_);
  ros::param::param<double>("~kinect/fltFocalConstantX", this->fltFocalConstantX_, this->fltFocalConstantX_);
  ros::param::param<double>("~kinect/fltFocalConstantY", this->fltFocalConstantY_, this->fltFocalConstantY_);
  ros::param::param<double>("~kinect/fltCenterX", this->fltCenterX_, this->fltCenterX_);
  ros::param::param<double>("~kinect/fltCenterY", this->fltCenterY_, this->fltCenterY_);

  ROS_INFO("kinect segmentation ~intDebug: %i", this->debug_);
  ROS_INFO("kinect segmentation ~strFileFloorPlane: %s", ss.str().c_str());
  ROS_INFO("kinect segmentation ~fltRANSACFloorDistThresh: %f", this->fltRANSACFloorDistThresh_);
  ROS_INFO("kinect segmentation ~fltVGLeafSize: %f", this->fltVGLeafSize_);


  this->SetParams();
}

template <typename T>
bool PtCldSegmentationNode<T>::srv_EstimateMainPlaneCoefs(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  return this->EstimateMainPlaneCoefs();
}

template <typename T>
bool PtCldSegmentationNode<T>::srv_RedoMainPlaneEstimation(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  this->bFloorPlaneEstimationDone = false;
  return true;
}

template <typename T>
bool PtCldSegmentationNode<T>::srv_LoadRosParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  LoadROSParams();
  return true;
}

template <typename T>
void PtCldSegmentationNode<T>::cb_PtCld(const ConstCloudPtr& msg_ptr_point_cloud)
{
  ros::Time beg = ros::Time::now();
  // Segment point cloud using PtCldSegmentation
  this->SetInputCloud(msg_ptr_point_cloud);
  if(!this->bFloorPlaneEstimationDone)
  {
    if (!this->EstimateMainPlaneCoefs())
    {
      ROS_DEBUG("Floor plane estimation failed.");
      return;
    }
  }
  this->Segment();
  ros::Duration elapsed = ros::Time::now() - beg;
  ROS_DEBUG("Segmentation done, in %ld ms\n", elapsed.toNSec()/1000000);

  std_msgs::Header header = pcl_conversions::fromPCL(msg_ptr_point_cloud->header);
  // Publish the vectors of segmented objects information
  size_t nbObjects = this->vCldPtrSegObjects_.size();
  sn_msgs::DetectionArray msg_v_clouds_seg_objects;
  msg_v_clouds_seg_objects.header = header;
  msg_v_clouds_seg_objects.detections.resize(nbObjects);
//  common_msgs_srvs::v_images_and_masks msg_v_image_and_mask_seg_objects;
//  msg_v_image_and_mask_seg_objects.header = header;
//  msg_v_image_and_mask_seg_objects.vImagesAndMasks.resize(nbObjects);
//  common_msgs_srvs::v_positions_and_bounding_boxes msg_v_position_and_bbox_seg_objects;
//  msg_v_position_and_bbox_seg_objects.header = header;
//  msg_v_position_and_bbox_seg_objects.vPosAndBBox.resize(nbObjects);
  for (size_t i = 0; i < nbObjects; ++i)
  {
    sensor_msgs::PointCloud2::Ptr msgPoints(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*this->vCldPtrSegObjects_[i], *msgPoints);
    sensor_msgs::PointCloud2::Ptr msgNormals(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*this->vNCldPtrSegObjects_[i], *msgNormals);
    msg_v_clouds_seg_objects.detections[i].header = msgPoints->header;
    msg_v_clouds_seg_objects.detections[i].cloud = *msgPoints;
    msg_v_clouds_seg_objects.detections[i].normals = *msgNormals;

    sn_msgs::ImageWithMask msg_image_and_mask;
    msg_image_and_mask.header = msgPoints->header;
    std::string encodingImg("bgr8");
    cv::Mat matImg(this->v2dBBoxSegObjects_[i].vMax-this->v2dBBoxSegObjects_[i].vMin+1, this->v2dBBoxSegObjects_[i].uMax-this->v2dBBoxSegObjects_[i].uMin+1, CV_8UC3, &this->vImageSegObjects_[i].at(0));
    cv_bridge::CvImagePtr imagePtrImg(new cv_bridge::CvImage(msgPoints->header, encodingImg, matImg));
    imagePtrImg->toImageMsg(msg_image_and_mask.image);
    msg_image_and_mask.uMin = this->v2dBBoxSegObjects_[i].uMin;
    msg_image_and_mask.uMax = this->v2dBBoxSegObjects_[i].uMax;
    msg_image_and_mask.vMin = this->v2dBBoxSegObjects_[i].vMin;
    msg_image_and_mask.vMax = this->v2dBBoxSegObjects_[i].vMax;
    std::string encodingMask("mono8");
    cv::Mat matMask(this->v2dBBoxSegObjects_[i].vMax-this->v2dBBoxSegObjects_[i].vMin+1, this->v2dBBoxSegObjects_[i].uMax-this->v2dBBoxSegObjects_[i].uMin+1, CV_8UC1, &this->vMaskSegObjects_[i].at(0));
    cv_bridge::CvImagePtr imagePtrMask(new cv_bridge::CvImage(msgPoints->header, encodingMask, matMask));
    imagePtrMask->toImageMsg(msg_image_and_mask.mask);
    msg_v_clouds_seg_objects.detections[i].img = msg_image_and_mask;

    sn_msgs::BoundingBox msg_pos_and_bbox;
    msg_pos_and_bbox.header = msgPoints->header;
    msg_pos_and_bbox.center.x = this->vPtCldPosAnd3dBBoxSegObjects_[i].pos[0];
    msg_pos_and_bbox.center.y = this->vPtCldPosAnd3dBBoxSegObjects_[i].pos[1];
    msg_pos_and_bbox.center.z = this->vPtCldPosAnd3dBBoxSegObjects_[i].pos[2];
    msg_pos_and_bbox.min.x = this->vPtCldPosAnd3dBBoxSegObjects_[i].min[0];
    msg_pos_and_bbox.min.y = this->vPtCldPosAnd3dBBoxSegObjects_[i].min[1];
    msg_pos_and_bbox.min.z = this->vPtCldPosAnd3dBBoxSegObjects_[i].min[2];
    msg_pos_and_bbox.max.x = this->vPtCldPosAnd3dBBoxSegObjects_[i].max[0];
    msg_pos_and_bbox.max.y = this->vPtCldPosAnd3dBBoxSegObjects_[i].max[1];
    msg_pos_and_bbox.max.z = this->vPtCldPosAnd3dBBoxSegObjects_[i].max[2];
    msg_v_clouds_seg_objects.detections[i].bbox = msg_pos_and_bbox;
  }
  if (this->debug_)
    pubPtCldDebug.publish(this->GetDebugCldPtr());
  if (nbObjects > 0)
  {
    pubPtCldSegObjects.publish(msg_v_clouds_seg_objects);
//    pubImageAndMaskSegObjects.publish(msg_v_image_and_mask_seg_objects);
//    pubPosAndBBoxSegObjects.publish(msg_v_position_and_bbox_seg_objects);
  }
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "kinect_segmentation_node");
  ros::NodeHandle n(std::string("~"));

  PtCldSegmentationNode<pcl::PointXYZRGB> node(n);
  ros::spin();

  return 0;
}
